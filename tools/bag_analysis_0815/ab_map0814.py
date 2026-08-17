#!/usr/bin/env python3
"""백에 실린 맵(잘린 388x317) vs 디스크의 my_track0814(418x228) 로 같은 분석 재실행.

백의 /map 은 0814 를 x>5.0 에서 잘라낸 것이다 (점유셀 3313 vs 3387, 좌표계 동일).
그 절단이 레이캐스트를 "벽 없음"으로 만들어 진단을 오염시켰는지 확인한다.
"""
import pickle
import sys
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt

D = pickle.load(open(Path(__file__).parent / "data.pkl", "rb"))
pf = D["/pf/pose/odom"]
XL = 0.28
MARGIN = 1.5
PROBE = np.radians(1.5)
UNSTABLE = 0.5
MAX_RANGE = 13.5

REAL_OBS = [(-10.92, 0.58), (-5.50, -0.19), (-2.23, -0.09), (4.10, 4.26)]


def read_pgm(p):
    data = Path(p).read_bytes()
    tok, i = [], 0
    while len(tok) < 4:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while i < len(data) and data[i:i + 1] != b"\n":
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        tok.append(data[i:j])
        i = j
    i += 1
    w, h = int(tok[1]), int(tok[2])
    return np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w), w, h


class Grid:
    def __init__(self, occ, res, ox, oy, name):
        self.occ = occ
        self.res, self.ox, self.oy = res, ox, oy
        self.h, self.w = occ.shape
        self.edt = (distance_transform_edt(~occ) * res).astype(float)
        self.hit = res
        self.steps = int(MAX_RANGE / res) + 8
        self.name = name

    def at(self, x, y):
        c = np.floor((x - self.ox) / self.res).astype(np.int64)
        r = np.floor((y - self.oy) / self.res).astype(np.int64)
        ok = (c >= 0) & (c < self.w) & (r >= 0) & (r < self.h)
        o = np.full(np.shape(x), np.nan)
        o[ok] = self.edt[r[ok], c[ok]]
        return o

    def cast(self, x0, y0, ux, uy):
        n = len(ux)
        t = np.zeros(n)
        done = np.zeros(n, bool)
        for _ in range(self.steps):
            live = ~done & (t < MAX_RANGE)
            if not live.any():
                break
            d = self.at(x0 + ux * t, y0 + uy * t)
            out = np.isnan(d)
            t = np.where(live & out, MAX_RANGE, t)
            done |= live & out
            hit = live & ~out & (d <= self.hit)
            done |= hit
            t = t + np.nan_to_num(np.where(live & ~out & ~hit, np.maximum(d, self.hit), 0.0))
        return np.minimum(t, MAX_RANGE)


M = D["map"]
bag = Grid(M["data"].reshape(M["h"], M["w"]) > 50, M["res"], M["ox"], M["oy"], "백 맵(잘림)")
img, w, h = read_pgm("/home/sangbum/IFAC_ws/maps/my_track0814.pgm")
m14 = Grid(np.flipud(img) < int(255 * 0.25), 0.05, -14.243026, -3.015387, "my_track0814")

for G in (bag, m14):
    print(f"{G.name:16s} {G.w}x{G.h} 점유 {int(G.occ.sum())}")
print()


def run(G):
    n_beam = n_over = 0
    fp = 0
    n_real = 0
    rej_tot = 0
    resid = []
    for ts, amin, ainc, r in D["scan"][::3]:
        i = np.clip(np.searchsorted(pf[:, 0], ts), 0, len(pf) - 1)
        px, py, yaw = pf[i, 1], pf[i, 2], pf[i, 3]
        lx, ly = px + XL * np.cos(yaw), py + XL * np.sin(yaw)
        ang = amin + ainc * np.arange(len(r))
        ok = np.isfinite(r) & (r > 0.1) & (r < 12) & (np.abs(ang) < np.radians(80))
        if ok.sum() < 20:
            continue
        a, rr = ang[ok], r[ok].astype(float)
        xl, yl = rr * np.cos(a), rr * np.sin(a)
        xm = lx + xl * np.cos(yaw) - yl * np.sin(yaw)
        ym = ly + xl * np.sin(yaw) + yl * np.cos(yaw)
        # 정합 잔차 (전체 빔, 벽필터 전)
        dxa, dya = xm - lx, ym - ly
        rhoa = np.hypot(dxa, dya)
        ea = G.cast(lx, ly, dxa / rhoa, dya / rhoa)
        v = ea < MAX_RANGE
        dd = (rhoa - ea)[v]
        resid += list(dd[np.abs(dd) < 1.0])
        # 벽필터 통과분에 대해 레이 필터
        wd = G.at(xm, ym)
        k = np.isfinite(wd) & (wd >= 0.25)
        xm2, ym2 = xm[k], ym[k]
        if not len(xm2):
            continue
        dx, dy = xm2 - lx, ym2 - ly
        rho = np.hypot(dx, dy)
        ux, uy = dx / rho, dy / rho
        e0 = G.cast(lx, ly, ux, uy)
        over = (e0 < MAX_RANGE) & (rho > e0 + 0.40)
        n_beam += len(rho)
        n_over += int(over.sum())
        cand = (e0 < MAX_RANGE) & (rho > e0 + MARGIN)
        rej = np.zeros(len(rho), bool)
        if cand.any():
            cp, sp = np.cos(PROBE), np.sin(PROBE)
            e1 = G.cast(lx, ly, cp * ux[cand] - sp * uy[cand], sp * ux[cand] + cp * uy[cand])
            e2 = G.cast(lx, ly, cp * ux[cand] + sp * uy[cand], -sp * ux[cand] + cp * uy[cand])
            sprd = np.maximum.reduce([e0[cand], e1, e2]) - np.minimum.reduce([e0[cand], e1, e2])
            rej[np.where(cand)[0][sprd <= UNSTABLE]] = True
        rej_tot += int(rej.sum())
        isreal = np.zeros(len(xm2), bool)
        for ox_, oy_ in REAL_OBS:
            isreal |= np.hypot(xm2 - ox_, ym2 - oy_) < 0.35
        n_real += int(isreal.sum())
        fp += int((rej & isreal).sum())
    res = np.array(resid)
    return dict(beam=n_beam, over=n_over, rej=rej_tot, real=n_real, fp=fp,
                res_med=np.median(np.abs(res)), res_p90=np.percentile(np.abs(res), 90),
                nowall=None)


print(f"{'':16s} {'벽필터통과':>10s} {'넘김':>8s} {'기각':>7s} {'실물근처':>9s} "
      f"{'실물오검출':>11s} {'|잔차|med':>10s} {'p90':>7s}")
out = {}
for G in (bag, m14):
    s = run(G)
    out[G.name] = s
    print(f"{G.name:16s} {s['beam']:10d} {s['over']:7d}"
          f"({100*s['over']/s['beam']:4.1f}%) {s['rej']:6d} {s['real']:9d} "
          f"{s['fp']:7d}({100*s['fp']/max(s['real'],1):4.2f}%) "
          f"{s['res_med']:9.3f} {s['res_p90']:7.3f}")

print()
print("=== t=46.49 대각선 27점: 두 맵의 기대거리 비교 ===")
cand = min(((abs(s[0] - 46.5),) + s for s in D["scan"]))
ts, amin, ainc, r = cand[1:]
i = np.clip(np.searchsorted(pf[:, 0], ts), 0, len(pf) - 1)
px, py, yaw = pf[i, 1], pf[i, 2], pf[i, 3]
lx, ly = px + XL * np.cos(yaw), py + XL * np.sin(yaw)
ang = amin + ainc * np.arange(len(r))
ok = np.isfinite(r) & (r > 0.1) & (r < 12) & (np.abs(ang) < np.radians(80))
a, rr = ang[ok], r[ok].astype(float)
xl, yl = rr * np.cos(a), rr * np.sin(a)
xm = lx + xl * np.cos(yaw) - yl * np.sin(yaw)
ym = ly + xl * np.sin(yaw) + yl * np.cos(yaw)
kb = np.isfinite(bag.at(xm, ym)) & (bag.at(xm, ym) >= 0.25)
xs, ys, rs, as_ = xm[kb], ym[kb], rr[kb], a[kb]
dx, dy = xs - lx, ys - ly
rho = np.hypot(dx, dy)
eb = bag.cast(lx, ly, dx / rho, dy / rho)
e8 = m14.cast(lx, ly, dx / rho, dy / rho)
print("  az      실측    백맵기대   0814기대   백:차이   0814:차이")
o = np.argsort(as_)
for j in o:
    print(f"  {np.degrees(as_[j]):+7.2f} {rho[j]:7.2f} {eb[j]:9.2f} {e8[j]:9.2f} "
          f"{rho[j]-eb[j]:+9.2f} {rho[j]-e8[j]:+9.2f}")
print(f"  벽 못 찾은 방위: 백맵 {int((eb>=MAX_RANGE).sum())}개 / 0814 {int((e8>=MAX_RANGE).sum())}개")
print(f"  0814 기준 이 27점의 맵 벽거리: med {np.median(m14.at(xs,ys)):.2f} "
      f"max {np.nanmax(m14.at(xs,ys)):.2f} m")
