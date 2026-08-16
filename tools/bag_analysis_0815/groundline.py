#!/usr/bin/env python3
"""지면 히트 판정 (기하 정정판).

LiDAR 가 피치 th 만큼 숙이면 스캔 평면과 바닥의 교선은 '직선'이다.
레이저 좌표계에서 그 직선은 x ~= h/tan(th) = const (roll 0 기준),
즉 짧은 빔들의 (x,y) 가 x=const 선 위에 늘어선다.
-> 짧은 빔 점군에 직선을 적합해서
   (a) 잔차 RMS 가 작고 (b) 선이 진행방향에 거의 수직이면 지면 히트.
장애물(콘)이면 점군이 한 점에 뭉치므로 y 범위가 좁고 선적합이 무의미하다.
"""
import pickle
from pathlib import Path

import numpy as np

D = pickle.load(open(Path(__file__).parent / "data.pkl", "rb"))
pf = D["/pf/pose/odom"]
MAP = D["map"]
H = 0.1374
XL = 0.28
grid = MAP["data"].reshape(MAP["h"], MAP["w"])
occ = grid > 50
res, ox, oy = MAP["res"], MAP["ox"], MAP["oy"]
W, Hh = MAP["w"], MAP["h"]

_step = res * 0.5
_ds = np.arange(1, int(10.0 / _step) + 1) * _step


def raycast_many(x0, y0, angs):
    """여러 방향 동시 레이캐스트 -> 첫 점유까지 거리."""
    ca = np.cos(angs)[:, None]
    sa = np.sin(angs)[:, None]
    xs = ((x0 + ca * _ds) - ox) / res
    ys = ((y0 + sa * _ds) - oy) / res
    ix = xs.astype(np.int32)
    iy = ys.astype(np.int32)
    good = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < Hh)
    hit = np.zeros(ix.shape, bool)
    hit[good] = occ[iy[good], ix[good]]
    any_hit = hit.any(axis=1)
    k = hit.argmax(axis=1)
    out = np.where(any_hit, _ds[k], 10.0)
    return out


def frame_stats(ts, amin, ainc, r):
    i = np.clip(np.searchsorted(pf[:, 0], ts), 0, len(pf) - 1)
    px, py, yaw = pf[i, 1], pf[i, 2], pf[i, 3]
    lx = px + XL * np.cos(yaw)
    ly = py + XL * np.sin(yaw)
    n = len(r)
    ang = amin + ainc * np.arange(n)
    ok = np.isfinite(r) & (r > 0.05) & (r < 10.0) & (np.abs(ang) < np.radians(110))
    idx = np.where(ok)[0]
    if len(idx) < 30:
        return None
    exp = raycast_many(lx, ly, yaw + ang[idx])
    meas = r[idx]
    short = meas < exp - 0.35
    ns = int(short.sum())
    res_out = dict(t=ts, ns=ns, ntot=len(idx))
    if ns < 6:
        return res_out
    a = ang[idx][short]
    m = meas[short]
    x = m * np.cos(a)          # 레이저 프레임
    y = m * np.sin(a)
    # 주성분 -> 직선 적합
    P = np.stack([x, y], 1)
    P0 = P - P.mean(0)
    u, s, vt = np.linalg.svd(P0, full_matrices=False)
    d1, d2 = vt[0], vt[1]
    resid = P0 @ d2
    ext = P0 @ d1
    res_out.update(
        rms=float(np.sqrt((resid ** 2).mean())),
        length=float(ext.max() - ext.min()),
        x_med=float(np.median(x)),
        y_span=float(y.max() - y.min()),
        # 선 방향이 차체 y축(횡)과 이루는 각: 0deg = 진행방향에 수직 = 지면선
        tilt=float(np.degrees(np.arccos(np.clip(abs(d1[1]), 0, 1)))),
        pitch_est=float(np.degrees(np.arctan2(H, max(np.median(x), 1e-3)))),
    )
    return res_out


def scan_range(t0, t1, label):
    print(f"\n===== {label} [{t0},{t1}] =====")
    print("   t   짧은빔/전체  선길이m  잔차RMS  x_med  y폭   선기울기°  피치추정°  판정")
    for ts, amin, ainc, r in D["scan"]:
        if not (t0 <= ts <= t1):
            continue
        st = frame_stats(ts, amin, ainc, r)
        if st is None:
            continue
        if "rms" not in st:
            print(f"{ts:6.2f} {st['ns']:4d}/{st['ntot']:<4d}   -")
            continue
        # 지면선: 길이 >= 0.5 m, 잔차 <= 6 cm, 선이 진행방향에 수직(기울기<35deg)
        gl = (st["length"] >= 0.5 and st["rms"] <= 0.06 and st["tilt"] <= 35)
        tag = "지면선!" if gl else ("길고평평" if st["length"] >= 0.5 and st["rms"] <= 0.08
                                 else "점덩어리")
        print(f"{ts:6.2f} {st['ns']:4d}/{st['ntot']:<4d} {st['length']:8.2f} "
              f"{st['rms']:8.3f} {st['x_med']:6.2f} {st['y_span']:5.2f} "
              f"{st['tilt']:9.1f} {st['pitch_est']:9.1f}  {tag}")


# ---- 전 구간 스윕: 지면선 프레임을 모두 찾는다 ----
print("########## 전 구간 지면선 스윕 ##########")
rows = []
for ts, amin, ainc, r in D["scan"]:
    st = frame_stats(ts, amin, ainc, r)
    if st is None or "rms" not in st:
        rows.append((ts, 0, 0, 0, 9, 9, 0))
        continue
    rows.append((ts, st["ns"], st["length"], st["rms"], st["tilt"],
                 st["x_med"], st["pitch_est"]))
A = np.array(rows)
np.save(Path(__file__).parent / "groundline.npy", A)

gl = (A[:, 2] >= 0.5) & (A[:, 3] <= 0.06) & (A[:, 4] <= 35) & (A[:, 1] >= 6)
print(f"지면선 판정 프레임: {gl.sum()} / {len(A)} ({100*gl.mean():.1f}%)")

fre = D["/car_state/frenet/odom"]
t = fre[:, 0]
v = fre[:, 4]
alon = np.convolve(np.gradient(v, t), np.ones(5) / 5, mode="same")
imu = D["/sensors/imu/raw"]
a_at = np.interp(A[:, 0], t, alon)
wy_at = np.interp(A[:, 0], imu[:, 0], imu[:, 3])

print("\n종가속 구간별 지면선 발생률:")
for lo, hi, lbl in ((-99, -2.0, "강한감속 a<-2"), (-2.0, -0.8, "감속"),
                    (-0.8, 0.8, "정상"), (0.8, 2.0, "가속"), (2.0, 99, "강한가속 a>2")):
    m = (a_at >= lo) & (a_at < hi)
    if m.sum() < 20:
        continue
    print(f"  {lbl:14s} n={m.sum():5d}  지면선 {100*gl[m].mean():5.1f}%  "
          f"|wy| med {np.median(np.abs(wy_at[m])):5.1f} deg/s")

print("\n피치각속도 구간별 지면선 발생률:")
for lo, hi in ((0, 5), (5, 10), (10, 20), (20, 40), (40, 999)):
    m = (np.abs(wy_at) >= lo) & (np.abs(wy_at) < hi)
    if m.sum() < 20:
        continue
    print(f"  |wy| {lo:3d}-{hi:3d} deg/s  n={m.sum():5d}  지면선 {100*gl[m].mean():5.1f}%")

print("\n지면선 프레임의 추정 피치각 분포:")
pe = A[gl, 6]
if len(pe):
    print(f"  n={len(pe)} med {np.median(pe):.2f}deg  p10 {np.percentile(pe,10):.2f} "
          f"p90 {np.percentile(pe,90):.2f}  x_med med {np.median(A[gl,5]):.2f} m")

print("\n지면선 발생 시각 (연속 구간 병합):")
idxs = np.where(gl)[0]
if len(idxs):
    runs = np.split(idxs, np.where(np.diff(idxs) > 3)[0] + 1)
    for rr in runs:
        if len(rr) < 2:
            continue
        print(f"  {A[rr[0],0]:6.2f}-{A[rr[-1],0]:6.2f}s  ({len(rr)}프레임) "
              f"a_lon {a_at[rr].min():+.1f}~{a_at[rr].max():+.1f}  "
              f"x_med {np.median(A[rr,5]):.2f}m  피치 {np.median(A[rr,6]):.1f}deg")

for lbl, a, b in (("34.3", 33.5, 34.9), ("46.6", 45.9, 47.2), ("60.6", 60.0, 61.2)):
    scan_range(a, b, lbl)
