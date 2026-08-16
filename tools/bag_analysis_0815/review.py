#!/usr/bin/env python3
"""전체 재검토. 앞선 결론 4개를 다시 친다.

A) 맵이 틀린 건가, 자세(PF)가 틀린 건가 — 강체 이동으로 스캔이 맵에 맞으면 자세.
B) 유령이 실제로 플래너까지 도달했나 (도달 못 하면 고쳐도 이득 없음).
C) lane_wall_no_room 260회 — 최대 거부 사유인데 아직 설명 안 됨.
D) 급감속이 유령보다 먼저인가 나중인가 (사용자 인과 방향 확인).
"""
import pickle
import re
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

D = pickle.load(open(Path(__file__).parent / "data.pkl", "rb"))
pf = D["/pf/pose/odom"]
fre = D["/car_state/frenet/odom"]
MAP = D["map"]
grid = MAP["data"].reshape(MAP["h"], MAP["w"])
occ = grid > 50
res, ox, oy = MAP["res"], MAP["ox"], MAP["oy"]
W, Hh = MAP["w"], MAP["h"]
XL = 0.28
_step = res * 0.5
_ds = np.arange(1, int(10.0 / _step) + 1) * _step


def raycast(x0, y0, angs):
    ca = np.cos(angs)[:, None]
    sa = np.sin(angs)[:, None]
    ix = (((x0 + ca * _ds) - ox) / res).astype(np.int32)
    iy = (((y0 + sa * _ds) - oy) / res).astype(np.int32)
    good = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < Hh)
    hit = np.zeros(ix.shape, bool)
    hit[good] = occ[iy[good], ix[good]]
    return np.where(hit.any(1), _ds[hit.argmax(1)], 10.0)


def residual(ts, amin, ainc, r, dx=0.0, dy=0.0, dth=0.0):
    i = np.clip(np.searchsorted(pf[:, 0], ts), 0, len(pf) - 1)
    px, py, yaw = pf[i, 1] + dx, pf[i, 2] + dy, pf[i, 3] + dth
    lx, ly = px + XL * np.cos(yaw), py + XL * np.sin(yaw)
    n = len(r)
    ang = amin + ainc * np.arange(n)
    ok = np.isfinite(r) & (r > 0.05) & (r < 8.0) & (np.abs(ang) < np.radians(110))
    idx = np.where(ok)[0][::2]
    if len(idx) < 30:
        return None
    exp = raycast(lx, ly, yaw + ang[idx])
    d = r[idx] - exp
    d = d[np.abs(d) < 1.0]            # 장애물/이상치 배제
    return np.median(np.abs(d)) if len(d) > 20 else None


print("=" * 68)
print("A) 맵 오류인가 자세 오류인가 — 강체 이동 탐색")
print("=" * 68)
print("각 시각에서 (dx,dy) 를 ±0.5 m 격자 탐색해 맵 잔차 중앙값을 최소화한다.")
print("보정으로 잔차가 크게 줄면 = 자세(PF) 오류.  안 줄면 = 맵 자체가 틀림.\n")
print("   t   ego_s  보정전  최적보정후  필요이동(dx,dy)  개선")
GRID = np.arange(-0.5, 0.51, 0.1)
for tq in (20.5, 33.6, 34.3, 46.2, 46.6, 60.5, 105.0, 106.5):
    cand = [(abs(ts - tq), ts, a, b, rr) for ts, a, b, rr in D["scan"]]
    _, ts, amin, ainc, r = min(cand)
    base = residual(ts, amin, ainc, r)
    if base is None:
        continue
    best, bdx, bdy = base, 0.0, 0.0
    for dx in GRID:
        for dy in GRID:
            v = residual(ts, amin, ainc, r, dx, dy)
            if v is not None and v < best:
                best, bdx, bdy = v, dx, dy
    egos = np.interp(ts, fre[:, 0], fre[:, 1])
    print(f"{ts:6.2f} {egos:5.1f} {base:7.3f} {best:10.3f}  "
          f"({bdx:+.1f},{bdy:+.1f})  {100*(1-best/base):5.1f}%")

print()
print("=" * 68)
print("B) 유령이 플래너까지 도달했나")
print("=" * 68)
# spliner trace 의 nobs = 그 장애물의 누적 관측수. obs id 별 최초 등장 trace.
seen_ids = set()
tr = []
for t, lvl, name, msg in D["rosout"]:
    m = re.search(r"obs id=(-?\d+) dist=(-?[\d.]+) .*?nobs=(-?\d+).*?\| (\w+)", msg)
    if m and "trace" in msg:
        tr.append((t, int(m.group(1)), float(m.group(2)), int(m.group(3)), m.group(4)))
print(f"trace 프레임 {len(tr)}개")
# 유령 id 집합
life = defaultdict(list)
for ts, lst in D["obs_frenet"]:
    for o in lst:
        life[o["id"]].append(ts)
ghost_ids = {k for k, v in life.items() if (max(v) - min(v)) < 0.4}
real_ids = {k for k, v in life.items() if (max(v) - min(v)) >= 0.4}
tr_ids = Counter(x[1] for x in tr)
print(f"유령 id {len(ghost_ids)}개 중 spliner 가 'closest obs' 로 채택한 것: "
      f"{len([i for i in ghost_ids if i in tr_ids])}개")
print(f"지속 id {len(real_ids)}개 중 채택: {len([i for i in real_ids if i in tr_ids])}개")
gf = sum(c for i, c in tr_ids.items() if i in ghost_ids)
rf = sum(c for i, c in tr_ids.items() if i in real_ids)
print(f"프레임 기준: 유령 기반 {gf}프레임 / 지속 기반 {rf}프레임 "
      f"= 유령이 {100*gf/max(gf+rf,1):.1f}%")
gm = Counter(x[4] for x in tr if x[1] in ghost_ids)
print(f"유령을 closest 로 잡았을 때 결과 모드: {gm.most_common()}")

print()
print("=" * 68)
print("C) lane_wall_no_room 260회 — 무엇이 막았나")
print("=" * 68)
rows = []
for t, lvl, name, msg in D["rosout"]:
    m = re.search(r"LaneSwitch (left|right) 레인 불가: d_hold=(-?[\d.]+) "
                  r"\(필요 (-?[\d.]+), 장애물 d\[(-?[\d.]+),(-?[\d.]+)\]\)", msg)
    if m:
        rows.append((t, m.group(1), float(m.group(2)), float(m.group(3)),
                     float(m.group(4)), float(m.group(5))))
print(f"로그로 잡힌 사례 {len(rows)}건 (throttle 2s 라 카운터 260 보다 적음)")
print("  side  d_hold  필요   장애물d[R,L]  부족분  ego_s")
for t, side, dh, need, dr, dl in rows[:25]:
    egos = np.interp(t, fre[:, 0], fre[:, 1])
    print(f"{t:7.2f} {side:5s} {dh:+6.2f} {need:+6.2f}  [{dr:+.2f},{dl:+.2f}] "
          f"{abs(need-dh):6.2f}  {egos:5.1f}")
if rows:
    short = np.array([abs(r[3] - r[2]) for r in rows])
    print(f"\n부족분: med {short.mean():.2f}  p90 {np.percentile(short,90):.2f} "
          f"max {short.max():.2f} m")
    print("사이드 분포:", Counter(r[1] for r in rows).most_common())
    es = np.array([np.interp(r[0], fre[:, 0], fre[:, 1]) for r in rows])
    print("ego_s 분포:", np.round(np.percentile(es, [10, 50, 90]), 1))

print()
print("=" * 68)
print("D) 급감속이 유령보다 먼저인가 나중인가")
print("=" * 68)
t = fre[:, 0]
v = fre[:, 4]
alon = np.convolve(np.gradient(v, t), np.ones(5) / 5, mode="same")
# 각 유령 발생 시각 기준 ±1.0 s 창의 종가속 프로파일
g0 = sorted(min(vs) for k, vs in life.items() if k in ghost_ids)
lags = np.arange(-1.0, 1.05, 0.2)
prof = np.array([[np.interp(gt + L, t, alon) for L in lags] for gt in g0])
print("유령 발생 시각 t=0 기준 종가속 평균 (음수=감속):")
for i, L in enumerate(lags):
    bar = "#" * int(abs(prof[:, i].mean()) * 10)
    print(f"  t{L:+.1f}s  a_lon {prof[:, i].mean():+6.2f} m/s^2  {bar}")
print(f"\n  유령 이전(-1.0~-0.2s) 평균 {prof[:, lags < -0.1].mean():+.2f}")
print(f"  유령 이후(+0.2~+1.0s) 평균 {prof[:, lags > 0.1].mean():+.2f}")
print("  -> 이후가 더 음수면 '유령을 보고 감속', 이전이 더 음수면 '감속이 먼저'")

print()
print("=" * 68)
print("E) trailing(감속 추종) 을 유발한 장애물 — 실물인가 유령인가")
print("=" * 68)
tm = [(x[0], x[1]) for x in tr if x[4] == "trailing"]
tc = Counter(i for _, i in tm)
print(f"trailing 프레임 {len(tm)}개")
print(f"  유령 유발: {sum(c for i,c in tc.items() if i in ghost_ids)} 프레임")
print(f"  지속 유발: {sum(c for i,c in tc.items() if i in real_ids)} 프레임")
print("  상위 유발 id:", [(i, c, "유령" if i in ghost_ids else "지속")
                      for i, c in tc.most_common(8)])
