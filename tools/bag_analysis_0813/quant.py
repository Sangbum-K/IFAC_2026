#!/usr/bin/env python3
"""정량 요약: 판단 분포, 오분류 타이밍, 구간별 시간손실, 드롭아웃."""
import pickle
import re
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

D = pickle.load(open(Path(__file__).parent / "data.pkl", "rb"))
TL = 32.07
fre = D["/car_state/frenet/odom"]
TR = re.compile(
    r"\[trace\] s=([-\d.]+) v=([-\d.]+) d=([-\d.]+) \| obs id=([-\d]+) "
    r"dist=([-\d.]+) d\[([-\d.]+),([-\d.]+)\] static=([-\d]+) nobs=([-\d]+) \| "
    r"evade=(\d) sector=(\d) commit=(\d) \| (\w+) \| (.*)")

traces = []
for t, l, n, m in D["rosout"]:
    g = TR.search(m)
    if g:
        s, v, d, oid, dist, dr, dl, st, nobs, ev, sec, cm, mode, rej = g.groups()
        traces.append(dict(t=t, s=float(s), v=float(v), d=float(d), oid=int(oid),
                           dist=float(dist), dr=float(dr), dl=float(dl),
                           st=int(st), mode=mode,
                           rej=[r for r in rej.split(",") if r and r != "-"]))
print(f"trace cycles: {len(traces)}")

with_obs = [x for x in traces if x["oid"] != 0 or x["st"] >= 0]
print(f"\n=== 판단 분포 (장애물이 있는 사이클 {len(with_obs)}) ===")
for k, v in Counter(x["mode"] for x in with_obs).most_common():
    print(f"  {k:16s} {v:5d}  ({100*v/len(with_obs):4.1f}%)")

print("\n=== 기각 사유 빈도 (장애물 있는 사이클 기준) ===")
c = Counter()
for x in with_obs:
    for r in set(x["rej"]):
        c[r] += 1
for k, v in c.most_common(20):
    print(f"  {k:38s} {v:5d}  ({100*v/len(with_obs):4.1f}%)")

# ── 구간별 속도 손실 ────────────────────────────────────────────────────────
print("\n=== 구간별 계획대비 실속 (랩 22개 평균) ===")
t, gb = D["global_wpnts"][0]
gs, gv = gb[:, 0], gb[:, 8]
spd = D["/monitor/current_speed"]
s_ego = fre[:, 1]
t_ego = fre[:, 0]
v_ego = fre[:, 4]
zones = [("S/F 직선 s29-32,0-3", lambda s: s >= 29.0 or s <= 3.0),
         ("코너 s3-9", lambda s: 3.0 < s <= 9.0),
         ("s12 장애물 s9-14", lambda s: 9.0 < s <= 14.0),
         ("나머지 s14-29", lambda s: 14.0 < s < 29.0)]
tot_lost = 0.0
for name, f in zones:
    m = np.array([f(s) for s in s_ego])
    if m.sum() == 0:
        continue
    vplan = np.interp(s_ego[m], gs, gv) * 0.96   # velocity_scale
    vact = v_ego[m]
    dt = np.gradient(t_ego)[m]
    # 시간 손실 ≈ ∫ ds (1/v_act - 1/v_plan)
    ds = vact * dt
    lost = np.sum(ds * (1.0 / np.maximum(vact, 0.2) - 1.0 / np.maximum(vplan, 0.2)))
    tot_lost += lost
    print(f"  {name:22s} v_act p50 {np.median(vact):4.2f} / v_plan p50 "
          f"{np.median(vplan):4.2f}  ({100*np.median(vact)/np.median(vplan):3.0f}%)  "
          f"누적손실 {lost:6.1f}s = {lost/22:.2f}s/랩")
print(f"  총 손실 {tot_lost:.1f}s / 22랩 = {tot_lost/22:.2f}s/랩 "
      f"(실측 평균 랩타임 {np.median(np.diff([19.1,28.6,37.7,46.9,56.3,66.4,77.2,86.5,95.7,105.6,114.9,124.2,135.9,147.9,157.3,166.6,176.,185.3,195.,204.5,214.])):.2f}s)")

# ── 인지 드롭아웃 ──────────────────────────────────────────────────────────
print("\n=== /perception/obstacles_frenet 드롭아웃 (장애물 4m 이내에서 빈 배열) ===")
prev_t, prev_n = None, 0
gaps = []
for tt, obs in D["obs_frenet"]:
    n = len(obs)
    if prev_n > 0 and n == 0:
        start = tt
    if prev_n == 0 and n > 0 and prev_t is not None:
        pass
    prev_t, prev_n = tt, n
# 간단히: 비어있는 연속 구간 길이
empty = [(tt, len(o)) for tt, o in D["obs_frenet"]]
runs = []
i = 0
while i < len(empty):
    if empty[i][1] == 0:
        j = i
        while j < len(empty) and empty[j][1] == 0:
            j += 1
        runs.append((empty[i][0], empty[j - 1][0] - empty[i][0]))
        i = j
    else:
        i += 1
long_runs = [r for r in runs if 0.1 <= r[1] <= 2.0]
print(f"  공백 구간 {len(runs)}개, 0.1~2.0s 공백 {len(long_runs)}개")
if long_runs:
    a = np.array([r[1] for r in long_runs])
    print(f"  길이 p50 {np.median(a):.2f}s p90 {np.percentile(a,90):.2f}s "
          f"max {a.max():.2f}s")

hold = D["/pitwall/avoidance_loss_hold"]
on = hold[hold[:, 1] > 0.5]
print(f"\n  avoidance_loss_hold 활성 샘플 {len(on)}/{len(hold)}")
