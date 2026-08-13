#!/usr/bin/env python3
"""run_0813_165312 전체 개관: 랩, 속도, 회피 이벤트, 기각 카운터, 트랙 id 생애."""
import pickle
from collections import defaultdict, Counter
from pathlib import Path

import numpy as np

D = pickle.load(open(Path(__file__).parent / "data.pkl", "rb"))

fre = D["/car_state/frenet/odom"]          # t, s, d, yaw?, v  (odom decode)
print("frenet odom cols:", fre.shape)
t = fre[:, 0]
s = fre[:, 1]
d = fre[:, 2]
v = fre[:, 4]

# 랩 경계: s wrap
track_len = s.max()
print(f"s range {s.min():.2f}..{s.max():.2f}  (track_len ~{track_len:.2f})")
wrap = np.where(np.diff(s) < -track_len * 0.5)[0]
lap_t = t[wrap]
print(f"\n랩 {len(lap_t)+1}개, 경계 t = {np.round(lap_t,1)}")
if len(lap_t) > 1:
    print("랩타임:", np.round(np.diff(lap_t), 2))

# 속도
spd = D["/monitor/current_speed"]
print(f"\n속도: p50 {np.median(spd[:,1]):.2f}  p90 {np.percentile(spd[:,1],90):.2f} "
      f"max {spd[:,1].max():.2f}")

# pitwall events
print("\n=== /pitwall/events (196) ===")
for tt, msg in D["events"]:
    print(f"{tt:7.2f}  {msg}")

# 기각 카운터: 누적이므로 diff 로 발동 시점
print("\n=== reject 카운터 최종값 ===")
for k in sorted(D):
    if k.startswith("/pitwall/reject"):
        a = D[k]
        if len(a):
            print(f"{k:60s} final={a[-1,1]:8.0f}  n_msg={len(a)}")

# escape / estop
for k in ("/planner/escape_active", "/safety/emergency_stop",
          "/pitwall/avoidance_loss_hold"):
    a = D[k]
    if len(a):
        on = a[a[:, 1] > 0.5]
        print(f"\n{k}: {len(on)}/{len(a)} true")
        if len(on):
            print("  구간 t:", np.round(on[:20, 0], 1))
