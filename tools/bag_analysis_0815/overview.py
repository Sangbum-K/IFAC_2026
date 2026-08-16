#!/usr/bin/env python3
"""run_0815_191056 개요: 랩, 이벤트, 리젝트 카운터, 장애물 목록."""
import pickle
import sys
from pathlib import Path

import numpy as np

D = pickle.load(open(Path(__file__).parent / "data.pkl", "rb"))
np.set_printoptions(suppress=True, precision=3)

fre = D["/car_state/frenet/odom"]        # t x=s y=d yaw v
print(f"duration {fre[-1,0]:.1f}s   frenet samples {len(fre)}")

s = fre[:, 1]
gw = D["global_wpnts"][0][1] if D["global_wpnts"] else None
if gw is not None:
    print(f"global wpnts n={len(gw)}  s_max={gw[-1,0]:.2f}  "
          f"d_left med={np.median(gw[:,5]):.2f} d_right med={np.median(gw[:,4]):.2f}")
    w = np.abs(gw[:, 4]) + np.abs(gw[:, 5])
    print(f"track width: min {w.min():.2f} @s={gw[w.argmin(),0]:.1f}  "
          f"med {np.median(w):.2f}  max {w.max():.2f}")

# --- 랩 경계 (s wrap) ---
laps = []
prev = s[0]
start = fre[0, 0]
for i in range(1, len(s)):
    if prev - s[i] > 5.0:            # wrap
        laps.append((start, fre[i, 0]))
        start = fre[i, 0]
    prev = s[i]
laps.append((start, fre[-1, 0]))
print("\n--- laps ---")
for i, (a, b) in enumerate(laps):
    m = (fre[:, 0] >= a) & (fre[:, 0] < b)
    print(f"lap{i}: {a:6.1f}-{b:6.1f}s ({b-a:5.1f}s)  v mean {fre[m,4].mean():.2f} "
          f"max {fre[m,4].max():.2f}  |d| max {np.abs(fre[m,2]).max():.2f}")

print("\n--- pitwall events ---")
for t, msg in D["events"]:
    print(f"{t:7.2f}  {msg}")

print("\n--- reject counters (마지막 값) ---")
rej = [(k, v[-1, 1]) for k, v in D.items()
       if isinstance(k, str) and k.startswith("/pitwall/reject")
       and isinstance(v, np.ndarray) and len(v)]
for k, v in sorted(rej, key=lambda x: -x[1]):
    print(f"{v:9.0f}  {k}")

print("\n--- rosout WARN+ (상위 30 유형) ---")
from collections import Counter
import re
cnt = Counter()
for t, lvl, name, msg in D["rosout"]:
    if lvl >= 30:
        key = re.sub(r"[-+]?\d*\.?\d+", "#", msg)[:110]
        cnt[(name, key)] += 1
for (name, k), c in cnt.most_common(30):
    print(f"{c:5d}  [{name}] {k}")
