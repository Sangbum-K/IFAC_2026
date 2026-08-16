#!/usr/bin/env python3
"""트랙 기하: s별 곡률/폭, 헤어핀 위치 식별. 장애물 s 분포."""
import pickle
from collections import defaultdict
from pathlib import Path

import numpy as np

D = pickle.load(open(Path(__file__).parent / "data.pkl", "rb"))
gw = D["global_wpnts"][0][1]        # s d x y d_r d_l psi kappa vx ax
s, x, y = gw[:, 0], gw[:, 2], gw[:, 3]
d_r, d_l, kap = np.abs(gw[:, 4]), np.abs(gw[:, 5]), gw[:, 7]
w = d_r + d_l
L = s[-1] + (s[1] - s[0])

print(f"track len {L:.2f}m, n={len(gw)}, ds={s[1]-s[0]:.3f}")
print("\n s(m)   x      y     kappa   d_left d_right width  vx")
for i in range(0, len(gw), 5):
    print(f"{s[i]:6.2f} {x[i]:6.2f} {y[i]:6.2f} {kap[i]:7.3f} "
          f"{d_l[i]:6.2f} {d_r[i]:6.2f} {w[i]:6.2f} {gw[i,8]:5.2f}")

# 헤어핀 = |kappa| 큰 연속 구간
print("\n--- 고곡률 구간 (|kappa|>1.0) ---")
hi = np.abs(kap) > 1.0
i = 0
while i < len(hi):
    if hi[i]:
        j = i
        while j + 1 < len(hi) and hi[j + 1]:
            j += 1
        seg = kap[i:j + 1]
        side = "LEFT(반시계)" if seg.mean() > 0 else "RIGHT(시계)"
        print(f"s {s[i]:5.2f}-{s[j]:5.2f} ({s[j]-s[i]:4.2f}m) kappa peak "
              f"{seg[np.abs(seg).argmax()]:+.2f} {side}  "
              f"width {w[i:j+1].min():.2f}-{w[i:j+1].max():.2f} "
              f"| 출구 이후 width @+1m {w[min(j+int(1/0.1),len(w)-1)]:.2f}")
        i = j + 1
    else:
        i += 1

# --- 장애물 s 분포 ---
print("\n--- 장애물 클러스터 (frenet, s별) ---")
seen = defaultdict(list)
for t, obs in D["obs_frenet"]:
    for o in obs:
        seen[o["id"]].append((t, o))
rows = []
for oid, lst in seen.items():
    ss = np.array([o["s_center"] for _, o in lst])
    dd = np.array([o["d_center"] for _, o in lst])
    t0, t1 = lst[0][0], lst[-1][0]
    rows.append((np.median(ss), np.median(dd), oid, len(lst), t0, t1,
                 np.median([o["s_end"] - o["s_start"] for _, o in lst]),
                 np.median([o["d_left"] - o["d_right"] for _, o in lst])))
rows.sort()
print("  s_med  d_med  id   n    t0     t1   len_s  width_d")
for r in rows:
    print(f"{r[0]:7.2f} {r[1]:6.2f} {r[2]:4d} {r[3]:4d} {r[4]:6.1f} {r[5]:6.1f} "
          f"{r[6]:6.2f} {r[7]:7.2f}")

# 정적 장애물 위치를 s로 히스토그램 (같은 물리 장애물 = 같은 s)
print("\n--- 물리 장애물 추정 (s 0.5m 빈) ---")
bins = defaultdict(lambda: [0, [], []])
for r in rows:
    b = round(r[0] * 2) / 2
    bins[b][0] += r[3]
    bins[b][1].append(r[2])
    bins[b][2].append(r[1])
for b in sorted(bins):
    n, ids, ds = bins[b]
    idx = int(round(b / (s[1] - s[0]))) % len(gw)
    print(f"s≈{b:5.1f}  frames={n:5d}  d_med={np.median(ds):+.2f}  "
          f"trackW={w[idx]:.2f} (L{d_l[idx]:.2f}/R{d_r[idx]:.2f}) "
          f"kappa={kap[idx]:+.2f}  ids={sorted(ids)[:8]}")
