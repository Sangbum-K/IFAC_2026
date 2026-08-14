#!/usr/bin/env python3
"""트랙 기하 + 장애물 물리 위치 클러스터링."""
import pickle
from pathlib import Path
import numpy as np

D = pickle.load(open(Path(__file__).parent / "data.pkl", "rb"))
t, gb = D["global_wpnts"][0]
# cols: s d x y d_r d_l psi kappa vx ax
s, dd, x, y, dr, dl, psi, kap, vx, ax = [gb[:, i] for i in range(10)]
print(f"global wpnts n={len(gb)}  s {s.min():.2f}..{s.max():.2f}")
print(f"track width: d_r p50 {np.median(dr):.2f}  d_l p50 {np.median(dl):.2f}")
print()
print("  s     x      y     kappa   R(m)   d_r   d_l   width  v_plan")
for i in range(0, len(gb), 4):
    R = 1.0/abs(kap[i]) if abs(kap[i]) > 1e-3 else 999
    print(f"{s[i]:6.2f} {x[i]:6.2f} {y[i]:6.2f} {kap[i]:+7.3f} {R:6.1f} "
          f"{dr[i]:5.2f} {dl[i]:5.2f} {dr[i]+dl[i]:5.2f} {vx[i]:5.2f}")
