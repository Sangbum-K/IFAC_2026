#!/usr/bin/env python3
"""run_0817_150059 개요."""
import pickle
import sys
import numpy as np

D = pickle.load(open(sys.argv[1] if len(sys.argv) > 1 else "d150059.pkl", "rb"))

print("=== topics with data ===")
for k, v in sorted(D.items()):
    if isinstance(v, np.ndarray):
        if len(v):
            print(f"{k:60s} arr{v.shape}")
    elif isinstance(v, list):
        if len(v):
            print(f"{k:60s} list[{len(v)}]")

print("\n=== ego ===")
pf = D["/pf/pose/odom"]
od = D["/odom"]
fr = D["/car_state/frenet/odom"]
print(f"pf {pf.shape} t {pf[0,0]:.1f}~{pf[-1,0]:.1f}")
print(f"frenet {fr.shape}")

print("\n=== speed timeline (1s bins) ===")
ack = D["/ackermann_cmd"]
for t in range(0, 44):
    m = (od[:, 0] >= t) & (od[:, 0] < t + 1)
    ma = (ack[:, 0] >= t) & (ack[:, 0] < t + 1)
    mf = (fr[:, 0] >= t) & (fr[:, 0] < t + 1) if len(fr) else None
    s = fr[mf, 1].mean() if mf is not None and mf.any() else float("nan")
    print(f"t={t:3d}  v_odom={od[m,4].mean() if m.any() else float('nan'):6.2f}"
          f"  cmd_v={ack[ma,2].mean() if ma.any() else float('nan'):6.2f}"
          f"  cmd_steer={ack[ma,1].mean() if ma.any() else float('nan'):7.3f}"
          f"  s={s:7.2f}")
