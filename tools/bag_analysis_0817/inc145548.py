#!/usr/bin/env python3
"""세 사건 구간의 spliner trace + 기억 주입 상태."""
import pickle
import sys
import numpy as np

D = pickle.load(open("d145548.pkl", "rb"))
t0, t1 = float(sys.argv[1]), float(sys.argv[2])
step = int(sys.argv[3]) if len(sys.argv) > 3 else 3

n = 0
for t, lvl, name, msg in D["rosout"]:
    if not (t0 <= t <= t1):
        continue
    if name == "spliner_node" and "[trace]" in msg and "s=" in msg:
        n += 1
        if n % step:
            continue
        print(f"{t:7.2f} {msg[:215]}")
    elif name == "spliner_node":
        print(f"{t:7.2f} ** {msg[:200]}")
    elif name == "perception_obstacle_bridge_node":
        print(f"{t:7.2f} BB {msg[:200]}")
    elif name == "control" and lvl >= 30:
        print(f"{t:7.2f} CT {msg[:160]}")
