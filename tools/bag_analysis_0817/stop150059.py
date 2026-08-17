#!/usr/bin/env python3
"""t=23~44 정지 구간 상세."""
import pickle
import numpy as np

D = pickle.load(open("d150059.pkl", "rb"))

print("=== reject 카운터 (마지막 값 / 증가 시점) ===")
for k in sorted(D):
    if not k.startswith("/pitwall/reject"):
        continue
    a = D[k]
    if not len(a):
        continue
    v = a[:, 1]
    if v.max() == 0:
        continue
    # 증가 구간
    jumps = []
    for i in range(1, len(a)):
        if v[i] > v[i - 1]:
            jumps.append((a[i, 0], v[i] - v[i - 1]))
    print(f"{k:58s} last={v[-1]:6.0f}  jumps={[f'{t:.1f}(+{d:.0f})' for t,d in jumps][:14]}")

print("\n=== rosout t=20~44 (spliner/control/safety) ===")
for t, lvl, name, msg in D["rosout"]:
    if t < 20:
        continue
    if lvl < 20:
        continue
    print(f"{t:7.2f} [{lvl}] {name}: {msg[:200]}")
