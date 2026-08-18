#!/usr/bin/env python3
"""과분할/중복 지표. usage: metrics.py <pkl> [label]"""
import pickle, sys
from collections import defaultdict
import numpy as np

D = pickle.load(open(sys.argv[1], "rb"))
label = sys.argv[2] if len(sys.argv) > 2 else sys.argv[1]
obs, pf = D["obs"], D["pf"]

def ego_at(t):
    i = np.clip(np.searchsorted(pf[:, 0], t), 0, len(pf) - 1)
    return pf[i, 1], pf[i, 2], pf[i, 3]

tl = defaultdict(list)
inst = dup = 0
nvis_frames = 0
for ht, o in obs:
    if any(x[8] == 0 for x in o):
        nvis_frames += 1
    for i, a in enumerate(o):
        inst += 1
        for j, b in enumerate(o):
            if i != j and np.hypot(a[1] - b[1], a[2] - b[2]) < 0.45:
                dup += 1
                break
    for (oid, px, py, vx, vy, dyn, conf, rad, vis) in o:
        tl[oid].append((ht, px, py, vis, rad))

drops = 0
frag_tracks = 0
lifetimes = []
for oid, rows in tl.items():
    rows.sort()
    lifetimes.append(rows[-1][0] - rows[0][0])
    rr = [r[4] for r in rows if r[3]]
    if rr and np.median(rr) < 0.10:
        frag_tracks += 1
    for a, b in zip(rows, rows[1:]):
        if a[3] == 1 and b[3] == 0:
            drops += 1

# 신규 트랙이 기존 트랙 0.45m 안에서 태어난 비율
first = {}
byt = {}
for ht, o in obs:
    byt[ht] = o
    for x in o:
        first.setdefault(x[0], (ht, x[1], x[2], x[7]))
near_spawn = 0
for oid, (ht, px, py, rad) in first.items():
    for x in byt[ht]:
        if x[0] != oid and np.hypot(x[1] - px, x[2] - py) < 0.45:
            near_spawn += 1
            break

print("=== %s ===" % label)
print("  발행 인스턴스        %6d" % inst)
print("  0.45m 안 중복        %6d  (%.1f%%)" % (dup, 100 * dup / max(inst, 1)))
print("  트랙 ID 수           %6d   (수명 p50 %.2fs, <1s %.0f%%)" % (
    len(tl), np.median(lifetimes), 100 * np.mean(np.array(lifetimes) < 1.0)))
print("  파편 트랙(r<0.10)    %6d  (%.0f%%)" % (frag_tracks, 100 * frag_tracks / max(len(tl), 1)))
print("  이웃 0.45m 안 스폰   %6d  (%.0f%%)" % (near_spawn, 100 * near_spawn / max(len(tl), 1)))
print("  미관측 전환          %6d" % drops)
print("  코스팅 포함 프레임   %6d  (%.1f%%)" % (nvis_frames, 100 * nvis_frames / max(len(obs), 1)))
