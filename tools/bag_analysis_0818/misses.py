#!/usr/bin/env python3
"""수정본이 놓친 base 검출의 정체 확인."""
import pickle, sys
import numpy as np
A = pickle.load(open(sys.argv[1], "rb")); B = pickle.load(open(sys.argv[2], "rb"))
fa = {round(h, 4): o for h, o in A["obs"]}; fb = {round(h, 4): o for h, o in B["obs"]}
common = sorted(set(fa) & set(fb))
rows = []
for t in common:
    ma = [x for x in fa[t] if x[8]]; mb = [x for x in fb[t] if x[8]]
    for x in ma:
        if any(np.hypot(x[1]-y[1], x[2]-y[2]) < 0.5 for y in mb):
            continue
        # new 쪽에 코스팅으로라도 있나 / base 쪽에 이웃(본체)이 있었나
        any_new = min([np.hypot(x[1]-y[1], x[2]-y[2]) for y in fb[t]], default=9)
        nb_base = min([np.hypot(x[1]-y[1], x[2]-y[2]) for y in ma if y is not x], default=9)
        rows.append((x[7], any_new, nb_base))
R = np.array(rows)
print("new 가 놓친 base 검출: %d" % len(R))
print("  반경 p50 %.3f p90 %.3f  | 반경<0.10 비율 %.0f%%" % (
    np.median(R[:, 0]), np.percentile(R[:, 0], 90), 100*(R[:, 0] < 0.10).mean()))
print("  new 쪽에 (코스팅 포함) 0.5m 안 트랙 있음: %.0f%%" % (100*(R[:, 1] < 0.5).mean()))
print("  base 쪽에서 0.45m 안에 다른 검출이 함께 있었음: %.0f%%  <- 파편이었다는 뜻" % (
    100*(R[:, 2] < 0.45).mean()))
print("  둘 다 아닌 '진짜 손실': %d (%.1f%% of base 검출)" % (
    ((R[:, 1] >= 0.5) & (R[:, 2] >= 0.45)).sum(),
    100*((R[:, 1] >= 0.5) & (R[:, 2] >= 0.45)).sum()/8545))

# '진짜 손실' 31건의 시간/위치 분포
loss = []
for t in common:
    ma = [x for x in fa[t] if x[8]]; mb = [x for x in fb[t] if x[8]]
    for x in ma:
        if any(np.hypot(x[1]-y[1], x[2]-y[2]) < 0.5 for y in mb): continue
        if min([np.hypot(x[1]-y[1], x[2]-y[2]) for y in fb[t]], default=9) < 0.5: continue
        if min([np.hypot(x[1]-y[1], x[2]-y[2]) for y in ma if y is not x], default=9) < 0.45: continue
        loss.append((t, x[1], x[2], x[7]))
loss.sort()
print("\n진짜 손실 %d 건 — 연속 구간으로 묶기:" % len(loss))
grp=[[loss[0]]]
for a,b in zip(loss, loss[1:]):
    if b[0]-a[0] < 0.5 and np.hypot(b[1]-a[1], b[2]-a[2]) < 1.0: grp[-1].append(b)
    else: grp.append([b])
for g in grp:
    print("  t %.2f~%.2f (%d프레임) (%.2f,%.2f) r%.2f" % (
        g[0][0]-loss[0][0]+0, g[-1][0]-loss[0][0]+0, len(g), g[0][1], g[0][2], g[0][3]))
