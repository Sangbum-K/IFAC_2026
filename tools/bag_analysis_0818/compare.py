#!/usr/bin/env python3
"""수정 전/후 재생 결과의 검출 커버리지 비교. usage: compare.py <base.pkl> <new.pkl>"""
import pickle, sys
from collections import defaultdict
import numpy as np

A = pickle.load(open(sys.argv[1], "rb"))
B = pickle.load(open(sys.argv[2], "rb"))
pf = A["pf"]

def frames(D):
    return {round(ht, 4): o for ht, o in D["obs"]}

fa, fb = frames(A), frames(B)
common = sorted(set(fa) & set(fb))
print("공통 프레임 %d (base %d / new %d)" % (len(common), len(fa), len(fb)))

def ego(t):
    i = np.clip(np.searchsorted(pf[:, 0], t), 0, len(pf) - 1)
    return pf[i, 1], pf[i, 2]

cov_ab = miss_ab = cov_ba = miss_ba = 0
missed_R = []
ra, rb = [], []
na = nb = 0
for t in common:
    ma = [x for x in fa[t] if x[8]]
    mb = [x for x in fb[t] if x[8]]
    na += len(ma); nb += len(mb)
    ra += [x[7] for x in ma]; rb += [x[7] for x in mb]
    ex, ey = ego(t)
    for x in ma:
        if any(np.hypot(x[1] - y[1], x[2] - y[2]) < 0.5 for y in mb):
            cov_ab += 1
        else:
            miss_ab += 1
            missed_R.append(np.hypot(x[1] - ex, x[2] - ey))
    for y in mb:
        if any(np.hypot(x[1] - y[1], x[2] - y[2]) < 0.5 for x in ma):
            cov_ba += 1
        else:
            miss_ba += 1
print("실측(is_visible) 인스턴스: base %d -> new %d" % (na, nb))
print("  base 검출이 new 에도 0.5m 안에 있음: %d / %d  (%.1f%%)" % (
    cov_ab, cov_ab + miss_ab, 100 * cov_ab / max(cov_ab + miss_ab, 1)))
print("  new 에만 있는 검출: %d (%.1f%%)" % (miss_ba, 100 * miss_ba / max(cov_ba + miss_ba, 1)))
if missed_R:
    print("  new 가 놓친 검출의 거리: p50 %.1f p90 %.1f m" % tuple(np.percentile(missed_R, [50, 90])))
print("클러스터 반경 p50: base %.3f -> new %.3f  (p90 %.3f -> %.3f)" % (
    np.median(ra), np.median(rb), np.percentile(ra, 90), np.percentile(rb, 90)))

# 물체별 최초 검출 시각 (같은 자리 0.5m 기준으로 묶어서)
def first_seen(F):
    sites, out = [], []
    for t in common:
        for x in F[t]:
            if not x[8]:
                continue
            hit = False
            for s in sites:
                if np.hypot(s[0] - x[1], s[1] - x[2]) < 0.5 and t - s[2] < 3.0:
                    s[0], s[1], s[2] = x[1], x[2], t
                    hit = True
                    break
            if not hit:
                ex, ey = ego(t)
                sites.append([x[1], x[2], t])
                out.append((t, np.hypot(x[1] - ex, x[2] - ey)))
    return out
oa, ob = first_seen(fa), first_seen(fb)
print("물체 최초검출 이벤트: base %d / new %d | 최초검출 거리 p50 %.2f -> %.2f m" % (
    len(oa), len(ob), np.median([r for _, r in oa]), np.median([r for _, r in ob])))
