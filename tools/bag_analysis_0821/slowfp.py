#!/usr/bin/env python3
"""'잠깐 느려진 상대'를 정지로 오인해 급감속할 위험 실측.

핵심: ObsVsMean 은 cls_tracks_.vs_hist(10프레임, **원시** vs)의 부호 있는 평균이다.
ClassifyObstacles 가 obs.vs=0 으로 덮어쓰기 **전** 값이라 정적 분류 중에도 살아 있다.
"""
import pickle, re, numpy as np, collections
L = 36.833
D = pickle.load(open("d.pkl", "rb"))
R = pickle.load(open("rosout2.pkl", "rb"))

# spliner 분류 전이 시계열
seq = collections.defaultdict(list)
for t, l, n, m in R:
    g = re.search(r"obstacle (\d+) classified (STATIC|DYNAMIC)", m)
    if g:
        seq[int(g.group(1))].append((t, g.group(2)))

tr = collections.defaultdict(list)
for t, o in D["obs_frenet"]:
    for x in o:
        tr[x["id"]].append((t, x["s_center"], x["vs"]))

def cls_at(i, t):
    s = seq.get(i)
    if not s: return None
    cur = None
    for tt, c in s:
        if tt <= t: cur = c
        else: break
    return cur

def mv(i):
    a = np.array(tr[i]); s = np.unwrap(a[:, 1], period=L)
    return s.max() - s.min()

moved   = {i for i in tr if len(tr[i]) >= 20 and mv(i) >= 0.5}
still   = {i for i in tr if len(tr[i]) >= 20 and mv(i) < 0.25}
print(f"이동 0.5m+ 트랙 {len(moved)}개 / 정지(0.25m 미만) 트랙 {len(still)}개")

def rows(ids, want_cls=None):
    out = []
    for i in ids:
        a = np.array(tr[i]); vs = a[:, 2]; t = a[:, 0]
        for k in range(9, len(vs)):
            if want_cls is not None and cls_at(i, t[k]) != want_cls:
                continue
            out.append((vs[k - 9:k + 1].mean(), vs[k]))
    return np.array(out) if out else np.zeros((0, 2))

A = rows(still)                 # 진짜 정지물
B = rows(moved, "STATIC")       # 움직이는데 STATIC 으로 분류된 구간  <- 위험 구간
C = rows(moved, "DYNAMIC")

for nm, X in (("진짜 정지물", A), ("움직이는데 STATIC 분류", B), ("움직이고 DYNAMIC", C)):
    if len(X) == 0:
        print(f"{nm:24s} 표본 없음"); continue
    m = X[:, 0]
    print(f"{nm:24s} n={len(X):5d}  ObsVsMean(부호평균) p10 {np.percentile(m,10):+.2f} "
          f"p50 {np.median(m):+.2f} p90 {np.percentile(m,90):+.2f}  |평균|<0.3 인 비율 {np.mean(np.abs(m)<0.3)*100:3.0f}%")

print(f"\n=== 위험 정량화 ===")
if len(B):
    print(f"'움직이는데 STATIC' 구간에서")
    print(f"  현행(obs.vs=0 강제)      목표속도 = 0.00 m/s  -> 바닥 2.5 가 가려 줌")
    print(f"  개정안(ObsVsMean 사용)   목표속도 p10 {np.percentile(B[:,0],10):+.2f} "
          f"p50 {np.median(B[:,0]):+.2f} p90 {np.percentile(B[:,0],90):+.2f} m/s")
    print(f"  -> 실제 그 순간 상대 속도 p50 {np.median(B[:,1]):+.2f} m/s 와 비교")
    print(f"  ObsVsMean 이 실제보다 0.5 m/s 이상 낮게 나오는 비율 "
          f"{np.mean(B[:,0] < B[:,1]-0.5)*100:.0f}%")
