#!/usr/bin/env python3
"""장애물 '기억' 타당성 분석.

1. 트랙 id 생애 / 랩 간 재사용 여부
2. 같은 물리 장애물이 랩마다 같은 (s,d)에 오는가  -> 기억의 근거
3. 접근 거리에 따른 d 추정 편향  -> 기억이 주는 정확도 이득
4. 동적 오분류 사건의 전후 맥락
"""
import pickle
from collections import defaultdict
from pathlib import Path

import numpy as np

D = pickle.load(open(Path(__file__).parent / "data.pkl", "rb"))
TL = 32.07
fre = D["/car_state/frenet/odom"]


def ego(t):
    i = np.searchsorted(fre[:, 0], t)
    i = min(max(i, 0), len(fre) - 1)
    return fre[i, 1], fre[i, 2], fre[i, 4]


def mod(a):
    return a % TL


# ── 1. 트랙 생애 ──────────────────────────────────────────────────────────
life = defaultdict(lambda: dict(t0=None, t1=None, n=0, s=[], d=[], dist=[],
                                static=[], vis=[]))
for t, obs in D["obs_frenet"]:
    s, d, v = ego(t)
    for o in obs:
        L = life[o["id"]]
        if L["t0"] is None:
            L["t0"] = t
        L["t1"] = t
        L["n"] += 1
        L["s"].append(o["s_center"])
        L["d"].append(o["d_center"])
        L["dist"].append(mod(o["s_center"] - s))
        L["static"].append(o["is_static"])
        L["vis"].append(o["is_visible"])

durs = np.array([L["t1"] - L["t0"] for L in life.values()])
print(f"=== 1. 트랙 id 생애 ({len(life)}개 id, 22랩, 227초) ===")
print(f"  수명   p50 {np.median(durs):.2f}s  p90 {np.percentile(durs,90):.2f}s  "
      f"max {durs.max():.2f}s")
print(f"  1랩(9.4s) 넘게 산 id: {int((durs>9.4).sum())}개")
print(f"  2초 넘게 산 id: {int((durs>2.0).sum())}개")
print(f"  id 총 발급 수 (최대 id): {max(life)}")

# ── 2. 랩 간 위치 일관성 ───────────────────────────────────────────────────
# 각 트랙의 "가장 가까이서 본" 관측 = 가장 정확한 위치
print("\n=== 2. 물리 장애물별 랩 간 위치 일관성 ===")
best = []
for tid, L in life.items():
    if L["n"] < 5:
        continue
    dist = np.array(L["dist"])
    k = int(np.argmin(dist))
    best.append((L["t0"], tid, L["s"][k], L["d"][k], dist[k], L["n"]))
best.sort()
# s 로 클러스터링 (같은 물리 물체)
clusters = []
for t0, tid, s, d, dm, n in best:
    hit = None
    for c in clusters:
        if abs(mod(s - c["s"][0] + TL / 2) - TL / 2) < 1.2:
            hit = c
            break
    if hit is None:
        clusters.append(dict(s=[s], d=[d], ids=[tid], t=[t0]))
    else:
        hit["s"].append(s)
        hit["d"].append(d)
        hit["ids"].append(tid)
        hit["t"].append(t0)
clusters.sort(key=lambda c: -len(c["ids"]))
for c in clusters:
    if len(c["ids"]) < 3:
        continue
    s = np.array(c["s"])
    d = np.array(c["d"])
    print(f"  물체@s≈{np.median(s):5.2f}: 관측 {len(c['ids']):2d}회 "
          f"(id {min(c['ids'])}~{max(c['ids'])})  "
          f"s std {s.std():.3f}m  d 중앙값 {np.median(d):+.3f} std {d.std():.3f}m")

# ── 3. 거리에 따른 d 추정 편향 ─────────────────────────────────────────────
print("\n=== 3. 접근 거리별 d 추정 편향 (동일 트랙 내, 최근접 관측 기준) ===")
bins = [(0.0, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 3.0), (3.0, 4.0),
        (4.0, 5.0), (5.0, 8.0)]
acc = {b: [] for b in bins}
for tid, L in life.items():
    if L["n"] < 8:
        continue
    dist = np.array(L["dist"])
    dd = np.array(L["d"])
    k = int(np.argmin(dist))
    if dist[k] > 0.6:
        continue
    ref = dd[k]
    for i in range(len(dist)):
        for b in bins:
            if b[0] <= dist[i] < b[1]:
                acc[b].append(dd[i] - ref)
print("   거리[m]     n    편향(평균)   |편향| p90    std")
for b in bins:
    a = np.array(acc[b])
    if len(a) < 5:
        continue
    print(f"  {b[0]:3.1f}~{b[1]:3.1f}  {len(a):5d}   {a.mean():+8.3f}   "
          f"{np.percentile(np.abs(a),90):8.3f}   {a.std():.3f}")

# ── 4. 정적/동적 플래그 뒤집힘 ─────────────────────────────────────────────
print("\n=== 4. is_static 플래그 뒤집힘 ===")
flips = []
for tid, L in life.items():
    st = np.array(L["static"])
    ch = np.where(np.diff(st.astype(int)) != 0)[0]
    if len(ch):
        flips.append((tid, L["t0"], L["t1"], len(ch), st[0], st[-1], L["n"]))
print(f"  플래그가 바뀐 트랙 {len(flips)} / 관측 5회 이상 트랙 "
      f"{sum(1 for L in life.values() if L['n']>=5)}")
for tid, t0, t1, nch, s0, s1, n in sorted(flips, key=lambda x: x[1]):
    print(f"   id{tid:4d} t {t0:7.2f}~{t1:7.2f} ({n:3d}관측) 변화 {nch}회  "
          f"{'STATIC' if s0 else 'DYN'} -> {'STATIC' if s1 else 'DYN'}")

# ── 5. 첫 탐지 거리 / 그때 속도 ────────────────────────────────────────────
print("\n=== 5. 첫 탐지 거리와 그때 자차 속도 ===")
rows = []
for tid, L in life.items():
    if L["n"] < 5:
        continue
    s0, d0, v0 = ego(L["t0"])
    rows.append((L["dist"][0], v0, tid))
rows.sort()
a = np.array([[r[0], r[1]] for r in rows])
print(f"  n={len(a)}  첫탐지거리 p10 {np.percentile(a[:,0],10):.2f} "
      f"p50 {np.median(a[:,0]):.2f} p90 {np.percentile(a[:,0],90):.2f} m")
print(f"  그때 속도  p50 {np.median(a[:,1]):.2f} p90 "
      f"{np.percentile(a[:,1],90):.2f} m/s")
budget = a[:, 0] / np.maximum(a[:, 1], 0.3)
print(f"  반응 예산(거리/속도) p10 {np.percentile(budget,10):.2f} "
      f"p50 {np.median(budget):.2f} s")
