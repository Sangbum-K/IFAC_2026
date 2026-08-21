#!/usr/bin/env python3
"""3-way 판정: 잡음 바닥(같은 빌드 2회) 대비 신호(변경 전/후)를 본다.

  python3 ab3.py <p0> <p1> <p1b>
     p0  = 변경 전
     p1  = 변경 후
     p1b = 변경 후, 같은 빌드 재실행  -> (p1 vs p1b) 가 잡음 바닥

재생은 결정적이지 않다. spliner 루프가 벽시계라 재생 스트림을 매번 다른
순간에 샘플링하고, committed_hold 는 이력 의존이라 한 번 어긋나면 이후가
연쇄로 갈린다. 그래서 (p0 vs p1) 차이는 반드시 (p1 vs p1b) 와 견줘야 한다.
"""
import sys, collections
import numpy as np
from ab import load, PAT          # noqa: F401  (ab.py 의 로더 재사용)

names = sys.argv[1:4]
R = [load(p) for p in names]
CNT = [r[0] for r in R]
CY = [r[1] for r in R]

def pair(i, j, label):
    A, B = CY[i], CY[j]
    com = sorted(set(A) & set(B))
    mode = [k for k in com if A[k]["mode"] != B[k]["mode"]]
    st = [k for k in com if A[k]["static"] == 1]
    st_d = [k for k in st if A[k]["mode"] != B[k]["mode"] or A[k]["tags"] != B[k]["tags"]]
    print(f"{label:28s} 공통 {len(com):5d}  모드변화 {len(mode):4d} "
          f"({len(mode)/max(len(com),1)*100:4.1f}%)   "
          f"static 프레임 {len(st):5d} 중 변화 {len(st_d):4d} "
          f"({len(st_d)/max(len(st),1)*100:4.1f}%)")
    return len(mode)/max(len(com),1), len(st_d)/max(len(st),1)

print("=== 사이클 단위 ===")
sig = pair(0, 1, "신호  (변경전 vs 변경후)")
noi = pair(1, 2, "잡음  (변경후 vs 같은빌드재실행)")
_   = pair(0, 2, "참고  (변경전 vs 재실행)")
print(f"\n  신호/잡음 비 — 모드변화 {sig[0]/max(noi[0],1e-9):.2f}x, "
      f"static {sig[1]/max(noi[1],1e-9):.2f}x")
print("  (1.0 근처 = 변경이 잡음에 묻힌다 = 그 지표로는 회귀 판정 불가)")

print("\n=== 카운터 단위 (증분) ===")
keys = sorted(set().union(*[set(c) for c in CNT]))
print(f"{'카운터':44s} {'변경전':>8} {'변경후':>8} {'재실행':>8}  판정")
for k in keys:
    v = [c.get(k, 0) for c in CNT]
    if max(v) < 30: continue
    d_sig = abs(v[1] - v[0]) / max(v[0], 1)
    d_noi = abs(v[2] - v[1]) / max(v[1], 1)
    if d_sig < 0.03 and d_noi < 0.03: continue
    verdict = ("실제 변화" if d_sig > 3 * max(d_noi, 0.01) else
               "잡음 수준" if d_sig <= max(d_noi, 0.02) * 1.5 else "판정 애매")
    print(f"  {k.split('/')[-1]:42s} {v[0]:8.0f} {v[1]:8.0f} {v[2]:8.0f}  {verdict}")
