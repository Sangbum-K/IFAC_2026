#!/usr/bin/env python3
"""S4(정적 회귀) 콘 배치 — /clicked_point 로 쏠 map 좌표를 계산한다.

정적 회피는 지금 잘 도는 유일한 부분이다(접촉률 8%, 통과여유 중앙 0.31 m).
동적을 고치다 그걸 깨는 것이 가장 큰 위험이라 매 단계 같이 돌린다.

배치 원칙: 실차 백에서 실제로 회피가 성립하던 폭 대역에 놓는다.
  - 필요 통과폭 = 0.30(간격) + 0.15(차반폭) + 0.25(경계여유) = 0.70 m
  - 콘 반경 0.17 -> 콘을 d 에 놓으면 반대쪽 여유 = wall - |d| - 0.17
"""
import pickle, sys
import numpy as np

D = pickle.load(open("d.pkl", "rb"))
gw = D["global_wpnts"][0][1]           # s d x y d_r d_l psi kappa vx ax
s, x, y, dr, dl, psi, kap = (gw[:, i] for i in (0, 2, 3, 4, 5, 6, 7))
NEED, R = 0.70, 0.17
n = int(sys.argv[1]) if len(sys.argv) > 1 else 3

# 후보: 통과가 성립하는 곳만 (한쪽에 NEED 이상 남는 d 가 존재)
cand = []
for i in range(0, len(s), 3):
    for side, wall, sgn in (("left", dl[i], +1), ("right", dr[i], -1)):
        opp = (dr[i] if sgn > 0 else dl[i])       # 반대쪽 벽까지
        d = sgn * max(wall - R - 0.05, 0.0)       # 벽에 붙여 놓는다
        # 반대쪽 통로 폭 = 반대쪽 벽까지 + 콘 안쪽 가장자리까지
        free = opp + (abs(d) - R)
        if free >= NEED + 0.10:
            cand.append((s[i], i, side, d, free, abs(kap[i])))
if not cand:
    print("통과 가능한 배치가 없다"); sys.exit(1)

# 트랙에 고르게 n 개
cand.sort()
pick = [cand[int(len(cand) * (j + 0.5) / n)] for j in range(n)]
print(f"{'s':>7} {'side':>6} {'콘 d':>7} {'반대쪽 통로':>11} {'|kappa|':>8}   map 좌표")
for si, i, side, d, free, k in pick:
    cx = x[i] - d * np.sin(psi[i])
    cy = y[i] + d * np.cos(psi[i])
    print(f"{si:7.2f} {side:>6} {d:+7.2f} {free:11.2f} {k:8.2f}   ({cx:+.3f}, {cy:+.3f})")
print("\n# 배치 명령:")
for si, i, side, d, free, k in pick:
    cx = x[i] - d * np.sin(psi[i]); cy = y[i] + d * np.cos(psi[i])
    print(f"ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped "
          f'"{{header: {{frame_id: \'map\'}}, point: {{x: {cx:.3f}, y: {cy:.3f}, z: 0.0}}}}"')
