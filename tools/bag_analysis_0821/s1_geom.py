#!/usr/bin/env python3
"""S1(봉쇄 정지) 배치 계산: 상대는 레이스라인(d=0) 위에 세우고, 그 옆에
콘 하나를 놓아 **양쪽 다 통과 불가**를 만든다.

통과 가능 조건(MoreSpace): 그쪽 여유 >= RequiredObstacleClearance + 경계여유
                          ~= 0.30 + 0.25 = 0.55 m  (실측 p50)
"""
import pickle, numpy as np
D = pickle.load(open("d.pkl", "rb"))
gw = D["global_wpnts"][0][1]      # s d x y d_r d_l psi kappa vx ax
s, x, y, dr, dl, psi, kap = (gw[:, i] for i in (0, 2, 3, 4, 5, 6, 7))
w = dr + dl
NEED = 0.55
OPP_HALF = 0.15     # 상대 반폭
CONE_R = 0.17       # static_cylinder_radius_m

print(f"{'s':>6} {'폭':>5} {'|k|':>5} {'우여유':>7} {'좌여유':>7} {'양쪽막힘?':>9}")
best = []
for i in range(0, len(s), 5):
    if abs(kap[i]) > 0.15:      # 직선만 (탐지 7.9 m 구간)
        continue
    # 상대가 d=0 에 섰을 때 남는 여유
    right_free = dr[i] - OPP_HALF
    left_free = dl[i] - OPP_HALF
    # 콘을 좌측에 놓아 좌측을 막는다: 콘 중심 d_c 이면 좌 여유 = dl - (d_c+CONE_R)
    # 좌측이 NEED 미만이 되려면 d_c >= dl - NEED - CONE_R
    d_cone = dl[i] - NEED - CONE_R
    # 그 콘이 상대와 겹치지 않아야 한다 (d_c - CONE_R > OPP_HALF)
    ok = (right_free < NEED) and (d_cone - CONE_R > OPP_HALF)
    print(f"{s[i]:6.2f} {w[i]:5.2f} {abs(kap[i]):5.2f} {right_free:7.2f} {left_free:7.2f} "
          f"{'예' if ok else ('우측 열림' if right_free>=NEED else '콘/상대 겹침'):>9}"
          + (f"  콘 d={d_cone:+.2f}" if ok else ""))
    if ok:
        best.append((i, d_cone))

if best:
    i, d_cone = best[len(best) // 2]
    cx = x[i] - d_cone * np.sin(psi[i])
    cy = y[i] + d_cone * np.cos(psi[i])
    print(f"\n권장: s={s[i]:.2f} 에서 콘 d={d_cone:+.2f}")
    print(f"  콘 map 좌표  x={cx:.3f} y={cy:.3f}")
    print(f"  상대 정지 s  {s[i]:.2f}  (레이스라인 위, map x={x[i]:.3f} y={y[i]:.3f})")
    print(f"\n  ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped "
          f"\"{{header: {{frame_id: 'map'}}, point: {{x: {cx:.3f}, y: {cy:.3f}, z: 0.0}}}}\"")
else:
    print("\n직선 중 우측이 자연히 막히는 지점이 없다 — 콘 2개를 써야 한다.")
    # 대안: 상대 좌측 + 우측에 각각 하나
    i = int(np.argmin(np.where(np.abs(kap) < 0.15, w, 1e9)))
    print(f"  가장 좁은 직선 s={s[i]:.2f} 폭 {w[i]:.2f} (d_r {dr[i]:.2f} / d_l {dl[i]:.2f})")
    for side, dd in (("좌", dl[i] - NEED - CONE_R), ("우", -(dr[i] - NEED - CONE_R))):
        cx = x[i] - dd * np.sin(psi[i]); cy = y[i] + dd * np.cos(psi[i])
        print(f"  {side}측 콘 d={dd:+.2f} -> map x={cx:.3f} y={cy:.3f}"
              f"  (상대와 간격 {abs(dd)-CONE_R-OPP_HALF:+.2f} m)")
