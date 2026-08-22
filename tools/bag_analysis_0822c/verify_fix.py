#!/usr/bin/env python3
"""수정 전/후 프로파일을 같은 식으로 전방적분해 접촉 속도를 비교."""
import numpy as np

LAG, A_CAP, WD = 0.3, 4.19, 0.1   # 구동계 지연, 실측 제동 능력, wpnt 간격

def v_stop(rem, floor, decel):
    aT = decel * LAG
    return max(-aT + np.sqrt(aT*aT + floor*floor + 2.0*decel*rem), 0.0)

def reachable(dist, v0, ramp_a=2.0):
    eff = max(dist - v0*LAG, 0.0)
    return np.sqrt(max(v0*v0 - 2.0*ramp_a*eff, 0.0))

def sim(v0, stop_dist, gap, floor, decel, label):
    """gap = 장애물 전면까지 남은 거리. stop_dist = gap - trailing_gap(1.5)."""
    v, x, dt = v0, 0.0, 0.005
    hist = []
    while x < gap and v > 0.01:
        rem = max(stop_dist - 0.0, 0.0)          # 명령 선두(dist=0)
        cmd = max(v_stop(rem, floor, decel), reachable(0.0, v), floor)
        # 컨트롤러: 명령보다 빠르면 제동(능력 한계), 느리면 유지
        a = -min(A_CAP, max((v - cmd)/LAG, 0.0)) if v > cmd else 0.0
        v = max(v + a*dt, 0.0)
        x += v*dt
        stop_dist = max(stop_dist - v*dt, 0.0)
        hist.append((x, v, cmd))
    return v, hist

print("=== 케이스3 (2:48) — 램프충돌 분기, t=167.50 ===")
print("   gap 3.53m, v0 4.51, trailing_gap 1.5 -> stop_dist 2.03")
for label, floor, decel in (("수정 전 (decel 1.0, 바닥 2.5)", 2.5, 1.0),
                             ("수정 후 (decel 3.0, 바닥 2.5)", 2.5, 3.0),
                             ("수정 후 + 래치(바닥 0.5)",      0.5, 3.0)):
    v, _ = sim(4.51, 2.03, 3.53, floor, decel, label)
    print(f"  {label:32s} -> 장애물 전면 통과 속도 {v:.2f} m/s")

print()
print("=== 케이스1 (1:40) — t=101.38, stop_dist 0.00 (이미 안쪽) ===")
print("   gap 0.5m, v0 1.5 (바닥 degraded 1.5)")
for label, floor, decel in (("수정 전 (decel 1.0, 바닥 1.5)", 1.5, 1.0),
                             ("수정 후 (접촉확정 -> 크리프 0.5)", 0.5, 3.0)):
    v, _ = sim(1.5, 0.0, 0.5, floor, decel, label)
    print(f"  {label:32s} -> 접촉 속도 {v:.2f} m/s")

print()
print("=== 접촉확정 판정이 통상 트레일링을 건드리는가 ===")
brake = 3.0
for floor in (2.5, 1.5):
    d = floor*floor/(2*brake)
    print(f"  바닥 {floor} -> 이 거리 미만에서만 크리프 전환: stop_dist < {d:.2f} m")
print("  (케이스3 t=167.50 stop_dist 2.03 / 케이스1 t=100.43 stop_dist 2.32 -> 전환 안 함)")
