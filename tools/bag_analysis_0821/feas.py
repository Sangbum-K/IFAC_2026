import numpy as np
a_dec=1.5; T=0.3           # trailing_decel_dyn, trailing_lag_sec
a_lat=8.83                 # path_lat_acc_max (real_car)
contact=0.44               # trailing_floor_contact_dist_m (실측)
print("[A] 접근 단계 — 꽁지 0.30 m(=planner dist 0.74)까지 붙는 데 필요한 거리")
print("    필요 = (v^2-vo^2)/(2a) + v*T + 0.74      a=1.5, T=0.3")
print(f"    {'자차v':>6} | " + " ".join(f"상대{vo:.1f}".rjust(8) for vo in (1.5,2.5,3.5,4.5)))
for v in (3.0,4.0,5.0,6.0):
    row=[]
    for vo in (1.5,2.5,3.5,4.5):
        if vo>=v: row.append("   -   "); continue
        row.append(f"{(v*v-vo*vo)/(2*a_dec)+v*T+0.74:7.2f}")
    print(f"    {v:6.1f} | " + " ".join(x.rjust(8) for x in row))
print("    실측 탐지 지평: 코너 2.85 m / 중앙 4.1 m / 직선 7.9 m")

print("\n[B] 꽁지에서 옆으로 빠지는 단계 — 횡 A 만큼 이동")
print("    최소 소요시간 T_lat = sqrt(6A/a_lat)  (속도 무관), 주행거리 = v*T_lat")
print(f"    {'A[m]':>6} {'T_lat[s]':>9} " + " ".join(f"v={v} 주행".rjust(11) for v in (2,3,4,5)))
for A in (0.35,0.45,0.60,0.80):
    Tl=np.sqrt(6*A/a_lat)
    print(f"    {A:6.2f} {Tl:9.3f} " + " ".join(f"{v*Tl:10.2f}m" for v in (2,3,4,5)))
print("    핵심: 뒤에 붙어 있으면 이 주행거리는 '장애물까지 남은 거리'와 무관하다.")
print("          지금 코드는 이 둘을 같은 것으로 보고 lane_geom_floor 로 기각한다.")

print("\n[C] 빠지는 0.55 s 동안 상대가 옆으로 움직이는 양 (실측 |Δd| 분포에서 환산)")
for name,p50,p90 in (("이 백(수동 RC, 흔들림 큼)",0.24,0.79),):
    print(f"    {name}: 1s p50 {p50:.2f}/p90 {p90:.2f} -> 0.55s p50 {p50*0.55:.2f}/p90 {p90*0.55:.2f} m")
print("    -> 목표 횡간격 0.45 m 중 p90 0.43 m 가 상대 흔들림으로 사라진다.")
print("       레이스라인을 따르는 진짜 상대라면 이 항이 크게 줄어든다 (이 백으로는 못 잼).")

print("\n[D] 꽁지 0.30 m 유지의 측정 가능성")
print(f"    planner dist 로는 0.74 m, 접촉선은 {contact:.2f} m -> 여유 0.30 m")
print("    실측 지터: std 0.126 / |잔차| p90 0.131 / 프레임간 s_start 점프 p99 0.360 m")
print("    -> p90 지터는 여유 0.30 안쪽이라 유지 가능. p99 점프(0.36)는 넘는다 = 가끔 오경보.")
print("       10프레임 평균(trailing_vs_filter 와 같은 창)이면 std 0.126 -> 0.040 m.")
