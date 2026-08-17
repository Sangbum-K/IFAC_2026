#!/usr/bin/env python3
"""급가감속 -> 피치 -> LiDAR 지면 히트 -> 유령 벽 가설 검증.

관심 시각: 34.3 / 46.6 / 60.6 s (사용자 지목), 105-108 s (회피 불안정).
IMU wy(피치각속도) / ax, 종가속(프레네 v 미분), 스캔 통계를 함께 본다.
"""
import pickle
from pathlib import Path

import numpy as np

D = pickle.load(open(Path(__file__).parent / "data.pkl", "rb"))
fre = D["/car_state/frenet/odom"]        # t s d yaw v
imu = D["/sensors/imu/raw"]              # t yaw wx wy wz ax ay az
cmd = D["/autonomy/ackermann_cmd"]       # t steer speed
core = D["/sensors/core"]                # t v_in i_motor erpm duty

print(f"imu n={len(imu)}  cols=t,yaw,wx,wy,wz,ax,ay,az")
print("imu 정지구간(0-1s) 평균:", np.round(imu[imu[:, 0] < 1.0, 1:].mean(axis=0), 3))
for j, nm in enumerate(("wx", "wy", "wz", "ax", "ay", "az"), start=2):
    c = imu[:, j]
    print(f"  {nm}: mean {c.mean():+8.3f} std {c.std():6.3f} "
          f"p1 {np.percentile(c,1):+7.3f} p99 {np.percentile(c,99):+7.3f} "
          f"min {c.min():+7.3f} max {c.max():+7.3f}")

# ---- 종가속: 프레네 v 미분 (0.2 s 중심차분) ----
t = fre[:, 0]
v = fre[:, 4]
ax_dyn = np.gradient(v, t)
# 노이즈 억제: 5샘플 이동평균 (~0.125 s @40 Hz)
k = 5
ax_s = np.convolve(ax_dyn, np.ones(k) / k, mode="same")

print("\n=== 종가속(프레네 v 미분, 평활) 분포 ===")
print(f"  p1 {np.percentile(ax_s,1):+.2f}  p5 {np.percentile(ax_s,5):+.2f}  "
      f"p95 {np.percentile(ax_s,95):+.2f}  p99 {np.percentile(ax_s,99):+.2f}  "
      f"min {ax_s.min():+.2f} max {ax_s.max():+.2f} m/s^2")

# ---- 스캔 통계: 지면 히트 지표 ----
# LiDAR 가 아래로 기울면 (a) 전 방위에 걸쳐 중거리 링이 생기고
# (b) 원거리 빔이 사라진다 -> r_p90 급락 + 근거리 빔 비율 급등.
rows = []
for ts, amin, ainc, r in D["scan"]:
    ok = (r > 0.05) & np.isfinite(r)
    rr = r[ok]
    if len(rr) < 20:
        rows.append((ts, np.nan, np.nan, np.nan, np.nan, np.nan))
        continue
    n = len(r)
    ang = amin + ainc * np.arange(n)
    front = ok & (np.abs(ang) < np.radians(30))
    rows.append((ts,
                 np.percentile(rr, 90),
                 np.percentile(rr, 50),
                 (rr < 1.5).mean(),
                 np.percentile(r[front], 50) if front.sum() > 5 else np.nan,
                 len(rr) / n))
SC = np.array(rows)   # t r_p90 r_p50 frac_near front_p50 valid_frac
np.save(Path(__file__).parent / "scanstat.npy", SC)

print("\n=== 스캔 통계 분포 ===")
for j, nm in enumerate(("r_p90", "r_p50", "frac<1.5m", "front_p50", "valid"), start=1):
    c = SC[:, j]
    c = c[np.isfinite(c)]
    print(f"  {nm:10s} p5 {np.percentile(c,5):6.3f} med {np.median(c):6.3f} "
          f"p95 {np.percentile(c,95):6.3f}")


def win(arr, t0, t1, col):
    m = (arr[:, 0] >= t0) & (arr[:, 0] <= t1)
    return arr[m, col]


EVENTS = [("34.3 지목", 33.0, 35.5), ("46.6 지목", 45.3, 47.5),
          ("60.6 지목", 59.3, 61.5), ("105-108 불안정", 104.5, 108.5)]

print("\n=== 지목 시각 주변 상세 (0.1 s 간격) ===")
for name, t0, t1 in EVENTS:
    print(f"\n--- {name}  [{t0}, {t1}] ---")
    print("   t     v   a_lon   wy(pitch)   ax_imu   r_p90  front_p50 near%  cmd_v  duty")
    for tq in np.arange(t0, t1, 0.1):
        print(f"{tq:6.2f} {np.interp(tq,t,v):5.2f} {np.interp(tq,t,ax_s):+6.2f} "
              f"{np.interp(tq,imu[:,0],imu[:,3]):+9.3f} "
              f"{np.interp(tq,imu[:,0],imu[:,6]):+8.3f} "
              f"{np.interp(tq,SC[:,0],SC[:,1]):7.2f} "
              f"{np.interp(tq,SC[:,0],SC[:,4]):8.2f} "
              f"{100*np.interp(tq,SC[:,0],SC[:,3]):5.1f} "
              f"{np.interp(tq,cmd[:,0],cmd[:,2]):6.2f} "
              f"{np.interp(tq,core[:,0],core[:,4]):6.2f}")

# ---- 전역 상관: |a_lon| 큰 순간에 r_p90 이 실제로 떨어지나 ----
print("\n=== 상관 분석: 종가속 vs 스캔 지표 ===")
sc_t = SC[:, 0]
ok = np.isfinite(SC[:, 1])
a_at_scan = np.interp(sc_t, t, ax_s)
v_at_scan = np.interp(sc_t, t, v)
wy_at_scan = np.interp(sc_t, imu[:, 0], imu[:, 3])
for lo, hi, lbl in ((-99, -2.0, "강한 감속 a<-2"), (-2.0, -0.8, "감속"),
                    (-0.8, 0.8, "정상"), (0.8, 2.0, "가속"),
                    (2.0, 99, "강한 가속 a>2")):
    m = ok & (a_at_scan >= lo) & (a_at_scan < hi)
    if m.sum() < 10:
        continue
    print(f"  {lbl:14s} n={m.sum():5d}  r_p90 med {np.median(SC[m,1]):5.2f}  "
          f"front_p50 med {np.median(SC[m,4]):5.2f}  "
          f"near% med {100*np.median(SC[m,3]):5.1f}  "
          f"|wy| med {np.median(np.abs(wy_at_scan[m])):5.3f}")

print("\n  참고: 속도 구간별 (가감속과 분리)")
for lo, hi in ((0, 1.5), (1.5, 3), (3, 4.5), (4.5, 9)):
    m = ok & (v_at_scan >= lo) & (v_at_scan < hi)
    if m.sum() < 10:
        continue
    print(f"  v {lo}-{hi}: n={m.sum():5d} r_p90 med {np.median(SC[m,1]):5.2f} "
          f"near% {100*np.median(SC[m,3]):5.1f}")
