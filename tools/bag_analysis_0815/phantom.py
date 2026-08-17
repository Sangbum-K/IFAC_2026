#!/usr/bin/env python3
"""유령 장애물의 방아쇠가 (a) 급가감속인가 (b) 특정 위치인가.

유령 = 수명이 짧은(<0.4 s) 트랙 id. 각 유령마다 발생 s, 그 순간 자차 s,
종가속, 피치각속도, 맵 잔차를 기록해 두 가설을 분리한다.
"""
import pickle
from collections import defaultdict
from pathlib import Path

import numpy as np

D = pickle.load(open(Path(__file__).parent / "data.pkl", "rb"))
fre = D["/car_state/frenet/odom"]
imu = D["/sensors/imu/raw"]
t = fre[:, 0]
v = fre[:, 4]
alon = np.convolve(np.gradient(v, t), np.ones(5) / 5, mode="same")

# ---- id별 생존 구간 ----
life = defaultdict(list)
for ts, lst in D["obs_frenet"]:
    for o in lst:
        life[o["id"]].append((ts, o["s_center"], o["d_center"], o["size"],
                              o["is_static"], o["is_visible"]))

rows = []
for oid, obs in life.items():
    obs.sort()
    t0, t1 = obs[0][0], obs[-1][0]
    dur = t1 - t0
    s_med = np.median([o[1] for o in obs])
    d_med = np.median([o[2] for o in obs])
    ego_s = np.interp(t0, t, fre[:, 1])
    rows.append((oid, t0, t1, dur, len(obs), s_med, d_med, ego_s,
                 np.interp(t0, t, alon),
                 np.interp(t0, imu[:, 0], imu[:, 3])))
R = np.array(rows)
R = R[R[:, 1].argsort()]

SHORT = 0.4
ph = R[:, 3] < SHORT
print(f"전체 트랙 id {len(R)}개.  수명<{SHORT}s 유령 {ph.sum()}개 "
      f"({100*ph.mean():.0f}%),  지속 {(~ph).sum()}개")
print(f"유령 수명: med {np.median(R[ph,3])*1000:.0f} ms  "
      f"관측횟수 med {np.median(R[ph,4]):.0f}")

print("\n=== 가설 A: 급가감속이 방아쇠인가 ===")
print("유령 발생 순간 종가속 분포 vs 전체 주행 시간 분포")
for lo, hi, lbl in ((-99, -2, "a<-2"), (-2, -0.8, "-2~-0.8"), (-0.8, 0.8, "정상"),
                    (0.8, 2, "0.8~2"), (2, 99, "a>2")):
    pm = (R[ph, 8] >= lo) & (R[ph, 8] < hi)
    bm = (alon >= lo) & (alon < hi)
    print(f"  {lbl:9s} 유령 {100*pm.mean():5.1f}%   주행시간 {100*bm.mean():5.1f}%  "
          f"-> 비율 {pm.mean()/max(bm.mean(),1e-9):.2f}x")

print("\n피치각속도 |wy|:")
for lo, hi in ((0, 5), (5, 10), (10, 20), (20, 40), (40, 999)):
    aw = np.abs(np.interp(t, imu[:, 0], imu[:, 3]))
    pm = (np.abs(R[ph, 9]) >= lo) & (np.abs(R[ph, 9]) < hi)
    bm = (aw >= lo) & (aw < hi)
    print(f"  {lo:3d}-{hi:3d} deg/s  유령 {100*pm.mean():5.1f}%   "
          f"주행시간 {100*bm.mean():5.1f}%  -> 비율 {pm.mean()/max(bm.mean(),1e-9):.2f}x")

print("\n=== 가설 B: 특정 위치가 방아쇠인가 ===")
print("유령이 '찍힌' 트랙 위치 s (2 m 빈):")
gw = D["global_wpnts"][0][1]
s_max = gw[-1, 0]
bins = np.arange(0, s_max + 2, 2.0)
h_ph, _ = np.histogram(R[ph, 5], bins=bins)
h_real, _ = np.histogram(R[~ph, 5], bins=bins)
# 자차가 각 s 를 지난 시간 (노출량)
h_exp, _ = np.histogram(fre[:, 1], bins=bins)
h_exp = h_exp / h_exp.sum()
for i in range(len(bins) - 1):
    if h_ph[i] == 0 and h_real[i] == 0:
        continue
    bar = "#" * int(h_ph[i])
    print(f"  s {bins[i]:5.1f}-{bins[i+1]:5.1f}  유령 {h_ph[i]:3d}  "
          f"지속 {h_real[i]:3d}   노출 {100*h_exp[i]:4.1f}%  {bar}")

print("\n유령 발생 시 자차 위치 s (2 m 빈):")
h_ego, _ = np.histogram(R[ph, 7], bins=bins)
for i in range(len(bins) - 1):
    if h_ego[i] == 0:
        continue
    print(f"  ego s {bins[i]:5.1f}-{bins[i+1]:5.1f}  {h_ego[i]:3d}  "
          f"{'#'*int(h_ego[i])}")

print("\n=== 지속 트랙 (수명>=0.4s) 목록 ===")
print("  id     t0     t1   dur   n   s_med  d_med  ego_s0")
for r in R[~ph]:
    print(f"{int(r[0]):5d} {r[1]:6.2f} {r[2]:6.2f} {r[3]:5.2f} {int(r[4]):4d} "
          f"{r[5]:6.1f} {r[6]:+6.2f} {r[7]:7.1f}")

print("\n=== 사용자 지목 시각 전후 3 s 유령 목록 ===")
for lbl, a, b in (("34.3", 31.3, 35.3), ("46.6", 43.6, 47.6),
                  ("60.6", 57.6, 61.6), ("108", 105.0, 109.0)):
    print(f"\n--- {lbl} ---")
    m = ph & (R[:, 1] >= a) & (R[:, 1] <= b)
    for r in R[m]:
        print(f"  #{int(r[0]):3d} t={r[1]:6.2f} dur={r[3]*1000:4.0f}ms n={int(r[4])} "
              f"s={r[5]:5.1f} d={r[6]:+5.2f}  ego_s={r[7]:5.1f} "
              f"a_lon={r[8]:+5.2f} wy={r[9]:+6.1f}")
