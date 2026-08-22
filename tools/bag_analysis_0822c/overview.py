#!/usr/bin/env python3
"""run_0822_172617: 충돌 3건(~100s, ~112s, ~168s) 오버뷰."""
import pickle
import numpy as np

D = pickle.load(open("/home/miru/IFAC_ws/tools/bag_analysis_0822c/d26.pkl", "rb"))

fo = D["/car_state/frenet/odom"]   # t x(=s) y(=d) yaw v
pf = D["/pf/pose/odom"]
spd = D["/monitor/current_speed"]
acmd = D["/autonomy/ackermann_cmd"]  # t steer speed
estop = D["/safety/emergency_stop"]
imu = D["/sensors/imu/raw"]        # t yaw wx wy wz ax ay az

print("=== 충돌 후보: IMU 종가속 급감(ax 스파이크) ===")
t = imu[:, 0]
ax = imu[:, 5]
# 5-sample smoothing
k = np.ones(5) / 5
axs = np.convolve(ax, k, "same")
for i in np.where(axs < -6.0)[0]:
    if i > 0 and axs[i] < axs[i-1]:
        pass
# find local minima below -5
idx = np.where(axs < -5.0)[0]
groups = []
for i in idx:
    if groups and t[i] - groups[-1][-1] < 1.0:
        groups[-1].append(t[i])
    else:
        groups.append([t[i]])
for g in groups:
    tm = g[0]
    print(f"  t={tm:7.2f}s  ax_min={axs[(t>=g[0])&(t<=g[-1])].min():.1f}")

print("\n=== 속도 개요 (2초 간격) ===")
print("estop 메시지 수:", len(estop), " True 구간:", estop[estop[:, 1] > 0.5][:, 0] if len(estop) else "-")

print("\n=== corridor_blocked / escape / evade_committed ===")
# corridor_blocked was bool topic - check extraction
import sqlite3
db = sqlite3.connect("/home/miru/IFAC_ws/rosbag0822/run_0822_172617/run_0822_172617_0.db3")
topics = {n: tid for tid, n, ty in db.execute("SELECT id,name,type FROM topics")}
t0 = D["t0_ns"]
import sys
sys.path.insert(0, "/home/miru/IFAC_ws/tools")
from bagviz import dec_bool
for name in ("/planner/corridor_blocked", "/planner/avoidance/evade_committed"):
    r = db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
                   (topics[name],)).fetchall()
    arr = np.array([((ts - t0) / 1e9, 1.0 if dec_bool(d) else 0.0) for ts, d in r])
    D[name] = arr
    if len(arr):
        # print transitions
        ch = np.where(np.diff(arr[:, 1]) != 0)[0]
        print(f"{name}: n={len(arr)} transitions:")
        for i in ch:
            print(f"    t={arr[i+1,0]:7.2f}s -> {arr[i+1,1]:.0f}")
    else:
        print(f"{name}: empty")

esc = D["/planner/escape_active"]
if len(esc):
    ch = np.where(np.diff(esc[:, 1]) != 0)[0]
    print("/planner/escape_active transitions:")
    for i in ch:
        print(f"    t={esc[i+1,0]:7.2f}s -> {esc[i+1,1]:.0f}")

print("\n=== 각 윈도우 상태 ===")
for tc in (100, 112, 168):
    w0, w1 = tc - 8, tc + 5
    m = (fo[:, 0] >= w0) & (fo[:, 0] <= w1)
    seg = fo[m]
    print(f"\n--- window {w0}..{w1}s ---")
    for row in seg[::10]:
        print(f"  t={row[0]:7.2f} s={row[1]:6.2f} d={row[2]:+.2f} v={row[4]:5.2f}")
