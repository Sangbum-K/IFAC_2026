#!/usr/bin/env python3
"""충돌/접촉 사건 탐지 + 스캔 기반 실제 벽 여유(계획과 독립)."""
import pickle
from pathlib import Path

import numpy as np

D = pickle.load(open(Path(__file__).parent / "data.pkl", "rb"))
fre = D["/car_state/frenet/odom"]        # t s d yaw v
pf = D["/pf/pose/odom"]
imu = D["/sensors/imu/raw"]              # t yaw wx wy wz ax ay az
core = D["/sensors/core"]                # t v_in i_motor erpm duty
cmd = D["/autonomy/ackermann_cmd"]       # t steer speed


def at(arr, t, col):
    return np.interp(t, arr[:, 0], arr[:, col])


# ---- 스캔에서 차체 좌/우 최소 여유 (laser frame, x 전방) ----
# 차체 옆 ±0.5m 종방향 창에서 좌/우 최소 |y|, 그리고 전방 3m 창의 좌/우 최소값.
rows = []
for t, amin, ainc, r in D["scan"]:
    n = len(r)
    ang = amin + ainc * np.arange(n)
    ok = r > 0.05
    xs = r * np.cos(ang)
    ys = r * np.sin(ang)
    side = ok & (np.abs(xs) < 0.35)               # 차체 옆
    fwd = ok & (xs > 0.0) & (xs < 2.5)            # 전방 2.5m
    def mn(m, sgn):
        v = ys[m & (np.sign(ys) == sgn)]
        return np.abs(v).min() if len(v) else np.nan
    rows.append((t, mn(side, 1), mn(side, -1), mn(fwd, 1), mn(fwd, -1),
                 r[ok].min() if ok.any() else np.nan))
S = np.array(rows)   # t, side_L, side_R, fwd_L, fwd_R, rmin
np.save(Path(__file__).parent / "scanclear.npy", S)

print("=== 접촉 후보 (차체 옆 여유 < 0.20m 또는 최근접 < 0.12m) ===")
hit = (np.nanmin(S[:, 1:3], axis=1) < 0.20) | (S[:, 5] < 0.12)
i = 0
ev = []
while i < len(S):
    if hit[i]:
        j = i
        while j + 1 < len(S) and (hit[j + 1] or (j + 3 < len(S) and hit[j:j+4].any())):
            j += 1
        t0, t1 = S[i, 0], S[j, 0]
        m = (fre[:, 0] >= t0 - 0.3) & (fre[:, 0] <= t1 + 0.3)
        if m.any():
            sm = fre[m]
            ev.append((t0, t1, sm[:, 1].mean(), sm[:, 2].mean(), sm[:, 4]))
            print(f"t {t0:6.2f}-{t1:6.2f} ({t1-t0:4.2f}s) s≈{sm[:,1].mean():5.1f} "
                  f"d≈{sm[:,2].mean():+5.2f} v {sm[:,4].max():.2f}->{sm[:,4].min():.2f} "
                  f"sideL={np.nanmin(S[i:j+1,1]):.2f} sideR={np.nanmin(S[i:j+1,2]):.2f} "
                  f"rmin={np.nanmin(S[i:j+1,5]):.2f}")
        i = j + 1
    else:
        i += 1

# ---- 급감속 사건 (충돌 신호) ----
print("\n=== 급감속 (0.3s 내 v 하락 > 1.5 m/s) ===")
v = fre[:, 4]
t = fre[:, 0]
for i in range(len(t)):
    j = np.searchsorted(t, t[i] + 0.3)
    if j < len(t) and v[i] - v[j] > 1.5:
        if not any(abs(t[i] - p) < 1.0 for p in globals().setdefault("_seen", [])):
            globals()["_seen"].append(t[i])
            print(f"t {t[i]:6.2f} s={fre[i,1]:5.1f} d={fre[i,2]:+5.2f} "
                  f"v {v[i]:.2f}->{v[j]:.2f}  cmd_v={at(cmd,t[i],2):.2f} "
                  f"steer={np.degrees(at(cmd,t[i],1)):+5.1f}deg "
                  f"sideL={np.interp(t[i],S[:,0],S[:,1]):.2f} "
                  f"sideR={np.interp(t[i],S[:,0],S[:,2]):.2f}")

# ---- 조향 포화 ----
print("\n=== 조향 명령 포화(|steer|>0.34 rad) 구간 ===")
st = np.abs(cmd[:, 1]) > 0.34
i = 0
while i < len(st):
    if st[i]:
        j = i
        while j + 1 < len(st) and st[j + 1]:
            j += 1
        if cmd[j, 0] - cmd[i, 0] > 0.15:
            print(f"t {cmd[i,0]:6.2f}-{cmd[j,0]:6.2f} ({cmd[j,0]-cmd[i,0]:4.2f}s) "
                  f"s≈{at(fre,cmd[i,0],1):5.1f} d={at(fre,cmd[i,0],2):+5.2f} "
                  f"steer_max={np.degrees(np.abs(cmd[i:j+1,1]).max()):+.1f}deg "
                  f"v={at(fre,cmd[i,0],4):.2f}")
        i = j + 1
    else:
        i += 1
