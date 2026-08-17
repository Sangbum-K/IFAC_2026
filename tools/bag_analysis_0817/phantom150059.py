#!/usr/bin/env python3
"""정지 구간(t>30)에 실제로 앞에 물체가 있었는가 — scan 직접 확인."""
import pickle
import numpy as np

D = pickle.load(open("d150059.pkl", "rb"))

print("=== raw perception (/perception/obstacles) t>26 ===")
n_empty = 0
for t, obs in D["obs_perc"]:
    if t < 26:
        continue
    if not obs:
        n_empty += 1
        continue
    print(f"{t:6.2f} " + " ".join(
        f"id={o['id']} xy=({o['x']:.2f},{o['y']:.2f}) r={o['radius']:.2f} "
        f"conf={o['conf']:.2f} vis={o['vis']}" for o in obs))
print(f"(빈 프레임 {n_empty}개)")

print("\n=== 정지 중 ego pose ===")
pf = D["/pf/pose/odom"]
m = pf[:, 0] > 31
print(f"x={pf[m,1].mean():.3f} y={pf[m,2].mean():.3f} yaw={pf[m,3].mean():.3f} "
      f"std_x={pf[m,1].std():.4f}")
ex, ey, eyaw = pf[m, 1].mean(), pf[m, 2].mean(), pf[m, 3].mean()

print("\n=== 정지 중 전방 스캔 (차량 좌표, 3m 이내) ===")
# 마지막 스캔 몇 개 평균
for t, amin, ainc, rr in D["scan"][-5:-4]:
    ang = amin + ainc * np.arange(len(rr))
    ok = np.isfinite(rr) & (rr > 0.05) & (rr < 4.0)
    a, r = ang[ok], rr[ok]
    x = r * np.cos(a)
    y = r * np.sin(a)
    print(f"t={t:.2f} beams={ok.sum()}")
    # 전방 ±60도, 0.2~3 m
    fwd = (np.abs(a) < np.deg2rad(70)) & (r < 3.0)
    idx = np.argsort(a[fwd])
    aa, rrr, xx, yy = a[fwd][idx], r[fwd][idx], x[fwd][idx], y[fwd][idx]
    for i in range(0, len(aa), 2):
        print(f"   ang={np.rad2deg(aa[i]):7.1f}  r={rrr[i]:5.2f}  "
              f"veh=({xx[i]:6.2f},{yy[i]:6.2f})")
