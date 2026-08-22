#!/usr/bin/env python3
"""맵 좌표 장면 재구성 + 스캔 조기가시성 검증.
usage: scene.py <t_center> <t_first_detect> <s_obs>
- 트랙 폭(global wpnts d_left/d_right) at s_obs
- t in [t_center-10, t_center+3]에서 pf pose로 스캔을 맵 좌표로 투영,
  장애물 무더기 반경 1.2m 내 스캔 포인트 수를 시간별로 셈 -> LiDAR가 언제부터 봤는지
- 첫 인지 시점의 장애물 map 위치 출처: obs_frenet의 x,y
"""
import pickle
import sys
import numpy as np

D = pickle.load(open("/home/miru/IFAC_ws/tools/bag_analysis_0822c/d26.pkl", "rb"))
TC = float(sys.argv[1])
TD = float(sys.argv[2])   # first detection time
S_OBS = float(sys.argv[3])

gw = D["global_wpnts"][0][1]  # s d x y d_r d_l psi kappa vx ax
print(f"track len ~ {gw[-1,0]:.2f}, n={len(gw)}")
i = np.argmin(np.abs(gw[:, 0] - S_OBS))
print(f"wpnt s={gw[i,0]:.2f}: x={gw[i,2]:.2f} y={gw[i,3]:.2f} d_right={gw[i,4]:.2f} d_left={gw[i,5]:.2f} psi={gw[i,6]:.2f} vx_plan={gw[i,8]:.2f}")

# 첫 인지 시점 장애물들의 맵 위치
obs = D["obs_frenet"]
obs_t = np.array([t for t, _ in obs])
j = np.searchsorted(obs_t, TD + 0.15)
pts = []
for tt, ol in obs[j:j+6]:
    for o in ol:
        ds = (o["s_start"] - S_OBS) % 34.44
        if ds < 2.5 or ds > 32:
            pts.append((o["id"], o["x"], o["y"], o["s_center"], o["d_center"]))
seen = {}
for oid, x, y, sc, dc in pts:
    seen.setdefault(oid, (x, y, sc, dc))
print("first-detection obstacles near pile:")
cx, cy, n = 0.0, 0.0, 0
for oid, (x, y, sc, dc) in seen.items():
    print(f"  id{oid}: map=({x:.2f},{y:.2f}) s={sc:.2f} d={dc:+.2f}")
    cx += x; cy += y; n += 1
cx /= max(n, 1); cy /= max(n, 1)
print(f"pile center approx ({cx:.2f},{cy:.2f})")

# pf pose 보간
pf = D["/pf/pose/odom"]  # t x y yaw v
scan = D["scan"]         # (t, amin, ainc, ranges[stride4])

def pose_at(t):
    i = np.searchsorted(pf[:, 0], t)
    i = min(max(i, 1), len(pf) - 1)
    return pf[i, 1], pf[i, 2], pf[i, 3]

print(f"\nscan hits within 0.9m of pile center, and ego dist to pile:")
print(f"{'t':>7} {'ego_dist':>8} {'n_hits':>6}  first_seen_range")
for tt, amin, ainc, rr in scan:
    if not (TC - 12 <= tt <= TC + 2):
        continue
    if int(tt * 5) != tt * 5 // 1 * 1 and False:
        pass
    x0, y0, yaw = pose_at(tt)
    ego_d = np.hypot(cx - x0, cy - y0)
    ang = amin + np.arange(len(rr)) * ainc * 4 + yaw
    m = np.isfinite(rr) & (rr > 0.05) & (rr < 12)
    px = x0 + rr[m] * np.cos(ang[m])
    py = y0 + rr[m] * np.sin(ang[m])
    dd = np.hypot(px - cx, py - cy)
    nh = int((dd < 0.9).sum())
    # print every ~0.5s
    if abs((tt * 2) - round(tt * 2)) < 0.06:
        print(f"{tt:7.2f} {ego_d:8.2f} {nh:6d}")
