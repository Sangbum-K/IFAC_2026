#!/usr/bin/env python3
"""트랙 코리도 3중 비교:
  (A) global_waypoints 의 d_left/d_right  — 플래너가 믿는 벽
  (B) 점유격자에서 수직 레이캐스트한 벽    — 지도의 실제 벽
  (C) 주행 중 LiDAR 를 프레네로 투영한 벽  — 실제 관측된 벽
"""
import pickle
from pathlib import Path

import numpy as np

P = Path(__file__).parent
D = pickle.load(open(P / "data.pkl", "rb"))
gw = D["global_wpnts"][0][1]           # s d x y d_r d_l psi kappa vx ax
s, gx, gy, psi = gw[:, 0], gw[:, 2], gw[:, 3], gw[:, 6]
A_l, A_r = np.abs(gw[:, 5]), np.abs(gw[:, 4])
mp = D["map"]
G, res, ox, oy = mp["data"], mp["res"], mp["ox"], mp["oy"]
H, W = G.shape


def occupied(px, py):
    j = int((px - ox) / res)
    i = int((py - oy) / res)
    if i < 0 or j < 0 or i >= H or j >= W:
        return True
    return G[i, j] != 0


def raycast(x0, y0, ang, maxd=3.0, step=0.02):
    d = 0.0
    while d < maxd:
        d += step
        if occupied(x0 + d * np.cos(ang), y0 + d * np.sin(ang)):
            return d
    return maxd


# (B) 지도 레이캐스트
B_l = np.array([raycast(gx[i], gy[i], psi[i] + np.pi / 2) for i in range(len(gw))])
B_r = np.array([raycast(gx[i], gy[i], psi[i] - np.pi / 2) for i in range(len(gw))])

# (C) LiDAR -> 프레네
pf = D["/pf/pose/odom"]                # t x y yaw v
LASER_X = 0.28
pts_s, pts_d = [], []
for t, amin, ainc, r in D["scan"][::3]:
    px = np.interp(t, pf[:, 0], pf[:, 1])
    py = np.interp(t, pf[:, 0], pf[:, 2])
    # yaw 는 unwrap 후 보간
    yaw = np.interp(t, pf[:, 0], np.unwrap(pf[:, 3]))
    ok = r > 0.05
    ang = amin + ainc * np.arange(len(r))
    lx = r * np.cos(ang) + LASER_X
    ly = r * np.sin(ang)
    wx = px + lx * np.cos(yaw) - ly * np.sin(yaw)
    wy = py + lx * np.sin(yaw) + ly * np.cos(yaw)
    wx, wy = wx[ok], wy[ok]
    if not len(wx):
        continue
    # 최근접 웨이포인트
    dx = wx[:, None] - gx[None, :]
    dy = wy[:, None] - gy[None, :]
    dist2 = dx * dx + dy * dy
    k = dist2.argmin(axis=1)
    # 부호 있는 횡거리 (왼쪽 +)
    sd = -np.sin(psi[k]) * dx[np.arange(len(wx)), k] + \
        np.cos(psi[k]) * dy[np.arange(len(wx)), k]
    pts_s.append(s[k])
    pts_d.append(sd)
pts_s = np.concatenate(pts_s)
pts_d = np.concatenate(pts_d)

ds = s[1] - s[0]
bin_i = np.clip((pts_s / ds).astype(int), 0, len(gw) - 1)
C_l = np.full(len(gw), np.nan)
C_r = np.full(len(gw), np.nan)
for i in range(len(gw)):
    m = bin_i == i
    if m.sum() < 20:
        continue
    dd = pts_d[m]
    lp, rp = dd[dd > 0.05], dd[dd < -0.05]
    if len(lp) > 10:
        C_l[i] = np.percentile(lp, 90)
    if len(rp) > 10:
        C_r[i] = np.percentile(rp, 10)

np.savez(P / "corridor.npz", s=s, A_l=A_l, A_r=A_r, B_l=B_l, B_r=B_r,
         C_l=C_l, C_r=-C_r, pts_s=pts_s, pts_d=pts_d)

print("   s    | 웨이포인트 L/R | 지도레이 L/R | 스캔 L/R   | 오차(웨이-지도) L/R")
for i in range(0, len(gw), 3):
    print(f"{s[i]:6.2f} | {A_l[i]:5.2f} {A_r[i]:5.2f} | {B_l[i]:5.2f} {B_r[i]:5.2f} | "
          f"{C_l[i]:5.2f} {-C_r[i]:5.2f} | {A_l[i]-B_l[i]:+5.2f} {A_r[i]-B_r[i]:+5.2f}")

print("\n=== 요약 ===")
for lbl, a, b in (("좌", A_l, B_l), ("우", A_r, B_r)):
    e = a - b
    print(f"{lbl}벽: 웨이포인트 - 지도레이캐스트  mean {e.mean():+.3f}  "
          f"med {np.median(e):+.3f}  min {e.min():+.3f} @s={s[e.argmin()]:.1f}  "
          f"max {e.max():+.3f} @s={s[e.argmax()]:.1f}")
for lbl, a, c in (("좌", A_l, C_l), ("우", A_r, -C_r)):
    m = np.isfinite(c)
    e = (a - c)[m]
    print(f"{lbl}벽: 웨이포인트 - 스캔관측       mean {e.mean():+.3f}  "
          f"med {np.median(e):+.3f}  min {e.min():+.3f}  max {e.max():+.3f}")

print("\n=== 관심 구간 (헤어핀2 출구 s=26~31, 직선 s=32~35) ===")
for lo, hi in ((26, 31), (32, 35), (6, 9)):
    m = (s >= lo) & (s <= hi)
    print(f"s {lo}-{hi}: 웨이L {A_l[m].min():.2f}~{A_l[m].max():.2f} "
          f"지도L {B_l[m].min():.2f}~{B_l[m].max():.2f} | "
          f"웨이R {A_r[m].min():.2f}~{A_r[m].max():.2f} "
          f"지도R {B_r[m].min():.2f}~{B_r[m].max():.2f}")
