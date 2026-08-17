#!/usr/bin/env python3
"""특정 시각의 씬 스냅샷: 스캔(프레네), 지도 벽, 인지 장애물, 계획 경로."""
import pickle
import sys
from pathlib import Path

import numpy as np

P = Path(__file__).parent
D = pickle.load(open(P / "data.pkl", "rb"))
gw = D["global_wpnts"][0][1]
gs, gx, gy, psi = gw[:, 0], gw[:, 2], gw[:, 3], gw[:, 6]
A_l, A_r = np.abs(gw[:, 5]), np.abs(gw[:, 4])
ds = gs[1] - gs[0]
mp = D["map"]
G, res, ox, oy = mp["data"], mp["res"], mp["ox"], mp["oy"]
pf = D["/pf/pose/odom"]
fre = D["/car_state/frenet/odom"]
LASER_X = 0.28


def frenet(wx, wy):
    d2 = (wx[:, None] - gx[None, :]) ** 2 + (wy[:, None] - gy[None, :]) ** 2
    k = d2.argmin(axis=1)
    dx, dy = wx - gx[k], wy - gy[k]
    sd = -np.sin(psi[k]) * dx + np.cos(psi[k]) * dy
    return gs[k], sd


def raycast(x0, y0, ang, maxd=3.5, step=0.02):
    d = 0.0
    while d < maxd:
        d += step
        j = int((x0 + d * np.cos(ang) - ox) / res)
        i = int((y0 + d * np.sin(ang) - oy) / res)
        if i < 0 or j < 0 or i >= G.shape[0] or j >= G.shape[1] or G[i, j] != 0:
            return d
    return maxd


def snap(T):
    k = min(range(len(D["scan"])), key=lambda i: abs(D["scan"][i][0] - T))
    t, amin, ainc, r = D["scan"][k]
    px = np.interp(t, pf[:, 0], pf[:, 1])
    py = np.interp(t, pf[:, 0], pf[:, 2])
    yaw = np.interp(t, pf[:, 0], np.unwrap(pf[:, 3]))
    ok = r > 0.05
    ang = amin + ainc * np.arange(len(r))
    lx, ly = r * np.cos(ang) + LASER_X, r * np.sin(ang)
    wx = px + lx * np.cos(yaw) - ly * np.sin(yaw)
    wy = py + lx * np.sin(yaw) + ly * np.cos(yaw)
    ss, dd = frenet(wx[ok], wy[ok])
    cs = np.interp(t, fre[:, 0], fre[:, 1])
    cd = np.interp(t, fre[:, 0], fre[:, 2])
    print(f"\n### t={t:.2f}  car s={cs:.2f} d={cd:+.2f}  pose=({px:.2f},{py:.2f}) "
          f"yaw={np.degrees(yaw):.0f}deg")
    print("  s_bin | 스캔점 d 범위(우..좌)                   | 지도벽 R/L | 웨이R/L")
    for sb in np.arange(round(cs) - 2, round(cs) + 4, 0.5):
        m = (ss >= sb) & (ss < sb + 0.5)
        i = int(round((sb % (gs[-1] + ds)) / ds)) % len(gw)
        mr = raycast(gx[i], gy[i], psi[i] - np.pi / 2)
        ml = raycast(gx[i], gy[i], psi[i] + np.pi / 2)
        if m.sum() < 3:
            hist = "(점 없음)"
        else:
            v = np.sort(dd[m])
            # 0.15m 간격으로 군집
            grp, cur = [], [v[0]]
            for a in v[1:]:
                if a - cur[-1] > 0.2:
                    grp.append(cur)
                    cur = [a]
                else:
                    cur.append(a)
            grp.append(cur)
            hist = "  ".join(f"[{g[0]:+.2f}..{g[-1]:+.2f}]x{len(g)}" for g in grp)
        print(f"  {sb:5.1f} | {hist:55s} | {-mr:+5.2f}/{ml:+5.2f} | "
              f"{-A_r[i]:+5.2f}/{A_l[i]:+5.2f}")
    # 인지 장애물
    ko = min(range(len(D["obs_frenet"])), key=lambda i: abs(D["obs_frenet"][i][0] - T))
    to, obs = D["obs_frenet"][ko]
    print(f"  인지 장애물 @t={to:.2f}: " + (", ".join(
        f"#{o['id']} s[{o['s_start']:.2f},{o['s_end']:.2f}] d[{o['d_right']:+.2f},"
        f"{o['d_left']:+.2f}] {'static' if o['is_static'] else 'dyn'}"
        f"{'' if o['is_visible'] else ' (invis)'}" for o in obs) or "없음"))


for T in [float(a) for a in sys.argv[1:]]:
    snap(T)
