#!/usr/bin/env python3
"""세 충돌 장면(맵 프레임): 맵 + 자차 경로 + 장애물 인지 + 스캔 스냅샷."""
import pickle
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

D = pickle.load(open("/home/miru/IFAC_ws/tools/bag_analysis_0822c/d26.pkl", "rb"))
M = D["map"]
pf = D["/pf/pose/odom"]
gw = D["global_wpnts"][0][1]
obs = D["obs_frenet"]
obs_t = np.array([t for t, _ in obs])
scan = D["scan"]
scan_t = np.array([s[0] for s in scan])

grid = M["data"].reshape(M["h"], M["w"])
extent = [M["ox"], M["ox"] + M["w"] * M["res"], M["oy"], M["oy"] + M["h"] * M["res"]]

def pose_at(t):
    i = np.clip(np.searchsorted(pf[:, 0], t), 1, len(pf) - 1)
    return pf[i, 1], pf[i, 2], pf[i, 3]

def scan_pts(t):
    i = np.clip(np.searchsorted(scan_t, t), 0, len(scan) - 1)
    tt, amin, ainc, rr = scan[i]
    x0, y0, yaw = pose_at(tt)
    ang = amin + np.arange(len(rr)) * ainc * 4 + yaw
    m = np.isfinite(rr) & (rr > 0.05) & (rr < 8)
    return x0 + rr[m] * np.cos(ang[m]), y0 + rr[m] * np.sin(ang[m])

CASES = [
    ("case1_t100  (1:40)", 99.5, 103.5, [100.5, 101.4, 102.0], (2.37, 0.61), 101.6),
    ("case2_t112  (1:52)", 111.0, 114.5, [111.7, 112.4, 113.0], (2.37, 0.61), 112.6),
    ("case3_t168  (2:48)", 166.5, 170.8, [167.7, 168.2, 169.0], (-2.77, 12.06), 168.6),
]

fig, axes = plt.subplots(1, 3, figsize=(21, 8))
for ax, (title, t0, t1, obs_times, pile, tscan) in zip(axes, CASES):
    ax.imshow(grid, cmap="gray_r", origin="lower", extent=extent, vmin=-1, vmax=100, alpha=0.6)
    ax.plot(gw[:, 2], gw[:, 3], "-", color="tab:blue", lw=1, alpha=0.5, label="raceline")
    m = (pf[:, 0] >= t0) & (pf[:, 0] <= t1)
    seg = pf[m]
    sc = ax.scatter(seg[:, 1], seg[:, 2], c=seg[:, 0], cmap="viridis", s=14, label="ego path")
    plt.colorbar(sc, ax=ax, shrink=0.6, label="t [s]")
    # 속도 주석 (0.5s마다)
    for tq in np.arange(t0, t1, 0.5):
        i = np.clip(np.searchsorted(pf[:, 0], tq), 0, len(pf) - 1)
        ax.annotate(f"{pf[i,4]:.1f}", (pf[i, 1], pf[i, 2]), fontsize=7, color="darkgreen")
    # 장애물 (각 시점 최근접 프레임)
    colors = ["red", "orange", "magenta"]
    for c, ot in zip(colors, obs_times):
        j = np.clip(np.searchsorted(obs_t, ot), 0, len(obs) - 1)
        _, ol = obs[j]
        for o in ol:
            dx, dy = np.hypot(o["x"] - pile[0], o["y"] - pile[1]), 0
            if dx < 2.0:
                ax.plot(o["x"], o["y"], "s", color=c, ms=8, mfc="none", mew=2)
                ax.annotate(f"{o['id']}@{ot:.1f}", (o["x"], o["y"]), fontsize=7, color=c)
    # 스캔 스냅샷
    sx, sy = scan_pts(tscan)
    ax.plot(sx, sy, ".", color="black", ms=2, label=f"scan@{tscan}")
    ax.plot(*pile, "*", color="purple", ms=16, label="pile")
    ax.set_title(title)
    ax.set_xlim(pile[0] - 6, pile[0] + 6)
    ax.set_ylim(pile[1] - 6, pile[1] + 6)
    ax.set_aspect("equal")
    ax.legend(fontsize=7, loc="lower right")
plt.tight_layout()
plt.savefig("/home/miru/IFAC_ws/tools/bag_analysis_0822c/scenes.png", dpi=110)
print("saved scenes.png")
