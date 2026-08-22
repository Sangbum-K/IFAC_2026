#!/usr/bin/env python3
"""스캔 -> 프레네 (s,d) 투영으로 무더기 구간의 실제 점유 d-히스토그램.
usage: gap.py <t_query> <s_lo> <s_hi>"""
import pickle
import sys
import numpy as np

D = pickle.load(open("/home/miru/IFAC_ws/tools/bag_analysis_0822c/d26.pkl", "rb"))
TQ, SLO, SHI = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])

gw = D["global_wpnts"][0][1]
pf = D["/pf/pose/odom"]
scan = D["scan"]
scan_t = np.array([s[0] for s in scan])

wx, wy, ws = gw[:, 2], gw[:, 3], gw[:, 0]

def to_frenet(px, py):
    d2 = (wx[None, :] - px[:, None]) ** 2 + (wy[None, :] - py[:, None]) ** 2
    i = np.argmin(d2, axis=1)
    s = ws[i]
    # 부호: 진행방향 왼쪽 = +d  (psi 기준)
    psi = gw[i, 6]
    dx, dy = px - wx[i], py - wy[i]
    d = -np.sin(psi) * dx + np.cos(psi) * dy
    return s, d

i = np.clip(np.searchsorted(scan_t, TQ), 0, len(scan) - 1)
tt, amin, ainc, rr = scan[i]
j = np.clip(np.searchsorted(pf[:, 0], tt), 1, len(pf) - 1)
x0, y0, yaw = pf[j, 1], pf[j, 2], pf[j, 3]
ang = amin + np.arange(len(rr)) * ainc * 4 + yaw
m = np.isfinite(rr) & (rr > 0.05) & (rr < 8)
px = x0 + rr[m] * np.cos(ang[m])
py = y0 + rr[m] * np.sin(ang[m])
s, d = to_frenet(px, py)
sel = (s >= SLO) & (s <= SHI)
print(f"scan@{tt:.2f}  ego=({x0:.2f},{y0:.2f})  points in s[{SLO},{SHI}]: {sel.sum()}")
dd = np.sort(d[sel])
# d-히스토그램 (0.1m bin)
lo, hi = -1.6, 1.2
bins = np.arange(lo, hi + 0.05, 0.1)
h, _ = np.histogram(dd, bins=bins)
iw = np.argmin(np.abs(ws - (SLO + SHI) / 2))
print(f"track bounds here: d_right room={gw[iw,4]:.2f} d_left room={gw[iw,5]:.2f}")
for b, c in zip(bins[:-1], h):
    print(f"  d {b:+.1f}..{b+0.1:+.1f}: {'#'*min(c,60)}{c}")
# 갭 찾기: 점유 bin 사이 빈 구간
occ = h > 0
print("free d-intervals (bin 단위):", end=" ")
runs = []
start = None
for k, o in enumerate(occ):
    if not o and start is None:
        start = bins[k]
    if o and start is not None:
        runs.append((start, bins[k]))
        start = None
if start is not None:
    runs.append((start, bins[-1]))
for a, b in runs:
    print(f"[{a:+.1f},{b:+.1f}]({b-a:.1f}m)", end=" ")
print()
