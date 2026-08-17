#!/usr/bin/env python3
"""장애물 조우별: 계획 d(otwpnts) vs 실제 d vs 지도 벽, 그리고 실제 벽 여유."""
import pickle
import sys
from pathlib import Path

import numpy as np

P = Path(__file__).parent
D = pickle.load(open(P / "data.pkl", "rb"))
S = np.load(P / "scanclear.npy")          # t sideL sideR fwdL fwdR rmin
gw = D["global_wpnts"][0][1]
gs, A_l, A_r = gw[:, 0], np.abs(gw[:, 5]), np.abs(gw[:, 4])
ds = gs[1] - gs[0]
L = gs[-1] + ds
fre = D["/car_state/frenet/odom"]         # t s d yaw v
cmd = D["/autonomy/ackermann_cmd"]        # t steer speed
lat = D["/pitwall/lat_err"]
esc = D["/planner/escape_active"]


def wall(s_, side):
    i = int(round((s_ % L) / ds)) % len(gw)
    return (A_l if side == "L" else A_r)[i]


def ipol(arr, t, c):
    return np.interp(t, arr[:, 0], arr[:, c]) if len(arr) else np.nan


# otwpnts: 각 시점 계획 경로의 (s, d) — apex(|d| 최대) 추출
ot = []
for t, w in D["otwpnts"]:
    if len(w) == 0:
        ot.append((t, np.nan, np.nan))
        continue
    k = np.abs(w[:, 1]).argmax()
    ot.append((t, w[k, 0], w[k, 1]))
ot = np.array(ot)

TARGETS = {"헤어핀2출구 s≈27.5": (26.5, 30.5), "직선 s≈33.0": (31.5, 35.0),
           "헤어핀1 s≈7.0": (6.0, 9.5)}

for name, (lo, hi) in TARGETS.items():
    print(f"\n{'='*100}\n### {name}   (지도 벽: "
          f"s={lo} L{wall(lo,'L'):.2f}/R{wall(lo,'R'):.2f} -> "
          f"s={hi} L{wall(hi,'L'):.2f}/R{wall(hi,'R'):.2f})\n{'='*100}")
    inz = (fre[:, 1] >= lo) & (fre[:, 1] <= hi)
    # 통과 구간 나누기
    idx = np.where(inz)[0]
    if not len(idx):
        continue
    splits = np.split(idx, np.where(np.diff(fre[idx, 0]) > 1.0)[0] + 1)
    for grp in splits:
        if len(grp) < 5:
            continue
        t0, t1 = fre[grp[0], 0], fre[grp[-1], 0]
        print(f"\n-- pass t={t0:.1f}~{t1:.1f}s --")
        print("   t      s     d_act  d_plan | mapR  mapL | scanR scanL | "
              "steer  v    lat_err esc")
        for i in grp[::3]:
            t, s_, d_, v = fre[i, 0], fre[i, 1], fre[i, 2], fre[i, 4]
            dp = np.interp(t, ot[:, 0], ot[:, 2]) if len(ot) else np.nan
            sr = np.interp(t, S[:, 0], S[:, 2])
            sl = np.interp(t, S[:, 0], S[:, 1])
            print(f"{t:7.2f} {s_:6.2f} {d_:+6.2f} {dp:+6.2f} | "
                  f"{wall(s_,'R'):5.2f} {wall(s_,'L'):5.2f} | "
                  f"{sr:5.2f} {sl:5.2f} | "
                  f"{np.degrees(ipol(cmd,t,1)):+6.1f} {v:4.2f} "
                  f"{ipol(lat,t,1):+6.2f} {ipol(esc,t,1):.0f}"
                  + ("   <<< 벽여유<0.20" if sr < 0.20 or sl < 0.20 else ""))
