#!/usr/bin/env python3
"""윈도우별 상세: 장애물(frenet), otwpnts, 이벤트, 리젝트 카운터, 명령속도."""
import pickle
import sys
import numpy as np

D = pickle.load(open("/home/miru/IFAC_ws/tools/bag_analysis_0822c/d26.pkl", "rb"))
W = float(sys.argv[1]) if len(sys.argv) > 1 else 100.0
w0, w1 = W - 10, W + 4

fo = D["/car_state/frenet/odom"]
acmd = D["/autonomy/ackermann_cmd"]

def at(arr, t):
    if len(arr) == 0:
        return None
    i = np.searchsorted(arr[:, 0], t)
    i = min(max(i, 0), len(arr) - 1)
    return arr[i]

print(f"===== window {w0:.0f}..{w1:.0f}s =====")

print("\n--- pitwall events ---")
for t, s in D["events"]:
    if w0 <= t <= w1:
        print(f"  {t:7.2f}  {s}")

print("\n--- rosout (planner/control/perception, WARN+) ---")
for t, lvl, name, msg in D["rosout"]:
    if w0 <= t <= w1 and lvl >= 30:
        print(f"  {t:7.2f} [{name}] {msg[:150]}")

print("\n--- reject counters active in window (value increments) ---")
for k in sorted(D.keys()):
    if not isinstance(k, str) or not k.startswith("/pitwall/"):
        continue
    arr = D[k]
    if not isinstance(arr, np.ndarray) or arr.ndim != 2 or len(arr) == 0:
        continue
    m = (arr[:, 0] >= w0) & (arr[:, 0] <= w1)
    seg = arr[m]
    if len(seg) == 0:
        continue
    if seg[:, 1].max() > 0:
        # print nonzero stretches compactly
        nz = seg[seg[:, 1] > 0]
        print(f"  {k}: nonzero {len(nz)}/{len(seg)}  t[{nz[0,0]:.2f}..{nz[-1,0]:.2f}] maxval={nz[:,1].max():.3f}")

print("\n--- ego + cmd + obstacles ahead (0.2s step) ---")
obs = D["obs_frenet"]
obs_t = np.array([t for t, _ in obs])
TRACK_L = 34.44  # wrap
for tq in np.arange(w0, w1, 0.2):
    e = at(fo, tq)
    c = at(acmd, tq)
    i = np.searchsorted(obs_t, tq)
    i = min(max(i, 0), len(obs) - 1)
    _, ol = obs[i]
    s_ego, d_ego, v_ego = e[1], e[2], e[4]
    ahead = []
    for o in ol:
        ds = (o["s_start"] - s_ego) % TRACK_L
        if ds < 12 or ds > TRACK_L - 1:
            ahead.append((ds, o))
    ahead.sort(key=lambda x: x[0])
    obs_str = " | ".join(
        f"id{o['id']}{'S' if o['is_static'] else 'D'}{'v' if o['is_visible'] else '.'} ds={ds:4.1f} d[{o['d_right']:+.2f},{o['d_left']:+.2f}]"
        for ds, o in ahead[:4])
    print(f"t={tq:7.1f} s={s_ego:5.1f} d={d_ego:+5.2f} v={v_ego:4.1f} cmd_v={c[2] if c is not None else -1:4.1f} | {obs_str}")
