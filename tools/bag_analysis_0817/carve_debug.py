#!/usr/bin/env python3
"""900005(150059) / 900016(145548) 프레임별 카빙 판정 디버그."""
import math
import pickle

import numpy as np

SLACK = 0.25
RANGE_MAX = 7.0
MIN_PASS = 1
LASER_FWD = 0.27
L = 38.7117295


def dbg(pkl, aid, t0, t1):
    D = pickle.load(open(pkl, 'rb'))
    g = D['global_wpnts'][0][1]

    def sd2xy(s, d):
        i = np.argmin(np.abs(g[:, 0] - s))
        psi = g[i, 6]
        return g[i, 2] - d * math.sin(psi), g[i, 3] + d * math.cos(psi)

    pf = D['/pf/pose/odom']
    fr = D['/car_state/frenet/odom']
    # 앵커 최신 주입 위치 타임라인
    anc = []
    for t, obs in D['obs_frenet']:
        for o in obs:
            if o['id'] == aid:
                anc.append((t, o['s_center'], o['d_center'], o['size'] / 2.0))
    anc = np.array(anc)
    streak = 0
    printed = 0
    for ts, amin, ainc, rr in D['scan']:
        if not (t0 <= ts <= t1):
            continue
        j = np.searchsorted(anc[:, 0], ts)
        j = min(max(j, 0), len(anc) - 1)
        _, s0, d0, half = anc[j]
        i = np.searchsorted(pf[:, 0], ts)
        i = min(max(i, 0), len(pf) - 1)
        ex, ey, eyaw = pf[i, 1], pf[i, 2], pf[i, 3]
        fi = np.searchsorted(fr[:, 0], ts)
        fi = min(max(fi, 0), len(fr) - 1)
        ego_s = fr[fi, 1]
        ahead = (s0 - ego_s) % L
        lx = ex + LASER_FWD * math.cos(eyaw)
        ly = ey + LASER_FWD * math.sin(eyaw)
        ax, ay = sd2xy(s0, d0)
        dx, dy = ax - lx, ay - ly
        r_a = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx) - eyaw
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        half_ang = math.atan2(max(half, 0.10), r_a)
        ainc4 = ainc * 4.0
        n = len(rr)
        i0 = int(math.ceil((bearing - half_ang - amin) / ainc4))
        i1 = int(math.floor((bearing + half_ang - amin) / ainc4))
        note = ''
        hits = passes = 0
        rs = []
        if not 0.3 <= r_a <= RANGE_MAX:
            note = 'RANGE'
        elif ahead < 0.5 or ahead > L / 2:
            note = 'AHEAD'
        elif i0 < 0 or i1 >= n or i1 < i0:
            note = 'FOV'
        else:
            for k in range(i0, i1 + 1):
                r = float(rr[k])
                if not math.isfinite(r):
                    passes += 1
                    rs.append(float('inf'))
                    continue
                rs.append(round(r, 2))
                if r < 0.02:
                    continue
                if r > r_a + SLACK:
                    passes += 1
                elif r >= r_a - SLACK:
                    hits += 1
            if hits > 0:
                streak = 0
            elif passes >= MIN_PASS:
                streak += 1
        if printed < 45:
            print(f'{ts:7.2f} ahead={ahead:5.2f} r_a={r_a:4.2f} '
                  f'brg={math.degrees(bearing):6.1f} half={half:.2f} '
                  f'win={max(0,i1-i0+1):2d} hit={hits:2d} pass={passes:2d} '
                  f'streak={streak:2d} {note} rs={rs[:12]}')
            printed += 1


print('=== 150059 / 900005 / 26.5~31.5 ===')
dbg('d150059.pkl', 900005, 26.5, 31.5)
print()
print('=== 145548 / 900016 / 144.0~146.0 ===')
dbg('d145548.pkl', 900016, 144.0, 146.0)
