#!/usr/bin/env python3
"""스캔 빈자리 카빙 오프라인 재생 검증 (2026-08-17 수정안).

perception_obstacle_bridge_node.cb_scan 의 판정 로직을 bag 데이터에 그대로
재생한다. 노드와 다른 점 두 가지만 명시한다:
  · bag 스캔은 빔 stride 4 데시메이션 -> 창 내 빔 수가 1/4. min_pass 를
    ceil(3/4)=1 로 낮춰 재생한다 (민감한 쪽으로 보수적 -> 오탐이 있으면
    여기서 더 잘 드러난다).
  · TF 대신 /pf/pose/odom + laser 전방 오프셋 0.27 m.

판정 기준 (노드와 동일):
  hit    |r - r_a| <= slack(0.25)  -> 스트릭 리셋
  pass   r > r_a + slack 또는 무반사/개방 -> 빈자리 증거
  block  r < r_a - slack          -> 보류
  hit==0 and passes>=min_pass -> streak+=1;  streak>=15 -> 카빙
"""
import math
import pickle
import sys

import numpy as np

SLACK = 0.25
RANGE_MAX = 7.0
# 근거리 오탐(옆 지나치는 순간)은 RANGE_MIN 이 아니라 전방 게이트(ahead>=0.5)
# 가 맡는다. RANGE_MIN 1.0 실험은 진짜 유령 카빙(900005 r 0.88~1.45 /
# 900016 r 0.44~0.84)까지 죽였다 — 유령은 벽이 slack 안에 걸리는 원거리에선
# 증명이 안 되고, 근거리에서만 관통이 열린다. 노드 하한 0.3 과 동일하게 둔다.
RANGE_MIN = 0.3
CLEAR_FRAMES = 15
MIN_PASS = 1          # 노드 3 / stride4 보정
LASER_FWD = 0.27
MEM_BASE = 900000


def load(name):
    return pickle.load(open(name, 'rb'))


def make_sd2xy(D):
    g = D['global_wpnts'][0][1]      # s d x y d_r d_l psi kappa vx ax

    def sd2xy(s, d):
        i = np.argmin(np.abs(g[:, 0] - s))
        psi = g[i, 6]
        return g[i, 2] - d * math.sin(psi), g[i, 3] + d * math.cos(psi)
    return sd2xy


def replay(name, checks):
    D = load(name)
    sd2xy = make_sd2xy(D)
    pf = D['/pf/pose/odom']
    pf_t = pf[:, 0]
    fr = D['/car_state/frenet/odom']         # 노드의 ego_s (전방 게이트)
    fr_t = fr[:, 0]
    L = 38.7117295

    # mem id 별 최신 주입 위치 (랩 경계 갱신 반영)
    anchor_pos = {}          # id -> (s, d, half)
    obs_frames = D['obs_frenet']
    obs_i = 0

    streak = {}
    carved = {}              # id -> [(t_carve, r_a), ...]
    carve_state = {}
    live_near = {}           # id -> 실측이 앵커 밴드 안에 있던 시각들

    for ts, amin, ainc, rr in D['scan']:
        # obs_frenet 을 스캔 시각까지 소비
        while obs_i < len(obs_frames) and obs_frames[obs_i][0] <= ts:
            _t, obs = obs_frames[obs_i]
            for o in obs:
                if o['id'] >= MEM_BASE:
                    anchor_pos[o['id']] = (
                        o['s_center'], o['d_center'], o['size'] / 2.0)
                else:
                    # 실측이 어떤 앵커 밴드(0.6/0.35) 안인가 (오탐 판정용)
                    for aid, (s0, d0, _h) in anchor_pos.items():
                        L = 38.7117295
                        ds = abs(((o['s_center'] - s0 + L / 2) % L) - L / 2)
                        if ds < 0.6 and abs(o['d_center'] - d0) < 0.35:
                            live_near.setdefault(aid, []).append(_t)
            obs_i += 1
        if not anchor_pos:
            continue
        i = np.searchsorted(pf_t, ts)
        i = min(max(i, 0), len(pf) - 1)
        ex, ey, eyaw = pf[i, 1], pf[i, 2], pf[i, 3]
        lx = ex + LASER_FWD * math.cos(eyaw)
        ly = ey + LASER_FWD * math.sin(eyaw)
        ainc4 = ainc * 4.0            # extract.py 의 빔 stride
        n = len(rr)
        fi = np.searchsorted(fr_t, ts)
        fi = min(max(fi, 0), len(fr) - 1)
        ego_s = fr[fi, 1]
        for aid, (s0, d0, half) in anchor_pos.items():
            # 노드와 동일한 전방 게이트 (오탐 근거는 노드 cb_scan 주석)
            ahead = (s0 - ego_s) % L
            if ahead < 0.5 or ahead > L / 2.0:
                continue
            ax, ay = sd2xy(s0, d0)
            dx, dy = ax - lx, ay - ly
            r_a = math.hypot(dx, dy)
            if not RANGE_MIN <= r_a <= RANGE_MAX:
                continue
            bearing = math.atan2(dy, dx) - eyaw
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            half_ang = math.atan2(max(half, 0.10), r_a)
            i0 = int(math.ceil((bearing - half_ang - amin) / ainc4))
            i1 = int(math.floor((bearing + half_ang - amin) / ainc4))
            if i0 < 0 or i1 >= n or i1 < i0:
                continue
            hits = passes = 0
            for k in range(i0, i1 + 1):
                r = float(rr[k])
                if not math.isfinite(r):
                    passes += 1
                    continue
                if r < 0.02:
                    continue
                if r > r_a + SLACK:
                    passes += 1
                elif r >= r_a - SLACK:
                    hits += 1
            if hits > 0:
                streak[aid] = 0
                if carve_state.get(aid):
                    carve_state[aid] = False   # 재점유 관찰 (노드는 실측 매칭만 부활)
                continue
            if passes < MIN_PASS:
                continue
            streak[aid] = streak.get(aid, 0) + 1
            if streak[aid] >= CLEAR_FRAMES and not carve_state.get(aid):
                carve_state[aid] = True
                carved.setdefault(aid, []).append((ts, r_a))

    print(f'━━ {name} ━━')
    for aid in sorted(anchor_pos):
        ev = carved.get(aid, [])
        ln = ', '.join(f'{t:.2f}s(r={r:.2f})' for t, r in ev[:6])
        near = live_near.get(aid, [])
        near_s = f' 실측매칭 {len(near)}프레임({near[0]:.1f}~{near[-1]:.1f}s)' \
            if near else ' 실측매칭 없음'
        print(f'  {aid}: 카빙 {len(ev)}회 [{ln}]{near_s}')
    for label, aid, cond in checks:
        ev = carved.get(aid, [])
        ok = cond(ev)
        print(f'  {"PASS" if ok else "FAIL"}  {label}')
    print()


replay('d150059.pkl', [
    ('900005(유령): 정지 시작(29.8s) 전에 카빙', 900005,
     lambda ev: any(t < 29.8 for t, _ in ev)),
    ('900006(유령): 정지 시작 전에 카빙', 900006,
     lambda ev: any(t < 29.8 for t, _ in ev)),
    ('900001(실물 벽콘): 자기 통과(18.7~21.9s) 전에 카빙 안 됨', 900001,
     lambda ev: not any(t < 18.7 for t, _ in ev)),
    ('900000(실물): 랩0 통과(4.7~6.5s) 중 카빙 안 됨', 900000,
     lambda ev: not any(t < 6.5 for t, _ in ev)),
])

replay('d145548.pkl', [
    # 900016(충돌 유령)은 전방게이트 때문에 카빙으로는 못 지운다 — 관통
    # 증거가 전부 ahead<0.5 구간에만 열렸다. 이 유령의 방어선은 중심 매칭
    # 게이트다: 아래 별도 체크가 "중심 매칭 0건 -> 3 m 근거리 게이트가 주입
    # 차단"을 확인한다.
    ('900011(실물 s=3.5): 확정 랩 통과 중 카빙 안 됨', 900011,
     lambda ev: not any(121.0 < t < 135.0 for t, _ in ev)),
    ('900012(실물 s=3.76): 확정 랩 통과 중 카빙 안 됨', 900012,
     lambda ev: not any(121.0 < t < 135.0 for t, _ in ev)),
])

# ── 145548 충돌 유령(900016)의 방어선: 중심 매칭 게이트 ──────────────────
D = load('d145548.pkl')
L = 38.7117295
central = 0
edge = 0
for t, obs in D['obs_frenet']:
    if not (134.2 < t < 149.4):          # 랩 8 (충돌 랩)
        continue
    for o in obs:
        if o['id'] >= MEM_BASE or not o['is_visible']:
            continue
        ds = abs(((o['s_center'] - 35.08 + L / 2) % L) - L / 2)
        dd = abs(o['d_center'] - 0.12)
        if ds <= 0.60 and dd <= 0.35:
            c = (ds / 0.6) ** 2 + (dd / 0.35) ** 2
            if c <= 0.5:
                central += 1
            else:
                edge += 1
ok = central == 0
print(f'  {"PASS" if ok else "FAIL"}  900016(충돌 유령): 랩8 중심 매칭 0건 '
      f'(중심 {central} / 가장자리 {edge}) -> matched_this_pass 미라치 '
      f'-> 3 m 근거리 게이트가 주입 차단')
