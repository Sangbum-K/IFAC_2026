#!/usr/bin/env python3
"""side_clears() 벽 구간 A/B — hold 창(장애물 앞뒤 0.5m) vs 접근 구간 전체.

봉쇄(lane_side_switch_blocked_mid_evasion) 프레임마다 잠긴 사이드의
side_clears() 를 두 가지 벽 구간으로 다시 계산해, 접근 구간을 쓰면 잠금이
양보되는(=반대편 후보가 열리는) 프레임이 몇인지 센다.

가정: 0814 백은 확정-정적 중심 누적(1c96844) 이전이라 UncertaintyMargin 이
항상 캡(max_uncertainty_m 0.12)에 붙어 있었다 — 추출기가 *_var 를 버려서
직접 못 읽으므로 캡으로 고정한다. need 는 두 분기에 똑같이 들어가므로
A/B 판정(플립 여부)에는 이 값이 아니라 벽 구간만 작용한다.
"""
import pickle
import re

import numpy as np

D = pickle.load(open('/home/sangbum/IFAC_ws/tools/bag_analysis_0814/data.pkl', 'rb'))

# ── 실차 파라미터 (params.yaml + environments/real_car.yaml) ─────────────
HALF_W = 0.15
PATH_CHECK_MARGIN = 0.15
MAX_UNC = 0.12
EVASION_DIST = 0.35
APEX_EPS = 0.05
BOUND_MINDIST = 0.20
# ⚠️ boundary_tracking_speed_gain 은 c376c12(08-15) 에서 0.0 -> 0.08 로 올랐다.
# 0814 백은 0.0 으로 굴렀다 — apex_port_check.py 가 gain 0.0 에서만 로그
# d_hold 와 100% 일치(58/58, 전부 벽 클램프가 결정한 표본)한다.
BT_M, BT_REF, BT_GAIN, BT_MAX = 0.10, 3.5, 0.00, 0.22
PLAN_SPEED_FLOOR = 1.0

gb = D['global_wpnts'][-1][-1]          # [n,10] s d x y d_r d_l psi kappa vx ax
GS = gb[:, 0]
D_RIGHT = np.abs(gb[:, 4])
D_LEFT = np.abs(gb[:, 5])
STEP = float(np.median(np.diff(GS)))
N = len(GS)
TRACK_LEN = float(GS[-1] + STEP)


def mod(x):
    return x % TRACK_LEN


def min_wall(s_from, s_to):
    """MinWallInSpan 이식."""
    span = min(mod(s_to - s_from), TRACK_LEN - STEP)
    steps = max(1, int(span / STEP))
    i0 = int(mod(s_from) / STEP) % N
    idx = (i0 + np.arange(steps + 1)) % N
    return float(D_LEFT[idx].min()), float(D_RIGHT[idx].min())


def boundary_margin(v):
    reserve = min(BT_M + BT_GAIN * max(v - BT_REF, 0.0), BT_MAX)
    return max(BOUND_MINDIST, HALF_W + reserve)


def need_clear():
    return max(PATH_CHECK_MARGIN, HALF_W) + MAX_UNC


def apex_for(side, obs, wall_l, wall_r, v):
    """ApexFor 이식 — 벽 값만 인자로 받는다."""
    bm = boundary_margin(v)
    need = need_clear() + APEX_EPS
    dist_min = need
    dist_target = max(EVASION_DIST, need)
    if side == 'left':
        corridor = abs(wall_l - obs['d_left'])
        surplus = max(corridor - dist_min - bm, 0.0)
        balanced = obs['d_left'] + dist_min + surplus / 2.0
        target = obs['d_left'] + dist_target
        return min(max(min(target, balanced), 0.0), max(wall_l - bm, 0.0))
    corridor = abs(wall_r + obs['d_right'])
    surplus = max(corridor - dist_min - bm, 0.0)
    balanced = obs['d_right'] - dist_min - surplus / 2.0
    target = obs['d_right'] - dist_target
    return max(min(max(target, balanced), 0.0), -max(wall_r - bm, 0.0))


def clears(side, apex, obs):
    nc = need_clear()
    return (apex >= obs['d_left'] + nc) if side == 'left' \
        else (apex <= obs['d_right'] - nc)


def main(gain):
    global BT_GAIN
    BT_GAIN = gain
    print("\n===== boundary_tracking_speed_gain = %.2f %s ====="
          % (gain, "(0814 실제 설정)" if gain == 0.0 else "(현재 real_car.yaml)"))
    # ── 봉쇄 프레임 수집 ────────────────────────────────────────────────────
    TRACE = re.compile(
        r'\[trace\] s=([-\d.]+) v=([-\d.]+) d=([-\d.]+) \| obs id=(-?\d+) '
        r'dist=([-\d.]+)')
    frames = []
    for t, lvl, name, msg in D['rosout']:
        if name != 'spliner_node' or not msg.startswith('[trace]'):
            continue
        tags = msg.split('|')[-1].strip().split(',')
        if 'lane_side_switch_blocked_mid_evasion' not in tags:
            continue
        m = TRACE.match(msg)
        if not m:
            continue
        frames.append((t, float(m.group(1)), float(m.group(2)),
                       float(m.group(3)), int(m.group(4))))

    obs_t = np.array([o[0] for o in D['obs_frenet']])
    obs_list = [o[1] for o in D['obs_frenet']]
    # PlanSpeed() 는 cur_vs_ (frenet 종속도). 추출기의 frenet 배열은
    # [t, s, d, 0, v] 이고 **col3 은 전부 0**이다 — 속도는 col4.
    fr = D['/car_state/frenet/odom']

    stat = dict(total=0, no_obs=0, hold_blocked=0, flip=0, same=0,
                opp_ok=0, opp_blocked=0)
    flips = []
    for t, s, v, d, oid in frames:
        stat['total'] += 1
        k = int(np.searchsorted(obs_t, t) - 1)
        if k < 0:
            stat['no_obs'] += 1
            continue
        obs = next((o for o in obs_list[k] if o['id'] == oid), None)
        if obs is None:
            stat['no_obs'] += 1
            continue
        side = 'left' if d > 0 else 'right'
        vp = max(abs(float(np.interp(t, fr[:, 0], fr[:, 4]))),
                 PLAN_SPEED_FLOOR)

        wl_h, wr_h = min_wall(mod(obs['s_start'] - 0.5), mod(obs['s_end'] + 0.5))
        wl_a, wr_a = min_wall(mod(s), mod(obs['s_end'] + 0.5))

        a_hold = apex_for(side, obs, wl_h, wr_h, vp)
        a_appr = apex_for(side, obs, wl_a, wr_a, vp)
        c_hold = clears(side, a_hold, obs)
        c_appr = clears(side, a_appr, obs)

        if not c_hold:
            # 재구성이 이미 "막힘"으로 본 프레임 (그룹 판정/예측 차이).
            stat['hold_blocked'] += 1
            continue
        if c_appr:
            stat['same'] += 1
            continue
        stat['flip'] += 1
        # 양보했을 때 반대편이 실제로 성립하는가 (현행 hold 창 기준 = 루프의 판정)
        opp = 'right' if side == 'left' else 'left'
        a_opp = apex_for(opp, obs, wl_h, wr_h, vp)
        if clears(opp, a_opp, obs):
            stat['opp_ok'] += 1
        else:
            stat['opp_blocked'] += 1
        flips.append((t, s, d, side, a_hold, a_appr, wl_h if side == 'left' else wr_h,
                      wl_a if side == 'left' else wr_a, obs['d_left'], obs['d_right']))

    print("봉쇄 프레임 %d" % stat['total'])
    print("  장애물 매칭 실패        %4d" % stat['no_obs'])
    print("  재구성상 이미 막힘      %4d  (그룹 apex/예측 차이 — A/B 대상 아님)"
          % stat['hold_blocked'])
    print("  hold=통과 & 접근=통과   %4d  (변화 없음)" % stat['same'])
    print("  hold=통과 & 접근=막힘   %4d  <- 잠금 양보로 바뀌는 프레임" % stat['flip'])
    print("      그중 반대편 성립    %4d / 반대편도 막힘 %d"
          % (stat['opp_ok'], stat['opp_blocked']))

    if flips:
        print("\n대표 프레임 (t, ego_s, ego_d, 잠긴사이드, apex_hold, apex_접근,"
              " 벽_hold, 벽_접근, 장애물 d[l,r])")
        for row in flips[:12]:
            print("  %7.2f s=%5.2f d=%+.2f %-5s  %+.2f -> %+.2f   %.2f -> %.2f"
                  "   [%+.2f,%+.2f]" % row)

        # 봉쇄 구간 단위로 환산
        ft = np.array([f[0] for f in flips])
        print("\n플립 프레임의 시간 분포: %.1f ~ %.1f s, 서로 다른 구간 %d개"
              % (ft.min(), ft.max(), 1 + int((np.diff(ft) > 0.5).sum())))


if __name__ == '__main__':
    main(0.00)
    main(0.08)
