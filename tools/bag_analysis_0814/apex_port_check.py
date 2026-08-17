#!/usr/bin/env python3
"""sideclears_ab.py 의 ApexFor 이식이 실제 노드와 같은 값을 내는지 검증.

"LaneSwitch <side> 레인 불가: d_hold=.. (필요 .., 장애물 d[..,..])" 로그는
노드가 계산한 d_hold(=ApexFor)와 need 를 그대로 찍는다. 같은 시각의
obs_frenet 에서 그 장애물을 찾아 이식 함수로 다시 계산해 비교한다.
"""
import pickle
import re

import numpy as np

import sideclears_ab as AB   # 이식 함수 재사용 (파라미터·트랙도 그대로)

D = AB.D
LINE = re.compile(
    r'LaneSwitch (left|right) 레인 불가: d_hold=([-\d.]+) \(필요 ([-\d.]+), '
    r'장애물 d\[([-\d.]+),([-\d.]+)\]\)')

obs_t = np.array([o[0] for o in D['obs_frenet']])
obs_list = [o[1] for o in D['obs_frenet']]
speed = D['/car_state/frenet/odom'][:, [0, 4]]  # [t, v] — col3 은 0

err = []
miss = 0
need_err = []
for t, lvl, nm, msg in D['rosout']:
    if nm != 'spliner_node':
        continue
    m = LINE.search(msg)
    if not m:
        continue
    # 로그 인자 순서는 (d_right, d_left) — 소스 확인 완료.
    side, d_hold, need, dr, dl = (m.group(1), float(m.group(2)),
                                  float(m.group(3)), float(m.group(4)),
                                  float(m.group(5)))
    k = int(np.searchsorted(obs_t, t) - 1)
    if k < 0:
        miss += 1
        continue
    # 같은 시각 장애물 중 로그의 d[d_right,d_left] 와 가장 가까운 것.
    cand = sorted(obs_list[k],
                  key=lambda o: abs(o['d_left'] - dl) + abs(o['d_right'] - dr))
    if not cand or abs(cand[0]['d_left'] - dl) > 0.02 \
            or abs(cand[0]['d_right'] - dr) > 0.02:
        miss += 1
        continue
    obs = cand[0]
    v = float(np.interp(t, speed[:, 0], speed[:, 1])) if len(speed) else 0.0
    vp = max(abs(v), AB.PLAN_SPEED_FLOOR)
    wl, wr = AB.min_wall(AB.mod(obs['s_start'] - 0.5),
                         AB.mod(obs['s_end'] + 0.5))
    apex = AB.apex_for(side, obs, wl, wr, vp)
    err.append(apex - d_hold)
    need_ref = (dl + AB.need_clear()) if side == 'left' else (dr - AB.need_clear())
    need_err.append(need_ref - need)

err = np.array(err)
need_err = np.array(need_err)
print("검증 표본 %d (매칭 실패 %d)" % (len(err), miss))
print("need 오차:  max |Δ| = %.4f m" % np.abs(need_err).max())
print("d_hold 오차: med %.4f, p90 %.4f, max |Δ| %.4f m"
      % (np.median(err), np.percentile(np.abs(err), 90), np.abs(err).max()))
print("  |Δ| <= 1cm: %.0f%%   <= 3cm: %.0f%%"
      % (100 * (np.abs(err) <= 0.01).mean(), 100 * (np.abs(err) <= 0.03).mean()))
