#!/usr/bin/env python3
"""A안(브리지 트랙 재연결)의 이득 상한 측정.

새로 태어난 id 를 세 갈래로 나눈다:
  hand-off : 바로 직전까지 같은 자리에 있던 트랙이 이번 프레임에 없다
             -> 같은 물체의 재등록. id·통계를 승계할 수 있다.
  duplicate: 그 이웃이 이번 프레임에도 **함께** 살아 있다
             -> 한 콘이 둘로 쪼개진 것. 승계로는 못 고친다(병합 문제).
  new      : 이웃이 없다. 진짜 새 물체.

승계했을 때 물려받는 관측수(n)가 static_mean_min_obs(30) 이상이면, 그
프레임부터 바로 누적 평균 중심이 발행된다 = 1c96844 가 코앞에서도 듣는다.
"""
import numpy as np

import sideclears_ab as AB

D = AB.D
FRAMES = D['obs_frenet']
fr = D['/car_state/frenet/odom']

GATE_S = 0.60      # 재연결 후보 s 게이트 [m]
GATE_D = 0.25      # 같은 d [m]
LOOKBACK = 1.0     # 직전 몇 초까지 사라진 트랙을 후보로 볼 것인가 [s]
MIN_OBS = 30       # static_mean_min_obs

times = np.array([f[0] for f in FRAMES])
lists = [f[1] for f in FRAMES]

# id 별 관측 이력
hist = {}
for k, (t, obs) in enumerate(FRAMES):
    for o in obs:
        h = hist.setdefault(o['id'], {'first_k': k, 'n': 0, 'static': 0,
                                      'last_k': k, 's': o['s_center'],
                                      'd': o['d_center']})
        h['n'] += 1
        h['static'] += bool(o['is_static'])
        h['last_k'] = k

kinds = {'hand-off': [], 'duplicate': [], 'new': []}
for k, (t, obs) in enumerate(FRAMES):
    ego = float(np.interp(t, fr[:, 0], fr[:, 1]))
    live = {o['id'] for o in obs}
    for o in obs:
        h = hist[o['id']]
        if h['first_k'] != k:
            continue
        gap = (o['s_center'] - ego) % AB.TRACK_LEN
        # 같은 자리의 이웃 찾기 — 이번 프레임(중복) 먼저, 없으면 직전 프레임들
        dup = [x for x in obs if x['id'] != o['id']
               and abs(AB.mod(x['s_center'] - o['s_center'] + AB.TRACK_LEN / 2)
                       - AB.TRACK_LEN / 2) < GATE_S
               and abs(x['d_center'] - o['d_center']) < GATE_D]
        if dup:
            kinds['duplicate'].append((t, o['id'], gap, dup[0]['id'],
                                       hist[dup[0]['id']]['n']))
            continue
        pred = None
        for j in range(k - 1, -1, -1):
            if t - times[j] > LOOKBACK:
                break
            for x in lists[j]:
                if x['id'] in live or x['id'] == o['id']:
                    continue
                if abs(AB.mod(x['s_center'] - o['s_center'] + AB.TRACK_LEN / 2)
                       - AB.TRACK_LEN / 2) < GATE_S \
                        and abs(x['d_center'] - o['d_center']) < GATE_D:
                    pred = x
                    break
            if pred:
                break
        if pred:
            ph = hist[pred['id']]
            kinds['hand-off'].append((t, o['id'], gap, pred['id'], ph['n'],
                                      ph['static'] / max(ph['n'], 1)))
        else:
            kinds['new'].append((t, o['id'], gap, -1, 0))

tot = sum(len(v) for v in kinds.values())
print("새 id 총 %d" % tot)
for name in ('hand-off', 'duplicate', 'new'):
    rows = kinds[name]
    near = [r for r in rows if r[2] < 2.0]
    print("  %-9s %4d (%4.1f%%)   그중 차 앞 2m 이내 %3d"
          % (name, len(rows), 100 * len(rows) / tot, len(near)))

ho = kinds['hand-off']
if ho:
    n_inherit = np.array([r[4] for r in ho])
    st = np.array([r[5] for r in ho])
    near = np.array([r[2] for r in ho]) < 2.0
    print("\nhand-off 승계 시 물려받는 관측수: p10 %d p50 %d p90 %d"
          % (np.percentile(n_inherit, 10), np.median(n_inherit),
             np.percentile(n_inherit, 90)))
    print("  n >= %d (바로 누적 평균 발행): %d / %d (%.0f%%)"
          % (MIN_OBS, (n_inherit >= MIN_OBS).sum(), len(ho),
             100 * (n_inherit >= MIN_OBS).mean()))
    print("  선행 트랙이 90%% 이상 정적으로 분류돼 있던 비율: %.0f%%"
          % (100 * (st >= 0.9).mean()))
    print("  차 앞 2m 이내 hand-off: %d 건 (여기가 커밋 파기가 나던 자리)"
          % near.sum())
    print("\n  예시 (t, 새id, 거리, 선행id, 선행 관측수):")
    for r in [r for r in ho if r[2] < 2.0][:10]:
        print("    %6.1f  %4d  +%.2fm  <- %4d (n=%d)" % r[:5])
