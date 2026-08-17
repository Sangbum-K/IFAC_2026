#!/usr/bin/env python3
"""s≈7.0 콘이 막히는 건의 진단 (run_0814_191443).

네 가지를 한 번에 낸다:
  (1) 시선 차단 — 좌(안쪽) 경계가 언제까지 콘을 가리는가
  (2) 실제 최초 관측 거리 (obs_frenet 에 그 id 가 처음 뜬 시각 기준)
  (3) 회랑 예산 — 좌/우 여유 vs 필요 간격 (그 백의 설정: BT_GAIN 0.0)
  (4) 같은 콘의 id 재등록(코앞 출생) 과 그 결과
"""
import re

import numpy as np

import sideclears_ab as AB

D = AB.D
AB.BT_GAIN = 0.0        # 0814 백의 실제 설정 (apex_port_check.py 로 확인)

S_LO, S_HI = 6.2, 7.4   # 대상 콘 구간
gb = D['global_wpnts'][-1][-1]
X, Y, PSI = gb[:, 2], gb[:, 3], gb[:, 6]
NX, NY = -np.sin(PSI), np.cos(PSI)
LB = np.stack([X + NX * np.abs(gb[:, 5]), Y + NY * np.abs(gb[:, 5])], 1)
RB = np.stack([X - NX * np.abs(gb[:, 4]), Y - NY * np.abs(gb[:, 4])], 1)
fr = D['/car_state/frenet/odom']        # [t, s, d, 0, v]


def pos(s, d):
    i = int(AB.mod(s) / AB.STEP) % len(gb)
    return np.array([X[i] + NX[i] * d, Y[i] + NY[i] * d])


def _hit(p, q, a, b):
    d1, d2 = q - p, b - a
    den = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(den) < 1e-12:
        return False
    t = ((a[0] - p[0]) * d2[1] - (a[1] - p[1]) * d2[0]) / den
    u = ((a[0] - p[0]) * d1[1] - (a[1] - p[1]) * d1[0]) / den
    return 1e-6 < t < 1 - 1e-6 and 1e-6 < u < 1 - 1e-6


def occluded(p, q):
    n = sum(_hit(p, q, LB[i], LB[(i + 1) % len(LB)]) for i in range(len(LB)))
    n += sum(_hit(p, q, RB[i], RB[(i + 1) % len(RB)]) for i in range(len(RB)))
    return n


print("(1) 시선 — 콘(s=6.74, d=-0.61) 을 자차 위치별로 볼 수 있는가")
cone = pos(6.74, -0.61)
open_s = None
for s in np.arange(-8.0, 6.1, 0.5):
    n = occluded(pos(s, -0.10), cone)
    if n == 0 and open_s is None:
        open_s = s
    if -6.0 <= s <= 4.0 and abs(s * 2 - round(s * 2)) < 1e-9 and s % 1 == 0:
        print("    자차 s=%6.2f  거리 %5.2f m  경계 교차 %d  %s"
              % (AB.mod(s), 6.74 - s, n, "가림" if n else "보임"))
print("    -> 시선이 열리는 자차 위치 s=%.2f (콘까지 %.2f m)"
      % (AB.mod(open_s), 6.74 - open_s))

print("\n(2) 실제 최초 관측 거리와 회랑 예산")
print("    콘id     t   s중심  콘 d[우,좌]   벽[좌,우]  좌여유 우여유 필요  최초관측거리")
first = {}
for t, obs in D['obs_frenet']:
    for o in obs:
        if S_LO <= o['s_center'] <= S_HI and o['id'] not in first:
            ego = float(np.interp(t, fr[:, 0], fr[:, 1]))
            first[o['id']] = (t, (o['s_center'] - ego) % AB.TRACK_LEN, o)
gaps, right_blocked, close_born = [], 0, 0
for oid, (t, gap, o) in sorted(first.items(), key=lambda x: x[1][0]):
    if gap > 15:
        continue
    v = max(abs(float(np.interp(t, fr[:, 0], fr[:, 4]))), AB.PLAN_SPEED_FLOOR)
    wl, wr = AB.min_wall(AB.mod(o['s_start'] - 0.5), AB.mod(o['s_end'] + 0.5))
    need = AB.need_clear() + AB.boundary_margin(v)
    lg, rg = wl - o['d_left'], wr + o['d_right']
    gaps.append(gap)
    right_blocked += rg <= need
    close_born += gap < 2.0
    print("    %4d %6.1f  %.2f  [%+.2f,%+.2f]  [%.2f,%.2f]  %.2f%s %.2f%s  %.2f  %.2f m"
          % (oid, t, o['s_center'], o['d_right'], o['d_left'], wl, wr,
             lg, "o" if lg > need else "x", rg, "o" if rg > need else "x",
             need, gap))
g = np.array(gaps)
print("    -> 최초 관측 거리 p10 %.1f / p50 %.1f / p90 %.1f m (lookahead 12.0 m)"
      % (np.percentile(g, 10), np.median(g), np.percentile(g, 90)))
print("    -> 우측 통과 불가 %d / %d, 2m 안에서 태어난 id %d"
      % (right_blocked, len(g), close_born))

print("\n(3) 같은 콘의 id 재등록 (코앞 출생) — 예: t=324.0")
k = int(np.searchsorted([o[0] for o in D['obs_frenet']], 324.0) - 1)
ego = float(np.interp(324.0, fr[:, 0], fr[:, 1]))
for o in sorted(D['obs_frenet'][k][1],
                key=lambda o: (o['s_center'] - ego) % AB.TRACK_LEN):
    if (o['s_center'] - ego) % AB.TRACK_LEN < 3:
        print("    id %4d  +%.2fm  s[%.2f,%.2f] d[%+.2f,%+.2f] size %.2f vis=%d"
              % (o['id'], (o['s_center'] - ego) % AB.TRACK_LEN, o['s_start'],
                 o['s_end'], o['d_right'], o['d_left'], o['size'],
                 o['is_visible']))
seen, born_near, twin = set(), 0, 0
for t, obs in D['obs_frenet']:
    ego = float(np.interp(t, fr[:, 0], fr[:, 1]))
    for o in obs:
        if o['id'] in seen:
            continue
        seen.add(o['id'])
        if (o['s_center'] - ego) % AB.TRACK_LEN < 2.0:
            born_near += 1
            if any(x['id'] != o['id'] and abs(x['d_center'] - o['d_center']) < 0.20
                   and abs(((x['s_center'] - o['s_center'] + AB.TRACK_LEN / 2)
                            % AB.TRACK_LEN) - AB.TRACK_LEN / 2) < 0.6
                   for x in obs):
                twin += 1
print("    -> 전체 새 id %d 중 차 앞 2m 안에서 태어난 것 %d (%.0f%%), 그중 쌍둥이 동반 %d"
      % (len(seen), born_near, 100 * born_near / len(seen), twin))
c = D['/pitwall/reject_commit_released_new_obs']
print("    -> commit_released_new_obs 누적 %.0f회" % (c[-1, 1] - c[0, 1]))

print("\n(4) 통과 결과 — 콘이 closest 인 프레임의 인카운터별 속도")
TR = re.compile(r'\[trace\] s=([-\d.]+) v=([-\d.]+) d=([-\d.]+) \| obs id=(-?\d+)')
obs_t = np.array([o[0] for o in D['obs_frenet']])
obs_l = [o[1] for o in D['obs_frenet']]
fs = []
for t, lvl, nm, m in D['rosout']:
    if nm != 'spliner_node' or not m.startswith('[trace]'):
        continue
    g2 = TR.match(m)
    if not g2:
        continue
    k = int(np.searchsorted(obs_t, t) - 1)
    if k < 0:
        continue
    o = next((x for x in obs_l[k] if x['id'] == int(g2.group(4))), None)
    if o is None or not (S_LO <= o['s_center'] <= S_HI):
        continue
    fs.append((t, float(g2.group(2)), m.split('|')[-2].strip()))
eps, cur = [], [fs[0]]
for a, b in zip(fs, fs[1:]):
    if b[0] - a[0] > 2.0:
        eps.append(cur)
        cur = []
    cur.append(b)
eps.append(cur)
for e in eps:
    v = [r[1] for r in e]
    tr = sum(1 for r in e if r[2] == 'trailing')
    print("    t %6.1f  n=%3d  v %.2f -> 최저 %.2f  trailing %d 프레임"
          % (e[0][0], len(e), max(v), min(v), tr))
