#!/usr/bin/env python3
"""replay_ab.sh 로 뽑은 open-loop 재생 로그들을 비교한다.

프레임 단위 비교는 (자차 s, 장애물 id) 로 짝을 짓는다 — 재생마다 사이클
타이밍이 조금씩 달라 프레임 수가 정확히 같지는 않기 때문이다.
"""
import re
import sys
from collections import Counter, defaultdict

TR = re.compile(
    r'\[trace\] s=([-\d.]+) v=([-\d.]+) d=([-\d.]+) \| obs id=(-?\d+) '
    r'dist=([-\d.]+) d\[([-\d.]+),([-\d.]+)\] static=(-?\d+) nobs=(-?\d+) \| '
    r'evade=(\d+) sector=(\d+) commit=(\d+) \| (\S+) \| (\S+)')


def load(path):
    rows = []
    for line in open(path, errors='replace'):
        m = TR.search(line)
        if not m:
            continue
        rows.append(dict(s=float(m.group(1)), v=float(m.group(2)),
                         d=float(m.group(3)), oid=int(m.group(4)),
                         dist=float(m.group(5)), mode=m.group(13),
                         tags=set(m.group(14).split(','))))
    return rows


def summary(name, rows):
    md = Counter(r['mode'] for r in rows)
    tg = Counter()
    for r in rows:
        tg.update(r['tags'])
    print("%-9s 프레임 %5d | trailing %4d (%4.1f%%) lane_switch %3d "
          "committed_hold %4d | 봉쇄 %3d 양보 %3d 양쪽막힘 %3d ok_lane %3d"
          % (name, len(rows), md['trailing'], 100 * md['trailing'] / max(len(rows), 1),
             md['lane_switch'], md['committed_hold'],
             tg['lane_side_switch_blocked_mid_evasion'],
             tg['lane_side_switch_yielded'], tg['side_both_blocked'],
             tg['ok_lane_switch']))
    return md, tg


def paired(a_rows, b_rows, na, nb):
    """(s 0.05m 격자, 장애물 id) 로 짝지어 모드가 달라진 프레임을 센다."""
    def key(r):
        return (round(r['s'] / 0.05), r['oid'])
    A, B = defaultdict(list), defaultdict(list)
    for r in a_rows:
        A[key(r)].append(r)
    for r in b_rows:
        B[key(r)].append(r)
    common = set(A) & set(B)
    same = diff = 0
    diffs = Counter()
    for k in common:
        ma, mb = A[k][0]['mode'], B[k][0]['mode']
        if ma == mb:
            same += 1
        else:
            diff += 1
            diffs[(ma, mb)] += 1
    print("  %s vs %s: 짝지은 지점 %d, 모드 동일 %d, 다름 %d (%.2f%%)"
          % (na, nb, len(common), same, diff, 100 * diff / max(len(common), 1)))
    for (ma, mb), c in diffs.most_common(6):
        print("      %-15s -> %-15s %4d" % (ma, mb, c))
    return diff / max(len(common), 1)


logs = sys.argv[1:] or ['replay_base_r1.log', 'replay_new_r1.log',
                        'replay_new2_r1.log']
data = {}
for p in logs:
    name = p.replace('replay_', '').replace('.log', '')
    try:
        data[name] = load(p)
    except FileNotFoundError:
        print("없음: %s" % p)
for name, rows in data.items():
    summary(name, rows)

print("\n짝 비교 (같은 자차 위치·같은 장애물에서 결정이 바뀌었나)")
names = list(data)
if 'base_r1' in data and 'new_r1' in data:
    signal = paired(data['base_r1'], data['new_r1'], 'base_r1', 'new_r1')
if 'new_r1' in data and 'new2_r1' in data:
    noise = paired(data['new_r1'], data['new2_r1'], 'new_r1', 'new2_r1')
    if 'base_r1' in data:
        print("\n  신호(수정) %.2f%%  vs  노이즈(같은 코드 재실행) %.2f%%"
              % (100 * signal, 100 * noise))
