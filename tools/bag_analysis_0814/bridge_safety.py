#!/usr/bin/env python3
"""브리지 재연결의 안전성 점검: 흡수 때문에 **비어 버린 자리**가 있는가.

id 비교는 의미가 없다 — 흡수는 대부분 개명(rename)이라 id 는 사라져도 물체는
그대로 발행된다. 그래서 프레임(header stamp)을 맞춘 뒤 **위치**로 비교한다:
off 의 장애물마다 on 에서 가장 가까운 장애물까지의 거리를 재고, 그 거리가
콘 반지름(0.15)+여유를 넘으면 "덮이지 않은 물체"로 센다.

사용: bridge_safety.py <off 디렉터리> <on 디렉터리>
"""
import pathlib
import sqlite3
import sys

import numpy as np

sys.path.insert(0, '/home/sangbum/IFAC_ws/tools')
from bagviz import CDR                                    # noqa: E402

TRACK_LEN = 39.4487


def read(bag):
    p = sorted(pathlib.Path(bag).glob('*.db3'))[0]
    con = sqlite3.connect(str(p))
    tid = con.execute(
        "select id from topics where name='/perception/obstacles_frenet'"
    ).fetchone()[0]
    out = {}
    for _, data in con.execute(
            "select timestamp, data from messages where topic_id=? "
            "order by timestamp", (tid,)):
        c = CDR(data)
        ht, _ = c.header()
        n = c.u32()
        obs = []
        for _ in range(n):
            oid = c.i32()
            c.f64n(3)
            v = c.f64n(14)
            c.u8()
            c.u8()
            c.i32()
            obs.append((oid, v[0], v[1]))
        out[round(ht, 3)] = obs
    con.close()
    return out


off_dir, on_dir = (sys.argv[1:3] + ['bridge_off', 'bridge_on'])[:2]
A, B = read(off_dir), read(on_dir)
common = sorted(set(A) & set(B))
gaps = []
for t in common:
    for oid, s, d in A[t]:
        if not B[t]:
            gaps.append((9.9, oid, t, s, d))
            continue
        dd = min(np.hypot(
            abs((s - s2 + TRACK_LEN / 2) % TRACK_LEN - TRACK_LEN / 2), d - d2)
            for _, s2, d2 in B[t])
        gaps.append((dd, oid, t, s, d))

g = np.array([x[0] for x in gaps])
print("%s vs %s — 공통 프레임 %d, off 장애물 %d"
      % (off_dir, on_dir, len(common), len(g)))
for th in (0.20, 0.30, 0.35, 0.50, 1.00):
    print("   on 최근접까지 > %.2f m : %4d (%.2f%%)"
          % (th, (g > th).sum(), 100 * (g > th).mean()))
bad = sorted([x for x in gaps if x[0] > 0.35], key=lambda x: -x[0])
print("   최대 공백 %.2f m" % (g.max() if len(g) else 0.0))
for b in bad[:5]:
    print("     %.2f m  id=%d t=%.2f s=%.2f d=%+.2f" % b)
