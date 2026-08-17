#!/usr/bin/env python3
"""브리지 재생 A/B — bridge/reassoc_enabled off vs on.

두 재생은 같은 입력(/perception/obstacles)을 먹었으므로 출력 메시지 개수가
같아야 하고, 차이는 전부 재연결/흡수에서 온다.

지표
  새 id 출생        : 프레임 간 id 집합 차이 (적을수록 좋다)
  동시 중복쌍       : 같은 프레임에 0.30 m 안에 있는 두 장애물
  누적평균 발행비율 : s_var <= 1e-3 인 장애물 비율
                      (= Welford 평균이 실제로 발행된 프레임. 1c96844 가
                       듣고 있는지를 직접 보는 값)
"""
import sys

sys.path.insert(0, '/home/sangbum/IFAC_ws/tools')
import pathlib
import sqlite3

import numpy as np
from bagviz import CDR

TRACK_LEN = 39.4487


def dec(b):
    c = CDR(b)
    c.header()
    n = c.u32()
    out = []
    for _ in range(n):
        oid = c.i32()
        c.f64n(3)
        (s_c, d_c, s_s, s_e, d_r, d_l, vs, vd,
         s_var, d_var, vs_var, vd_var, size, theta) = c.f64n(14)
        is_static = c.u8() != 0
        c.u8()
        c.i32()
        out.append((oid, s_c, d_c, s_var, size, is_static))
    return out


def read(bagdir):
    p = sorted(pathlib.Path(bagdir).glob('*.db3'))[0]
    con = sqlite3.connect(str(p))
    tid = con.execute(
        "select id from topics where name='/perception/obstacles_frenet'"
    ).fetchone()[0]
    rows = con.execute(
        "select timestamp, data from messages where topic_id=? order by timestamp",
        (tid,)).fetchall()
    con.close()
    return [dec(d) for _, d in rows]


def stats(name, frames):
    seen = set()
    births = 0
    dup_frames = 0
    n_obs = 0
    n_mean = 0
    for obs in frames:
        ids = {o[0] for o in obs}
        births += len(ids - seen)
        seen |= ids
        for i, a in enumerate(obs):
            n_obs += 1
            if a[3] <= 1e-3:
                n_mean += 1
            for b in obs[i + 1:]:
                ds = abs((a[1] - b[1] + TRACK_LEN / 2) % TRACK_LEN - TRACK_LEN / 2)
                if ds <= 0.30 and abs(a[2] - b[2]) <= 0.25:
                    dup_frames += 1
                    break
    print("%-4s 프레임 %5d | 장애물 %6d | 새 id %4d | 동시중복 %4d | "
          "누적평균 발행 %5.1f%%"
          % (name, len(frames), n_obs, births, dup_frames,
             100 * n_mean / max(n_obs, 1)))
    return dict(frames=len(frames), obs=n_obs, births=births,
                dup=dup_frames, mean=n_mean)


off = stats('off', read('bridge_off'))
on = stats('on', read('bridge_on'))
print()
print("새 id 출생      %d -> %d  (%+.0f%%)"
      % (off['births'], on['births'],
         100 * (on['births'] - off['births']) / max(off['births'], 1)))
print("동시 중복쌍     %d -> %d  (%+.0f%%)"
      % (off['dup'], on['dup'],
         100 * (on['dup'] - off['dup']) / max(off['dup'], 1)))
print("누적평균 발행   %.1f%% -> %.1f%%"
      % (100 * off['mean'] / max(off['obs'], 1),
         100 * on['mean'] / max(on['obs'], 1)))
print("발행 장애물 수  %d -> %d  (%+.1f%%)"
      % (off['obs'], on['obs'],
         100 * (on['obs'] - off['obs']) / max(off['obs'], 1)))
