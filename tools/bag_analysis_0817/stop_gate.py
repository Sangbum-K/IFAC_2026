#!/usr/bin/env python3
"""②(정지 권한에 관측수 게이트) 정량화.

spliner 의 allow_stop 은 `!closest.is_static` 이었다. ClassifyObstacles 는
관측이 cls_min_nb_meas(5) 를 넘기 전에는 재분류를 건너뛰고 perception 플래그를
그대로 통과시키므로, 갓 뜬 1프레임 검출이 is_static=false 로 내려와 정지 권한을
온전히 받았다. run_0817_123650 좌벽 충돌의 시작점이다 (id=8 nobs=0).

게이트를 걸면 얼마나 달라지는가를 백으로 센다:

  · 트레일링 대상(전방 최근접)이 동적으로 내려온 사이클 수
  · 그중 관측수가 문턱 이하라 **정지 권한이 유보되는** 사이클 수
  · 유보되는 트랙들의 수명 분포 — 유령이면 짧고 실물이면 길어야 한다
    (짧은 쪽만 걸리면 게이트가 유령만 겨냥한다는 근거가 된다)

사용법:  python3 stop_gate.py [<bag.db3>] [<문턱>]
"""
import glob
import sqlite3
import sys
from collections import Counter, defaultdict
from pathlib import Path

sys.path.insert(0, "/home/miru/IFAC_ws/tools")
import numpy as np  # noqa: E402
from bagviz import CDR  # noqa: E402

CLS_MIN_NB_MEAS = 5      # spliner/cls_min_nb_meas
LOOKAHEAD_M = 10.0       # spliner/lookahead_m
BAG_DEFAULT = "/home/miru/Downloads/run_0817_123650"


def dec_frenet_obs(b):
    c = CDR(b)
    c.header()
    n = c.u32()
    out = []
    for _ in range(n):
        oid = c.i32()
        c.f64n(3)
        (s_c, d_c, s_s, s_e, d_r, d_l, vs, vd,
         _sv, _dv, _vsv, _vdv, size, _th) = c.f64n(14)
        is_static = c.u8() != 0
        is_visible = c.u8() != 0
        c.i32()
        out.append(dict(id=oid, s_center=s_c, d_center=d_c, s_start=s_s,
                        s_end=s_e, d_right=d_r, d_left=d_l, vs=vs, vd=vd,
                        size=size, is_static=is_static,
                        is_visible=is_visible))
    return out


def dec_odom_s(b):
    c = CDR(b)
    c.header()
    c.string()
    px, _py, _pz = c.f64n(3)
    c.f64n(4)
    c.f64n(36)
    vx, _vy, _vz = c.f64n(3)
    return px, vx


def read(bag):
    path = Path(bag)
    if path.is_dir():
        cands = sorted(glob.glob(str(path / "*.db3")))
        if not cands:
            raise SystemExit(f"db3 를 찾을 수 없다: {bag}")
        path = Path(cands[0])
    con = sqlite3.connect(str(path))
    topics = {name: tid for tid, name in con.execute(
        "select id,name from topics")}

    def rows(name):
        return con.execute(
            "select timestamp,data from messages where topic_id=? "
            "order by timestamp", (topics[name],)).fetchall()

    obs = [(t * 1e-9, dec_frenet_obs(d))
           for t, d in rows("/perception/obstacles_frenet")]
    odom = [(t * 1e-9,) + dec_odom_s(d)
            for t, d in rows("/car_state/frenet/odom")]
    gw = rows("/global_waypoints")[-1][1]
    c = CDR(gw)
    c.header()
    n = c.u32()
    s_all = []
    for _ in range(n):
        c.i32()
        s_all.append(c.f64n(10)[0])
    step = (s_all[-1] - s_all[0]) / max(len(s_all) - 1, 1)
    return obs, odom, s_all[-1] + step


def main():
    bag = sys.argv[1] if len(sys.argv) > 1 else BAG_DEFAULT
    thresh = int(sys.argv[2]) if len(sys.argv) > 2 else CLS_MIN_NB_MEAS
    obs_msgs, odom, track_len = read(bag)

    o_t = np.array([r[0] for r in odom])
    o_s = np.array([r[1] for r in odom])

    def ego_s(t):
        i = min(max(int(np.searchsorted(o_t, t)), 0), len(o_t) - 1)
        return o_s[i]

    lifetime = Counter()          # id -> 총 관측 프레임 수
    for _t, obs in obs_msgs:
        for o in obs:
            lifetime[o["id"]] += 1

    seen = defaultdict(int)       # id -> 지금까지 본 프레임 수 (인과 순서)
    n_dyn_target = 0
    n_gated = 0
    gated_ids = set()
    dyn_target_ids = set()

    for t, obs in obs_msgs:
        cur_s = ego_s(t)
        for o in obs:
            seen[o["id"]] += 1
        ahead = [(( o["s_start"] - cur_s) % track_len, o) for o in obs]
        ahead = [(d, o) for d, o in ahead if 0.0 <= d <= LOOKAHEAD_M]
        if not ahead:
            continue
        ahead.sort(key=lambda p: p[0])
        target = ahead[0][1]
        if target["is_static"]:
            continue
        n_dyn_target += 1
        dyn_target_ids.add(target["id"])
        # 코드와 같은 비교 방향: ObsSeen(id) > cls_min_nb_meas_
        if seen[target["id"]] <= thresh:
            n_gated += 1
            gated_ids.add(target["id"])

    print(f"bag={bag}  track_len={track_len:.2f} m  문턱={thresh}")
    print(f"\n트레일링 대상이 동적으로 내려온 사이클: {n_dyn_target}"
          f"  (고유 트랙 {len(dyn_target_ids)}개)")
    print(f"그중 정지 권한이 유보되는 사이클: {n_gated}"
          f" ({100.0 * n_gated / max(n_dyn_target, 1):.1f}%)"
          f"  (고유 트랙 {len(gated_ids)}개)")

    def dist(ids, label):
        if not ids:
            print(f"\n{label}: 없음")
            return
        lens = sorted(lifetime[i] for i in ids)
        arr = np.array(lens)
        print(f"\n{label} 트랙의 총 관측 프레임 수 (n={len(lens)})")
        print(f"  중앙 {np.median(arr):.0f}  평균 {arr.mean():.1f}  "
              f"최소 {arr.min()}  최대 {arr.max()}")
        print(f"  <=10 프레임: {int((arr <= 10).sum())}개 "
              f"({100.0 * (arr <= 10).mean():.0f}%)")

    dist(gated_ids, "유보된")
    dist(dyn_target_ids - gated_ids, "유보되지 않은")

    print("\n해석: 유보된 쪽이 짧은 수명에 몰려 있으면 게이트가 유령만 겨냥한다는"
          "\n      뜻이다. 유보되지 않은 쪽에 긴 수명이 남아 있어야 실물 상대의"
          "\n      정지 권한이 살아 있다는 뜻이다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
