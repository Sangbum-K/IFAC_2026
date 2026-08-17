#!/usr/bin/env python3
"""③b(트레일링 램프 충돌검사 사정거리 절단)의 fail-open 검증.

0817 리뷰에서 남은 유일한 실차 전 확인 항목이다. 변경은
BuildTrailing 의 램프 충돌검사에서 차단점이 **이 기동의 사정거리**

    maneuver_reach = Mod(target.s_end - cur_s) + trailing_gap_m

밖이면 제동 곡선을 미루고 다음 사이클에 맡긴다. 보호를 없애는 게 아니라
미루는 것이라는 논거인데, 0811 coasting fail-open 이 정확히 같은 논거로
틀렸던 사례이므로 근거 없이 넘길 수 없다.

미루는 것이 안전하려면 이 조건이 성립해야 한다:

    **미룬 차단점이 처음 사정거리 안에 들어오는 순간, 그 시점의 차속에서
    제동으로 세울 수 있는 거리가 남아 있어야 한다.**

각 장애물마다 그 순간을 찾아 (남은거리, 차속)을 재고, 트레일링 제동 모델

    d_req = v*trailing_lag_sec + v^2 / (2*trailing_decel)

과 비교한다. 여유가 음수인 장애물이 하나라도 있으면 절단은 실차에 올릴 수
없다는 뜻이다.

사용법:  python3 reach_defer.py [<bag.db3>]
"""
import glob
import sqlite3
import sys
from pathlib import Path

sys.path.insert(0, "/home/miru/IFAC_ws/tools")
import numpy as np  # noqa: E402
from bagviz import CDR  # noqa: E402

# planning_pkg/config/params.yaml (spliner_node)
TRAILING_GAP_M = 1.5
TRAILING_DECEL = 1.0
TRAILING_LAG_SEC = 0.3
LOOKAHEAD_M = 10.0

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
         _s_var, _d_var, _vs_var, _vd_var, size, _theta) = c.f64n(14)
        is_static = c.u8() != 0
        is_visible = c.u8() != 0
        c.i32()
        out.append(dict(id=oid, s_center=s_c, d_center=d_c, s_start=s_s,
                        s_end=s_e, d_right=d_r, d_left=d_l, vs=vs, vd=vd,
                        size=size, is_static=is_static, is_visible=is_visible))
    return out


def dec_odom_sd(b):
    c = CDR(b)
    c.header()
    c.string()          # child_frame_id
    px, py, _pz = c.f64n(3)
    c.f64n(4)           # orientation
    c.f64n(36)          # pose covariance
    vx, vy, _vz = c.f64n(3)
    return px, py, vx, vy


def read(bag):
    path = Path(bag)
    if path.is_dir():
        cands = sorted(glob.glob(str(path / "*.db3")))
        if not cands:
            raise SystemExit(f"db3 를 찾을 수 없다: {bag}")
        path = Path(cands[0])
    con = sqlite3.connect(str(path))
    topics = {name: (tid, tp) for tid, name, tp in con.execute(
        "select id,name,type from topics")}

    def rows(name):
        tid = topics[name][0]
        return con.execute(
            "select timestamp,data from messages where topic_id=? "
            "order by timestamp", (tid,)).fetchall()

    obs = [(t * 1e-9, dec_frenet_obs(d))
           for t, d in rows("/perception/obstacles_frenet")]
    odom = [(t * 1e-9,) + dec_odom_sd(d)
            for t, d in rows("/car_state/frenet/odom")]
    # 트랙 길이는 global_waypoints 의 마지막 s + 간격으로 잡는다.
    # Wpnt = int32 id + float64 x10 (s_m, d_m, x_m, y_m, d_right, d_left,
    # psi_rad, kappa_radpm, vx_mps, ax_mps2).
    gw = rows("/global_waypoints")[-1][1]
    c = CDR(gw)
    c.header()
    n = c.u32()
    s_all = []
    for _ in range(n):
        c.i32()
        vals = c.f64n(10)
        s_all.append(vals[0])
    step = (s_all[-1] - s_all[0]) / max(len(s_all) - 1, 1)
    return obs, odom, s_all[-1] + step


def main():
    bag = sys.argv[1] if len(sys.argv) > 1 else BAG_DEFAULT
    obs_msgs, odom, track_len = read(bag)
    print(f"bag={bag}")
    print(f"track_len={track_len:.2f} m  "
          f"obstacles msgs={len(obs_msgs)}  odom={len(odom)}")

    o_t = np.array([r[0] for r in odom])
    o_s = np.array([r[1] for r in odom])
    o_vs = np.array([r[3] for r in odom])

    def mod(x):
        return x % track_len

    def ego_at(t):
        i = int(np.searchsorted(o_t, t))
        i = min(max(i, 0), len(o_t) - 1)
        return o_s[i], o_vs[i]

    # 장애물별로 "처음 사정거리 안에 들어온 순간"을 찾는다.
    # 사정거리는 그 사이클의 **트레일링 대상**(전방 최근접 장애물) 기준이다.
    first_in_reach = {}   # id -> (t, remaining_dist, v)
    deferred_cycles = 0
    total_multi_cycles = 0

    for t, obs in obs_msgs:
        cur_s, v = ego_at(t)
        ahead = []
        for o in obs:
            d = mod(o["s_start"] - cur_s)
            if 0.0 <= d <= LOOKAHEAD_M:
                ahead.append((d, o))
        if len(ahead) < 2:
            continue
        total_multi_cycles += 1
        ahead.sort(key=lambda p: p[0])
        target = ahead[0][1]
        reach = mod(target["s_end"] - cur_s) + TRAILING_GAP_M

        saw_defer = False
        for d, o in ahead[1:]:
            in_reach = d <= reach
            if in_reach:
                if o["id"] not in first_in_reach:
                    first_in_reach[o["id"]] = (t, d, v)
            else:
                saw_defer = True
        if saw_defer:
            deferred_cycles += 1

    print(f"\n장애물 2개 이상 전방 사이클: {total_multi_cycles}")
    print(f"그중 사정거리 밖 차단점이 있던 사이클(= 절단이 발동): "
          f"{deferred_cycles} "
          f"({100.0 * deferred_cycles / max(total_multi_cycles, 1):.1f}%)")

    print("\n미룬 차단점이 처음 사정거리 안에 들어온 순간의 제동 여유")
    print("  id       t[s]   남은거리[m]  차속[m/s]  필요제동[m]   여유[m]")
    worst = None
    for oid, (t, d, v) in sorted(first_in_reach.items(),
                                 key=lambda kv: kv[1][0]):
        d_req = v * TRAILING_LAG_SEC + v * v / (2.0 * TRAILING_DECEL)
        margin = d - d_req
        flag = "  <-- 부족" if margin < 0 else ""
        print(f"  {oid:6d}  {t - o_t[0]:7.2f}  {d:9.2f}  {v:9.2f}  "
              f"{d_req:10.2f}  {margin:8.2f}{flag}")
        if worst is None or margin < worst[1]:
            worst = (oid, margin, v, d)

    if worst is None:
        print("  (해당 사례 없음)")
        return 0
    print(f"\n최악: id={worst[0]} 여유 {worst[1]:.2f} m "
          f"(차속 {worst[2]:.2f} m/s, 남은거리 {worst[3]:.2f} m)")
    if worst[1] < 0.0:
        print("판정: 절단이 제동 여유를 먹는다 — 실차 금지, 사정거리를 늘려야 한다.")
        return 1
    print("판정: 미룬 차단점이 전부 제동 가능한 거리에서 다시 잡힌다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
