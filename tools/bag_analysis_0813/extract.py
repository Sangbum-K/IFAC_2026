#!/usr/bin/env python3
"""run_0812_013630 분석용 추출기. bagviz의 CDR 디코더를 재사용."""
import math
import pathlib
import pickle
import sqlite3
import sys
from pathlib import Path

sys.path.insert(0, "/home/sangbum/IFAC_ws/tools")
import numpy as np
from bagviz import (CDR, dec_ackermann, dec_bool, dec_float32, dec_float64,
                    dec_log, dec_odom, dec_string, quat_yaw, read_vesc_calib)

BAG = "/home/sangbum/IFAC_ws/rosbag_0813/run_0813_165312/run_0813_165312_0.db3"
OUT = Path(__file__).parent / "data.pkl"


def dec_frenet_obs(b):
    """planning_msgs/ObstacleArray -> list of dict (프레네 필드 전체)."""
    c = CDR(b)
    ht, frame = c.header()
    n = c.u32()
    out = []
    for _ in range(n):
        oid = c.i32()
        px, py, _pz = c.f64n(3)
        (s_c, d_c, s_s, s_e, d_r, d_l, vs, vd,
         s_var, d_var, vs_var, vd_var, size, theta) = c.f64n(14)
        is_static = c.u8() != 0
        is_visible = c.u8() != 0
        lap = c.i32()
        out.append(dict(id=oid, x=px, y=py, s_center=s_c, d_center=d_c,
                        s_start=s_s, s_end=s_e, d_right=d_r, d_left=d_l,
                        vs=vs, vd=vd, size=size, theta=theta,
                        is_static=is_static, is_visible=is_visible, lap=lap))
    return ht, frame, out


def dec_perc_obs(b):
    """perception_msgs/ObstacleArray (base_link/laser frame)."""
    c = CDR(b)
    ht, frame = c.header()
    n = c.u32()
    out = []
    for _ in range(n):
        oid = c.i32()
        px, py, _pz = c.f64n(3)
        vx, vy, _vz = c.f64n(3)
        dyn = c.u8() != 0
        conf = c.f32()
        radius = c.f32()
        vis = c.u8() != 0
        out.append(dict(id=oid, x=px, y=py, vx=vx, vy=vy, dyn=dyn,
                        conf=conf, radius=radius, vis=vis))
    return ht, frame, out


def dec_wpnts_full(b):
    """WpntArray -> (header_t, ndarray[n,10])  s d x y d_r d_l psi kappa vx ax"""
    c = CDR(b)
    ht, _f = c.header()
    n = c.u32()
    if n == 0:
        return ht, np.zeros((0, 10))
    arr = np.empty((n, 10))
    for i in range(n):
        c.i32()
        arr[i] = c.f64n(10)
    return ht, arr


def dec_vesc_state(b):
    """vesc_msgs/VescStateStamped: header + state{...}"""
    c = CDR(b)
    ht, _f = c.header()
    # VescState: float64 voltage_input, temperature_pcb, current_motor,
    # current_input, speed(erpm), duty_cycle, charge_drawn, charge_regen,
    # energy_drawn, energy_regen, displacement, distance_traveled,
    # int32 fault_code
    f = c.f64n(6)
    return ht, dict(v_in=f[0], temp=f[1], i_motor=f[2], i_in=f[3],
                    erpm=f[4], duty=f[5])


def dec_imu_stamped(b):
    """vesc_msgs/VescImuStamped: header + VescImu{ ... }"""
    c = CDR(b)
    ht, _f = c.header()
    return ht, None


def dec_imu_raw(b):
    """sensor_msgs/Imu -> yaw rate (wz), accel"""
    c = CDR(b)
    ht, _f = c.header()
    qx, qy, qz, qw = c.f64n(4)
    c.skip_f64(9)          # orientation_covariance
    wx, wy, wz = c.f64n(3)
    c.skip_f64(9)          # angular_velocity_covariance
    ax, ay, az = c.f64n(3)
    return ht, quat_yaw(qx, qy, qz, qw), wx, wy, wz, ax, ay, az


def dec_overlay_text(b):
    """rviz_2d_overlay_msgs/OverlayText -> text field."""
    c = CDR(b)
    # action(int32), width, height, horizontal_distance, vertical_distance(int32 x?)
    # 정확한 레이아웃 모름 -> text 만 필요하므로 문자열 스캔으로 대체
    return None


def main():
    db = sqlite3.connect(BAG)
    topics = {n: (tid, ty) for tid, n, ty in
              db.execute("SELECT id,name,type FROM topics")}
    t0_ns = db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]

    def rows(name):
        tid = topics[name][0]
        return db.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (tid,)).fetchall()

    D = {"t0_ns": t0_ns, "calib": read_vesc_calib()}

    def rel(ts):
        return (ts - t0_ns) / 1e9

    # ---- 스칼라 Float32/64 토픽 일괄 ----
    scalar32 = [n for n in topics if topics[n][1].endswith("Float32")]
    scalar64 = [n for n in topics if topics[n][1].endswith("Float64")]
    for n in scalar32:
        r = rows(n)
        D[n] = np.array([(rel(ts), dec_float32(d)) for ts, d in r]) if r else np.zeros((0, 2))
    for n in scalar64:
        r = rows(n)
        D[n] = np.array([(rel(ts), dec_float64(d)) for ts, d in r]) if r else np.zeros((0, 2))
    print(f"scalars: {len(scalar32)+len(scalar64)}", flush=True)

    # ---- Bool ----
    for n in ("/planner/escape_active", "/safety/emergency_stop",
              "/pf/localization_ready"):
        r = rows(n)
        D[n] = np.array([(rel(ts), 1.0 if dec_bool(d) else 0.0) for ts, d in r]) \
            if r else np.zeros((0, 2))

    # ---- Ackermann ----
    for n in ("/ackermann_cmd", "/autonomy/ackermann_cmd", "/teleop"):
        r = rows(n)
        out = []
        for ts, d in r:
            st, sp = dec_ackermann(d)
            out.append((rel(ts), st, sp))
        D[n] = np.array(out) if out else np.zeros((0, 3))
    print("ackermann done", flush=True)

    # ---- Odometry ----
    for n in ("/odom", "/pf/pose/odom", "/car_state/frenet/odom"):
        r = rows(n)
        out = []
        for ts, d in r:
            ht, x, y, yaw, v = dec_odom(d)
            out.append((rel(ts), x, y, yaw, v))
        D[n] = np.array(out) if out else np.zeros((0, 5))
    print("odom done", flush=True)

    # ---- IMU raw (yaw rate) ----
    r = rows("/sensors/imu/raw")
    out = []
    for ts, d in r:
        try:
            ht, yaw, wx, wy, wz, ax, ay, az = dec_imu_raw(d)
            out.append((rel(ts), yaw, wx, wy, wz, ax, ay, az))
        except Exception:
            pass
    D["/sensors/imu/raw"] = np.array(out) if out else np.zeros((0, 8))
    print(f"imu: {len(out)}", flush=True)

    # ---- VESC core (erpm, current) ----
    r = rows("/sensors/core")
    out = []
    for ts, d in r:
        try:
            ht, s = dec_vesc_state(d)
            out.append((rel(ts), s["v_in"], s["i_motor"], s["erpm"], s["duty"]))
        except Exception:
            pass
    D["/sensors/core"] = np.array(out) if out else np.zeros((0, 5))
    print(f"core: {len(out)}", flush=True)

    # ---- 장애물 ----
    r = rows("/perception/obstacles_frenet")
    D["obs_frenet"] = [(rel(ts), dec_frenet_obs(d)[2]) for ts, d in r]
    r = rows("/perception/obstacles")
    D["obs_perc"] = [(rel(ts), dec_perc_obs(d)[2]) for ts, d in r]
    r = rows("/opponent_prediction/obstacles")
    D["obs_pred"] = [(rel(ts), dec_frenet_obs(d)[2]) for ts, d in r]
    print(f"obs: frenet={len(D['obs_frenet'])} perc={len(D['obs_perc'])} "
          f"pred={len(D['obs_pred'])}", flush=True)

    # ---- 웨이포인트 (otwpnts = 회피경로, local = 추종중, global = 레이스라인) ----
    r = rows("/planner/avoidance/otwpnts")
    D["otwpnts"] = [(rel(ts), dec_wpnts_full(d)[1]) for ts, d in r]
    r = rows("/local_waypoints")
    D["local_wpnts"] = [(rel(ts), dec_wpnts_full(d)[1]) for ts, d in r]
    r = rows("/global_waypoints")
    D["global_wpnts"] = [(rel(ts), dec_wpnts_full(d)[1]) for ts, d in r[:3]]
    print(f"wpnts: ot={len(D['otwpnts'])} local={len(D['local_wpnts'])}", flush=True)

    # ---- rosout (경고 로그: 백스톱 발화 등) ----
    r = rows("/rosout")
    out = []
    for ts, d in r:
        try:
            lvl, name, msg = dec_log(d)
            out.append((rel(ts), lvl, name, msg))
        except Exception:
            pass
    D["rosout"] = out
    print(f"rosout: {len(out)}", flush=True)

    # ---- pitwall events (문자열) ----
    r = rows("/pitwall/events")
    D["events"] = [(rel(ts), dec_string(d)) for ts, d in r]
    r = rows("/integration/status")
    D["integration"] = [(rel(ts), dec_string(d)) for ts, d in r]
    r = rows("/pf/localization_quality")
    D["pf_quality"] = [(rel(ts), dec_string(d)) for ts, d in r]

    with open(OUT, "wb") as f:
        pickle.dump(D, f)
    print(f"saved {OUT}  duration={rel(db.execute('SELECT MAX(timestamp) FROM messages').fetchone()[0]):.1f}s")


if __name__ == "__main__":
    main()
