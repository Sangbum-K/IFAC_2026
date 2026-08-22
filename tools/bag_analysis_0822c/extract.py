#!/usr/bin/env python3
"""run_0815_191056 추출기. bag_analysis_0813b/extract.py 구조를 그대로 따름."""
import pickle
import sqlite3
import sys
from pathlib import Path

sys.path.insert(0, "/home/mina/IFAC_ws/tools")
import numpy as np
from bagviz import (CDR, dec_ackermann, dec_bool, dec_float32, dec_float64,
                    dec_grid, dec_log, dec_odom, dec_point_stamped, dec_scan,
                    dec_string, dec_tf_static, quat_yaw, read_vesc_calib)

BAG = sys.argv[1] if len(sys.argv)>1 else str(Path(__file__).parent / "bag.db3")
OUT = Path(__file__).parent / (sys.argv[2] if len(sys.argv)>2 else "data.pkl")


def dec_frenet_obs(b):
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
    """WpntArray -> ndarray[n,10]  s d x y d_r d_l psi kappa vx ax"""
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
    c = CDR(b)
    ht, _f = c.header()
    f = c.f64n(6)
    return ht, dict(v_in=f[0], temp=f[1], i_motor=f[2], i_in=f[3],
                    erpm=f[4], duty=f[5])


def dec_imu_raw(b):
    c = CDR(b)
    ht, _f = c.header()
    qx, qy, qz, qw = c.f64n(4)
    c.skip_f64(9)
    wx, wy, wz = c.f64n(3)
    c.skip_f64(9)
    ax, ay, az = c.f64n(3)
    return ht, quat_yaw(qx, qy, qz, qw), wx, wy, wz, ax, ay, az


def main():
    db = sqlite3.connect(BAG)
    topics = {n: (tid, ty) for tid, n, ty in
              db.execute("SELECT id,name,type FROM topics")}
    t0_ns = db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]

    def rows(name):
        if name not in topics:
            return []
        return db.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topics[name][0],)).fetchall()

    D = {"t0_ns": t0_ns, "calib": read_vesc_calib()}

    def rel(ts):
        return (ts - t0_ns) / 1e9

    for n in topics:
        ty = topics[n][1]
        if ty.endswith("Float32"):
            r = rows(n)
            D[n] = np.array([(rel(ts), dec_float32(d)) for ts, d in r]) if r else np.zeros((0, 2))
        elif ty.endswith("Float64"):
            r = rows(n)
            D[n] = np.array([(rel(ts), dec_float64(d)) for ts, d in r]) if r else np.zeros((0, 2))
    print("scalars done", flush=True)

    for n in ("/planner/escape_active", "/safety/emergency_stop",
              "/pf/localization_ready"):
        r = rows(n)
        D[n] = np.array([(rel(ts), 1.0 if dec_bool(d) else 0.0) for ts, d in r]) \
            if r else np.zeros((0, 2))

    for n in ("/ackermann_cmd", "/autonomy/ackermann_cmd", "/teleop"):
        r = rows(n)
        out = [(rel(ts),) + dec_ackermann(d) for ts, d in r]
        D[n] = np.array(out) if out else np.zeros((0, 3))
    print("ackermann done", flush=True)

    for n in ("/odom", "/pf/pose/odom", "/car_state/frenet/odom"):
        r = rows(n)
        out = []
        for ts, d in r:
            ht, x, y, yaw, v = dec_odom(d)
            out.append((rel(ts), x, y, yaw, v))
        D[n] = np.array(out) if out else np.zeros((0, 5))
    print("odom done", flush=True)

    r = rows("/sensors/imu/raw")
    out = []
    for ts, d in r:
        try:
            ht, yaw, wx, wy, wz, ax, ay, az = dec_imu_raw(d)
            out.append((rel(ts), yaw, wx, wy, wz, ax, ay, az))
        except Exception:
            pass
    D["/sensors/imu/raw"] = np.array(out) if out else np.zeros((0, 8))

    r = rows("/sensors/core")
    out = []
    for ts, d in r:
        try:
            ht, s = dec_vesc_state(d)
            out.append((rel(ts), s["v_in"], s["i_motor"], s["erpm"], s["duty"]))
        except Exception:
            pass
    D["/sensors/core"] = np.array(out) if out else np.zeros((0, 5))
    print("core done", flush=True)

    r = rows("/perception/obstacles_frenet")
    D["obs_frenet"] = [(rel(ts), dec_frenet_obs(d)[2]) for ts, d in r]
    r = rows("/perception/obstacles")
    D["obs_perc"] = [(rel(ts), dec_perc_obs(d)[2]) for ts, d in r]
    r = rows("/opponent_prediction/obstacles")
    D["obs_pred"] = [(rel(ts), dec_frenet_obs(d)[2]) for ts, d in r]
    print(f"obs: frenet={len(D['obs_frenet'])} perc={len(D['obs_perc'])}", flush=True)

    r = rows("/planner/avoidance/otwpnts")
    D["otwpnts"] = [(rel(ts), dec_wpnts_full(d)[1]) for ts, d in r]
    r = rows("/local_waypoints")
    D["local_wpnts"] = [(rel(ts), dec_wpnts_full(d)[1]) for ts, d in r]
    r = rows("/global_waypoints")
    D["global_wpnts"] = [(rel(ts), dec_wpnts_full(d)[1]) for ts, d in r[:2]]
    print("wpnts done", flush=True)

    # scan: 4x 데시메이션 (빔 stride 4)
    r = rows("/scan")
    out = []
    for ts, d in r:
        ht, amin, ainc, tinc, rr = dec_scan(d, 2)
        out.append((rel(ts), amin, ainc, rr.astype(np.float32)))
    D["scan"] = out
    print(f"scan: {len(out)}", flush=True)

    r = rows("/control/lookahead_point")
    D["lookahead"] = np.array([(rel(ts),) + dec_point_stamped(d) for ts, d in r]) \
        if r else np.zeros((0, 3))

    r = rows("/rosout")
    out = []
    for ts, d in r:
        try:
            lvl, name, msg = dec_log(d)
            out.append((rel(ts), lvl, name, msg))
        except Exception:
            pass
    D["rosout"] = out

    for key, topic in (("events", "/pitwall/events"),
                       ("integration", "/integration/status"),
                       ("pf_quality", "/pf/localization_quality")):
        D[key] = [(rel(ts), dec_string(d)) for ts, d in rows(topic)]

    r = rows("/map")
    if r:
        res, ox, oy, w, h, data = dec_grid(r[0][1])
        D["map"] = dict(res=res, ox=ox, oy=oy, w=w, h=h, data=np.array(data))
    r = rows("/tf_static")
    if r:
        D["tf_static"] = dec_tf_static(r[0][1])

    with open(OUT, "wb") as f:
        pickle.dump(D, f)
    print(f"saved {OUT} dur={rel(db.execute('SELECT MAX(timestamp) FROM messages').fetchone()[0]):.1f}s")


if __name__ == "__main__":
    main()
