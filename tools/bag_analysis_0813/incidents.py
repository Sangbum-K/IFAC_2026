#!/usr/bin/env python3
"""4개 사고 구간 정밀 분석."""
import pathlib
import pickle
import re
import sys

import numpy as np

D = pickle.load(open(str(pathlib.Path(__file__).parent / "data.pkl"), "rb"))
CAL = D["calib"]
GAIN, OFF = CAL["gain"], CAL["offset"]
SMIN, SMAX = CAL["servo_min"], CAL["servo_max"]
MAX_STEER = 0.36            # planning/control 상한 (CLAUDE.md)
MECH_STEER = 0.3677         # 대칭 기계 한계
WHEELBASE = 0.33

TRACE = re.compile(
    r"\[trace\] s=(?P<s>[-\d.]+) v=(?P<v>[-\d.]+) d=(?P<d>[-\d.]+) \| "
    r"obs id=(?P<oid>[-\d]+) dist=(?P<dist>[-\d.]+) "
    r"d\[(?P<dr>[-\d.]+),(?P<dl>[-\d.]+)\] static=(?P<st>\d+) nobs=(?P<nobs>\d+) \| "
    r"evade=(?P<ev>\d+) sector=(?P<sec>\d+) commit=(?P<cm>\d+) \| "
    r"(?P<mode>\w+) \| (?P<rej>.*)")


def parse_traces():
    out = []
    for t, lvl, name, msg in D["rosout"]:
        m = TRACE.search(msg)
        if not m:
            continue
        g = m.groupdict()
        out.append(dict(t=t, s=float(g["s"]), v=float(g["v"]), d=float(g["d"]),
                        oid=int(g["oid"]), dist=float(g["dist"]),
                        dr=float(g["dr"]), dl=float(g["dl"]),
                        static=int(g["st"]), nobs=int(g["nobs"]),
                        evade=int(g["ev"]), sector=int(g["sec"]),
                        commit=int(g["cm"]), mode=g["mode"],
                        rej=[r for r in g["rej"].split(",") if r and r != "-"]))
    return out


def at(arr, t, col=1):
    """arr[:,0]=t 에서 t 시점 최근접 값."""
    if len(arr) == 0:
        return float("nan")
    i = np.searchsorted(arr[:, 0], t)
    i = min(max(i, 0), len(arr) - 1)
    return arr[i, col]


def window(arr, t0, t1):
    if len(arr) == 0:
        return arr
    m = (arr[:, 0] >= t0) & (arr[:, 0] <= t1)
    return arr[m]


def servo_from_delta(delta):
    return GAIN * delta + OFF


def delta_from_servo(sv):
    return (sv - OFF) / GAIN


INCIDENTS = [(104.0, "1:44 충돌"), (129.0, "2:09 회피후 감속"),
             (140.0, "2:20 회피후 감속"), (167.0, "2:47 동적 오분류")]


def main():
    traces = parse_traces()
    print(f"trace 파싱: {len(traces)}개\n")

    ack = D["/ackermann_cmd"]                 # t, steer, speed
    servo = D["/commands/servo/position"]     # t, val
    spd = D["/monitor/current_speed"]         # t, val
    imu = D["/sensors/imu/raw"]               # t,yaw,wx,wy,wz,ax,ay,az
    fren = D["/car_state/frenet/odom"]        # t,x=s,y=d,yaw,v
    esc = D["/planner/escape_active"]
    lat = D["/pitwall/lat_err"]
    core = D["/sensors/core"]                 # t,v_in,i_motor,erpm,duty

    for tc, label in INCIDENTS:
        t0, t1 = tc - 6.0, tc + 4.0
        print("=" * 100)
        print(f"### {label}  (t={tc:.0f}s, 창 {t0:.0f}~{t1:.0f}s)")
        print("=" * 100)

        # --- 스플라이너 판단 흐름 ---
        tw = [x for x in traces if t0 <= x["t"] <= t1]
        print(f"\n[스플라이너 판단] {len(tw)} cycles")
        if tw:
            prev = None
            for x in tw:
                key = (x["mode"], tuple(x["rej"]), x["evade"], x["commit"])
                mark = " " if key == prev else ">"
                prev = key
                print(f" {mark}{x['t']:7.2f} s={x['s']:6.2f} v={x['v']:4.2f} "
                      f"d={x['d']:+5.2f} | obs{x['oid']:>4} dist={x['dist']:5.2f} "
                      f"d[{x['dr']:+.2f},{x['dl']:+.2f}] st={x['static']} "
                      f"nobs={x['nobs']:>2} | ev={x['evade']} sec={x['sector']} "
                      f"cm={x['commit']} | {x['mode']:<12} | {','.join(x['rej'])}")

        # --- 관련 로그 (trace 제외) ---
        logs = [(t, lvl, msg) for t, lvl, n, msg in D["rosout"]
                if t0 <= t <= t1 and "[trace]" not in msg
                and "Monitor Running" not in msg and "PF sync" not in msg
                and "[Frenet]" not in msg and "Visualization Running" not in msg
                and "Published" not in msg]
        print(f"\n[로그] {len(logs)}")
        for t, lvl, msg in logs:
            print(f"  {t:7.2f} lvl{lvl} {msg[:150]}")

        # --- 조향/속도/서보 ---
        aw = window(ack, t0, t1)
        sw = window(servo, t0, t1)
        iw = window(imu, t0, t1)
        print(f"\n[조향·속도] 명령 {len(aw)} 샘플")
        print("      t   cmd_v  act_v  cmd_δ(deg) sat%  servo  포화?  "
              "yawrate  δ_kin(deg)  실측/기하비")
        for r in aw[::4]:
            t, st, sp = r
            av = at(spd, t)
            sv = at(sw, t) if len(sw) else float("nan")
            satr = abs(st) / MAX_STEER
            wz = at(iw, t, 4) if len(iw) else float("nan")
            # 실측 yaw rate 로 역산한 유효 조향각 (자전거 모델)
            dkin = np.arctan2(wz * WHEELBASE, av) if av > 0.3 else float("nan")
            ratio = (dkin / st) if (abs(st) > 0.02 and np.isfinite(dkin)) else float("nan")
            clip = ""
            if np.isfinite(sv):
                if sv <= SMIN + 1e-6:
                    clip = "MIN클립"
                elif sv >= SMAX - 1e-6:
                    clip = "MAX클립"
            print(f" {t:7.2f} {sp:6.2f} {av:6.2f} {np.degrees(st):+9.2f} "
                  f"{100*satr:5.1f} {sv:6.3f} {clip:>7}  {wz:+7.3f} "
                  f"{np.degrees(dkin):+9.2f}  {ratio:5.2f}")

        # --- 요약 통계 ---
        if len(aw):
            print(f"\n[요약] cmd_δ max |{np.degrees(np.abs(aw[:,1]).max()):.2f}|deg "
                  f"(포화율 {100*np.abs(aw[:,1]).max()/MAX_STEER:.0f}%)  "
                  f"기계한계까지 여유 {np.degrees(MECH_STEER - np.abs(aw[:,1]).max()):.2f}deg")
            svw = sw[:, 1] if len(sw) else np.array([])
            if len(svw):
                print(f"       servo 범위 [{svw.min():.3f},{svw.max():.3f}] "
                      f"vs 한계 [{SMIN},{SMAX}]  "
                      f"MIN클립 {np.sum(svw<=SMIN+1e-6)} MAX클립 {np.sum(svw>=SMAX-1e-6)}")
            aws = window(spd, t0, t1)
            if len(aws):
                print(f"       실측속도 {aws[:,1].min():.2f}~{aws[:,1].max():.2f} m/s")
            ew = window(esc, t0, t1)
            if len(ew):
                print(f"       escape_active: {int(ew[:,1].sum())}/{len(ew)} 샘플 활성")
            lw = window(lat, t0, t1)
            if len(lw):
                print(f"       lat_err {lw[:,1].min():+.2f}~{lw[:,1].max():+.2f}")
        print()


if __name__ == "__main__":
    main()
