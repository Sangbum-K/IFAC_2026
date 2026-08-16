#!/usr/bin/env python3
"""구현된 로직(spliner_node.cpp 2026-08-16)을 그대로 옮겨 로그로 검증.

  ManeuverWorstMargin : hold(장애물 앞뒤 0.5m) + 복귀 블렌드. 진입은 제외.
  TrackingReserve     : min(0.10 + 0.08*max(v-3.5,0), 0.22)
  MoreSpace           : 둘 다 막힘 분기의 비교 기준을 gap -> 기동 최악여유
  BlendPlanSpeed      : max(PlanSpeed, min(레이스라인 구간최대, 도달가능))
"""
import pickle
from collections import defaultdict
from pathlib import Path

import numpy as np

P = Path(__file__).parent
D = pickle.load(open(P / "data.pkl", "rb"))
gw = D["global_wpnts"][0][1]
gs, VX, KAP = gw[:, 0], gw[:, 8], gw[:, 7]
A_l, A_r = np.abs(gw[:, 5]), np.abs(gw[:, 4])
DS = gs[1] - gs[0]
L = gs[-1] + DS
N = len(gw)
fre = D["/car_state/frenet/odom"]

# --- 실차 파라미터 ---
HALF, TRACK_M, BOUND_REQ = 0.15, 0.10, 0.20
PATH_CHECK, MAX_UNC, EVASION, APEX_EPS = 0.15, 0.12, 0.35, 0.05
BLEND_OUT_MIN, BLEND_ALAT, PLAN_FLOOR = 5.0, 8.83, 1.5
PATH_ACCEL, GEOM_MARGIN, GEOM_KRATIO = 3.0, 0.92, 0.30
KAPPA_STEER_MAX = 1.0 * np.tan(0.3578) / 0.33            # SteerKappaMax()
CLEAR = max(PATH_CHECK, HALF) + MAX_UNC                  # 0.27
NEED = CLEAR + APEX_EPS                                  # 0.32
# 속도 의존 예비 (신규)
RES_REF, RES_GAIN, RES_MAX = 3.5, 0.08, 0.22

OVER_P90 = [(2.0, 0.046), (3.0, 0.092), (4.0, 0.078), (5.0, 0.214), (99, 0.222)]


def over_p90(v):
    return next(val for hi, val in OVER_P90 if v < hi)


def reserve(v, speed_dep):
    if not speed_dep:
        return TRACK_M
    return min(TRACK_M + RES_GAIN * max(v - RES_REF, 0.0), RES_MAX)


def bound_margin(v, speed_dep):
    return max(BOUND_REQ, HALF + reserve(v, speed_dep))


def i_at(s):
    return int((s % L) / DS) % N


def min_wall(s0, s1):
    n = max(1, int((((s1 - s0) % L)) / DS))
    i = (i_at(s0) + np.arange(n + 1)) % N
    return A_l[i].min(), A_r[i].min()


def apex_for(side, o, wl, wr, bnd):
    if side == "left":
        sur = max(abs(wl - o["d_left"]) - NEED - bnd, 0.0)
        return float(np.clip(min(o["d_left"] + max(EVASION, NEED),
                                 o["d_left"] + NEED + sur / 2),
                             0.0, max(wl - bnd, 0.0)))
    sur = max(abs(wr + o["d_right"]) - NEED - bnd, 0.0)
    return float(np.clip(max(o["d_right"] - max(EVASION, NEED),
                             o["d_right"] - NEED - sur / 2),
                         -max(wr - bnd, 0.0), 0.0))


def blend_plan_speed(s0, span, v0):
    """BlendPlanSpeed()"""
    n = max(1, int(min(span, L - DS) / DS))
    i = (i_at(s0) + np.arange(n + 1)) % N
    v_line = max(v0, VX[i].max())
    v_reach = np.sqrt(v0 * v0 + 2.0 * PATH_ACCEL * min(span, L - DS))
    return max(v0, min(v_line, v_reach))


def blend_dist(amp, v):
    return abs(v) * np.sqrt(6.0 * abs(amp) / BLEND_ALAT)


def maneuver_worst_margin(apex, o, v, speed_dep):
    """ManeuverWorstMargin() — hold + 복귀. reachable 도 함께."""
    a = abs(apex)
    wall = A_l if apex >= 0 else A_r
    hold_s0 = (o["s_start"] - 0.5) % L
    hold_span = ((o["s_end"] + 0.5) - (o["s_start"] - 0.5)) % L
    hi = (i_at(hold_s0) + np.arange(max(1, int(hold_span / DS)) + 1)) % N
    worst = (wall[hi] - a - HALF).min()

    ret_s0 = (o["s_end"] + 0.5) % L
    out_v = v
    for _ in range(2):
        out_v = blend_plan_speed(ret_s0,
                                 max(BLEND_OUT_MIN, blend_dist(a, out_v)), v)
    blend_out = min(max(BLEND_OUT_MIN, blend_dist(a, out_v)), L - DS)
    n = max(1, int(blend_out / DS))
    res = reserve(v, speed_dep)
    reach = True
    for k in range(n + 1):
        dist = k * DS
        w = k / n
        prof = a * (1 - w * w * (3 - 2 * w))
        gi = i_at(ret_s0 + dist)
        worst = min(worst, wall[gi] - prof - HALF)
        if dist < DS:
            continue
        drop = a - max(wall[gi] - HALF - res, 0.0)
        if drop <= 0:
            continue
        spare = GEOM_MARGIN * max(KAPPA_STEER_MAX - abs(KAP[gi]),
                                  GEOM_KRATIO * KAPPA_STEER_MAX)
        if 6.0 * drop / (dist * dist) > spare:
            reach = False
    return worst, reach


def more_space(o, v, new_logic, speed_dep):
    bnd = bound_margin(v, speed_dep)
    wl, wr = min_wall((o["s_start"] - 0.5) % L, (o["s_end"] + 0.5) % L)
    al, ar = apex_for("left", o, wl, wr, bnd), apex_for("right", o, wl, wr, bnd)
    lg, rg = wl - o["d_left"], wr + o["d_right"]
    passable = CLEAR + bnd
    lc = al >= o["d_left"] + CLEAR
    rc = ar <= o["d_right"] - CLEAR
    ml, okl = maneuver_worst_margin(al, o, v, speed_dep)
    mr, okr = maneuver_worst_margin(ar, o, v, speed_dep)

    if rg > passable and lg < passable:
        side = "right"
    elif lg > passable and rg < passable:
        side = "left"
    elif lc != rc:
        side = "left" if lc else "right"
    elif not lc and not rc:
        if new_logic:
            side = "left" if (ml - (0 if okl else 1)) >= (mr - (0 if okr else 1)) \
                else "right"
        else:
            side = "left" if lg >= rg else "right"
    else:
        pref_left = abs(al) <= abs(ar)
        side = "left" if pref_left else "right"
        if new_logic:
            if pref_left and not okl and okr:
                side = "right"
            elif not pref_left and not okr and okl:
                side = "left"
    apex = al if side == "left" else ar
    m, ok = (ml, okl) if side == "left" else (mr, okr)
    m_real, _ = maneuver_worst_margin(apex, o, v, speed_dep)
    return dict(side=side, apex=apex, m_plan=m, reach=ok,
                clears=(lc if side == "left" else rc),
                m_real=m_real - over_p90(v))


ZONES = {"헤어핀2 출구 s≈27.6": (27.3, 28.0), "헤어핀1     s≈7.0": (6.5, 7.4),
         "직선        s≈33.0": (32.7, 33.4), "직선말      s≈36.4": (36.0, 36.9)}
samples = defaultdict(dict)
for t, obs in D["obs_frenet"]:
    v = float(np.interp(t, fre[:, 0], fre[:, 4]))
    for o in obs:
        if o["is_static"]:
            for lbl, (lo, hi) in ZONES.items():
                if lo <= o["s_center"] <= hi:
                    samples[lbl].setdefault(o["id"], []).append((t, v, o))

print("=== 구역별 (각 조우의 중간 관측 1건씩) ===")
print(f"{'구역':20s} {'안':24s} {'좌선택':>6s} {'회피성립':>7s} {'복귀도달':>7s} "
      f"{'계획최악예비':>11s} {'실측반영예비':>12s} {'벽침범':>6s}")
VAR = [("A 현행", False, False), ("B = A+C1(사이드)", True, False),
       ("C = B+C2(속도예비)", True, True)]
tot = {k: [0, 0] for k, _, _ in VAR}
for lbl, byid in samples.items():
    reps = [lst[len(lst) // 2] for lst in byid.values()]
    print("-" * 102)
    for vname, nl, sd in VAR:
        r = [more_space(o, v, nl, sd) for _, v, o in reps]
        tot[vname][0] += sum(x["m_real"] < 0 for x in r)
        tot[vname][1] += len(r)
        print(f"{lbl:20s} {vname:24s} "
              f"{100*np.mean([x['side']=='left' for x in r]):5.0f}% "
              f"{100*np.mean([x['clears'] for x in r]):6.0f}% "
              f"{100*np.mean([x['reach'] for x in r]):6.0f}% "
              f"{np.median([x['m_plan'] for x in r]):11.2f} "
              f"{np.median([x['m_real'] for x in r]):12.2f} "
              f"{100*np.mean([x['m_real']<0 for x in r]):5.0f}%")
print("\n전체 벽침범(실측 p90 오버슈트 반영) 비율:")
for k, (bad, n) in tot.items():
    print(f"  {k:24s} {bad}/{n} = {100*bad/n:.0f}%")

print("\n\n=== 헤어핀2 출구 조우별 (사고 지점) ===")
print("  id     t    v   | 현행: side apex 최악예비 도달 | 신규: side apex "
      "최악예비 도달")
for oid, lst in sorted(samples["헤어핀2 출구 s≈27.6"].items(),
                       key=lambda kv: kv[1][0][0]):
    t, v, o = lst[len(lst) // 2]
    a = more_space(o, v, False, False)
    b = more_space(o, v, True, True)
    mark = "   <== 뒤집힘" if a["side"] != b["side"] else ""
    print(f"{oid:4d} {t:6.1f} {v:4.1f} | {a['side']:5s} {a['apex']:+5.2f} "
          f"{a['m_plan']:+9.2f} {'O' if a['reach'] else 'X':>4s} | "
          f"{b['side']:5s} {b['apex']:+5.2f} {b['m_plan']:+9.2f} "
          f"{'O' if b['reach'] else 'X':>4s}{mark}")

print("\n\n=== C3. 블렌드 길이/피크 횡가속 (BlendPlanSpeed 적용 전후) ===")
print("  구간            A     v_now  L(현행)  a_lat(현행)  v_블렌드  L(신규)  "
      "a_lat(신규)")
CASES = [("s=32.3 직선회피", 32.3, 0.67, 3.53), ("s=24.0 헤어핀2진입", 24.0, 0.83, 2.9),
         ("s=30.5 복귀", 30.5, 0.50, 2.2), ("s=5.0 헤어핀1", 5.0, 0.36, 4.0)]
for lbl, s0, amp, v_now in CASES:
    v0 = max(v_now, PLAN_FLOOR)
    l_old = max(blend_dist(amp, v0), 0.3)
    v_new = v0
    for _ in range(2):
        v_new = blend_plan_speed(s0, max(blend_dist(amp, v_new), 0.3), v0)
    l_new = max(blend_dist(amp, v_new), 0.3)
    # 실제로 낼 속도로 평가한 피크 횡가속
    v_real = blend_plan_speed(s0, l_old, v0)
    a_old = 6 * amp * v_real ** 2 / l_old ** 2
    a_new = 6 * amp * v_new ** 2 / l_new ** 2
    print(f"{lbl:16s} {amp:5.2f} {v0:6.2f} {l_old:8.2f} {a_old:11.1f}  "
          f"{v_new:8.2f} {l_new:8.2f} {a_new:11.1f}   (예산 {BLEND_ALAT})")
