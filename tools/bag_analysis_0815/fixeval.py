#!/usr/bin/env python3
"""수정안 3건을 로그의 실제 장애물로 재현 평가.

  C1  사이드 선택 기준 = 장애물 스팬 gap  ->  기동 전체(진입+유지+복귀) 최악 벽예비
  C2  추적 예비 0.10 고정            ->  속도 의존 (실측 오버슈트 p90)
  C3  블렌드 길이 v=PlanSpeed()      ->  블렌드 구간에서 실제로 낼 속도

각 안에 대해: 사이드 선택 / 계획 최악 벽예비 / 실측 오버슈트 반영 예비 /
회피 성립 여부 / 복귀 조향 기하 성립 여부.
"""
import pickle
from collections import defaultdict
from pathlib import Path

import numpy as np

P = Path(__file__).parent
D = pickle.load(open(P / "data.pkl", "rb"))
gw = D["global_wpnts"][0][1]           # s d x y d_r d_l psi kappa vx ax
gs = gw[:, 0]
A_l, A_r = np.abs(gw[:, 5]), np.abs(gw[:, 4])
VX = gw[:, 8]
KAP = gw[:, 7]
ds = gs[1] - gs[0]
L = gs[-1] + ds
N = len(gw)
fre = D["/car_state/frenet/odom"]

# --- real_car.yaml 실차값 ---
HALF = 0.15
TRACK_MARGIN = 0.10          # boundary_tracking_margin_m
BOUND_REQ = 0.20             # spline_bound_mindist
PATH_CHECK = 0.15
MAX_UNC = 0.12
EVASION = 0.35
APEX_EPS = 0.05
BLEND_OUT = 5.0              # lane_blend_out_m (real_car)
BLEND_ALAT = 8.83            # blend_lat_acc_max
STEER_KAPPA_MAX = np.tan(0.36) / 0.32    # 3.75 rad/m? -> 실제 1.17

CLEAR = max(PATH_CHECK, HALF) + MAX_UNC          # 0.27  회피에 필요한 장애물 간격
NEED = CLEAR + APEX_EPS                          # 0.32

# trackerr.py 실측: |d_plan|>0.15 구간의 "벽 방향" 추종오차 p90 [m]
OVER_P90 = [(2.0, 0.046), (3.0, 0.092), (4.0, 0.078), (5.0, 0.214), (99, 0.222)]


def over_p90(v):
    for hi, val in OVER_P90:
        if v < hi:
            return val
    return 0.222


def track_margin(v, speed_dep):
    """C2. 0.10 고정  vs  실측 p90 을 덮는 속도 의존."""
    if not speed_dep:
        return TRACK_MARGIN
    return min(TRACK_MARGIN + 0.08 * max(v - 3.5, 0.0), 0.22)


def bound_margin(v, speed_dep):
    return max(BOUND_REQ, HALF + track_margin(v, speed_dep))


def idx(s_):
    return int(round((s_ % L) / ds)) % N


def span_idx(s0, span):
    n = max(1, int(round(span / ds)))
    return (idx(s0) + np.arange(n + 1)) % N


def min_wall(s0, s1):
    i = span_idx(s0, (s1 - s0) % L)
    return A_l[i].min(), A_r[i].min()


def apex_for(side, o, wl, wr, bnd):
    if side == "left":
        surplus = max(abs(wl - o["d_left"]) - NEED - bnd, 0.0)
        return float(np.clip(min(o["d_left"] + max(EVASION, NEED),
                                 o["d_left"] + NEED + surplus / 2),
                             0.0, max(wl - bnd, 0.0)))
    surplus = max(abs(wr + o["d_right"]) - NEED - bnd, 0.0)
    return float(np.clip(max(o["d_right"] - max(EVASION, NEED),
                             o["d_right"] - NEED - surplus / 2),
                         -max(wr - bnd, 0.0), 0.0))


def smoothstep(t):
    return t * t * (3 - 2 * t)


def blend_len(amp, v):
    """BlendDist 의 그립 항. 기하 하한은 여기선 무시(둘 다 같은 항)."""
    return max(abs(v) * np.sqrt(6.0 * abs(amp) / BLEND_ALAT), 0.3)


def maneuver(apex, o, v_hold, v_out, extra=0.0):
    """진입 블렌드 + hold + 복귀 블렌드 전 구간의 최악 벽예비와,
       복귀에 필요한 곡률이 조향 한계 안인지."""
    a = abs(apex)
    hold_s0 = (o["s_start"] - 0.5) % L
    hold_span = (((o["s_end"] + 0.5) - (o["s_start"] - 0.5)) % L)
    wall = A_l if apex >= 0 else A_r

    worst = (wall[span_idx(hold_s0, hold_span)] - a - HALF - extra).min()

    # 복귀
    lo = blend_len(a, v_out)
    ret0 = (o["s_end"] + 0.5) % L
    n = max(1, int(round(lo / ds)))
    ri = (idx(ret0) + np.arange(n + 1)) % N
    prof = a * (1 - smoothstep(np.linspace(0, 1, len(ri))))
    worst = min(worst, (wall[ri] - prof - HALF - extra).min())

    # 진입
    li = blend_len(a, v_hold)
    n2 = max(1, int(round(li / ds)))
    ei = (idx(hold_s0) - np.arange(n2 + 1)[::-1]) % N
    prof_e = a * smoothstep(np.linspace(0, 1, len(ei)))
    worst = min(worst, (wall[ei] - prof_e - HALF - extra).min())

    # 복귀가 벽 수축을 이길 수 있는가: 각 지점에서 요구되는 |d| 상한을
    # 조향 기하(최소 회전반경)로 도달 가능한가
    reach_ok = True
    for k, i in enumerate(ri):
        dist = (k + 1) * ds
        limit = wall[i] - HALF - extra          # 그 지점의 |d| 상한
        need_drop = a - max(limit, 0.0)
        if need_drop <= 0:
            continue
        # smoothstep 으로 need_drop 만큼 줄이는 데 필요한 피크 곡률
        kap_req = 6.0 * need_drop / max(dist * dist, 1e-6)
        if kap_req > STEER_KAPPA_MAX:
            reach_ok = False
            break
    return worst, reach_ok


def plan(o, v, side_mode, speed_dep):
    bnd = bound_margin(v, speed_dep)
    wl, wr = min_wall((o["s_start"] - 0.5) % L, (o["s_end"] + 0.5) % L)
    al = apex_for("left", o, wl, wr, bnd)
    ar = apex_for("right", o, wl, wr, bnd)
    lc = al >= o["d_left"] + CLEAR
    rc = ar <= o["d_right"] - CLEAR
    lg, rg = wl - o["d_left"], wr + o["d_right"]
    passable = CLEAR + bnd
    ml, okl = maneuver(al, o, v, v)
    mr, okr = maneuver(ar, o, v, v)

    if rg > passable and lg < passable:
        side = "right"
    elif lg > passable and rg < passable:
        side = "left"
    elif lc != rc:
        side = "left" if lc else "right"
    elif not lc and not rc:
        if side_mode == "current":
            side = "left" if lg >= rg else "right"     # 장애물 스팬 gap
        else:
            # C1: 기동 전체 최악 벽예비 (도달 불가 사이드는 강등)
            sl = ml - (0.0 if okl else 1.0)
            sr = mr - (0.0 if okr else 1.0)
            side = "left" if sl >= sr else "right"
    else:
        side = "left" if abs(al) <= abs(ar) else "right"

    apex = al if side == "left" else ar
    m_plan, ok_reach = (ml, okl) if side == "left" else (mr, okr)
    m_real, _ = maneuver(apex, o, v, v, extra=over_p90(v))
    return dict(side=side, apex=apex, clears=(lc if side == "left" else rc),
                m_plan=m_plan, m_real=m_real, reach=ok_reach)


ZONES = {"헤어핀2 출구 s≈27.6": (27.3, 28.0),
         "헤어핀1     s≈7.0": (6.5, 7.4),
         "직선        s≈33.0": (32.7, 33.4),
         "직선말      s≈36.4": (36.0, 36.9)}

samples = defaultdict(dict)
for t, obs in D["obs_frenet"]:
    v = float(np.interp(t, fre[:, 0], fre[:, 4]))
    for o in obs:
        if not o["is_static"]:
            continue
        for lbl, (lo, hi) in ZONES.items():
            if lo <= o["s_center"] <= hi:
                samples[lbl].setdefault(o["id"], []).append((t, v, o))

VAR = [("A 현행", "current", False),
       ("B = A + C1(사이드)", "maneuver", False),
       ("C = B + C2(속도예비)", "maneuver", True)]

print(f"{'구역':20s} {'안':22s} {'좌선택':>6s} {'회피성립':>7s} {'복귀도달':>7s} "
      f"{'계획최악예비':>11s} {'실측반영예비':>12s} {'벽침범':>6s}")
for lbl, byid in samples.items():
    reps = [lst[len(lst) // 2] for lst in byid.values()]
    print("-" * 100)
    for vname, sm, sd in VAR:
        res = [plan(o, v, sm, sd) for _, v, o in reps]
        print(f"{lbl:20s} {vname:22s} "
              f"{100*np.mean([r['side']=='left' for r in res]):5.0f}% "
              f"{100*np.mean([r['clears'] for r in res]):6.0f}% "
              f"{100*np.mean([r['reach'] for r in res]):6.0f}% "
              f"{np.median([r['m_plan'] for r in res]):11.2f} "
              f"{np.median([r['m_real'] for r in res]):12.2f} "
              f"{100*np.mean([r['m_real'] < 0 for r in res]):5.0f}%")

print("\n\n=== 헤어핀2 출구: 조우별 상세 (현행 -> C1) ===")
print("  id    t    v     장애물 d[..]      | 현행 사이드 apex  최악예비 도달 | "
      "C1 사이드 apex  최악예비 도달")
for oid, lst in sorted(samples["헤어핀2 출구 s≈27.6"].items(),
                       key=lambda kv: kv[1][0][0]):
    t, v, o = lst[len(lst) // 2]
    a = plan(o, v, "current", False)
    b = plan(o, v, "maneuver", False)
    print(f"{oid:4d} {t:6.1f} {v:4.1f} [{o['d_right']:+5.2f},{o['d_left']:+5.2f}] "
          f"| {a['side']:5s} {a['apex']:+5.2f} {a['m_plan']:+8.2f} "
          f"{'O' if a['reach'] else 'X':>4s} | "
          f"{b['side']:5s} {b['apex']:+5.2f} {b['m_plan']:+8.2f} "
          f"{'O' if b['reach'] else 'X':>4s}")

# ---------------- C3: 블렌드 길이 ----------------
print("\n\n=== C3. 블렌드 길이: 계획시점 속도 vs 블렌드 중 실제 속도 ===")
print("  회피 진입 블렌드가 끝나는 지점에서 차가 실제로 내는 속도로 다시 계산.")
print("  s0     A     v_plan  L(현행)  v_실제  L(수정)  피크 a_lat(현행) [m/s^2]")
for s0, amp in ((32.3, 0.67), (24.0, 0.83), (5.0, 0.36), (30.5, 0.50)):
    i0 = idx(s0)
    v_plan = VX[i0]
    l_now = max(blend_len(amp, v_plan), 0.3)
    i1 = idx(s0 + l_now)
    v_real = VX[span_idx(s0, l_now)].max()
    l_new = blend_len(amp, v_real)
    a_now = 6.0 * amp * v_real ** 2 / l_now ** 2
    print(f"{s0:5.1f} {amp:5.2f} {v_plan:7.2f} {l_now:8.2f} {v_real:7.2f} "
          f"{l_new:8.2f} {a_now:12.1f}  (예산 {BLEND_ALAT})")

print("\n  ※ 실제 사고(t=109.5~110.4, s=32.3->35.6, A=0.67): 계획 L=3.3m, "
      "블렌드 끝 v=5.9 m/s")
print(f"     -> 피크 a_lat = {6*0.67*5.9**2/3.3**2:.1f} m/s^2  (예산 {BLEND_ALAT})"
      f" = {6*0.67*5.9**2/3.3**2/BLEND_ALAT:.2f}배 초과")
print(f"     -> v=5.9 로 산정하면 L = {blend_len(0.67, 5.9):.2f} m, "
      f"피크 a_lat = {6*0.67*5.9**2/blend_len(0.67,5.9)**2:.1f}")
