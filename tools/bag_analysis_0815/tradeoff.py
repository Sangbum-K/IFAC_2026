#!/usr/bin/env python3
"""수정안 득실 정량화.
   A) 현행: MoreSpace(pad 0.5, |apex| 최소) + 고정 벽여유 0.10
   B) 기동 전체(진입+hold+복귀) 최악 여유로 사이드 선택
   C) B + 속도 의존 추적여유
   각 안에 대해: 회피 성사율 / 필요 횡이동량 / 최악 벽여유(계획·실측 오버슈트 포함)
"""
import pickle
from collections import defaultdict
from pathlib import Path

import numpy as np

P = Path(__file__).parent
D = pickle.load(open(P / "data.pkl", "rb"))
gw = D["global_wpnts"][0][1]
gs = gw[:, 0]
A_l, A_r = np.abs(gw[:, 5]), np.abs(gw[:, 4])
ds = gs[1] - gs[0]
L = gs[-1] + ds
fre = D["/car_state/frenet/odom"]

HALF_W, PATH_CHECK, MAX_UNC, EVASION, APEX_EPS = 0.15, 0.15, 0.12, 0.35, 0.05
CLEAR = max(PATH_CHECK, HALF_W) + MAX_UNC          # 0.27
NEED = CLEAR + APEX_EPS                            # 0.32
BLEND_OUT = 2.5                                    # 복귀 블렌드 대표 길이 [m]

# 실측 오버슈트 p90 (trackerr.py, |d_plan|>0.15)
OVER_P90 = {2: 0.046, 3: 0.092, 4: 0.078, 5: 0.214, 9: 0.222}


def over_p90(v):
    for k in sorted(OVER_P90):
        if v < k:
            return OVER_P90[k]
    return 0.222


def track_margin(v, mode):
    if mode == "fixed":
        return 0.10
    return min(0.10 + 0.055 * max(v - 3.0, 0.0), 0.25)   # 제안: 속도 의존


def idxs(s0, s1):
    n = max(1, int(((s1 - s0) % L) / ds))
    return (int((s0 % L) / ds) + np.arange(n + 1)) % len(gw)


def apex_for(side, o, wl, wr, bnd):
    if side == "left":
        corridor = abs(wl - o["d_left"])
        surplus = max(corridor - NEED - bnd, 0.0)
        return float(np.clip(min(o["d_left"] + max(EVASION, NEED),
                                 o["d_left"] + NEED + surplus / 2),
                             0.0, max(wl - bnd, 0.0)))
    corridor = abs(wr + o["d_right"])
    surplus = max(corridor - NEED - bnd, 0.0)
    return float(np.clip(max(o["d_right"] - max(EVASION, NEED),
                             o["d_right"] - NEED - surplus / 2),
                         -max(wr - bnd, 0.0), 0.0))


def maneuver_margin(apex, o, extra=0.0):
    """진입(장애물앞 0.5m) + hold + 복귀(BLEND_OUT) 전 구간 최악 차체-벽 여유.
       복귀 구간은 |d| 가 smoothstep 으로 감소한다."""
    hold = idxs((o["s_start"] - 0.5) % L, (o["s_end"] + 0.5) % L)
    wall = A_l[hold] if apex >= 0 else A_r[hold]
    worst = (wall - abs(apex) - HALF_W - extra).min()
    n = max(1, int(BLEND_OUT / ds))
    ret = (int(((o["s_end"] + 0.5) % L) / ds) + np.arange(n + 1)) % len(gw)
    w = np.linspace(0, 1, len(ret))
    prof = abs(apex) * (1 - (w * w * (3 - 2 * w)))
    wallr = A_l[ret] if apex >= 0 else A_r[ret]
    return min(worst, (wallr - prof - HALF_W - extra).min())


def plan(o, v, mode_side, mode_margin):
    bnd = HALF_W + track_margin(v, mode_margin)
    pad = 0.5
    i = idxs((o["s_start"] - pad) % L, (o["s_end"] + pad) % L)
    wl, wr = A_l[i].min(), A_r[i].min()
    al, ar = apex_for("left", o, wl, wr, bnd), apex_for("right", o, wl, wr, bnd)
    lc = al >= o["d_left"] + CLEAR
    rc = ar <= o["d_right"] - CLEAR
    ml, mr = maneuver_margin(al, o), maneuver_margin(ar, o)
    if mode_side == "current":
        lg, rg = wl - o["d_left"], wr + o["d_right"]
        passable = CLEAR + bnd
        if rg > passable and lg < passable:
            side = "right"
        elif lg > passable and rg < passable:
            side = "left"
        elif lc != rc:
            side = "left" if lc else "right"
        elif not lc and not rc:
            side = "left" if lg >= rg else "right"
        else:
            side = "left" if abs(al) <= abs(ar) else "right"
    else:                                   # 기동 전체 최악여유가 큰 쪽
        if lc != rc:
            side = "left" if lc else "right"
        elif not lc and not rc:
            side = "left" if ml >= mr else "right"
        else:
            side = "left" if ml >= mr else "right"
    apex = al if side == "left" else ar
    ok = (lc if side == "left" else rc) and maneuver_margin(apex, o) >= 0.0
    return dict(side=side, apex=apex, ok=ok,
                m_plan=maneuver_margin(apex, o),
                m_real=maneuver_margin(apex, o, extra=over_p90(v)))


# 각 물리 장애물 조우 = (랩, s빈) 대표 관측 여러 개
zones = {"헤어핀2출구 s≈27.6": (27.0, 28.2), "직선 s≈33.0": (32.5, 33.6),
         "헤어핀1 s≈7.0": (6.5, 7.4), "직선말 s≈36.4": (36.0, 36.9)}
samples = defaultdict(list)
for t, obs in D["obs_frenet"]:
    v = float(np.interp(t, fre[:, 0], fre[:, 4]))
    for o in obs:
        for lbl, (lo, hi) in zones.items():
            if lo <= o["s_center"] <= hi and o["is_static"]:
                samples[lbl].append((t, v, o))

VARIANTS = [("A 현행", "current", "fixed"),
            ("B 기동최악여유 사이드", "worst", "fixed"),
            ("C  B+속도의존여유", "worst", "speed")]

print(f"{'구역':22s} {'안':22s} {'회피성사':>7s} {'좌선택':>6s} "
      f"{'|apex|평균':>9s} {'계획최악여유':>11s} {'p90오버슈트 반영':>14s} {'<0 비율':>7s}")
for lbl in zones:
    ss = samples[lbl]
    if not ss:
        continue
    print("-" * 108)
    for vname, msd, mmg in VARIANTS:
        res = [plan(o, v, msd, mmg) for _, v, o in ss]
        ok = np.mean([r["ok"] for r in res])
        lft = np.mean([r["side"] == "left" for r in res])
        ap = np.mean([abs(r["apex"]) for r in res])
        mp = np.percentile([r["m_plan"] for r in res], 5)
        mr = np.percentile([r["m_real"] for r in res], 5)
        neg = np.mean([r["m_real"] < 0 for r in res])
        print(f"{lbl:22s} {vname:22s} {100*ok:6.0f}% {100*lft:5.0f}% "
              f"{ap:9.2f} {mp:11.2f} {mr:14.2f} {100*neg:6.0f}%")

print("\n(계획최악여유/p90오버슈트 반영 = 해당 구역 표본의 5퍼센타일 [m]. "
      "음수면 차체가 벽을 침범.)")
print(f"속도의존 추적여유: v<=3 -> 0.10, v=5 -> {track_margin(5,'speed'):.2f}, "
      f"v>=7.5 -> {track_margin(9,'speed'):.2f}")
