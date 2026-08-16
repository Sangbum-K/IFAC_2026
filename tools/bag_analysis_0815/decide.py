#!/usr/bin/env python3
"""MoreSpace / ApexFor 를 로그의 장애물로 그대로 재현하고,
   MinWallInSpan 범위만 바꿨을 때 결정이 어떻게 달라지는지 비교."""
import pickle
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

# real_car.yaml + params.yaml 실차값
PATH_CHECK_MARGIN = 0.15
HALF_W = 0.15
BOUND_TRACK = 0.10
SPLINE_BOUND_MINDIST = 0.20
MAX_UNC = 0.12
EVASION_DIST = 0.35
APEX_EPS = 0.05

BOUNDARY = max(SPLINE_BOUND_MINDIST, HALF_W + BOUND_TRACK)          # 0.25
CLEAR = max(PATH_CHECK_MARGIN, HALF_W) + MAX_UNC                    # 0.27 (σ는 캡 포화)
NEED = CLEAR + APEX_EPS                                             # 0.32
PASSABLE = CLEAR + BOUNDARY                                         # 0.52


def min_wall(s0, s1):
    span = (s1 - s0) % L
    n = max(1, int(span / ds))
    i0 = int((s0 % L) / ds) % len(gw)
    idx = (i0 + np.arange(n + 1)) % len(gw)
    return A_l[idx].min(), A_r[idx].min()


def apex_for(side, o, wl, wr):
    if side == "left":
        corridor = abs(wl - o["d_left"])
        surplus = max(corridor - NEED - BOUNDARY, 0.0)
        balanced = o["d_left"] + NEED + surplus / 2
        target = o["d_left"] + max(EVASION_DIST, NEED)
        return np.clip(min(target, balanced), 0.0, max(wl - BOUNDARY, 0.0))
    corridor = abs(wr + o["d_right"])
    surplus = max(corridor - NEED - BOUNDARY, 0.0)
    balanced = o["d_right"] - NEED - surplus / 2
    target = o["d_right"] - max(EVASION_DIST, NEED)
    return np.clip(max(target, balanced), -max(wr - BOUNDARY, 0.0), 0.0)


def decide(o, pad):
    """pad=0.5 가 현재 구현. pad=0 이면 장애물 자체 구간만 본다."""
    wl, wr = min_wall((o["s_start"] - pad) % L, (o["s_end"] + pad) % L)
    lg = wl - o["d_left"]
    rg = wr + o["d_right"]
    al, ar = apex_for("left", o, wl, wr), apex_for("right", o, wl, wr)
    lc = al >= o["d_left"] + CLEAR
    rc = ar <= o["d_right"] - CLEAR
    if rg > PASSABLE and lg < PASSABLE:
        side, apex = "right", ar
    elif lg > PASSABLE and rg < PASSABLE:
        side, apex = "left", al
    elif lc != rc:
        side, apex = ("left", al) if lc else ("right", ar)
    elif not lc and not rc:
        side, apex = ("left", al) if lg >= rg else ("right", ar)
    else:
        side, apex = ("left", al) if abs(al) <= abs(ar) else ("right", ar)
    return dict(side=side, apex=apex, wl=wl, wr=wr, lg=lg, rg=rg,
                al=al, ar=ar, lc=lc, rc=rc)


def worst_margin(apex, o, pad):
    """apex 를 유지하는 구간 전체에서 차체-벽 최소 여유."""
    s0, s1 = (o["s_start"] - pad) % L, (o["s_end"] + pad) % L
    n = max(1, int(((s1 - s0) % L) / ds))
    idx = (int((s0 % L) / ds) + np.arange(n + 1)) % len(gw)
    wall = A_l[idx] if apex >= 0 else A_r[idx]
    return (wall - abs(apex) - HALF_W).min()


print(f"상수: BOUNDARY={BOUNDARY:.2f} CLEAR={CLEAR:.2f} NEED={NEED:.2f} "
      f"PASSABLE={PASSABLE:.2f}\n")

# 대표 장애물: s≈27.5 (헤어핀2 출구), 각 랩의 첫 안정 관측
targets = {}
for t, obs in D["obs_frenet"]:
    for o in obs:
        c = o["s_center"]
        for lo, hi, lbl in ((27.0, 28.2, "헤어핀2출구 s≈27.6"),
                            (32.5, 33.6, "직선 s≈33.0"),
                            (6.5, 7.4, "헤어핀1 s≈7.0"),
                            (36.0, 36.9, "직선말 s≈36.4")):
            if lo <= c <= hi:
                targets.setdefault(lbl, {}).setdefault(o["id"], []).append((t, o))

for lbl, byid in targets.items():
    print(f"\n{'='*104}\n### {lbl}\n{'='*104}")
    print(" id   t     장애물 s[..]     d[..]      | 현재(pad0.5): 벽L/R  gapL/R  "
          "apexL/R  선택 여유 | 대안(pad0.0): 선택 여유")
    for oid, lst in sorted(byid.items(), key=lambda kv: kv[1][0][0]):
        # 궤적 중 대표: 관측 중간
        t, o = lst[len(lst) // 2]
        a = decide(o, 0.5)
        b = decide(o, 0.0)
        ma = worst_margin(a["apex"], o, 0.5)
        mb = worst_margin(b["apex"], o, 0.0)
        # pad0 결정을 pad0.5 구간(실제 hold 구간)에서 평가
        mb_full = worst_margin(b["apex"], o, 0.5)
        print(f"{oid:4d} {t:6.1f} [{o['s_start']:5.2f},{o['s_end']:5.2f}] "
              f"[{o['d_right']:+5.2f},{o['d_left']:+5.2f}] | "
              f"{a['wl']:4.2f}/{a['wr']:4.2f} {a['lg']:+5.2f}/{a['rg']:+5.2f} "
              f"{a['al']:+5.2f}/{a['ar']:+5.2f}  {a['side']:5s} {ma:+5.2f} | "
              f"{b['side']:5s} apex{b['apex']:+5.2f} 여유{mb:+5.2f}(hold전체{mb_full:+5.2f})")
