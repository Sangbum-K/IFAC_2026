#!/usr/bin/env python3
"""회귀 감시 지표를 한 번에 뽑는다 (실차 백 / 시뮬 백 공통).

  python3 extract.py <bag_dir>/<x>.db3 <x>.pkl
  python3 metrics.py <x>.pkl

PLAN.md §5 의 '회귀 감시' 표와 같은 항목이다. 기준선(실차 run_0820_235246):
  정적 통과여유 중앙 0.31 / 접촉률 8%,  동적 0.11 / 43%
  자유주행 속도비 85%,  랩타임 7.45~7.70 s
"""
import pickle, sys, collections
import numpy as np

D = pickle.load(open(sys.argv[1] if len(sys.argv) > 1 else "d.pkl", "rb"))
gw = D["global_wpnts"][0][1]
L = gw[-1, 0] + (gw[1, 0] - gw[0, 0])
fo = D["/car_state/frenet/odom"]
dur = fo[-1, 0] - fo[0, 0]
print(f"길이 {dur:.0f} s, track_len {L:.2f} m")

# ── 랩타임 ────────────────────────────────────────────────────────────────
s, t = fo[:, 1], fo[:, 0]
wrap = np.where(np.diff(s) < -L * 0.5)[0]
lt = np.diff(t[wrap]) if len(wrap) > 1 else np.array([])
if len(lt):
    print(f"랩 {len(lt)}회: 최속 {lt.min():.2f} p50 {np.median(lt):.2f} s")

# ── 통과 여유 (|Δs|<0.45 순간의 차체 횡여유) ───────────────────────────────
def ego_at(tt):
    i = np.searchsorted(fo[:, 0], tt); i = min(max(i, 0), len(fo) - 1)
    return fo[i, 1], fo[i, 2], fo[i, 4]

tr = collections.defaultdict(list)
for tt, obs in D["obs_frenet"]:
    es, ed, ev = ego_at(tt)
    for x in obs:
        ds = (x["s_center"] - es) % L
        if ds > L / 2: ds -= L
        tr[x["id"]].append((tt, ds, x["d_center"], ed, x["d_right"], x["d_left"],
                            x["is_static"], ev))

ev_rows = []
for i, v in tr.items():
    a = np.array(v)
    if len(a) < 20: continue
    m = np.abs(a[:, 1]) < 0.45
    if m.sum() < 2: continue
    lat = np.abs(a[m, 2] - a[m, 3])
    half = (a[m, 5] - a[m, 4]) / 2.0
    ev_rows.append((1 - a[:, 6].mean(), (lat - half - 0.15).min()))
if ev_rows:
    E = np.array(ev_rows)
    for lo, hi, nm, ref in ((0.5, 1.1, "동적", (0.11, 43)), (-0.1, 0.5, "정적", (0.31, 8))):
        m = (E[:, 0] > lo) & (E[:, 0] <= hi)
        if not m.sum(): continue
        sl = E[m, 1]
        print(f"{nm} 통과 {int(m.sum()):3d}회: 차체여유 중앙 {np.median(sl):+.2f} m "
              f"(기준 {ref[0]:+.2f}) / 접촉(<=0) {int((sl <= 0).sum())}회 "
              f"= {(sl <= 0).mean() * 100:.0f}% (기준 {ref[1]}%)")

# ── 속도비 / 봉쇄 체류 ────────────────────────────────────────────────────
vref = np.interp(fo[:, 1] % L, gw[:, 0], gw[:, 8])
near = np.zeros(len(fo), bool)
for tt, obs in D["obs_frenet"]:
    if not obs: continue
    es, _, _ = ego_at(tt)
    if any(0 < (x["s_start"] - es) % L < 5.0 for x in obs):
        k = np.searchsorted(fo[:, 0], tt)
        if k < len(near): near[k] = True
free = ~near
if free.sum():
    print(f"자유주행 속도/레이스라인 {np.median(fo[free, 4] / vref[free]) * 100:.0f}% (기준 85%)")
if near.sum():
    print(f"장애물 5m 이내 체류 {near.mean() * 100:.0f}%, 그때 속도비 "
          f"{np.median(fo[near, 4] / vref[near]) * 100:.0f}%")

# ── 기각 카운터 (증분!) ───────────────────────────────────────────────────
print("\n주요 카운터 (증분, 초당):")
for k in ("reject_lane_wall_no_room", "reject_lane_geom_floor", "outcome_trailing",
          "outcome_ok_lane_switch", "outcome_blocked_both_sides",
          "outcome_trailing_stop_denied_ever_dynamic", "outcome_escape_attempted"):
    a = D.get("/pitwall/" + k)
    if a is None or len(a) < 2: continue
    inc = a[-1, 1] - a[0, 1]
    print(f"  {k:46s} {inc:7.0f}  ({inc / dur:5.2f}/s)")
