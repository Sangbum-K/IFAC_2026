#!/usr/bin/env python3
"""정량화: 사이드락 오발화율, 회피 성공률, 트레일링 감속 실패, 반사실 계산."""
import pathlib
import pickle
import re

import numpy as np

SP = str(pathlib.Path(__file__).parent) + "/"
D = pickle.load(open(SP + "data.pkl", "rb"))
L = 0.33
MAX_STEER_PLAN = 0.3578          # spliner/vehicle_max_steering_rad
MECH = 0.3677
GEOM_MARGIN = 0.92
KAPPA_RATIO = 0.30
ESC_ALAT = 5.5
BLEND_ALAT = 5.0
SIDE_TRESH = 0.20
KMAX = np.tan(MAX_STEER_PLAN) / L


def sec(title):
    print("\n" + "=" * 78)
    print(title)
    print("=" * 78)


# ---------------------------------------------------------------- 1) 사이드락
sec("1) side_switch_d_tresh=0.20 오발화 — |d|는 장애물 없이도 0.20을 넘는가")
fren = D["/car_state/frenet/odom"]        # t, s, d, yaw, v
obs_t = np.array([t for t, o in D["obs_frenet"]])
obs_n = np.array([len(o) for t, o in D["obs_frenet"]])
# 장애물 0개인 시각 (±0.3s 이내 모두 0)
d_all = np.abs(fren[:, 2])
nobs_i = np.interp(fren[:, 0], obs_t, obs_n) if len(obs_t) else np.zeros(len(fren))
free = nobs_i < 0.01
print(f"전체 {len(fren)} 샘플, 장애물 0개 구간 {free.sum()} ({100*free.mean():.0f}%)")
for nm, m in (("장애물 없음", free), ("전체", np.ones(len(fren), bool))):
    if m.sum() < 50:
        continue
    dd = d_all[m]
    print(f"  {nm}: |d| p50 {np.percentile(dd,50):.3f}  p90 {np.percentile(dd,90):.3f}  "
          f"p95 {np.percentile(dd,95):.3f}  p99 {np.percentile(dd,99):.3f}  max {dd.max():.3f}")
    print(f"          |d|>0.20 인 시간 비율 = {100*np.mean(dd > SIDE_TRESH):.1f}%")

# ---------------------------------------------------------------- 2) 성공률
sec("2) 회피 판단 결과 집계 (trace 1725 사이클)")
TR = re.compile(r"\[trace\].*\| (\w+) \| (.*)$")
modes = {}
for t, lvl, n, msg in D["rosout"]:
    m = TR.search(msg)
    if not m:
        continue
    modes[m.group(1)] = modes.get(m.group(1), 0) + 1
tot = sum(modes.values())
for k, v in sorted(modes.items(), key=lambda x: -x[1]):
    print(f"  {k:<16} {v:5d}  {100*v/tot:5.1f}%")
print(f"\n  실제 회피 경로 생성(lane_switch+free_spline) = "
      f"{modes.get('lane_switch',0)+modes.get('free_spline',0)}"
      f" / 감속만(trailing) = {modes.get('trailing',0)}")

sec("3) '다른 쪽은 성립하는데 거부' — 사이드락이 삼킨 회피 횟수")
pat_fb = re.compile(r"OT side (\w+) infeasible \(d_apex=([-\d.]+)\) -> (\w+) 폴백 \(성립\)")
pat_rej = re.compile(r"OT side 전환 거부: 이미 (\w+)로 회피 중 \(cur_d=([-\d.]+)\)")
fb, rj = [], []
for t, lvl, n, msg in D["rosout"]:
    m = pat_fb.search(msg)
    if m:
        fb.append((t, m.group(1), float(m.group(2)), m.group(3)))
    m = pat_rej.search(msg)
    if m:
        rj.append((t, m.group(1), float(m.group(2))))
print(f"  '반대쪽 폴백 (성립)' 로그: {len(fb)}회")
print(f"  '사이드 전환 거부(mid-evasion)' 로그: {len(rj)}회")
print(f"  거부 시점 |cur_d| 분포: {[abs(d) for _,_,d in rj]}")
if rj:
    dd = np.array([abs(d) for _, _, d in rj])
    print(f"    -> 임계 0.20 을 넘은 폭: 중앙 {np.median(dd)-SIDE_TRESH:+.3f} m, "
          f"최대 {dd.max()-SIDE_TRESH:+.3f} m")
    print(f"    -> 0.25 로만 올려도 살아날 건수: {(dd<=0.25).sum()}/{len(dd)}")
    print(f"    -> 0.30 이면: {(dd<=0.30).sum()}/{len(dd)}")
print("\n  누적 카운터 기준:")
for k in ("side_switch_blocked_mid_evasion", "lane_side_switch_blocked_mid_evasion"):
    a = D[f"/pitwall/reject_{k}"]
    print(f"    {k}: {int(a[:,1].max())} 프레임")

# ---------------------------------------------------------------- 4) 트레일링
sec("4) 3:14 — 트레일링이 v_target=0 을 걸었는데 차는 가속했다")
ack = D["/ackermann_cmd"]
spd = D["/monitor/current_speed"]
lw = D["local_wpnts"]
print("      t     계획속도(local wpnt[0..3] vx)  ack_cmd_v   실측v")
for t0 in np.arange(194.4, 195.7, 0.1):
    # 가장 가까운 local_waypoints
    i = min(range(len(lw)), key=lambda k: abs(lw[k][0] - t0))
    arr = lw[i][1]
    vx = arr[:4, 8] if len(arr) >= 4 else np.array([np.nan])
    ai = np.searchsorted(ack[:, 0], t0)
    ai = min(ai, len(ack) - 1)
    si = np.searchsorted(spd[:, 0], t0)
    si = min(si, len(spd) - 1)
    print(f"  {t0:7.2f}   {np.round(vx,2)}        {ack[ai,2]:5.2f}     {spd[si,1]:5.2f}")

m = (ack[:, 0] >= 194.4) & (ack[:, 0] <= 195.6)
print(f"\n  구간 ack 속도명령: {ack[m,2].min():.2f} ~ {ack[m,2].max():.2f} m/s")
ms = (spd[:, 0] >= 194.4) & (spd[:, 0] <= 195.6)
print(f"  구간 실측 속도    : {spd[ms,1].min():.2f} ~ {spd[ms,1].max():.2f} m/s "
      f"(시작 {spd[ms,1][0]:.2f} -> 충돌시점 {spd[ms,1][-1]:.2f})")

# ---------------------------------------------------------------- 5) 반사실
sec("5) 반사실 — escape_arc_lat_acc 를 실측 그립으로 올리면 살아났는가")


def arc_floor(a_amp, v, alat, kappa_track=0.0):
    k_steer = GEOM_MARGIN * max(KMAX - abs(kappa_track), KAPPA_RATIO * KMAX)
    k_grip = alat / (v * v)
    k = min(k_steer, k_grip)
    return np.sqrt(2.0 * a_amp / k), k, k_steer, k_grip


print(f"  SteerKappaMax = tan({MAX_STEER_PLAN})/{L} = {KMAX:.3f} 1/m")
print(f"  (기계 한계로 하면 tan({MECH})/{L} = {np.tan(MECH)/L:.3f} 1/m, +{100*(np.tan(MECH)/np.tan(MAX_STEER_PLAN)-1):.1f}%)\n")
cases = [("2:37 (158.10s)", 0.56, 2.9, 1.30),
         ("3:14 (195.10s)", 0.53, 5.3, 1.61)]
print("  사례            횡이동  v     가용   | a_lat=5.5(현재)      a_lat=8.0        a_lat=8.8(p99)")
for nm, amp, v, avail in cases:
    row = f"  {nm:<15} {amp:.2f}m  {v:.1f}  {avail:.2f}m |"
    for alat in (5.5, 8.0, 8.8):
        f, k, ks, kg = arc_floor(amp, v, alat)
        ok = "통과" if f <= avail else "거부"
        row += f"  {f:.2f}m/k={k:.2f} {ok}"
    print(row)
print("\n  * a_lat 을 올려도 kappa_steer 상한(= 0.92*1.133 = "
      f"{GEOM_MARGIN*KMAX:.3f})에서 멈춘다 -> 그 위로는 조향각이 진짜 구속.")

# 필요한 v: 현재 a_lat 로 통과 가능한 속도
print("\n  현재 a_lat=5.5 로 각 사례를 통과시키려면 필요한 속도:")
for nm, amp, v, avail in cases:
    # arc_floor <= avail  ->  2A/k <= avail^2 -> k >= 2A/avail^2
    k_need = 2.0 * amp / (avail ** 2)
    v_need = np.sqrt(ESC_ALAT / k_need) if k_need > 0 else np.inf
    k_steer = GEOM_MARGIN * KMAX
    feasible = k_need <= k_steer
    print(f"    {nm}: kappa 필요 {k_need:.3f} -> v <= {v_need:.2f} m/s "
          f"(실제 {v:.1f}) {'조향으로도 가능' if feasible else '조향한계 초과 = 불가'}")

# ---------------------------------------------------------------- 6) 인지
sec("6) 인지 성능 — 첫 탐지 거리 (유저 말대로 좋아졌는가)")
TRD = re.compile(r"\[trace\] s=[-\d.]+ v=([-\d.]+) d=[-\d.]+ \| obs id=(\d+) dist=([-\d.]+)")
first = {}
for t, lvl, n, msg in D["rosout"]:
    m = TRD.search(msg)
    if not m:
        continue
    oid, dist, v = int(m.group(2)), float(m.group(3)), float(m.group(1))
    if oid not in first:
        first[oid] = (t, dist, v)
ds = np.array([d for _, d, _ in first.values()])
print(f"  고유 장애물 트랙 {len(first)}개, 첫 등장 거리:")
print(f"    p10 {np.percentile(ds,10):.2f}  p50 {np.percentile(ds,50):.2f}  "
      f"p90 {np.percentile(ds,90):.2f}  max {ds.max():.2f} m")
print(f"    2.0m 미만에서 처음 본 비율: {100*np.mean(ds<2.0):.0f}%")
print(f"    4.0m 이상에서 본 비율:      {100*np.mean(ds>=4.0):.0f}%")
# 필요 제동거리 대비
print("\n  참고: 회피 기하 하한(0.5m 횡이동, a_lat=5.5) 이 요구하는 조주거리")
for v in (3.0, 4.0, 5.0, 5.5):
    f, k, ks, kg = arc_floor(0.5, v, ESC_ALAT)
    print(f"    v={v}: {f:.2f} m  (a_lat=8.8 이면 {arc_floor(0.5,v,8.8)[0]:.2f} m)")
