#!/usr/bin/env python3
"""핵심 질문: 플래너가 '발행한 경로 자체'가 벽 여유를 만족하는가?
   그리고 실제 주행은 얼마나 소비했는가?  (지도 기준 + 스캔 기준)
   margin = 벽거리 - |d| - 차반폭   (0 이면 차체가 벽에 닿음)
"""
import pickle
from pathlib import Path

import numpy as np

P = Path(__file__).parent
D = pickle.load(open(P / "data.pkl", "rb"))
S = np.load(P / "scanclear.npy")
gw = D["global_wpnts"][0][1]
gs = gw[:, 0]
A_l, A_r = np.abs(gw[:, 5]), np.abs(gw[:, 4])
ds = gs[1] - gs[0]
L = gs[-1] + ds
fre = D["/car_state/frenet/odom"]
HALF_W = 0.15
BUDGET = 0.10          # boundary_tracking_margin_m (real_car.yaml)


def widx(s_):
    return (np.round((np.asarray(s_) % L) / ds).astype(int)) % len(gw)


# ---------- 1) 발행된 경로의 자체 여유 ----------
print("=== 1) 발행 경로(local_waypoints)의 계획상 벽 여유 ===")
worst = []
for t, w in D["local_wpnts"]:
    if len(w) < 2:
        continue
    i = widx(w[:, 0])
    wall = np.where(w[:, 1] >= 0, A_l[i], A_r[i])
    mar = wall - np.abs(w[:, 1]) - HALF_W
    k = mar.argmin()
    worst.append((t, mar[k], w[k, 0], w[k, 1]))
worst = np.array(worst)
print(f"n={len(worst)}  min {worst[:,1].min():+.3f} @t={worst[worst[:,1].argmin(),0]:.1f} "
      f"s={worst[worst[:,1].argmin(),2]:.1f} d={worst[worst[:,1].argmin(),3]:+.2f}")
for q in (1, 5, 25, 50):
    print(f"  p{q:02d} = {np.percentile(worst[:,1], q):+.3f}")
bad = worst[worst[:, 1] < BUDGET]
print(f"  계획 여유 < 예산 0.10m 인 프레임: {len(bad)} / {len(worst)} "
      f"({100*len(bad)/len(worst):.1f}%)")
bad2 = worst[worst[:, 1] < 0]
print(f"  계획 자체가 벽 침범(<0): {len(bad2)} 프레임")
if len(bad2):
    b = bad2[np.argsort(bad2[:, 1])][:10]
    for r in b:
        print(f"    t={r[0]:.2f} s={r[2]:5.1f} d={r[3]:+.2f} margin={r[1]:+.3f}")

# ---------- 2) 실제 주행의 여유 ----------
print("\n=== 2) 실제 주행 여유 (지도 기준 / 스캔 기준) ===")
i = widx(fre[:, 1])
wall = np.where(fre[:, 2] >= 0, A_l[i], A_r[i])
mar_act = wall - np.abs(fre[:, 2]) - HALF_W
scan_side = np.where(fre[:, 2] >= 0,
                     np.interp(fre[:, 0], S[:, 0], S[:, 1]),
                     np.interp(fre[:, 0], S[:, 0], S[:, 2])) - HALF_W
for lbl, v in (("지도", mar_act), ("스캔", scan_side)):
    print(f"{lbl}: min {np.nanmin(v):+.3f}  p01 {np.nanpercentile(v,1):+.3f}  "
          f"p05 {np.nanpercentile(v,5):+.3f}  p50 {np.nanpercentile(v,50):+.3f}")
print(f"지도여유 < 0 인 시간: {(mar_act<0).sum()/len(mar_act)*100:.1f}%  "
      f"< 0.10 인 시간: {(mar_act<0.10).sum()/len(mar_act)*100:.1f}%")

# ---------- 3) s 구간별 최악 여유 (랩별) ----------
print("\n=== 3) s 1m 구간별 실제 최소 여유(지도) — 위험 구간 ===")
print("  s   | 랩별 최소여유 ...                                    | 전체최소")
for sb in range(0, int(L) + 1):
    m = (fre[:, 1] >= sb) & (fre[:, 1] < sb + 1)
    if m.sum() < 5:
        continue
    v = mar_act[m]
    if v.min() > 0.25:
        continue
    tt = fre[m, 0]
    # 랩 단위로 쪼개기
    cuts = np.where(np.diff(tt) > 2.0)[0] + 1
    per = [g.min() for g in np.split(v, cuts) if len(g)]
    print(f" {sb:3d}  | " + " ".join(f"{x:+5.2f}" for x in per) +
          f"   | {v.min():+.2f}")
