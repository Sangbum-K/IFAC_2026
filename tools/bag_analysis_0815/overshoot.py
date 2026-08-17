#!/usr/bin/env python3
"""회피 기동별 계획 대비 실제 횡오차(오버슈트) 정량화 + 실제 벽 여유 소비량."""
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

# 계획 경로: local_waypoints 를 (s -> d) 로 보고, 차량의 현재 s 에서의 계획 d
plan = []          # (t, d_plan_at_car_s, v_plan)
for (t, w) in D["local_wpnts"]:
    if len(w) < 2:
        continue
    cs = np.interp(t, fre[:, 0], fre[:, 1])
    rel = (w[:, 0] - cs + L / 2) % L - L / 2
    k = np.abs(rel).argmin()
    plan.append((t, w[k, 1], w[k, 8]))
plan = np.array(plan)

t = plan[:, 0]
d_plan = plan[:, 1]
d_act = np.interp(t, fre[:, 0], fre[:, 2])
s_act = np.interp(t, fre[:, 0], fre[:, 1])
v_act = np.interp(t, fre[:, 0], fre[:, 4])
err = d_act - d_plan

print("=== 계획 경로 대비 횡추종오차 (전 구간) ===")
for lo, hi, lbl in ((0, 99, "전체"), (0, 2.0, "저속<2m/s"),
                    (2.0, 4.0, "2~4m/s"), (4.0, 99, ">4m/s")):
    m = (v_act >= lo) & (v_act < hi)
    if m.sum() < 10:
        continue
    print(f"{lbl:10s} n={m.sum():5d}  |err| mean {np.abs(err[m]).mean():.3f}  "
          f"p50 {np.percentile(np.abs(err[m]),50):.3f}  "
          f"p95 {np.percentile(np.abs(err[m]),95):.3f}  "
          f"max {np.abs(err[m]).max():.3f}")

# --- 회피 기동 구간: |d_plan| > 0.25 가 0.3s 이상 지속 ---
print("\n=== 회피 기동별 (|d_plan|>0.25) ===")
act = np.abs(d_plan) > 0.25
i = 0
rows = []
while i < len(act):
    if act[i]:
        j = i
        while j + 1 < len(act) and (act[j + 1] or act[j:j + 6].any()):
            j += 1
        if t[j] - t[i] > 0.4:
            sl = slice(i, j + 1)
            side = "L" if d_plan[sl].mean() > 0 else "R"
            k = np.abs(d_plan[sl]).argmax()
            ka = np.abs(d_act[sl]).argmax()
            # 실제 벽 여유 최소값 (스캔)
            m2 = (S[:, 0] >= t[i]) & (S[:, 0] <= t[j])
            sc = np.nanmin(S[m2, 1 if side == "L" else 2]) if m2.any() else np.nan
            # 지도 기준 여유: 벽 - |d| - 차반폭
            idxs = (np.round((s_act[sl] % L) / ds).astype(int)) % len(gw)
            wallv = (A_l if side == "L" else A_r)[idxs]
            mapmar = (wallv - np.abs(d_act[sl]) - HALF_W).min()
            mapmar_plan = (wallv - np.abs(d_plan[sl]) - HALF_W).min()
            rows.append(dict(
                t0=t[i], t1=t[j], s0=s_act[i], s1=s_act[j], side=side,
                apex_plan=d_plan[sl][k], apex_act=d_act[sl][ka],
                over=abs(d_act[sl][ka]) - abs(d_plan[sl][k]),
                v=v_act[sl].mean(), vmax=v_act[sl].max(),
                scan=sc, mapmar=mapmar, mapmar_plan=mapmar_plan))
        i = j + 1
    else:
        i += 1

print("  t0     t1    s0    s1  side apex계획 apex실제 오버슈트  v평균 "
      "지도여유(계획/실제) 스캔최소")
for r in rows:
    flag = ""
    if r["scan"] < 0.15:
        flag = "  ***접촉"
    elif r["scan"] < 0.25:
        flag = "  **아슬"
    print(f"{r['t0']:6.1f} {r['t1']:6.1f} {r['s0']:5.1f} {r['s1']:5.1f}  {r['side']}  "
          f"{r['apex_plan']:+6.2f}  {r['apex_act']:+6.2f}  {r['over']:+6.2f}   "
          f"{r['v']:4.2f}   {r['mapmar_plan']:+5.2f} / {r['mapmar']:+5.2f}    "
          f"{r['scan']:5.2f}{flag}")

ov = np.array([r["over"] for r in rows])
print(f"\n오버슈트: n={len(ov)}  mean {ov.mean():+.3f}  p50 {np.median(ov):+.3f}  "
      f"p90 {np.percentile(ov,90):+.3f}  max {ov.max():+.3f}")
print(f"계획 지도여유 최소: {min(r['mapmar_plan'] for r in rows):+.3f}  "
      f"실제 지도여유 최소: {min(r['mapmar'] for r in rows):+.3f}")
lowv = [r for r in rows if r["v"] < 2.5]
hiv = [r for r in rows if r["v"] >= 2.5]
for lbl, g in (("v<2.5", lowv), ("v>=2.5", hiv)):
    if g:
        o = np.array([r["over"] for r in g])
        print(f"  {lbl}: n={len(g)} 오버슈트 mean {o.mean():+.3f} max {o.max():+.3f}")
