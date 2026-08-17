#!/usr/bin/env python3
"""횡추종오차 모델: |err| 을 속도 v 와 계획 횡변화율 |dd/ds| 로 회귀.
   RequiredBoundaryMargin 의 tracking 항을 데이터로 정하기 위한 것."""
import pickle
from pathlib import Path

import numpy as np

P = Path(__file__).parent
D = pickle.load(open(P / "data.pkl", "rb"))
fre = D["/car_state/frenet/odom"]
L = 38.51

rows = []
for (t, w) in D["local_wpnts"]:
    if len(w) < 30:
        continue
    cs = np.interp(t, fre[:, 0], fre[:, 1])
    cd = np.interp(t, fre[:, 0], fre[:, 2])
    cv = np.interp(t, fre[:, 0], fre[:, 4])
    rel = (w[:, 0] - cs + L / 2) % L - L / 2
    k = np.abs(rel).argmin()
    d_plan = w[k, 1]
    # 앞으로 2m 구간의 계획 횡변화율 |dd/ds|
    m = (rel > 0) & (rel < 2.0)
    slope = 0.0
    if m.sum() > 3:
        slope = abs(np.polyfit(rel[m], w[m, 1], 1)[0])
    kap = abs(w[k, 7])
    rows.append((t, cs, cd, cv, d_plan, cd - d_plan, slope, kap))
R = np.array(rows)
t, cs, cd, cv, dp, err, slope, kap = R.T

# 충돌/정지 구간 제외
good = (cv > 0.5) & ~(((t > 105.2) & (t < 108.5)) | (t > 110.9)) & (t > 1.5)
print(f"표본 {good.sum()} / {len(R)} (충돌·정지 제외)\n")

print("=== |횡오차| : 속도 x 계획 횡변화율 ===")
vb = [(0.5, 2), (2, 3), (3, 4), (4, 5), (5, 9)]
sb = [(0, 0.05), (0.05, 0.15), (0.15, 0.30), (0.30, 9)]
hdr = "v \\ |dd/ds|"
print(f"{hdr:12s}" + "".join(f"{a:.2f}-{b:.2f}   " for a, b in sb))
for v0, v1 in vb:
    line = f"{v0}-{v1} m/s   "
    for s0, s1 in sb:
        m = good & (cv >= v0) & (cv < v1) & (slope >= s0) & (slope < s1)
        line += (f"{np.abs(err[m]).mean():.3f}/{np.percentile(np.abs(err[m]),95):.3f}"
                 f"({m.sum():4d}) " if m.sum() > 20 else f"{'-':>19s} ")
    print(line)
print("  (셀 = 평균 / p95 (표본수))")

print("\n=== 선형 모델 |err| ≈ a + b·v + c·|dd/ds|·v ===")
m = good
X = np.column_stack([np.ones(m.sum()), cv[m], slope[m] * cv[m]])
y = np.abs(err[m])
beta, *_ = np.linalg.lstsq(X, y, rcond=None)
pred = X @ beta
print(f"  a={beta[0]:+.4f}  b={beta[1]:+.4f} [s]  c={beta[2]:+.4f} [s]")
print(f"  R²={1-((y-pred)**2).sum()/((y-y.mean())**2).sum():.3f}   "
      f"잔차 p95={np.percentile(y-pred,95):+.3f}")
print("  → p95 포락선 (계수 x 2.4 근사):")
for v in (2, 3, 4, 5, 6):
    for sl in (0.0, 0.15, 0.30):
        pass
    print(f"    v={v}: 직진 {beta[0]+beta[1]*v:.3f}  "
          f"|dd/ds|=0.15 {beta[0]+beta[1]*v+beta[2]*0.15*v:.3f}  "
          f"|dd/ds|=0.30 {beta[0]+beta[1]*v+beta[2]*0.30*v:.3f}")

print("\n=== 현행 예산(0.10m) 대비 실측 초과 ===")
for v0, v1 in vb:
    m = good & (cv >= v0) & (cv < v1)
    if m.sum() < 20:
        continue
    e = np.abs(err[m])
    print(f"v {v0}-{v1}: p50 {np.percentile(e,50):.3f}  p90 {np.percentile(e,90):.3f} "
          f" p95 {np.percentile(e,95):.3f}  p99 {np.percentile(e,99):.3f}  "
          f"0.10 초과 비율 {100*(e>0.10).mean():.0f}%")

# 벽 쪽으로 나가는 오차만 (안쪽으로 파고드는 것은 벽 위험이 아님)
out = np.where(dp >= 0, err, -err)      # +면 계획보다 벽 쪽(바깥)으로
print("\n=== 벽 방향(오버슈트) 성분만 ===")
for v0, v1 in vb:
    m = good & (cv >= v0) & (cv < v1) & (np.abs(dp) > 0.15)
    if m.sum() < 20:
        continue
    o = out[m]
    print(f"v {v0}-{v1} (|d_plan|>0.15, n={m.sum():4d}): "
          f"p50 {np.percentile(o,50):+.3f}  p90 {np.percentile(o,90):+.3f}  "
          f"p95 {np.percentile(o,95):+.3f}  max {o.max():+.3f}")
