#!/usr/bin/env python3
"""레이 일관성 필터 A/B — C++ 구현(ray_consistency_mask)을 그대로 옮겨 백에 적용.

확인할 것
  1) 넘김 빔이 실제로 줄어드는가
  2) **실물 콘이 살아남는가** (이게 통과 못 하면 필터를 넣으면 안 된다)
  3) 안정성 게이트가 얼마나 보류시키는가
  4) 연산 비용
"""
import pickle
import time
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt

D = pickle.load(open(Path(__file__).parent / "data.pkl", "rb"))
pf = D["/pf/pose/odom"]
fre = D["/car_state/frenet/odom"]
M = D["map"]
g = M["data"].reshape(M["h"], M["w"])
occ = g > 50
RES, OX, OY, W, H = M["res"], M["ox"], M["oy"], M["w"], M["h"]
EDT = (distance_transform_edt(~occ) * RES).astype(np.float64)
XL = 0.28

# real_car.yaml 값
MARGIN = 1.5
PROBE = np.radians(1.5)
UNSTABLE = 0.5
RANGE_MAX = 12.0
MAX_RANGE = RANGE_MAX + MARGIN
HIT_EPS = RES
MAX_STEPS = int(MAX_RANGE / RES) + 8

# 실물 지속 트랙의 맵 좌표 (수명>=0.4s, 관측>=50). 이 근처 점은 살아야 한다.
REAL_OBS = np.array([
    (-10.92, 0.58),   # s=27.6  콘  (#47/70/100/133/148/182/211)
    (-5.50, -0.19),   # s=33.0  콘  (#31/51/83/102/136)
    (-2.23, -0.09),   # s=36.4  콘  (#196/215)
    (4.10, 4.26),     # s= 7.0  콘  (#96/144/171/199)
])


def edt_at(x, y):
    """C++ distance_at 과 같은 규칙 (origin_yaw=0). 맵 밖은 nan."""
    col = np.floor((x - OX) / RES).astype(np.int64)
    row = np.floor((y - OY) / RES).astype(np.int64)
    ok = (col >= 0) & (col < W) & (row >= 0) & (row < H)
    out = np.full(x.shape, np.nan)
    out[ok] = EDT[row[ok], col[ok]]
    return out


def raycast(x0, y0, ux, uy):
    """벡터화 sphere tracing. C++ WallDistanceMap::raycast 와 같은 규칙."""
    n = ux.shape[0]
    t = np.zeros(n)
    done = np.zeros(n, bool)
    for _ in range(MAX_STEPS):
        live = ~done & (t < MAX_RANGE)
        if not live.any():
            break
        d = edt_at(x0 + ux * t, y0 + uy * t)
        out = np.isnan(d)                      # 맵 밖 -> max_range
        t = np.where(live & out, MAX_RANGE, t)
        done |= live & out
        hit = live & ~out & (d <= HIT_EPS)
        done |= hit
        step = np.where(live & ~out & ~hit, np.maximum(d, HIT_EPS), 0.0)
        t = t + np.nan_to_num(step)
    return np.minimum(t, MAX_RANGE)


def wall_dist_mask(x, y, thr=0.25):
    d = edt_at(x, y)
    return np.isfinite(d) & (d >= thr)


tot_before = tot_after = tot_pts = 0
tot_unstable = 0
n_over_before = n_over_after = n_beam = 0
real_hits_before = real_hits_after = 0
t_cpu = 0.0
per_scan = []

SCANS = D["scan"][::2]
for ts, amin, ainc, r in SCANS:
    i = np.clip(np.searchsorted(pf[:, 0], ts), 0, len(pf) - 1)
    px, py, yaw = pf[i, 1], pf[i, 2], pf[i, 3]
    lx, ly = px + XL * np.cos(yaw), py + XL * np.sin(yaw)
    ang = amin + ainc * np.arange(len(r))
    ok = np.isfinite(r) & (r > 0.1) & (r < RANGE_MAX) & (np.abs(ang) < np.radians(80))
    if ok.sum() < 20:
        continue
    a, rr = ang[ok], r[ok].astype(float)
    xl, yl = rr * np.cos(a), rr * np.sin(a)
    xm = lx + xl * np.cos(yaw) - yl * np.sin(yaw)
    ym = ly + xl * np.sin(yaw) + yl * np.cos(yaw)

    # ②.6 벽거리 필터 먼저 (C++ 순서와 동일)
    keep_wall = wall_dist_mask(xm, ym)
    xm, ym, rr = xm[keep_wall], ym[keep_wall], rr[keep_wall]
    if len(rr) == 0:
        continue

    t0 = time.perf_counter()
    # ②.62 레이 일관성 필터
    dx, dy = xm - lx, ym - ly
    rho = np.hypot(dx, dy)
    keep = np.ones(len(rr), bool)
    cand = rho > MARGIN
    if cand.any():
        ux, uy = dx[cand] / rho[cand], dy[cand] / rho[cand]
        e0 = raycast(lx, ly, ux, uy)
        rej = (e0 < MAX_RANGE) & (rho[cand] > e0 + MARGIN)
        if rej.any():
            cp, sp = np.cos(PROBE), np.sin(PROBE)
            u1x, u1y = cp * ux[rej] - sp * uy[rej], sp * ux[rej] + cp * uy[rej]
            u2x, u2y = cp * ux[rej] + sp * uy[rej], -sp * ux[rej] + cp * uy[rej]
            e1 = raycast(lx, ly, u1x, u1y)
            e2 = raycast(lx, ly, u2x, u2y)
            spread = np.maximum.reduce([e0[rej], e1, e2]) - np.minimum.reduce([e0[rej], e1, e2])
            stable = spread <= UNSTABLE
            tot_unstable += int((~stable).sum())
            drop_idx = np.where(cand)[0][np.where(rej)[0][stable]]
            keep[drop_idx] = False
    t_cpu += time.perf_counter() - t0

    # --- 지표 ---
    exp_all = np.full(len(rr), MAX_RANGE)
    sub = rho > MARGIN
    if sub.any():
        exp_all[sub] = raycast(lx, ly, dx[sub] / rho[sub], dy[sub] / rho[sub])
    over = (exp_all < MAX_RANGE) & (rho > exp_all + 0.40)
    n_beam += len(rr)
    n_over_before += int(over.sum())
    n_over_after += int((over & keep).sum())
    tot_before += len(rr)
    tot_after += int(keep.sum())

    # 실물 콘 근처(반경 0.35m) 점이 살아남는가
    for ox_, oy_ in REAL_OBS:
        near = np.hypot(xm - ox_, ym - oy_) < 0.35
        real_hits_before += int(near.sum())
        real_hits_after += int((near & keep).sum())
    per_scan.append((ts, len(rr), int((~keep).sum())))

P = np.array(per_scan)
print("=" * 66)
print("레이 일관성 필터 A/B  (margin=1.5 probe=1.5deg unstable=0.5)")
print("=" * 66)
print(f"스캔 {len(P)}개 · 벽필터 통과 포인트 {tot_before}")
print()
print("1) 넘김 빔 (맵 벽보다 0.4m 이상 뒤)")
print(f"   before {n_over_before:7d} / {n_beam} = {100*n_over_before/n_beam:5.2f}%")
print(f"   after  {n_over_after:7d} / {n_beam} = {100*n_over_after/n_beam:5.2f}%"
      f"   -> {100*(1-n_over_after/max(n_over_before,1)):.1f}% 제거")
print()
print("2) 실물 콘 4개 근처(반경 0.35m) 포인트 생존")
print(f"   before {real_hits_before}  after {real_hits_after}  "
      f"-> 생존율 {100*real_hits_after/max(real_hits_before,1):.2f}%")
if real_hits_after < real_hits_before:
    print(f"   !! 실물 포인트 {real_hits_before-real_hits_after}개가 버려졌다 — 확인 필요")
else:
    print("   OK — 실물은 한 점도 버려지지 않았다")
print()
print("3) 전체 포인트")
print(f"   {tot_before} -> {tot_after}  ({100*(tot_before-tot_after)/tot_before:.2f}% 기각)")
print(f"   안정성 게이트로 판단 보류한 빔 {tot_unstable}개 "
      f"({100*tot_unstable/max(tot_before,1):.2f}%)")
print()
print("4) 연산 비용")
print(f"   필터 총 {1000*t_cpu:.0f} ms / {len(P)} 스캔 = {1000*t_cpu/len(P):.2f} ms/스캔")
print("   (파이썬 벡터화 기준. C++ 는 이보다 빠르다 — 상한으로 읽을 것)")
print()
print("5) 스캔당 기각 분포")
dr = P[:, 2]
print(f"   0개 기각한 스캔 {100*(dr==0).mean():.1f}%   med {np.median(dr):.0f}  "
      f"p90 {np.percentile(dr,90):.0f}  max {dr.max():.0f}")
print()
print("6) 사용자 지목 시각")
for tq in (33.6, 34.3, 46.5, 60.5, 105.0):
    j = np.argmin(np.abs(P[:, 0] - tq))
    print(f"   t={P[j,0]:6.2f}  포인트 {int(P[j,1]):3d} 중 {int(P[j,2]):3d} 기각")
