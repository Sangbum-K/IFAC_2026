#!/usr/bin/env python3
"""레이스라인을 가로지르는 대각선 점열의 정체.

핵심 판정: 그 점열이
  (a) 맵 좌표에 고정 = 실물/맵누락 구조물
  (b) 차 기준 상대위치에 고정 = 센서 아티팩트
  (c) 매번 다른 곳 = 반사/노이즈
"""
import pickle
from pathlib import Path

import numpy as np

D = pickle.load(open(Path(__file__).parent / "data.pkl", "rb"))
pf = D["/pf/pose/odom"]
fre = D["/car_state/frenet/odom"]
MAP = D["map"]
grid = MAP["data"].reshape(MAP["h"], MAP["w"])
occ = grid > 50
res, ox, oy = MAP["res"], MAP["ox"], MAP["oy"]
W, Hh = MAP["w"], MAP["h"]
XL = 0.28

# ---- 맵 벽까지 거리 (EDT, 인지 노드의 wall_dist_min_m 과 같은 기준) ----
try:
    from scipy.ndimage import distance_transform_edt
    edt = distance_transform_edt(~occ) * res
except ImportError:
    ys, xs = np.nonzero(occ)
    wall_xy = np.stack([xs * res + ox, ys * res + oy], 1)
    edt = None


def wall_dist(px, py):
    if edt is not None:
        ix = np.clip(((px - ox) / res).astype(int), 0, W - 1)
        iy = np.clip(((py - oy) / res).astype(int), 0, Hh - 1)
        return edt[iy, ix]
    d = np.full(len(px), 9.0)
    for i in range(len(px)):
        d[i] = np.hypot(wall_xy[:, 0] - px[i], wall_xy[:, 1] - py[i]).min()
    return d


def scan_at(tq):
    return min(((abs(ts - tq), ts, a, b, r) for ts, a, b, r in D["scan"]))[1:]


def pts(ts, amin, ainc, r, rmax=8.0):
    """스캔 -> 맵 좌표 점군 + 레이저 좌표."""
    i = np.clip(np.searchsorted(pf[:, 0], ts), 0, len(pf) - 1)
    px, py, yaw = pf[i, 1], pf[i, 2], pf[i, 3]
    lx, ly = px + XL * np.cos(yaw), py + XL * np.sin(yaw)
    n = len(r)
    ang = amin + ainc * np.arange(n)
    ok = np.isfinite(r) & (r > 0.1) & (r < rmax) & (np.abs(ang) < np.radians(100))
    a, rr = ang[ok], r[ok].astype(float)
    xl, yl = rr * np.cos(a), rr * np.sin(a)          # 레이저 프레임
    xm = lx + xl * np.cos(yaw) - yl * np.sin(yaw)    # 맵 프레임
    ym = ly + xl * np.sin(yaw) + yl * np.cos(yaw)
    return dict(t=ts, ego=(px, py, yaw), ang=a, r=rr,
                xl=xl, yl=yl, xm=xm, ym=ym, wd=wall_dist(xm, ym))


print("=" * 72)
print("1) t=46.5 부근: 벽에서 먼(=필터 통과) 점들의 정체")
print("=" * 72)
P = pts(*scan_at(46.5))
print(f"scan t={P['t']:.2f}  ego=({P['ego'][0]:.2f},{P['ego'][1]:.2f}) "
      f"yaw={np.degrees(P['ego'][2]):.1f}deg  유효빔 {len(P['r'])}")
surv = P["wd"] > 0.25                      # wall_dist_min_m
print(f"맵 벽에서 0.25m 초과 = 필터 통과: {surv.sum()}점 ({100*surv.mean():.1f}%)")

sel = surv
if sel.sum() >= 5:
    XY = np.stack([P["xl"][sel], P["yl"][sel]], 1)
    C = XY - XY.mean(0)
    _, sv, vt = np.linalg.svd(C, full_matrices=False)
    ext = C @ vt[0]
    resid = C @ vt[1]
    print(f"\n필터 통과 점들의 형상 (레이저 프레임):")
    print(f"  주축 길이 {ext.max()-ext.min():.2f} m   직선 잔차 RMS {np.sqrt((resid**2).mean()):.3f} m")
    print(f"  주축 방향: 차 진행방향과 {np.degrees(np.arctan2(vt[0][1], vt[0][0])):+.1f}deg")
    print(f"  거리 범위 {P['r'][sel].min():.2f} ~ {P['r'][sel].max():.2f} m")
    print(f"  방위 범위 {np.degrees(P['ang'][sel]).min():+.1f} ~ "
          f"{np.degrees(P['ang'][sel]).max():+.1f} deg")
    print(f"  맵 벽까지 거리 {P['wd'][sel].min():.2f} ~ {P['wd'][sel].max():.2f} m")
    print("\n  통과 점 목록 (방위순, 레이저 프레임 x=전방):")
    o = np.argsort(P["ang"][sel])
    aa, rr = P["ang"][sel][o], P["r"][sel][o]
    xx, yy = P["xl"][sel][o], P["yl"][sel][o]
    wd = P["wd"][sel][o]
    for k in range(len(aa)):
        print(f"    az {np.degrees(aa[k]):+7.2f}deg  r {rr[k]:5.2f}  "
              f"(x {xx[k]:+5.2f}, y {yy[k]:+5.2f})  벽거리 {wd[k]:4.2f}")

print()
print("=" * 72)
print("2) 이 점열이 맵 고정인가 차 고정인가")
print("=" * 72)
print("여러 시각에서 '필터 통과 점군'의 중심을 맵 좌표 / 레이저 좌표 양쪽으로 본다.")
print("맵 좌표가 일정 -> 실물.  레이저 좌표가 일정 -> 센서 아티팩트.\n")
print("    t   ego_s  n   맵중심(x,y)        레이저중심(x,y)   r_med  az_med  길이")
rows = []
for tq in (33.6, 34.3, 46.2, 46.5, 46.9, 58.5, 59.0, 60.5, 71.0, 82.5, 94.0, 105.5):
    P = pts(*scan_at(tq))
    s = P["wd"] > 0.25
    if s.sum() < 4:
        print(f"{P['t']:6.2f}  {np.interp(P['t'],fre[:,0],fre[:,1]):5.1f}  {s.sum():2d}   -")
        continue
    XY = np.stack([P["xl"][s], P["yl"][s]], 1)
    C = XY - XY.mean(0)
    _, _, vt = np.linalg.svd(C, full_matrices=False)
    ext = C @ vt[0]
    rows.append((P["t"], P["xm"][s].mean(), P["ym"][s].mean(),
                 P["xl"][s].mean(), P["yl"][s].mean()))
    print(f"{P['t']:6.2f}  {np.interp(P['t'],fre[:,0],fre[:,1]):5.1f}  {s.sum():2d}   "
          f"({P['xm'][s].mean():+6.2f},{P['ym'][s].mean():+6.2f})   "
          f"({P['xl'][s].mean():+5.2f},{P['yl'][s].mean():+5.2f})  "
          f"{np.median(P['r'][s]):5.2f} {np.degrees(np.median(P['ang'][s])):+6.1f} "
          f"{ext.max()-ext.min():5.2f}")
if rows:
    A = np.array(rows)
    print(f"\n  맵 좌표 중심의 산포:    std_x {A[:,1].std():.2f}  std_y {A[:,2].std():.2f} m")
    print(f"  레이저 좌표 중심의 산포: std_x {A[:,3].std():.2f}  std_y {A[:,4].std():.2f} m")
    print("  -> 작은 쪽이 '고정된' 좌표계")

print()
print("=" * 72)
print("3) 전 구간: 필터를 통과하는 점이 트랙 어디에 몰리나 (맵 좌표 히트맵)")
print("=" * 72)
acc_m = []
acc_l = []
for ts, amin, ainc, r in D["scan"][::5]:
    P = pts(ts, amin, ainc, r)
    s = P["wd"] > 0.25
    if s.sum():
        acc_m.append(np.stack([P["xm"][s], P["ym"][s]], 1))
        acc_l.append(np.stack([P["xl"][s], P["yl"][s]], 1))
M = np.concatenate(acc_m)
L = np.concatenate(acc_l)
print(f"필터 통과 점 총 {len(M)}개 (5프레임마다 샘플)")
print(f"  맵 좌표 산포:    std_x {M[:,0].std():.2f}  std_y {M[:,1].std():.2f} m")
print(f"  레이저 좌표 산포: std_x {L[:,0].std():.2f}  std_y {L[:,1].std():.2f} m")

print("\n레이저 프레임 (차 기준) 분포 — 0.5m 격자, 개수:")
xb = np.arange(-1, 6.01, 0.5)
yb = np.arange(-3, 3.01, 0.5)
H, _, _ = np.histogram2d(L[:, 0], L[:, 1], bins=[xb, yb])
print("      y(좌+)", " ".join(f"{v:+5.1f}" for v in yb[:-1]))
for i in range(len(xb) - 1):
    row = " ".join(f"{int(H[i,j]):5d}" for j in range(len(yb) - 1))
    print(f"  x {xb[i]:+5.1f}  {row}")
