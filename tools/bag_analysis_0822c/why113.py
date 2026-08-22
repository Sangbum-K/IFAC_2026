#!/usr/bin/env python3
"""case3의 1.13s가 어느 필터에서 나갔나: 벽카빙(0.25) vs track_filter(8.0) vs 점수(8)."""
import pickle, numpy as np
from scipy.ndimage import distance_transform_edt

D = pickle.load(open("d26.pkl","rb"))
M = D["map"]; gw = D["global_wpnts"][0][1]; pf = D["/pf/pose/odom"]
fo = D["/car_state/frenet/odom"]; scan = D["scan"]
grid = M["data"].reshape(M["h"], M["w"])
occ = grid >= 50
edt = distance_transform_edt(~occ) * M["res"]          # 각 셀 -> 최근접 벽 거리
wx, wy, ws = gw[:,2], gw[:,3], gw[:,0]

def wall_dist(px, py):
    ix = np.clip(((px - M["ox"]) / M["res"]).astype(int), 0, M["w"]-1)
    iy = np.clip(((py - M["oy"]) / M["res"]).astype(int), 0, M["h"]-1)
    return edt[iy, ix]

def frenet(px, py):
    d2 = (wx[None,:]-px[:,None])**2 + (wy[None,:]-py[:,None])**2
    k = np.argmin(d2, axis=1)
    return ws[k], -np.sin(gw[k,6])*(px-wx[k]) + np.cos(gw[k,6])*(py-wy[k]), k

import sys
SLO, SHI, PILE_S = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
T0, T1 = float(sys.argv[1]), float(sys.argv[1]) + 1.7
print(f"{'t':>7} {'ego_s':>6} {'s갭':>5} {'박스점':>6} {'카빙후':>6} {'뷰내':>5}  판정")
for tt, amin, ainc, rr in scan:
    if not (T0 <= tt <= T1):
        continue
    j = np.clip(np.searchsorted(pf[:,0], tt), 1, len(pf)-1)
    x0, y0, yaw = pf[j,1], pf[j,2], pf[j,3]
    i = np.clip(np.searchsorted(fo[:,0], tt), 0, len(fo)-1)
    es = fo[i,1]
    ang = amin + np.arange(len(rr))*ainc*4 + yaw
    m = np.isfinite(rr) & (rr > 0.05) & (rr < 12)
    px, py = x0 + rr[m]*np.cos(ang[m]), y0 + rr[m]*np.sin(ang[m])
    s, d, k = frenet(px, py)
    box = (s >= SLO) & (s <= SHI) & (np.abs(d) < 1.1)
    if box.sum() == 0:
        continue
    wd = wall_dist(px[box], py[box])
    carved = wd > 0.25                                   # 벽카빙 생존
    dl, dr = gw[k[box],5], gw[k[box],4]                  # d_left/d_right room
    inb = (d[box] < dl - 0.10) & (d[box] > -(dr - 0.10)) # track_filter 횡
    sgap = (PILE_S - es) % 34.44
    inview = sgap < 8.0
    surv = int((carved & inb).sum())
    # 4x 데시메이션이므로 실제 점수 ~= x4
    verdict = []
    if not inview: verdict.append("track_filter(뷰 8m 초과)")
    if surv*4 < 8: verdict.append("점수부족")
    if box.sum() and carved.sum() < box.sum()*0.5: verdict.append(f"카빙 {box.sum()-carved.sum()}/{box.sum()} 삭제")
    print(f"{tt:7.2f} {es:6.2f} {sgap:5.2f} {box.sum():6d} {int(carved.sum()):6d} {surv:5d}  {'; '.join(verdict) if verdict else 'OK(발행 가능)'}")
