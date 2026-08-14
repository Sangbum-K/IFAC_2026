#!/usr/bin/env python3
"""장애물별·랩별 통과 분석: 어느 쪽으로 얼마나 비켰나, 속도는."""
import pickle, re
from pathlib import Path
import numpy as np

D = pickle.load(open("data.pkl","rb")); OB = pickle.load(open("obst.pkl","rb"))
_, gb = D["global_wpnts"][0]
gs, gdr, gdl, gvx = gb[:,0], gb[:,4], gb[:,5], gb[:,8]
TL = 32.07
fre = D["/car_state/frenet/odom"]     # t,s,d,_,v
t_e, s_e, d_e, v_e = fre[:,0], fre[:,1], fre[:,2], fre[:,4]
# 랩 경계
wrap = np.where(np.diff(s_e) < -TL*0.5)[0]
lap_t = np.concatenate([[t_e[0]], t_e[wrap], [t_e[-1]]])
print(f"랩 {len(lap_t)-1}개")

# 라인 기준 d (레이스라인은 d=0) → 회피 = |d| 가 커짐
print("\n" + "="*112)
print("장애물별 랩 통과표:  d@obs = 장애물 s 에서 ego 의 d,  Δ = 장애물 중심에서 벗어난 거리")
print("="*112)
for ob in OB:
    so, do = ob["s"], ob["d"]
    half = ob["size"]/2 + 0.15          # 장애물 반경 + 차체 반폭 여유 (차폭 0.30)
    print(f"\n### {ob['name']}  s={so:.2f} d={do:+.2f} size={ob['size']:.2f} "
          f"트랙d[{-ob['dr']:+.2f},{ob['dl']:+.2f}] v_plan={np.interp(so,gs,gvx):.2f} "
          f"static={ob['stat']:.2f}")
    print("  lap    t     d@obs    Δ(obs중심)  판정   v@obs  v_plan  v@-2m  v@+2m  min|Δ|(±1m)")
    hits = 0; tot = 0
    for li in range(len(lap_t)-1):
        m = (t_e >= lap_t[li]) & (t_e < lap_t[li+1])
        if m.sum() < 5: continue
        ss, dd, vv, tt = s_e[m], d_e[m], v_e[m], t_e[m]
        # 장애물 s 를 지나는 지점
        j = np.argmin(np.abs(ss - so))
        if abs(ss[j]-so) > 0.6: continue
        d_at = dd[j]; v_at = vv[j]; t_at = tt[j]
        # ±1m 구간 최소 이격
        w = np.abs(ss-so) < 1.0
        gap = np.min(np.abs(dd[w]-do)) if w.sum() else np.nan
        # 앞뒤 2m 속도
        j2 = np.argmin(np.abs(ss-(so-2))); j3 = np.argmin(np.abs(ss-(so+2)))
        clear = abs(d_at-do) - half
        verd = "충돌위험" if clear < 0.0 else ("스침" if clear < 0.12 else "회피OK")
        tot += 1; hits += (clear < 0.0)
        print(f"  {li:3d} {t_at:7.1f} {d_at:+7.2f}   {d_at-do:+7.2f}    {verd:6s} "
              f"{v_at:5.2f}  {np.interp(so,gs,gvx)*0.96:5.2f}  {vv[j2]:5.2f}  {vv[j3]:5.2f}   {gap:5.2f}")
    print(f"  → {tot}회 통과 중 충돌위험 {hits}회 ({100*hits/max(tot,1):.0f}%)")
