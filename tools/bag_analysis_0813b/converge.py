#!/usr/bin/env python3
"""O2(네모코너) 장애물의 추정 (s,d) 가 접근거리에 따라 어떻게 변하나."""
import pickle, numpy as np
D=pickle.load(open("data.pkl","rb")); TL=32.07
fre=D["/car_state/frenet/odom"]
def es(t): return np.interp(t,fre[:,0],fre[:,1])
TGT={"O2 네모코너":6.6,"O6 헤어핀":22.1,"O7 직선":28.9,"O3 상단":10.45,"O4 대각":15.0}
for nm,so in TGT.items():
    rows=[]
    for t,obs in D["obs_frenet"]:
        e=es(t)
        gap=(so-e)%TL
        if gap>10: continue
        # 이 장애물 근처로 보고된 것 중 최근접
        cand=[o for o in obs if abs(((o["s_center"]-so+TL/2)%TL)-TL/2)<1.2]
        if not cand: continue
        o=min(cand,key=lambda o:abs(o["s_center"]-so))
        rows.append((gap,o["s_center"],o["d_center"],o["d_right"],o["d_left"],
                     o["size"],1 if o["is_static"] else 0,o["s_end"]-o["s_start"]))
    R=np.array(rows)
    if not len(R): continue
    print(f"\n### {nm} (진짜 s≈{so})  관측 {len(R)}")
    print("  접근거리   n    s_est    d_est   d_right  d_left   size  span_s  static%")
    for lo,hi in [(0,0.5),(0.5,1),(1,1.5),(1.5,2),(2,2.5),(2.5,3),(3,4),(4,5),(5,6),(6,8)]:
        m=(R[:,0]>=lo)&(R[:,0]<hi)
        if m.sum()<3: continue
        print(f"   {lo:.1f}-{hi:.1f}m {m.sum():5d} {np.median(R[m,1]):7.2f} "
              f"{np.median(R[m,2]):+8.2f} {np.median(R[m,3]):+8.2f} {np.median(R[m,4]):+7.2f} "
              f"{np.median(R[m,5]):6.2f} {np.median(R[m,7]):6.2f} {100*R[m,6].mean():6.0f}")
