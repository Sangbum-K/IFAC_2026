#!/usr/bin/env python3
"""회피 에피소드: 장애물 획득 → 판단 전이 → 실제 횡이동/속도 결과."""
import pickle, numpy as np
from collections import Counter
D=pickle.load(open("data.pkl","rb")); TR=pickle.load(open("trace.pkl","rb"))
TL=32.07
fre=D["/car_state/frenet/odom"]; t_e,s_e,d_e,v_e=fre[:,0],fre[:,1],fre[:,2],fre[:,4]
_,gb=D["global_wpnts"][0]; gs,gvx=gb[:,0],gb[:,8]
ot=D["otwpnts"]; ot_t=np.array([t for t,_ in ot]); ot_n=np.array([len(a) for _,a in ot])
CONE={"O2":6.63,"O3":10.45,"O4":14.99,"O6":22.11,"O7":28.86}
wrap=np.where(np.diff(s_e)<-TL*0.5)[0]; lapb=np.concatenate([[t_e[0]],t_e[wrap],[t_e[-1]]])
def win(a,t0,t1,c=1):
    m=(a[:,0]>=t0)&(a[:,0]<=t1); return a[m]
print("에피소드 = 랩 x 콘. mode 시퀀스 / 최초 회피시도 거리 / 실제 도달 d / 속도")
for nm,so in CONE.items():
    print(f"\n{'='*118}\n### {nm} (s={so})  계획속도 {np.interp(so,gs,gvx)*0.96:.2f} m/s\n{'='*118}")
    print(" lap  획득s   최초lane시도 최초성공  통과d   여유   v_min  v@obs  v_plan  손실s  mode시퀀스")
    for li in range(len(lapb)-1):
        t0,t1=lapb[li],lapb[li+1]
        tw=[x for x in TR if t0<=x["t"]<t1 and x["dist"]>=0
            and abs(((x["s"]+x["dist"]-so+TL/2)%TL)-TL/2)<1.0]
        if len(tw)<3: continue
        seq=[]; prev=None
        for x in tw:
            if x["mode"]!=prev: seq.append((x["mode"],round(x["dist"],1))); prev=x["mode"]
        acq=tw[0]["dist"]
        lane_try=[x for x in tw if "lane_wall_no_room" not in x["rej"] and
                  any(r.startswith("lane_") or r=="ok_lane_switch" for r in x["rej"])]
        first_ok=[x for x in tw if x["mode"] in ("lane_switch","free_spline")]
        m=(t_e>=t0)&(t_e<t1)
        ss,dd,vv=s_e[m],d_e[m],v_e[m]
        j=np.argmin(np.abs(ss-so))
        wnear=np.abs(ss-so)<2.0
        vmin=vv[wnear].min() if wnear.sum() else np.nan
        vplan=np.interp(so,gs,gvx)*0.96
        # 시간손실: obs 앞뒤 4m 구간
        w4=np.abs(ss-so)<4.0
        if w4.sum()>3:
            vpl=np.interp(ss[w4],gs,gvx)*0.96
            dt=np.gradient(t_e[m])[w4]; ds=vv[w4]*dt
            lost=np.sum(ds*(1/np.maximum(vv[w4],0.2)-1/np.maximum(vpl,0.2)))
        else: lost=np.nan
        sq=" > ".join(f"{a}@{b:.1f}" for a,b in seq[:6])
        print(f" {li:3d} {acq:6.2f} {'':>12s} "
              f"{(first_ok[0]['dist'] if first_ok else np.nan):8.2f} {dd[j]:+7.2f} "
              f"{'':5s} {vmin:6.2f} {vv[j]:6.2f} {vplan:6.2f} {lost:6.2f}  {sq}")
