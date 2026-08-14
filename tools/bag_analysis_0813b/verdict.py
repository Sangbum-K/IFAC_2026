#!/usr/bin/env python3
"""회피가 '설계된 판단'인지 '우연'인지 판정."""
import pickle, numpy as np
D=pickle.load(open("data.pkl","rb")); TR=pickle.load(open("trace.pkl","rb"))
TL=32.07
fre=D["/car_state/frenet/odom"]; t_e,s_e,d_e,v_e=fre[:,0],fre[:,1],fre[:,2],fre[:,4]
ot=D["otwpnts"]
wrap=np.where(np.diff(s_e)<-TL*0.5)[0]; lapb=np.concatenate([[t_e[0]],t_e[wrap],[t_e[-1]]])
CONE={"O2 네모코너":(6.63,-0.28),"O3 상단":(10.45,-0.52),"O4 대각":(14.99,+0.54),
      "O6 헤어핀":(22.11,-0.32),"O7 직선":(28.86,+0.22)}
HALF=0.15+0.15   # 콘 반경 + 차 반폭
print(f"{'콘':12s} {'랩':>3s} | {'판정':<22s} | {'회피경로':>8s} | {'통과 d':>7s} {'이격':>6s} {'판정':>6s} | v@obs")
S={}
for nm,(so,do) in CONE.items():
    S[nm]=dict(plan=0,trail=0,none=0,ok=0,graze=0,hit=0,tot=0)
    for li in range(1,len(lapb)-2):
        t0,t1=lapb[li],lapb[li+1]
        tw=[x for x in TR if t0<=x["t"]<t1 and 0<=x["dist"]<6.0
            and abs(((x["s"]+x["dist"]-so+TL/2)%TL)-TL/2)<1.0]
        m=(t_e>=t0)&(t_e<t1)
        if m.sum()<5: continue
        ss,dd,vv=s_e[m],d_e[m],v_e[m]
        j=np.argmin(np.abs(ss-so))
        if abs(ss[j]-so)>0.6: continue
        clr=abs(dd[j]-do)-HALF
        verd="OK" if clr>=0.10 else ("스침" if clr>=0.0 else "접촉")
        if not tw:
            cls="인지 없음(콘 부재/미탐지)"; S[nm]["none"]+=1; otn=0
        else:
            has=any(x["mode"] in ("lane_switch","free_spline") for x in tw)
            # 접근 중 otwpnts 발행 여부
            otn=sum(1 for t,a in ot if t0<=t<t1 and len(a)>0 and
                    abs(((np.median(a[:,0])-so+TL/2)%TL)-TL/2)<6)
            if has: cls="회피경로 생성됨(설계)"; S[nm]["plan"]+=1
            else:   cls="trailing만(감속뿐)"; S[nm]["trail"]+=1
        S[nm]["tot"]+=1; S[nm]["ok"+""]+= (verd=="OK"); S[nm]["graze"]+=(verd=="스침"); S[nm]["hit"]+=(verd=="접촉")
        print(f"{nm:12s} {li:3d} | {cls:<22s} | {otn:8d} | {dd[j]:+7.2f} {clr:+6.2f} {verd:>6s} | {vv[j]:.2f}")
print("\n=== 요약 ===")
for nm,s in S.items():
    print(f" {nm:12s} 통과 {s['tot']:2d}회 | 회피경로생성 {s['plan']:2d} / trailing만 {s['trail']:2d} / 인지없음 {s['none']:2d} "
          f"|| OK {s['ok']:2d} 스침 {s['graze']:2d} 접촉 {s['hit']:2d}")
