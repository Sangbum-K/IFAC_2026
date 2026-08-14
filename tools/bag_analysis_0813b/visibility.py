#!/usr/bin/env python3
"""콘별·랩별 인지 가시성: ego_s 어디서 보였나 / 안 보였나."""
import pickle, numpy as np
D=pickle.load(open("data.pkl","rb")); TL=32.07
fre=D["/car_state/frenet/odom"]; t_e,s_e=fre[:,0],fre[:,1]
wrap=np.where(np.diff(s_e)<-TL*0.5)[0]; lapb=np.concatenate([[t_e[0]],t_e[wrap],[t_e[-1]]])
CONE={"O2 네모코너":6.63,"O3 상단":10.45,"O4 대각":14.99,"O6 헤어핀":22.11,"O7 직선":28.86}
for nm,so in CONE.items():
    print(f"\n### {nm} s={so}")
    print(" lap | 인지된 ego_s 구간(콘까지 남은거리 m) → 마지막 인지 시점의 잔여거리 | 총 인지 사이클")
    for li in range(len(lapb)-1):
        t0,t1=lapb[li],lapb[li+1]
        seen=[]
        for t,obs in D["obs_frenet"]:
            if not (t0<=t<t1): continue
            e=np.interp(t,t_e,s_e); gap=(so-e)%TL
            if gap>9: continue
            if any(abs(((o["s_center"]-so+TL/2)%TL)-TL/2)<1.2 for o in obs):
                seen.append(gap)
        if not seen:
            print(f" {li:3d} | ─── 한 번도 인지되지 않음 ───"); continue
        seen=np.array(seen)
        # 구간화
        seen_sorted=np.sort(seen)[::-1]
        segs=[]; a=seen_sorted[0]; p=a
        for g in seen_sorted[1:]:
            if p-g>0.45: segs.append((a,p)); a=g
            p=g
        segs.append((a,p))
        ss=", ".join(f"{x:.1f}→{y:.1f}" for x,y in segs)
        print(f" {li:3d} | {ss:60s} | 마지막 {seen.min():.2f}m | n={len(seen)}")
