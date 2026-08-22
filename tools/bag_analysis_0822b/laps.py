#!/usr/bin/env python3
"""랩 분할 + 랩별 시간/평균속도, 장애물 조우 구간 표시"""
import pickle,sys,numpy as np
D=pickle.load(open(sys.argv[1],"rb"))
fre=D["/car_state/frenet/odom"]; pf=D["/pf/pose/odom"]
gw=D["global_wpnts"][0][1]; L=gw[-1,0]
print("track_len=%.2f  gb_v: min %.2f max %.2f mean %.2f"%(L,gw[:,8].min(),gw[:,8].max(),gw[:,8].mean()))
t=fre[:,0]; s=fre[:,1]
# wrap 검출
laps=[]; start=None
for i in range(1,len(s)):
    if s[i-1]-s[i] > L*0.5:
        if start is not None: laps.append((start,t[i]))
        start=t[i]
    elif start is None and t[i]>3.0:
        start=t[i]
print("laps: %d"%len(laps))
def vat(a,tt): return a[np.argmin(np.abs(a[:,0]-tt))]
obs=D["obs_frenet"]
for k,(a,b) in enumerate(laps):
    m=(pf[:,0]>=a)&(pf[:,0]<b)
    vv=pf[m,4]
    # 이 랩에 장애물이 lookahead 안에 있던 시간 비율
    n=0;tot=0
    for tt,o in obs:
        if a<=tt<b:
            tot+=1
            f=vat(fre,tt)
            if any(0<((ob["s_start"]-f[1])%L)<12.0 for ob in o): n+=1
    print("lap %2d  %6.1f-%6.1f  %5.2fs  v mean %.2f min %.2f | obs<12m %4.1f%% (%d/%d)"%(
        k,a,b,b-a,vv.mean() if len(vv) else -1,vv.min() if len(vv) else -1,
        100*n/max(tot,1),n,tot))
