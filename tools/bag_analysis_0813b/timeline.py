#!/usr/bin/env python3
"""랩 x s 그리드로 '어디서 장애물이 보고됐나' 지도."""
import pickle, numpy as np
D=pickle.load(open("data.pkl","rb")); TL=32.07
fre=D["/car_state/frenet/odom"]; t_e,s_e=fre[:,0],fre[:,1]
wrap=np.where(np.diff(s_e)<-TL*0.5)[0]; lapb=np.concatenate([[t_e[0]],t_e[wrap],[t_e[-1]]])
NB=64; G=np.zeros((len(lapb)-1,NB),int)
for t,obs in D["obs_frenet"]:
    li=np.searchsorted(lapb,t)-1
    if li<0 or li>=len(lapb)-1: continue
    for o in obs:
        b=int(o["s_center"]/TL*NB)%NB
        G[li,b]+=1
print("행=랩, 열=장애물 s (0.5m bin). 숫자=보고 횟수(0=공백, 자리수 표기)")
print("      "+"".join(f"{int(i*TL/NB)%10}" for i in range(NB)))
for li in range(len(lapb)-1):
    row="".join("." if G[li,b]==0 else ("#" if G[li,b]>60 else str(min(9,G[li,b]//7))) for b in range(NB))
    print(f" lap{li:2d} {row}  (t {lapb[li]:6.1f}-{lapb[li+1]:6.1f})")
print("\n실제 콘 s: O2=6.6  O3=10.5  O4=15.0  O6=22.1  O7=28.9  (벽오탐 O1=4.3 O5=21.3)")
# s 5~9 상세
print("\n=== 랩별 s=5~9 구간 보고 (O2 영역) ===")
for li in range(len(lapb)-1):
    t0,t1=lapb[li],lapb[li+1]
    ss=[]
    for t,obs in D["obs_frenet"]:
        if not(t0<=t<t1): continue
        for o in obs:
            if 4.8<o["s_center"]<9.5: ss.append((o["s_center"],o["d_center"],o["size"]))
    if ss:
        A=np.array(ss)
        print(f" lap{li:2d}: n={len(A):4d} s p50={np.median(A[:,0]):.2f} d p50={np.median(A[:,1]):+.2f} sz p50={np.median(A[:,2]):.2f}")
    else:
        print(f" lap{li:2d}: (없음)")
