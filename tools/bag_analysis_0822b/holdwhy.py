#!/usr/bin/env python3
"""avoidance_loss_hold 이 걸린 이유: 장애물이 뒤로 지나가서인가, 인지 드롭아웃인가"""
import pickle,numpy as np
D=pickle.load(open("d17.pkl","rb"))
hold=D["/pitwall/avoidance_loss_hold"]; fre=D["/car_state/frenet/odom"]
gw=D["global_wpnts"][0][1]; TL=gw[-1,0]; obs=D["obs_frenet"]
def near(a,t): return a[np.argmin(np.abs(a[:,0]-t))]
def at(l,t,tol=0.1):
    b=None;bd=9
    for tt,o in l:
        if abs(tt-t)<bd: bd=abs(tt-t);b=o
    return b if bd<tol else None
H=hold[(hold[:,0]>4)&(hold[:,0]<170)&(hold[:,1]>0.5)]
cat={"전방 장애물 있음(진짜 드롭아웃 방어)":0,"전방 없음 — 이미 지나침":0,"장애물 배열 자체가 빔":0}
for t,_ in H:
    f=near(fre,t); o=at(obs,t)
    if o is None: cat["장애물 배열 자체가 빔"]+=1; continue
    ahead=[ob for ob in o if 0<((ob["s_start"]-f[1])%TL)<12.0]
    if ahead: cat["전방 장애물 있음(진짜 드롭아웃 방어)"]+=1
    elif o: cat["전방 없음 — 이미 지나침"]+=1
    else: cat["장애물 배열 자체가 빔"]+=1
n=len(H)
print("hold 활성 사이클 %d (%.1f s)"%(n,n*0.0333))
for k,v in cat.items(): print("  %-34s %5d  %5.1f%%"%(k,v,100*v/n))
