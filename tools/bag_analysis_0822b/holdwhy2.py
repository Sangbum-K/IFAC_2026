#!/usr/bin/env python3
"""hold 중 '마지막으로 본 장애물'을 이미 지났는가"""
import pickle,numpy as np
D=pickle.load(open("d17.pkl","rb"))
hold=D["/pitwall/avoidance_loss_hold"]; fre=D["/car_state/frenet/odom"]
gw=D["global_wpnts"][0][1]; TL=gw[-1,0]; obs=D["obs_frenet"]
otw=D["otwpnts"]
ot=[(t,w) for t,w in otw if len(w)]
OT=np.array([t for t,w in ot])
def near(a,t): return a[np.argmin(np.abs(a[:,0]-t))]
H=hold[(hold[:,0]>4)&(hold[:,0]<170)&(hold[:,1]>0.5)]
# 각 hold 시점 직전의 마지막 비어있지 않은 장애물 프레임
OT_t=np.array([t for t,_ in obs]);
passed=0; ahead=0; unknown=0; margins=[]
for t,_ in H:
    f=near(fre,t)
    # 직전 2초 안의 마지막 '전방 장애물이 있던' 프레임에서 그 장애물 s_end
    last=None
    for tt,o in obs:
        if tt>t or tt<t-2.0: continue
        for ob in o:
            if 0<((ob["s_start"]-near(fre,tt)[1])%TL)<12.0:
                last=(tt,ob["s_end"])
    if last is None: unknown+=1; continue
    gap=(f[1]-last[1])%TL          # ego 가 그 장애물 뒤끝을 얼마나 지났나
    if gap<TL/2 and gap>0.3: passed+=1; margins.append(gap)
    else: ahead+=1
n=len(H)
print("hold 활성 %d 사이클"%n)
print("  마지막 회피대상 뒤끝을 0.3 m 이상 지난 상태: %d (%.1f%%)  통과량 중앙 %.2f m"%(
    passed,100*passed/n,np.median(margins) if margins else -1))
print("  아직 안 지났거나 판정 불가:                   %d (%.1f%%)"%(ahead,100*ahead/n))
print("  최근 2 s 안에 전방 장애물 기록 없음:            %d (%.1f%%)"%(unknown,100*unknown/n))
