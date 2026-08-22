#!/usr/bin/env python3
"""전체 개요: 속도/명령/장애물/모드 시간선"""
import pickle,sys,numpy as np
D=pickle.load(open(sys.argv[1],"rb"))
pf=D["/pf/pose/odom"]; ack=D["/ackermann_cmd"]; aut=D["/autonomy/ackermann_cmd"]
tel=D["/teleop"]; fre=D["/car_state/frenet/odom"]
print("dur %.1f  pf %d  ack %d  autonomy %d  teleop %d"%(pf[-1,0],len(pf),len(ack),len(aut),len(tel)))
# 자율 명령이 ackermann_cmd 로 나가는 구간 = mux 가 autonomy 선택
print("\n== 속도 프로필 (2초 버킷) ==")
print(" t   v_pf  cmd_v  |obs_ahead(<3m)  esc  estop")
esc=D["/planner/escape_active"]; est=D["/safety/emergency_stop"]
obs=D["obs_frenet"]
L=None
gw=D["global_wpnts"]
if gw: L=gw[0][1][-1,0]
print("track L=%s"%L)
def near(a,t):
    if len(a)==0: return None
    return a[np.argmin(np.abs(a[:,0]-t))]
for t in np.arange(0,pf[-1,0],2.0):
    p=near(pf,t); a=near(ack,t); f=near(fre,t); e=near(esc,t); s=near(est,t)
    n=0; dmin=99
    for tt,o in obs:
        if abs(tt-t)<0.3:
            for ob in o:
                ds=(ob["s_center"]-f[1])%L
                if ds<3.0: n+=1; dmin=min(dmin,ds)
            break
    print("%5.1f %5.2f %6.2f  n%d dmin%5.2f  %s %s"%(t,p[4],a[2],n,dmin,
        "E" if e is not None and e[1]>0.5 else ".", "STOP" if s is not None and s[1]>0.5 else "."))
