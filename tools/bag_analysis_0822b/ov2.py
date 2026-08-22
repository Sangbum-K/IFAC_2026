#!/usr/bin/env python3
import pickle,sys,numpy as np
D=pickle.load(open(sys.argv[1],"rb"))
pf=D["/pf/pose/odom"]; ack=D["/ackermann_cmd"]; aut=D["/autonomy/ackermann_cmd"]
tel=D["/teleop"]; fre=D["/car_state/frenet/odom"]
esc=D["/planner/escape_active"]; est=D["/safety/emergency_stop"]
obs=D["obs_frenet"]; L=D["global_wpnts"][0][1][-1,0]
def near(a,t):
    return a[np.argmin(np.abs(a[:,0]-t))] if len(a) else None
def rate(a,t,w=1.0):  # 메시지 수 (활성 여부)
    return int(np.sum(np.abs(a[:,0]-t)<w/2)) if len(a) else 0
print("  t   v_pf  ack_v  aut_v  tel_v  tel_hz | obs<3m dmin  esc estop")
for t in np.arange(0,pf[-1,0],2.0):
    p=near(pf,t); a=near(ack,t); au=near(aut,t); te=near(tel,t); f=near(fre,t)
    e=near(esc,t); s=near(est,t)
    n=0; dmin=99
    for tt,o in obs:
        if abs(tt-t)<0.3:
            for ob in o:
                ds=(ob["s_center"]-f[1])%L
                if ds<3.0: n+=1; dmin=min(dmin,ds)
            break
    print("%5.1f %5.2f %6.2f %6.2f %6.2f %5d | n%d %5.2f  %s %s"%(t,p[4],a[2],
        au[2] if au is not None else -9, te[2] if te is not None else -9, rate(tel,t),
        n,dmin,"E" if e is not None and e[1]>0.5 else ".",
        "STOP" if s is not None and s[1]>0.5 else "."))
