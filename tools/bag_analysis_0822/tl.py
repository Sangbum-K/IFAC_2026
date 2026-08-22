#!/usr/bin/env python3
"""프레임 타임라인: 자차 s,d,v / 명령 / 장애물(frenet) / otwpnts"""
import pickle, sys, numpy as np
D = pickle.load(open(sys.argv[1],"rb"))
t0,t1 = float(sys.argv[2]), float(sys.argv[3])
step = float(sys.argv[4]) if len(sys.argv)>4 else 0.2
fre=D["/car_state/frenet/odom"]; pf=D["/pf/pose/odom"]; ack=D["/ackermann_cmd"]
esc=D["/planner/escape_active"]; spl=D["/pitwall/spliced_wpnts"]
cmt=D["/planner/avoidance/evade_committed"]; sfl=D["/planner/avoidance/speed_floor"]
L=D["global_wpnts"][0][1][-1,0]
def near(a,t):
    return a[np.argmin(np.abs(a[:,0]-t))] if len(a) else None
obs=D["obs_frenet"]; otw=D["otwpnts"]; lw=D["local_wpnts"]
def at(lst,t):
    best=None; bd=9
    for tt,o in lst:
        if abs(tt-t)<bd: bd=abs(tt-t); best=o
    return best,bd
print("  t     s     d    v   cmdv  steer  spl esc cmt sfl | obs: id Ds d[dr,dl] vs sz vis dyn")
for t in np.arange(t0,t1,step):
    f=near(fre,t); p=near(pf,t); a=near(ack,t); s_=near(spl,t); e=near(esc,t)
    c=near(cmt,t); sf=near(sfl,t)
    o,bd=at(obs,t); ow,bdo=at(otw,t)
    otxt=""
    if ow is not None and len(ow)>0 and bdo<0.3:
        otxt="  ot[s %.1f..%.1f d %.2f..%.2f v %.1f..%.1f n%d]"%(
            ow[0,0],ow[-1,0],ow[:,1].min(),ow[:,1].max(),ow[:,8].min(),ow[:,8].max(),len(ow))
    lwp,bdl=at(lw,t); ltxt=""
    if lwp is not None and len(lwp)>0 and bdl<0.3:
        ltxt="  lw[v %.2f..%.2f d %.2f..%.2f]"%(lwp[:,8].min(),lwp[:,8].max(),lwp[:,1].min(),lwp[:,1].max())
    ostr=""
    if o and bd<0.3:
        for ob in sorted(o,key=lambda z:z["s_center"]):
            ds=(ob["s_center"]-f[1])%L
            if ds>15: ds-=L
            ostr+=" #%d(Ds%+.2f d%+.2f[%+.2f,%+.2f] vs%+.2f sz%.2f %s%s)"%(
                ob["id"],ds,ob["d_center"],ob["d_right"],ob["d_left"],ob["vs"],ob["size"],
                "V" if ob["is_visible"] else "-", "S" if ob["is_static"] else "D")
    print("%6.2f %6.2f %+5.2f %4.2f %5.2f %+6.3f %4.0f %s %s %4.2f |%s%s%s"%(
        t,f[1],f[2],p[4],a[2],a[1],s_[1],"E" if e is not None and e[1]>0.5 else ".",
        "C" if c is not None and c[1]>0.5 else ".", sf[1] if sf is not None else -1,
        ostr,otxt,ltxt))
