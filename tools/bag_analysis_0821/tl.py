#!/usr/bin/env python3
"""프레임 타임라인: 자차 s,d,v / 명령 / 장애물(frenet) / otwpnts"""
import pickle, sys, numpy as np
D = pickle.load(open("/home/sangbum/IFAC_ws/tools/bag_analysis_0821/d.pkl","rb"))
t0,t1 = float(sys.argv[1]), float(sys.argv[2])
step = float(sys.argv[3]) if len(sys.argv)>3 else 0.2
fre=D["/car_state/frenet/odom"]; pf=D["/pf/pose/odom"]; ack=D["/ackermann_cmd"]
esc=D["/planner/escape_active"]; spl=D["/pitwall/spliced_wpnts"]
lat=D["/pitwall/lat_err"]
def near(a,t,col=None):
    if len(a)==0: return None
    i=np.argmin(np.abs(a[:,0]-t)); return a[i]
obs=D["obs_frenet"]; pred=D["obs_pred"]; otw=D["otwpnts"]; lw=D["local_wpnts"]
L=36.833
def obs_at(lst,t):
    best=None; bd=9
    for tt,o in lst:
        if abs(tt-t)<bd: bd=abs(tt-t); best=(tt,o)
    return best
print("  t     s     d    v   cmdv  steer  spl esc | obstacles: id s_c d_c[dr..dl] vs size vis dyn lap")
for t in np.arange(t0,t1,step):
    f=near(fre,t); p=near(pf,t); a=near(ack,t); s_=near(spl,t); e=near(esc,t)
    tt,o = obs_at(obs,t)
    ow = None
    bd=9
    for t2,w in otw:
        if abs(t2-t)<bd: bd=abs(t2-t); ow=w
    otxt=""
    if ow is not None and len(ow)>0 and bd<0.3:
        otxt = "  ot[s %.1f..%.1f d %.2f..%.2f v %.1f..%.1f n%d]"%(ow[0,0],ow[-1,0],ow[:,1].min(),ow[:,1].max(),ow[:,8].min(),ow[:,8].max(),len(ow))
    ostr=""
    if o and abs(tt-t)<0.3:
        for ob in sorted(o,key=lambda z:z["s_center"]):
            ds = (ob["s_center"]-f[1]) % L
            if ds>15: ds-=L
            ostr+=" #%d(Δs%+.2f d%+.2f[%+.2f,%+.2f] vs%+.2f sz%.2f %s%s)"%(
                ob["id"],ds,ob["d_center"],ob["d_right"],ob["d_left"],ob["vs"],ob["size"],
                "V" if ob["is_visible"] else "-", "S" if ob["is_static"] else "D")
    print("%6.2f %6.2f %+5.2f %4.2f %5.2f %+6.3f %4.0f %s |%s%s"%(
        t,f[1],f[2],p[4],a[2],a[1],s_[1],"E" if e[1]>0.5 else ".",ostr,otxt))
