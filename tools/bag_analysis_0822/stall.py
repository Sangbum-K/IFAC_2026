#!/usr/bin/env python3
"""정지 교착 분석: v~0 사이클에서 명령/계획선두속도/evade_committed/모드"""
import pickle,sys,numpy as np
D=pickle.load(open(sys.argv[1],"rb"))
pf=D["/pf/pose/odom"]; ack=D["/ackermann_cmd"]; fre=D["/car_state/frenet/odom"]
cmt=D["/planner/avoidance/evade_committed"]; lw=D["local_wpnts"]; otw=D["otwpnts"]
obs=D["obs_frenet"]; L=D["global_wpnts"][0][1][-1,0]
def near(a,t):
    return a[np.argmin(np.abs(a[:,0]-t))] if len(a) else None
def at(lst,t,tol=0.15):
    best=None;bd=9
    for tt,o in lst:
        if abs(tt-t)<bd: bd=abs(tt-t);best=o
    return (best if bd<tol else None)
# 정지 구간 추출
v=pf[:,4]; stop=np.abs(v)<0.10
segs=[];i=0
while i<len(pf):
    if stop[i]:
        j=i
        while j+1<len(pf) and stop[j+1]: j+=1
        if pf[j,0]-pf[i,0]>1.5: segs.append((pf[i,0],pf[j,0]))
        i=j+1
    else: i+=1
print("정지구간(>1.5s): %d 개"%len(segs))
print(" t0     t1    dur | ego s      obs(가장가까운) Ds  static | cmd0%%  head_v(중앙)  cmt_true%%  ot있음%%")
for a,b in segs:
    ts=np.arange(a,b,0.05)
    c0=0;hv=[];ct=0;oo=0;n=0;dsl=[];stat=[]
    for t in ts:
        A=near(ack,t); C=near(cmt,t); f=near(fre,t)
        if A is None: continue
        n+=1
        if abs(A[2])<0.05: c0+=1
        if C is not None and abs(C[0]-t)<0.2 and C[1]>0.5: ct+=1
        w=at(lw,t)
        if w is not None and len(w): hv.append(w[0,8])
        ow=at(otw,t)
        if ow is not None and len(ow): oo+=1
        o=at(obs,t)
        if o:
            best=None
            for ob in o:
                ds=(ob["s_center"]-f[1])%L
                if ds>20: continue
                if best is None or ds<best[0]: best=(ds,ob["is_static"])
            if best: dsl.append(best[0]); stat.append(best[1])
    if n==0: continue
    print("%6.1f %6.1f %5.1f | %6.2f   Ds=%s %s | %5.1f%%  %5.2f  %5.1f%%  %5.1f%%"%(
        a,b,b-a,near(fre,a)[1],
        ("%.2f"%np.median(dsl)) if dsl else "  -  ",
        ("static" if np.mean(stat)>0.5 else "dyn") if stat else "-",
        100*c0/n, np.median(hv) if hv else -1, 100*ct/n, 100*oo/n))
