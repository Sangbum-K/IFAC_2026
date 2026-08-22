#!/usr/bin/env python3
"""모드별 속도/커브 분석 + 회피 경로 속도 캡의 지배 요인"""
import pickle,re,sys,collections,numpy as np
D=pickle.load(open("d17.pkl","rb")); L=pickle.load(open("log17.pkl","rb"))
fre=D["/car_state/frenet/odom"]; pf=D["/pf/pose/odom"]; ack=D["/ackermann_cmd"]
gw=D["global_wpnts"][0][1]; TL=gw[-1,0]; otw=D["otwpnts"]
pat=re.compile(r"s=([-\d.]+) v=([-\d.]+) d=([-\d.]+) \| obs id=(-?\d+) dist=([-\d.]+) d\[([-\d.]+),([-\d.]+)\] static=(\d) nobs=(\d+) \| evade=(\d) sector=(\d) commit=(\d) \| (\w+) \| (.*)$")
R=[]
for t,lvl,n,m in L:
    if "[trace]" not in m or not (4<=t<=170): continue
    g=pat.search(m)
    if g: R.append(dict(t=t,s=float(g[1]),v=float(g[2]),dist=float(g[5]),
        mode=g[13],tags=g[14]))
def near(a,t): return a[np.argmin(np.abs(a[:,0]-t))]
def gbv(s): return gw[np.argmin(np.abs(gw[:,0]-(s%TL))),8]
def at(l,t,tol=0.05):
    b=None;bd=9
    for tt,o in l:
        if abs(tt-t)<bd: bd=abs(tt-t);b=o
    return b if bd<tol else None
print("모드별 (n / 실제v 중앙 / 명령v 중앙 / gb중앙 / 비율 / 경로head 중앙)")
by=collections.defaultdict(list)
for r in R: by[r["mode"]].append(r)
for k,v in sorted(by.items(),key=lambda x:-len(x[1])):
    vv=[r["v"] for r in v]; gg=[gbv(r["s"]) for r in v]
    cc=[near(ack,r["t"])[2] for r in v]
    hh=[]
    for r in v:
        w=at(otw,r["t"])
        if w is not None and len(w): hh.append(w[0,8])
    print("  %-15s %5d | %.2f | %.2f | %.2f | %.2f | %s"%(k,len(v),np.median(vv),
        np.median(cc),np.median(gg),np.median(np.array(cc)/np.array(gg)),
        "%.2f"%np.median(hh) if hh else "-"))
# 회피 경로 속도 캡 지배 요인
print("\n[발행 회피경로 웨이포인트: vx 를 무엇이 정했나]")
A_LAT=8.83
rows=[]
for t,w in otw:
    if not (4<=t<=170) or len(w)==0: continue
    for j in range(0,min(len(w),40),4):
        s,kap,vx=w[j,0],w[j,7],w[j,8]
        vgrip=np.sqrt(A_LAT/max(abs(kap),1e-6))
        rows.append((vx,vgrip,gbv(s),abs(kap)))
A=np.array(rows)
print("  n=%d  vx 중앙 %.2f | 그립캡 sqrt(8.83/|k|) 중앙 %.2f | gb 중앙 %.2f | |kappa| 중앙 %.3f"%(
    len(A),np.median(A[:,0]),np.median(A[:,1]),np.median(A[:,2]),np.median(A[:,3])))
for name,col in (("그립캡에 붙음(vx>=0.95*grip)",A[:,0]>=0.95*A[:,1]),
                 ("gb 계획에 붙음(vx>=0.95*gb)",A[:,0]>=0.95*A[:,2]),
                 ("둘 다 아님(더 낮은 이유 존재)",(A[:,0]<0.95*A[:,1])&(A[:,0]<0.95*A[:,2]))):
    print("  %-32s %5.1f%%"%(name,100*np.mean(col)))
