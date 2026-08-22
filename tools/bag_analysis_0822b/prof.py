#!/usr/bin/env python3
"""모드별 발행 회피경로 속도 프로파일 형태"""
import pickle,re,numpy as np
D=pickle.load(open("d17.pkl","rb")); L=pickle.load(open("log17.pkl","rb"))
otw=D["otwpnts"]; gw=D["global_wpnts"][0][1]; TL=gw[-1,0]
fre=D["/car_state/frenet/odom"]; pf=D["/pf/pose/odom"]
pat=re.compile(r"\| (\w+) \| ")
modes=[]
for t,lvl,n,m in L:
    if "[trace]" in m and 4<=t<=170:
        g=re.search(r"commit=(\d) \| (\w+) \|",m)
        if g: modes.append((t,g[2]))
modes=np.array([(t,) for t,_ in modes]),[m for _,m in modes]
MT=np.array([t for t,_ in zip([x[0] for x in modes[0]],modes[1])])
MM=modes[1]
def mode_at(t):
    i=np.argmin(np.abs(MT-t))
    return MM[i] if abs(MT[i]-t)<0.06 else None
def gbv(s): return gw[np.argmin(np.abs(gw[:,0]-(s%TL))),8]
def near(a,t): return a[np.argmin(np.abs(a[:,0]-t))]
agg={}
for t,w in otw:
    if not(4<=t<=170) or len(w)<20: continue
    mo=mode_at(t)
    if mo is None: continue
    x,y=w[:,2],w[:,3]
    ds=np.r_[0,np.cumsum(np.hypot(np.diff(x),np.diff(y)))]
    g=np.array([gbv(s) for s in w[:,0]])
    a=agg.setdefault(mo,[])
    row=[]
    for target in (0.0,0.25,0.5,1.0,2.0,3.0,5.0,8.0):
        k=int(np.searchsorted(ds,target))
        if k>=len(w): row.append((np.nan,np.nan)); continue
        row.append((w[k,8],g[k]))
    a.append((row,near(pf,t)[4],near(fre,t)[4]))
print("경로 위 거리별 계획속도 / 그 지점 gb / 비율   (mode: n)")
hdr="  d[m]:"+"".join("%8.1f"%x for x in (0,0.25,0.5,1,2,3,5,8))
for mo,rows in sorted(agg.items(),key=lambda x:-len(x[1])):
    R=np.array([r[0] for r in rows])   # n,8,2
    print("\n%s  n=%d  실제v중앙 %.2f  frenet vs중앙 %.2f"%(mo,len(rows),
        np.median([r[1] for r in rows]),np.median([r[2] for r in rows])))
    print(hdr)
    print("  vx  :"+"".join("%8.2f"%np.nanmedian(R[:,i,0]) for i in range(8)))
    print("  gb  :"+"".join("%8.2f"%np.nanmedian(R[:,i,1]) for i in range(8)))
    print("  비율 :"+"".join("%8.2f"%np.nanmedian(R[:,i,0]/R[:,i,1]) for i in range(8)))
