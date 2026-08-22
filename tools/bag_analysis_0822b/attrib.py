#!/usr/bin/env python3
"""사이클 단위로: 발행 vx 가 (a) cur_vs_ 전방가속 엔벨로프 (b) 기하천장 중 어디에 붙었나
   그리고 엔벨로프 seed 를 실제 속도로 바꾸면 얼마나 올라가나 (반사실)"""
import pickle,re,numpy as np
D=pickle.load(open("d17.pkl","rb")); L=pickle.load(open("log17.pkl","rb"))
otw=D["otwpnts"]; gw=D["global_wpnts"][0][1]; TL=gw[-1,0]
fre=D["/car_state/frenet/odom"]; pf=D["/pf/pose/odom"]
A_LAT=8.83; A_ACC=3.0
md={}
for t,lvl,n,m in L:
    if "[trace]" in m and 4<=t<=170:
        g=re.search(r"commit=(\d) \| (\w+) \|",m)
        if g: md[round(t,2)]=g[2]
MT=np.array(sorted(md)); MV=[md[k] for k in MT]
def mode_at(t):
    i=np.argmin(np.abs(MT-t)); return MV[i] if abs(MT[i]-t)<0.06 else None
def near(a,t): return a[np.argmin(np.abs(a[:,0]-t))]
def gbv(s): return gw[np.argmin(np.abs(gw[:,0]-(s%TL))),8]
def env(v0,ds,kap):
    """전방 가속 엔벨로프 (friction-circle derate 포함)"""
    out=np.empty(len(ds)+1); out[0]=v0; p=v0
    for i,d in enumerate(ds):
        lat=abs(kap[i])*p*p
        a=A_ACC*np.sqrt(max(1-min(lat/A_LAT,1.0)**2,0.0))
        p=np.sqrt(p*p+2*a*d); out[i+1]=p
    return out
rows=[]
for t,w in otw:
    if not(4<=t<=170) or len(w)<20: continue
    mo=mode_at(t)
    if mo not in ("committed_hold","lane_switch","free_spline"): continue
    x,y,k,vx=w[:,2],w[:,3],w[:,7],w[:,8]
    ds=np.hypot(np.diff(x),np.diff(y))
    vs=near(fre,t)[4]; vtrue=near(pf,t)[4]
    e0=env(max(vs,0.1),ds,k); e1=env(max(vtrue,0.1),ds,k)
    cum=np.r_[0,np.cumsum(ds)]
    j=int(np.searchsorted(cum,1.0)); j=min(j,len(w)-1)
    rows.append((mo,vx[0],e0[0],vx[j],e0[j],e1[j],gbv(w[j,0]),vtrue,vs,
                 np.mean(vx>=0.98*e0)))
import collections
by=collections.defaultdict(list)
for r in rows: by[r[0]].append(r)
print("반사실: 전방가속 엔벨로프 seed 를 cur_vs_(s투영) -> 실제속도 로 바꿨을 때")
print("       (기하천장은 그대로이므로 실제로는 min(기하천장, 새 엔벨로프) 이다)")
for mo,g in sorted(by.items(),key=lambda x:-len(x[1])):
    A=np.array([[r[1],r[2],r[3],r[4],r[5],r[6],r[7],r[8],r[9]] for r in g],float)
    print("\n%s n=%d"%(mo,len(A)))
    print("  현재 vs %.2f / 실제 v %.2f  (차이 %.2f m/s, %.0f%%)"%(
        np.median(A[:,7]),np.median(A[:,6]),np.median(A[:,6]-A[:,7]),
        100*(np.median(A[:,6])/np.median(A[:,7])-1)))
    print("  경로 1 m 지점: 발행 %.2f | 현 엔벨로프 %.2f | 새 엔벨로프 %.2f | gb %.2f"%(
        np.median(A[:,2]),np.median(A[:,3]),np.median(A[:,4]),np.median(A[:,5])))
    # 실제 상한 = 기하천장 추정 = 발행 vx (엔벨로프에 안 붙은 점)
    newv=np.minimum(A[:,4],np.maximum(A[:,2],A[:,3]))   # 보수적: 기존 기하천장 = max(발행, 현엔벨)
    print("  -> 1 m 지점 계획속도 중앙 %.2f -> %.2f (+%.0f%%)"%(
        np.median(A[:,2]),np.median(newv),100*(np.median(newv)/np.median(A[:,2])-1)))
    print("  발행 vx 가 엔벨로프에 붙은 점 비율 중앙 %.0f%%"%(100*np.median(A[:,8])))
