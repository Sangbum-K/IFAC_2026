#!/usr/bin/env python3
"""지정 구간만 재검증 + 발진 시점 추정"""
import pickle,re,sys,numpy as np
D=pickle.load(open(sys.argv[1],"rb")); L=pickle.load(open(sys.argv[2],"rb"))
T0,T1=float(sys.argv[3]),float(sys.argv[4]); MIN_M=0.25
lw=D["local_wpnts"]; cmt=D["/planner/avoidance/evade_committed"]
pat=re.compile(r"s=([-\d.]+) v=([-\d.]+) d=([-\d.]+) \| obs id=(-?\d+) dist=([-\d.]+) d\[([-\d.]+),([-\d.]+)\].*?\| (\w+) \| (.*)$")
rows=[]
for t,lvl,n,m in L:
    if "[trace]" not in m or not (T0<=t<=T1): continue
    g=pat.search(m)
    if g: rows.append((t,float(g.group(2)),float(g.group(3)),float(g.group(5)),
                       float(g.group(6)),float(g.group(7)),g.group(8),g.group(9)))
def at(l,t,tol=0.15):
    b=None;bd=9
    for tt,o in l:
        if abs(tt-t)<bd: bd=abs(tt-t);b=o
    return b if bd<tol else None
def near(a,t): return a[np.argmin(np.abs(a[:,0]-t))] if len(a) else None
n=0;og_n=0;ng_n=0;co=[];cn=[];first=None;HALF=0.15;head=[]
for t,v,ego_d,dist,odr,odl,mode,tags in rows:
    if abs(v)>=0.10: continue
    w=at(lw,t)
    if w is None or len(w)==0: continue
    n+=1
    C=near(cmt,t); og = C is not None and abs(C[0]-t)<0.2 and C[1]>0.5
    ng = og or (mode=="trailing" and "trailing_retry_clamp_raised" in tags)
    og_n+=og; ng_n+=ng
    ds=np.r_[0,np.cumsum(np.hypot(np.diff(w[:,2]),np.diff(w[:,3])))]
    k=min(int(np.searchsorted(ds,MIN_M)),len(w)-1)
    a=w[0,8] if not og else w[k,8]; b=w[0,8] if not ng else w[k,8]
    co.append(a); cn.append(b)
    if b>=0.05 and first is None:
        first=(t,b,dist,(ego_d-HALF)<odl and (ego_d+HALF)>odr,mode)
    head.append(w[0,8])
co=np.array(co); cn=np.array(cn)
print("구간 %.1f-%.1f s | 정지 사이클 %d"%(T0,T1,n))
print("  현재(옛 게이트): 명령 0 이 %.1f%%, 중앙 %.2f m/s  -> 출발 못 함"%(100*np.mean(co<0.05),np.median(co)))
print("  수정 후:        명령 0 이 %.1f%%, 중앙 %.2f m/s"%(100*np.mean(cn<0.05),np.median(cn)))
print("  게이트 열림 %.1f%% -> %.1f%%"%(100*og_n/n,100*ng_n/n))
if first:
    t,b,dist,ov,mode=first
    print("  첫 발진 예상: t=%.2f s (구간 시작 +%.2f s), 명령 %.2f m/s, 모드 %s"%(t,t-T0,b,mode))
    print("    그때 장애물: 간격 %.2f m, 정면(횡중첩) %s"%(dist,"예" if ov else "아니오 — 옆으로 비켜 있음"))
print("  계획 선두속도(=현재 컨트롤러가 읽는 값) 중앙 %.2f"%np.median(head))
