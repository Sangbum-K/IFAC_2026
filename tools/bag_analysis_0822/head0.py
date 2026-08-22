#!/usr/bin/env python3
"""정지 사이클의 계획 프로파일 머리 모양 분류"""
import pickle,sys,numpy as np
D=pickle.load(open(sys.argv[1],"rb"))
pf=D["/pf/pose/odom"]; ack=D["/ackermann_cmd"]; lw=D["local_wpnts"]; otw=D["otwpnts"]
cmt=D["/planner/avoidance/evade_committed"]
hold=D.get("/pitwall/avoidance_loss_hold",np.zeros((0,2)))
def near(a,t):
    return a[np.argmin(np.abs(a[:,0]-t))] if len(a) else None
segs=[]
v=pf[:,4]; stop=np.abs(v)<0.10; i=0
while i<len(pf):
    if stop[i]:
        j=i
        while j+1<len(pf) and stop[j+1]: j+=1
        if pf[j,0]-pf[i,0]>1.5: segs.append((pf[i,0],pf[j,0]))
        i=j+1
    else: i+=1
def at(l,t,tol=0.15):
    b=None;bd=9
    for tt,o in l:
        if abs(tt-t)<bd: bd=abs(tt-t);b=o
    return b if bd<tol else None
print("구간          n | 머리모양 분류                                | v@0.25m 중앙 | hold중앙 cmt%")
for a,b in segs:
    kinds={"anchor0(0 한점)":0,"all0(전구간 0)":0,"multi0(2~10점)":0,"정상(>0)":0,"경로없음":0}
    v25=[]; hh=[]; ct=0; n=0
    for t in np.arange(a,b,0.05):
        n+=1
        C=near(cmt,t)
        if C is not None and abs(C[0]-t)<0.2 and C[1]>0.5: ct+=1
        H=near(hold,t)
        if H is not None and abs(H[0]-t)<0.5: hh.append(H[1])
        w=at(lw,t)
        if w is None or len(w)==0: kinds["경로없음"]+=1; continue
        vx=w[:,8]
        nz=int(np.argmax(vx>1e-6)) if (vx>1e-6).any() else len(vx)
        if nz==0: kinds["정상(>0)"]+=1
        elif nz==len(vx): kinds["all0(전구간 0)"]+=1
        elif nz==1: kinds["anchor0(0 한점)"]+=1
        else: kinds["multi0(2~10점)"]+=1
        ds=np.r_[0,np.cumsum(np.hypot(np.diff(w[:,2]),np.diff(w[:,3])))]
        k=int(np.searchsorted(ds,0.25))
        v25.append(vx[min(k,len(vx)-1)])
    ks=" ".join("%s=%.0f%%"%(k,100*c/max(n,1)) for k,c in kinds.items() if c)
    print("%5.1f-%5.1f %4d | %-44s | %5.2f | %5.2f %4.1f%%"%(
        a,b,n,ks,np.median(v25) if v25 else -1, np.median(hh) if hh else -1, 100*ct/n))
