#!/usr/bin/env python3
"""36.5 s 손실을 모드별로 배분 + '이미 지나친 장애물에 트레일링' 계측"""
import pickle,re,numpy as np,collections
D=pickle.load(open("d17.pkl","rb")); L=pickle.load(open("log17.pkl","rb"))
pf=D["/pf/pose/odom"]; fre=D["/car_state/frenet/odom"]; gw=D["global_wpnts"][0][1]; TL=gw[-1,0]
pat=re.compile(r"s=([-\d.]+) v=([-\d.]+) d=([-\d.]+) \| obs id=(-?\d+) dist=([-\d.]+) d\[([-\d.]+),([-\d.]+)\] static=(\d) nobs=(\d+) \| evade=(\d) sector=(\d) commit=(\d) \| (\w+) \| (.*)$")
T=[];M=[];DI=[]
for t,lvl,n,m in L:
    if "[trace]" not in m or not(4<=t<=170): continue
    g=pat.search(m)
    if g: T.append(t); M.append(g[13]); DI.append(float(g[5]))
T=np.array(T); DI=np.array(DI)
def gbv(s): return gw[np.argmin(np.abs(gw[:,0]-(s%TL))),8]
hold=D["/pitwall/avoidance_loss_hold"]
P=pf[(pf[:,0]>4)&(pf[:,0]<170)]
FS=np.array([fre[np.argmin(np.abs(fre[:,0]-t)),1] for t in P[:,0]])
ds=np.diff(FS); ds=np.where(ds<-TL/2,ds+TL,ds); ds=np.clip(ds,0,1)
acc=collections.defaultdict(float); dist=collections.defaultdict(float)
for i in range(len(ds)):
    t=P[i,0]; dt=P[i+1,0]-P[i,0]
    if ds[i]<=0: continue
    vref=0.89*gbv(FS[i])
    loss=dt-ds[i]/max(vref,0.3)
    j=np.argmin(np.abs(T-t))
    if abs(T[j]-t)<0.06: key=M[j]+("(지나침)" if M[j]=="trailing" and DI[j]<0 else "")
    else:
        k=np.argmin(np.abs(hold[:,0]-t))
        key="hold(회피상실)" if abs(hold[k,0]-t)<0.1 and hold[k,1]>0.5 else "장애물 없음"
    acc[key]+=loss; dist[key]+=ds[i]
tot=sum(acc.values())
print("주행구간 총 손실 %.1f s (기준 = 회피없는 랩 페이스 0.89*gb)"%tot)
print("%-22s %8s %8s"%("구간","손실 s","거리 m"))
for k,v in sorted(acc.items(),key=lambda x:-x[1]):
    print("  %-20s %+7.2f  %7.1f"%(k,v,dist[k]))
print()
tr=[(t,d) for t,m,d in zip(T,M,DI) if m=="trailing"]
neg=[x for x in tr if x[1]<0]
print("trailing 사이클 %d 중 장애물이 이미 뒤에 있는(dist<0) 사이클 %d (%.0f%%)"%(
    len(tr),len(neg),100*len(neg)/len(tr)))
