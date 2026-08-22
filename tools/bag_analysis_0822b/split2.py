#!/usr/bin/env python3
"""손실 배분 — 기준선을 '이 트랙에서 실제로 낸 페이스'(s별 무장애물 중앙값)로"""
import pickle,re,numpy as np,collections
D=pickle.load(open("d17.pkl","rb")); L=pickle.load(open("log17.pkl","rb"))
pf=D["/pf/pose/odom"]; fre=D["/car_state/frenet/odom"]; gw=D["global_wpnts"][0][1]; TL=gw[-1,0]
otw=D["otwpnts"]; hold=D["/pitwall/avoidance_loss_hold"]
pat=re.compile(r"dist=([-\d.]+) .*commit=(\d) \| (\w+) \|")
T=[];M=[];DI=[]
for t,lvl,n,m in L:
    if "[trace]" not in m or not(4<=t<=170): continue
    g=pat.search(m)
    if g: T.append(t); DI.append(float(g[1])); M.append(g[3])
T=np.array(T); DI=np.array(DI)
P=pf[(pf[:,0]>4)&(pf[:,0]<170)]
FS=np.array([fre[np.argmin(np.abs(fre[:,0]-t)),1] for t in P[:,0]])
ot=np.array([t for t,w in otw if len(w)])
act=np.array([np.min(np.abs(ot-t))<0.06 for t in P[:,0]])
NB=70; bs=TL/NB
b=(FS//bs).astype(int)%NB
ref=np.zeros(NB)
for i in range(NB):
    m=(b==i)&(~act)
    ref[i]=np.median(P[m,4]) if m.sum()>=5 else np.nan
# 결측은 인접 보간
ok=~np.isnan(ref); ref=np.interp(np.arange(NB),np.arange(NB)[ok],ref[ok])
print("s별 무장애물 페이스: 중앙 %.2f m/s, 이 페이스로 한 바퀴 = %.2f s"%(
    np.median(ref),np.sum(bs/ref)))
print("실측 최속랩 7.37 s / 장애물랩 평균 9.5 s")
ds=np.diff(FS); ds=np.where(ds<-TL/2,ds+TL,ds); ds=np.clip(ds,0,1)
acc=collections.defaultdict(float); dd=collections.defaultdict(float); nn=collections.Counter()
for i in range(len(ds)):
    if ds[i]<=0: continue
    t=P[i,0]; dt=P[i+1,0]-P[i,0]
    loss=dt-ds[i]/ref[b[i]]
    j=np.argmin(np.abs(T-t))
    if abs(T[j]-t)<0.06:
        key=M[j]
        if key=="trailing": key+= "(장애물 뒤)" if DI[j]<0 else "(장애물 앞)"
    else:
        k=np.argmin(np.abs(hold[:,0]-t))
        key="hold(회피 상실 후 1s)" if abs(hold[k,0]-t)<0.1 and hold[k,1]>0.5 else "장애물 없음"
    acc[key]+=loss; dd[key]+=ds[i]; nn[key]+=1
print("\n%-24s %8s %8s %7s"%("구간","손실 s","거리 m","시간 s"))
for k,v in sorted(acc.items(),key=lambda x:-x[1]):
    print("  %-22s %+7.2f  %7.1f  %6.1f"%(k,v,dd[k],nn[k]*0.0333))
print("\n합계 손실 %.1f s / 주행 166 s"%sum(v for v in acc.values() if v>0))
