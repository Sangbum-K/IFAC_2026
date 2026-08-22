#!/usr/bin/env python3
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
H=hold[hold[:,1]>0.5][:,0]
def classify(t):
    j=np.argmin(np.abs(T-t))
    if abs(T[j]-t)<0.08:
        k=M[j]
        return k+("(장애물 뒤)" if k=="trailing" and DI[j]<0 else "(장애물 앞)" if k=="trailing" else "")
    if len(ot) and np.min(np.abs(ot-t))<0.08: return "회피경로만(트레이스 없음)"
    if len(H) and np.min(np.abs(H-t))<0.08: return "hold(회피 상실 후 1s)"
    return "무장애물"
key=[classify(t) for t in P[:,0]]
NB=70; bs=TL/NB; b=(FS//bs).astype(int)%NB
clean=np.array([k=="무장애물" for k in key])
ref=np.full(NB,np.nan)
for i in range(NB):
    m=(b==i)&clean
    if m.sum()>=5: ref[i]=np.median(P[m,4])
ok=~np.isnan(ref); ref=np.interp(np.arange(NB),np.arange(NB)[ok],ref[ok])
# 기준선 보정: 무장애물 구간의 손실이 정확히 0 이 되도록 스케일
ds0=np.diff(FS); ds0=np.where(ds0<-TL/2,ds0+TL,ds0); ds0=np.clip(ds0,0,1)
num=0.0;den=0.0
for i in range(len(ds0)):
    if ds0[i]<=0 or key[i]!="무장애물": continue
    num+=ds0[i]/ref[b[i]]; den+=P[i+1,0]-P[i,0]
ref=ref*(num/den)
print("기준선 스케일 보정 x%.3f"%(num/den))
print("무장애물 기준 페이스로 한 바퀴 = %.2f s (실측 최속랩 7.37 s)"%np.sum(bs/ref))
ds=np.diff(FS); ds=np.where(ds<-TL/2,ds+TL,ds); ds=np.clip(ds,0,1)
acc=collections.defaultdict(float); dd=collections.defaultdict(float); tt=collections.defaultdict(float)
for i in range(len(ds)):
    if ds[i]<=0: continue
    dt=P[i+1,0]-P[i,0]
    acc[key[i]]+=dt-ds[i]/ref[b[i]]; dd[key[i]]+=ds[i]; tt[key[i]]+=dt
print("\n%-26s %8s %8s %8s %8s"%("구간","손실 s","거리 m","시간 s","평균 v"))
for k,v in sorted(acc.items(),key=lambda x:-x[1]):
    print("  %-24s %+7.2f %7.1f %7.1f %7.2f"%(k,v,dd[k],tt[k],dd[k]/max(tt[k],1e-6)))
print("\n장애물 관련 총 손실 %.1f s  = 랩당 %.2f s (18랩)"%(
    sum(v for k,v in acc.items() if k!="무장애물"),
    sum(v for k,v in acc.items() if k!="무장애물")/18))
