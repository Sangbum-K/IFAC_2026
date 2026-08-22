#!/usr/bin/env python3
"""회피 구간의 속도 손실 분해: gb 계획 -> 발행 회피경로 -> 로컬 -> 명령 -> 실제"""
import pickle,sys,numpy as np
D=pickle.load(open(sys.argv[1],"rb"))
fre=D["/car_state/frenet/odom"]; pf=D["/pf/pose/odom"]; ack=D["/ackermann_cmd"]
gw=D["global_wpnts"][0][1]; L=gw[-1,0]
otw=D["otwpnts"]; lw=D["local_wpnts"]
T0,T1=4.0,170.0
def near(a,t):
    i=np.argmin(np.abs(a[:,0]-t)); return a[i]
def gbv(s):
    i=int(round((s%L)/ (L/len(gw)))) % len(gw)
    return gw[np.argmin(np.abs(gw[:,0]-(s%L))),8]
# otwpnts 발행 시각 집합 -> 회피 활성
ot_t=np.array([t for t,w in otw if len(w)])
rows=[]
for t,w in otw:
    if not (T0<=t<=T1) or len(w)==0: continue
    f=near(fre,t); p=near(pf,t); a=near(ack,t)
    s=f[1]; v=p[4]
    head=w[0,8]
    # 경로 위 0.5m/1.5m 앞 계획속도
    ds=np.r_[0,np.cumsum(np.hypot(np.diff(w[:,2]),np.diff(w[:,3])))]
    def vat(x):
        k=int(np.searchsorted(ds,x)); return w[min(k,len(w)-1),8]
    rows.append((t,s,v,a[2],head,vat(0.5),vat(1.5),gbv(s),w[:,8].min(),np.median(w[:,8])))
A=np.array(rows)
print("회피(otwpnts 비어있지 않음) 사이클 %d, 총 %.1f s (주행구간 %.0f s 의 %.1f%%)"%(
    len(A),len(A)*0.033,T1-T0,100*len(A)*0.033/(T1-T0)))
lab=["실제 v","명령 v","경로 head","경로 0.5m","경로 1.5m","gb 계획","경로 min","경로 중앙"]
idx=[2,3,4,5,6,7,8,9]
print("\n            p10    p50    p90   mean")
for l,i in zip(lab,idx):
    x=A[:,i]; print("%-10s %6.2f %6.2f %6.2f %6.2f"%(l,np.percentile(x,10),np.median(x),np.percentile(x,90),x.mean()))
print("\ngb 대비 손실: 명령/gb 중앙 %.2f  경로1.5m/gb 중앙 %.2f  실제/gb 중앙 %.2f"%(
    np.median(A[:,3]/A[:,7]),np.median(A[:,6]/A[:,7]),np.median(A[:,2]/A[:,7])))
# 회피 없는 구간
mask=np.ones(len(pf),bool)
for t in ot_t:
    mask &= ~(np.abs(pf[:,0]-t)<0.05)
free=pf[(pf[:,0]>T0)&(pf[:,0]<T1)&mask]
fv=[]
for r in free:
    fv.append(r[4]/gbv(near(fre,r[0])[1]))
print("회피 없는 사이클 실제/gb 중앙 %.2f (n=%d)"%(np.median(fv),len(fv)))
