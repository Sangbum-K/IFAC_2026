#!/usr/bin/env python3
"""발행 회피경로에서 각 속도 캡을 재구성해 무엇이 지배하는지 본다."""
import pickle,numpy as np
D=pickle.load(open("d17.pkl","rb"))
gw=D["global_wpnts"][0][1]; TL=gw[-1,0]; otw=D["otwpnts"]
A_LAT=8.83; WB=0.33; RATE=2.0; SCALE=1.0
def gbv(s): return gw[np.argmin(np.abs(gw[:,0]-(s%TL))),8]
res=[]
for t,w in otw:
    if not (4<=t<=170) or len(w)<8: continue
    x,y,k,vx=w[:,2],w[:,3],w[:,7],w[:,8]
    ds=np.hypot(np.diff(x),np.diff(y))
    st=np.arctan(WB*k)
    dst=np.abs(np.diff(st))
    vr=np.full(len(w),np.inf)
    with np.errstate(divide='ignore',invalid='ignore'):
        seg=np.where(dst>1e-5, ds*RATE/np.maximum(dst,1e-9), np.inf)
    vr[:-1]=np.minimum(vr[:-1],seg); vr[1:]=np.minimum(vr[1:],seg)
    vg=np.sqrt(A_LAT/np.maximum(np.abs(k),1e-6))
    vb=np.array([gbv(s) for s in w[:,0]])
    for j in range(len(w)):
        res.append((vx[j]*SCALE,vg[j],vr[j],vb[j],abs(k[j]),j,len(w)))
A=np.array(res)
v,vg,vr,vb=A[:,0],A[:,1],A[:,2],A[:,3]
print("웨이포인트 %d"%len(A))
print("발행 vx   중앙 %.2f  p10 %.2f  p90 %.2f"%(np.median(v),*np.percentile(v,[10,90])))
print("그립캡    중앙 %.2f   (vx가 이 캡의 95%% 이상: %.1f%%)"%(np.median(vg),100*np.mean(v>=0.95*vg)))
print("조향률캡  중앙 %.2f   (vx가 이 캡의 95%% 이상: %.1f%%)"%(np.median(vr),100*np.mean(v>=0.95*vr)))
print("gb 계획   중앙 %.2f   (vx가 이 값의 95%% 이상: %.1f%%)"%(np.median(vb),100*np.mean(v>=0.95*vb)))
mn=np.minimum(np.minimum(vg,vr),vb)
print("세 캡의 min 중앙 %.2f | vx/min 중앙 %.2f | vx가 min의 95%% 이상: %.1f%%"%(
    np.median(mn),np.median(v/mn),100*np.mean(v>=0.95*mn)))
who=np.where(vr<np.minimum(vg,vb),"조향률",np.where(vg<vb,"그립","gb"))
import collections
print("셋 중 가장 낮은 캡:",dict(collections.Counter(who)))
print()
print("[조향률 캡이 실제로 얼마나 낮은가]")
for lo,hi in ((0,1),(1,2),(2,3),(3,4),(4,6),(6,99)):
    m=(vr>=lo)&(vr<hi)
    if m.sum(): print("  vr %d-%d m/s: %6.1f%% 의 점, 그 점들의 발행 vx 중앙 %.2f"%(lo,hi,100*m.mean(),np.median(v[m])))
print()
print("[경로상 위치별] j=0 머리")
for a,b in ((0,3),(3,10),(10,20),(20,40),(40,200)):
    m=(A[:,5]>=a)&(A[:,5]<b)
    if m.sum(): print("  idx %3d-%3d: vx중앙 %.2f  그립 %.2f  조향률 %.2f  gb %.2f  |k|중앙 %.3f"%(
        a,b,np.median(v[m]),np.median(vg[m]),np.median(vr[m]),np.median(vb[m]),np.median(A[m,4])))
