#!/usr/bin/env python3
import pickle,numpy as np
D=pickle.load(open("d17.pkl","rb"))
fre=D["/car_state/frenet/odom"]; pf=D["/pf/pose/odom"]; otw=D["otwpnts"]
gw=D["global_wpnts"][0][1]; TL=gw[-1,0]
def near(a,t): return a[np.argmin(np.abs(a[:,0]-t))]
R=[]
for t,w in otw:
    if not (4<=t<=170) or len(w)<8: continue
    f=near(fre,t); p=near(pf,t)
    lead=(w[0,0]-f[1])%TL
    if lead>TL/2: lead-=TL
    R.append((t,f[4],p[4],w[0,8],w[:,8].max(),lead,w[0,1],f[2],len(w)))
A=np.array(R)
print("n=%d"%len(A))
print("frenet vs 중앙 %.2f | pf v 중앙 %.2f | head vx 중앙 %.2f | head/vs 중앙 %.2f"%(
    np.median(A[:,1]),np.median(A[:,2]),np.median(A[:,3]),np.median(A[:,3]/np.maximum(A[:,1],0.1))))
print("경로 머리의 자차 앞 거리(lead) 중앙 %.2f  p10 %.2f p90 %.2f"%(
    np.median(A[:,5]),*np.percentile(A[:,5],[10,90])))
print("head vx < frenet vs 인 사이클: %.1f%%   (head < 0.8*vs: %.1f%%)"%(
    100*np.mean(A[:,3]<A[:,1]),100*np.mean(A[:,3]<0.8*A[:,1])))
print("경로 최대 vx 중앙 %.2f  |  점 수 중앙 %d"%(np.median(A[:,4]),int(np.median(A[:,8]))))
print()
print("[head/vs 비율 구간별]")
r=A[:,3]/np.maximum(A[:,1],0.1)
for lo,hi in ((0,0.3),(0.3,0.6),(0.6,0.8),(0.8,0.95),(0.95,1.05),(1.05,9)):
    m=(r>=lo)&(r<hi)
    if m.sum(): print("  %.2f-%.2f: %5.1f%%  vs중앙 %.2f head중앙 %.2f lead중앙 %.2f"%(
        lo,hi,100*m.mean(),np.median(A[m,1]),np.median(A[m,3]),np.median(A[m,5])))
