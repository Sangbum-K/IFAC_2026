#!/usr/bin/env python3
import pickle,numpy as np
D=pickle.load(open("d17.pkl","rb"))
T0,T1=4,170
h=D["/pitwall/avoidance_loss_hold"]; cap=D["/pitwall/avoidance_loss_hold_cap"]
mem=D["/pitwall/avoidance_loss_hold_memory"]; pf=D["/pf/pose/odom"]; fre=D["/car_state/frenet/odom"]
gw=D["global_wpnts"][0][1]; TL=gw[-1,0]
def near(a,t): return a[np.argmin(np.abs(a[:,0]-t))]
def gbv(s): return gw[np.argmin(np.abs(gw[:,0]-(s%TL))),8]
m=(h[:,0]>T0)&(h[:,0]<T1); H=h[m]
on=H[H[:,1]>0.5]
print("avoidance_loss_hold: 발행 %d, 그중 활성 %d (%.1f%%)"%(len(H),len(on),100*len(on)/len(H)))
if len(cap):
    mc=(cap[:,0]>T0)&(cap[:,0]<T1); C=cap[mc]
    print("hold cap: n=%d  p10 %.2f p50 %.2f p90 %.2f"%(len(C),*np.percentile(C[:,1],[10,50,90])))
    rr=[]
    for t,c in C:
        f=near(fre,t); rr.append((c,gbv(f[1]),near(pf,t)[4]))
    R=np.array(rr)
    print("  hold 중: cap 중앙 %.2f / gb 중앙 %.2f / 실제v 중앙 %.2f  -> cap이 gb의 %.0f%%"%(
        np.median(R[:,0]),np.median(R[:,1]),np.median(R[:,2]),100*np.median(R[:,0]/R[:,1])))
    print("  cap 이 실제 속도보다 낮았던 사이클(= 실제 감속 강제): %.1f%%"%(100*np.mean(R[:,0]<R[:,2])))
# hold 지속 구간
t=H[:,0]; v=H[:,1]>0.5
segs=[];i=0
while i<len(H):
    if v[i]:
        j=i
        while j+1<len(H) and v[j+1]: j+=1
        segs.append((t[i],t[j])); i=j+1
    else: i+=1
segs=[s for s in segs if s[1]-s[0]>0.02]
print("hold 연속구간 %d개 총 %.1fs 최장 %.2fs 중앙 %.2fs"%(
    len(segs),sum(b-a for a,b in segs),max(b-a for a,b in segs),np.median([b-a for a,b in segs])))
