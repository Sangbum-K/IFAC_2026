#!/usr/bin/env python3
"""회랑 기하: 각 장애물에 대해 좌/우 레인이 성립하는가 + 반사실(반경/마진 축소)"""
import pickle,numpy as np
D=pickle.load(open("d17.pkl","rb"))
gw=D["global_wpnts"][0][1]; TL=gw[-1,0]; N=len(gw)
fre=D["/car_state/frenet/odom"]
S=gw[:,0]; DR=gw[:,4]; DL=gw[:,5]
def minwall(s0,s1):
    a=int(np.argmin(np.abs(S-(s0%TL)))); b=int(np.argmin(np.abs(S-(s1%TL))))
    idx=np.arange(a,b+1)%N if b>=a else np.r_[np.arange(a,N),np.arange(0,b+1)]
    return DL[idx].min(),DR[idx].min()
def near(a,t): return a[np.argmin(np.abs(a[:,0]-t))]
NEED=0.20; BM=0.25
rows=[]
for t,o in D["obs_frenet"]:
    if not(4<=t<=170) or not o: continue
    f=near(fre,t)
    for ob in o:
        d=(ob["s_start"]-f[1])%TL
        if d>12.0: continue
        wl,wr=minwall(ob["s_start"]-0.5,ob["s_end"]+0.5)
        rows.append((d,ob["d_left"],ob["d_right"],wl,wr,ob["d_left"]-ob["d_right"]))
A=np.array(rows)
print("장애물-사이클 %d (12 m 안)"%len(A))
def report(tag,shrink=0.0,need=NEED,bm=BM):
    dl=A[:,1]-shrink; dr=A[:,2]+shrink
    cl=A[:,3]-dl      # 좌 회랑
    cr=A[:,4]+dr      # 우 회랑
    okl=cl>=need+bm; okr=cr>=need+bm
    print("  %-46s 좌가능 %5.1f%%  우가능 %5.1f%%  **둘 다 불가 %5.1f%%**"%(
        tag,100*okl.mean(),100*okr.mean(),100*(~okl&~okr).mean()))
print("레인 성립 조건: 회랑 >= need(0.20) + 경계여유(0.25) = 0.45")
report("현재 (반경 0.25 고정원통)")
report("반경 0.25->0.17 (콘 반폭 -0.08)",shrink=0.08)
report("반경 0.25->0.15",shrink=0.10)
report("path_check_margin 0.20->0.15",need=0.15)
report("boundary_tracking_margin 0.10->0.05 (BM 0.20)",bm=0.20)
report("반경-0.08 & margin 0.15 & BM 0.20",shrink=0.08,need=0.15,bm=0.20)
print()
print("[회랑 여유 분포]")
cl=A[:,3]-A[:,1]; cr=A[:,4]+A[:,2]; best=np.maximum(cl,cr)
print("  넓은쪽 회랑 p10 %.2f p25 %.2f p50 %.2f p75 %.2f"%(*np.percentile(best,[10,25,50,75]),))
print("  0.45 미만 %.1f%% | 0.45~0.55 %.1f%% | 0.55~0.70 %.1f%%"%(
    100*np.mean(best<0.45),100*np.mean((best>=0.45)&(best<0.55)),100*np.mean((best>=0.55)&(best<0.70))))
