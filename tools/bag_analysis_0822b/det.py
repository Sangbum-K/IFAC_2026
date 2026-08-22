#!/usr/bin/env python3
"""트랙(id) 단위 수명/위치/벽거리 — 유령 서명(짧은 수명, 벽 근접) 계측"""
import pickle,sys,numpy as np
from scipy import ndimage
D=pickle.load(open(sys.argv[1],"rb"))
t0,t1=(float(sys.argv[2]),float(sys.argv[3])) if len(sys.argv)>3 else (0,1e9)
m=D["map"]; occ=np.array(m["data"]).reshape(m["h"],m["w"])>=65
edt=ndimage.distance_transform_edt(~occ)*m["res"]
def wd(x,y):
    j=int((x-m["ox"])/m["res"]); i=int((y-m["oy"])/m["res"])
    return float(edt[i,j]) if 0<=i<m["h"] and 0<=j<m["w"] else -1.0
tr={}
for t,o in D["obs_frenet"]:
    if not(t0<=t<=t1): continue
    for ob in o:
        tr.setdefault(ob["id"],[]).append((t,ob["x"],ob["y"],ob["is_visible"],ob["is_static"],ob["size"]))
life=[];rows=[]
for i,v in tr.items():
    a=np.array([[x[0],x[1],x[2]] for x in v])
    dur=a[-1,0]-a[0,0]
    disp=float(np.max(np.hypot(a[:,1]-a[0,1],a[:,2]-a[0,2])))
    w=np.median([wd(x,y) for _,x,y in a])
    rows.append((dur,len(v),i,w,disp,np.mean([x[3] for x in v])))
    life.append(dur)
life=np.array(life); rows.sort(reverse=True)
print("트랙 %d개  수명: p50 %.2fs p90 %.2fs max %.2fs | <0.3s %d개(%.0f%%) <1.0s %d개(%.0f%%)"%(
    len(rows),np.median(life),np.percentile(life,90),life.max(),
    int((life<0.3).sum()),100*np.mean(life<0.3),int((life<1.0).sum()),100*np.mean(life<1.0)))
short=[r for r in rows if r[0]<1.0]; long=[r for r in rows if r[0]>=1.0]
for nm,g in (("수명<1s (유령 후보)",short),("수명>=1s",long)):
    if not g: continue
    w=np.array([r[3] for r in g])
    print("  %-18s %3d개  벽거리 p10/p50/p90 = %.2f/%.2f/%.2f  발행수 합 %d"%(
        nm,len(g),np.percentile(w,10),np.median(w),np.percentile(w,90),sum(r[1] for r in g)))
print("  상위 수명 트랙:")
for dur,n,i,w,disp,vis in rows[:8]:
    print("    id=%-4d %6.1fs n=%-5d 벽거리 %.2f 이동 %.2fm 가시 %.0f%%"%(i,dur,n,w,disp,100*vis))
