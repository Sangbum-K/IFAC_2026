#!/usr/bin/env python3
"""발행 장애물을 공간 클러스터링 -> 어디에 무엇이 얼마나 오래 보였나"""
import pickle,sys,numpy as np
from scipy import ndimage
D=pickle.load(open(sys.argv[1],"rb"))
t0,t1=(float(sys.argv[2]),float(sys.argv[3])) if len(sys.argv)>3 else (0,1e9)
m=D["map"]; res=m["res"]; ox=m["ox"]; oy=m["oy"]; W=m["w"]; H=m["h"]
grid=np.array(m["data"]).reshape(H,W); occ=grid>=65
edt=ndimage.distance_transform_edt(~occ)*res
def wd(x,y):
    j=int((x-ox)/res); i=int((y-oy)/res)
    return float(edt[i,j]) if 0<=i<H and 0<=j<W else -1.0
gw=D["global_wpnts"][0][1]; gxy=gw[:,2:4]
def sd(x,y):
    k=int(np.argmin(np.sum((gxy-[x,y])**2,axis=1)))
    dx,dy=x-gw[k,2],y-gw[k,3]; psi=gw[k,6]
    d=-np.sin(psi)*dx+np.cos(psi)*dy
    return gw[k,0],d,gw[k,4],gw[k,5]
pts=[]
for t,o in D["obs_frenet"]:
    if not(t0<=t<=t1): continue
    for ob in o: pts.append((t,ob["x"],ob["y"],ob["is_static"],ob["is_visible"],ob["size"],ob["id"]))
if not pts: print("없음"); sys.exit()
P=np.array([[p[1],p[2]] for p in pts])
# 0.35 m 단일연결 클러스터링
used=np.zeros(len(P),bool); groups=[]
order=np.argsort(P[:,0])
for i in order:
    if used[i]: continue
    stack=[i]; used[i]=True; g=[i]
    while stack:
        k=stack.pop()
        dd=np.sum((P-P[k])**2,axis=1)
        nb=np.where((dd<0.35**2)&(~used))[0]
        for j in nb: used[j]=True; g.append(j); stack.append(j)
    groups.append(g)
groups.sort(key=len,reverse=True)
print("%-3s %5s %6s %6s %6s %7s %6s %6s %6s %5s"%("#","n","x","y","s","d","벽거리","d_r","d_l","static%"))
for gi,g in enumerate(groups[:20]):
    xs=P[g,0].mean(); ys=P[g,1].mean()
    ts=np.array([pts[i][0] for i in g])
    s,d,dr,dl=sd(xs,ys)
    st=np.mean([pts[i][3] for i in g])
    print("%-3d %5d %6.2f %6.2f %6.2f %+6.2f %6.2f %6.2f %6.2f %4.0f%%  t %.1f-%.1f (%.1fs 커버)"%(
        gi,len(g),xs,ys,s,d,wd(xs,ys),dr,dl,100*st,ts.min(),ts.max(),
        len(np.unique(np.round(ts,1)))*0.1))
