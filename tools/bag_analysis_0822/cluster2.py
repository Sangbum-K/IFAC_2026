#!/usr/bin/env python3
"""클러스터별 인스턴스 단위 통계 (단일연결 체이닝 확인용)"""
import pickle,sys,numpy as np
from scipy import ndimage
D=pickle.load(open(sys.argv[1],"rb"))
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
    return gw[k,0],-np.sin(psi)*dx+np.cos(psi)*dy,gw[k,4],gw[k,5]
pts=[]
for t,o in D["obs_frenet"]:
    for ob in o: pts.append((t,ob["x"],ob["y"]))
P=np.array([[p[1],p[2]] for p in pts])
used=np.zeros(len(P),bool); groups=[]
for i in np.argsort(P[:,0]):
    if used[i]: continue
    stack=[i]; used[i]=True; g=[i]
    while stack:
        k=stack.pop()
        nb=np.where((np.sum((P-P[k])**2,axis=1)<0.35**2)&(~used))[0]
        for j in nb: used[j]=True; g.append(j); stack.append(j)
    groups.append(g)
groups.sort(key=len,reverse=True)
print("#   n     s범위        d(p50)  퍼짐m  벽거리 p5/p50/p95   시간범위")
for gi,g in enumerate(groups[:8]):
    xs=P[g,0]; ys=P[g,1]
    ss=[];dds=[];ws=[]
    for x,y in zip(xs,ys):
        s,d,dr,dl=sd(x,y); ss.append(s); dds.append(d); ws.append(wd(x,y))
    ss=np.array(ss);ws=np.array(ws)
    ts=np.array([pts[i][0] for i in g])
    spread=max(xs.max()-xs.min(), ys.max()-ys.min())
    print("%-3d %5d  %5.1f..%5.1f  %+5.2f  %5.2f  %.2f/%.2f/%.2f   %.0f-%.0fs"%(
        gi,len(g),ss.min(),ss.max(),np.median(dds),spread,
        np.percentile(ws,5),np.median(ws),np.percentile(ws,95),ts.min(),ts.max()))
