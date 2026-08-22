#!/usr/bin/env python3
"""벽 오인지 계측: 발행 장애물 centroid 의 맵 벽거리(EDT) 분포"""
import pickle,sys,numpy as np
from scipy import ndimage
D=pickle.load(open(sys.argv[1],"rb"))
m=D["map"]; res=m["res"]; ox=m["ox"]; oy=m["oy"]; W=m["w"]; H=m["h"]
grid=np.array(m["data"]).reshape(H,W)
occ=grid>=65
# 자유공간에서 가장 가까운 점유셀까지 거리
edt=ndimage.distance_transform_edt(~occ)*res
def wd(x,y):
    j=int((x-ox)/res); i=int((y-oy)/res)
    if i<0 or j<0 or i>=H or j>=W: return -1.0
    return float(edt[i,j])
gw=D["global_wpnts"][0][1]
gxy=gw[:,2:4]
def track_dl_dr(x,y):
    k=int(np.argmin(np.sum((gxy-[x,y])**2,axis=1)))
    return gw[k,0],gw[k,4],gw[k,5]
for key,label in (("obs_perc","detector /perception/obstacles"),
                  ("obs_frenet","planner  /perception/obstacles_frenet")):
    ds=[];n=0;frames=0
    for t,o in D[key]:
        frames+=1
        for ob in o:
            n+=1; ds.append(wd(ob["x"],ob["y"]))
    ds=np.array(ds)
    if len(ds)==0: print("%s: 0"%label); continue
    q=np.percentile(ds[ds>=0],[5,10,25,50,75,90])
    print("%-42s 인스턴스 %6d / %5d 프레임 | 벽거리 p5/10/25/50/75/90 = %s | <0.15 %4.1f%%  <0.30 %4.1f%%  맵밖 %d"%(
        label,n,frames," ".join("%.2f"%v for v in q),
        100*np.mean(ds<0.15),100*np.mean(ds<0.30),int((ds<0).sum())))
