#!/usr/bin/env python3
"""자차 정지 구간에서 위치별 '보인 프레임 비율' — 진짜 콘 vs 유령 분리
정지 중이므로 시야는 고정이다. 실물이면 거의 매 프레임 보이고, 유령은 깜빡인다."""
import pickle,sys,numpy as np
from scipy import ndimage
D=pickle.load(open(sys.argv[1],"rb")); t0,t1=float(sys.argv[2]),float(sys.argv[3])
m=D["map"]; res=m["res"]; ox=m["ox"]; oy=m["oy"]; W=m["w"]; H=m["h"]
occ=np.array(m["data"]).reshape(H,W)>=65
edt=ndimage.distance_transform_edt(~occ)*res
def wd(x,y):
    j=int((x-ox)/res); i=int((y-oy)/res)
    return float(edt[i,j]) if 0<=i<H and 0<=j<W else -1.0
gw=D["global_wpnts"][0][1]; gxy=gw[:,2:4]
def sd(x,y):
    k=int(np.argmin(np.sum((gxy-[x,y])**2,axis=1)))
    dx,dy=x-gw[k,2],y-gw[k,3]; psi=gw[k,6]
    return gw[k,0],-np.sin(psi)*dx+np.cos(psi)*dy,gw[k,4],gw[k,5]
key=sys.argv[4] if len(sys.argv)>4 else "obs_frenet"
frames=[(t,o) for t,o in D[key] if t0<=t<=t1]
pts=[];fidx=[]
for k,(t,o) in enumerate(frames):
    for ob in o:
        pts.append((ob["x"],ob["y"],ob.get("size",0),ob.get("is_visible",True))); fidx.append(k)
if not pts: print("장애물 없음"); sys.exit()
P=np.array([[p[0],p[1]] for p in pts]); fidx=np.array(fidx)
# 0.20 m 격자 기반 그리드 클러스터 (체이닝 방지)
cell=np.round(P/0.20).astype(int)
uniq,inv=np.unique(cell,axis=0,return_inverse=True)
# 인접 셀 병합
lab=-np.ones(len(uniq),int); nl=0
for i in range(len(uniq)):
    if lab[i]>=0: continue
    st=[i]; lab[i]=nl
    while st:
        k=st.pop()
        nb=np.where((lab<0)&(np.abs(uniq[:,0]-uniq[k,0])<=1)&(np.abs(uniq[:,1]-uniq[k,1])<=1))[0]
        for j in nb: lab[j]=nl; st.append(j)
    nl+=1
grp=lab[inv]
N=len(frames)
print("정지구간 t=%.1f..%.1f  프레임 %d  (%s)"%(t0,t1,N,key))
print("loc    n   보인프레임%%   s      d     벽거리  size   가시%%")
rows=[]
for g in range(nl):
    sel=grp==g
    if sel.sum()<5: continue
    x,y=P[sel,0].mean(),P[sel,1].mean()
    s,d,dr,dl=sd(x,y)
    cov=len(np.unique(fidx[sel]))/N
    sz=np.median([pts[i][2] for i in np.where(sel)[0]])
    vis=np.mean([pts[i][3] for i in np.where(sel)[0]])
    rows.append((cov,sel.sum(),s,d,wd(x,y),sz,vis))
rows.sort(reverse=True)
for cov,n,s,d,w,sz,vis in rows[:25]:
    print("     %5d  %6.1f%%  %6.2f %+6.2f  %5.2f  %.2f  %4.0f%%"%(n,100*cov,s,d,w,sz,100*vis))
print("총 위치 %d, 프레임당 평균 장애물 %.2f"%(len(rows),len(pts)/N))
