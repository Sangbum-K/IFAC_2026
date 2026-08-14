#!/usr/bin/env python3
"""맵에 없는 LiDAR 점(=콘) 추출 → 콘 위치 정밀 추정 + 통과 가능 폭 계산."""
import pickle, sqlite3, sys
import numpy as np
sys.path.insert(0,"/home/miru/IFAC_ws/tools")
from bagviz import dec_grid
from scipy import ndimage
BAG="/home/miru/IFAC_ws/rosbag_0813/run_0813_221645/run_0813_221645_0.db3"
db=sqlite3.connect(BAG); T={n:i for i,n in db.execute("select id,name from topics")}
res,ox,oy,w,h,grid=dec_grid(db.execute("select data from messages where topic_id=?",(T['/map'],)).fetchone()[0])
occ=(grid>50)
# 맵 벽에서 0.20m 이내는 '맵으로 설명됨'
dist_map=ndimage.distance_transform_edt(~occ)*res
P=np.load("scanpts.npy")
ix=np.clip(((P[:,0]-ox)/res).astype(int),0,w-1); iy=np.clip(((P[:,1]-oy)/res).astype(int),0,h-1)
dmap=dist_map[iy,ix]
NEW=P[dmap>0.22]                       # 맵에 없는 점
print(f"scan {len(P)}  맵밖 {len(NEW)}")
# 격자 밀도 -> 콘 후보
H,xe,ye=np.histogram2d(NEW[:,0],NEW[:,1],bins=[np.arange(ox,ox+w*res,0.10),np.arange(oy,oy+h*res,0.10)])
lab,n=ndimage.label(H>25)
print(f"콘 후보 {n}")
cands=[]
for i in range(1,n+1):
    m=lab==i
    if H[m].sum()<400: continue
    cx=np.average((xe[:-1]+0.05)[np.where(m)[0]],weights=H[m])
    cy=np.average((ye[:-1]+0.05)[np.where(m)[1]],weights=H[m])
    cands.append((cx,cy,H[m].sum(),m.sum()*0.01))
cands.sort(key=lambda c:-c[2])
D=pickle.load(open("data.pkl","rb")); _,gb=D["global_wpnts"][0]
gs,gx,gy,gdr,gdl,gpsi,gkap=gb[:,0],gb[:,2],gb[:,3],gb[:,4],gb[:,5],gb[:,6],gb[:,7]
def to_frenet(px,py):
    d2=(gx-px)**2+(gy-py)**2; i=np.argmin(d2)
    nx,ny=-np.sin(gpsi[i]),np.cos(gpsi[i])
    return gs[i], (px-gx[i])*nx+(py-gy[i])*ny
print("\n=== 맵에 없는 물체(콘) 후보 ===")
CONES=[]
for cx,cy,cnt,area in cands:
    s,d=to_frenet(cx,cy)
    i=np.argmin(np.abs(gs-s))
    print(f"  xy=({cx:+6.2f},{cy:+6.2f}) pts={cnt:7.0f} area={area:.2f}m² -> s={s:6.2f} d={d:+5.2f} "
          f"트랙d[{-gdr[i]:+.2f},{gdl[i]:+.2f}] kappa={gkap[i]:+.3f}")
    CONES.append(dict(x=cx,y=cy,s=s,d=d,pts=cnt,area=area))
pickle.dump(CONES,open("cones.pkl","wb"))
