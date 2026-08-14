#!/usr/bin/env python3
import pickle, sqlite3, sys
import numpy as np
sys.path.insert(0,"/home/miru/IFAC_ws/tools")
from bagviz import dec_grid
from scipy import ndimage
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
BAG="/home/miru/IFAC_ws/rosbag_0813/run_0813_221645/run_0813_221645_0.db3"
db=sqlite3.connect(BAG); T={n:i for i,n in db.execute("select id,name from topics")}
res,ox,oy,w,h,grid=dec_grid(db.execute("select data from messages where topic_id=?",(T['/map'],)).fetchone()[0])
occ=grid>50; dmapg=ndimage.distance_transform_edt(~occ)*res
P=np.load("scanpts.npy")
ix=np.clip(((P[:,0]-ox)/res).astype(int),0,w-1); iy=np.clip(((P[:,1]-oy)/res).astype(int),0,h-1)
NEW=P[dmapg[iy,ix]>0.22]
D=pickle.load(open("data.pkl","rb")); _,gb=D["global_wpnts"][0]
gs,gx,gy,gdr,gdl,gpsi=gb[:,0],gb[:,2],gb[:,3],gb[:,4],gb[:,5],gb[:,6]
nx,ny=-np.sin(gpsi),np.cos(gpsi)
pf=D["/pf/pose/odom"]
Z=[("O2 square-right s6.5",5.02,2.65,1.5),("O7 straight s28.9",-2.7,-0.05,1.5),
   ("O6 hairpin s22.1",-8.6,2.8,1.5),("O3 top s10.5",1.55,4.55,1.5)]
fig,axs=plt.subplots(2,2,figsize=(15,13))
ext=[ox,ox+w*res,oy,oy+h*res]
for ax,(nm,cx,cy,r) in zip(axs.ravel(),Z):
    ax.imshow(np.where(occ,1,0),origin="lower",extent=ext,cmap="Greys",vmin=0,vmax=1.6,alpha=.55)
    m=(np.abs(NEW[:,0]-cx)<r)&(np.abs(NEW[:,1]-cy)<r)
    ax.scatter(NEW[m,0],NEW[m,1],s=1.5,c="crimson",alpha=.10)
    ax.plot(pf[:,1],pf[:,2],lw=.4,color="tab:blue",alpha=.5)
    ax.plot(gx,gy,lw=1.5,color="tab:orange")
    ax.plot(gx+nx*gdl,gy+ny*gdl,lw=1,color="tab:green",ls="--")
    ax.plot(gx-nx*gdr,gy-ny*gdr,lw=1,color="tab:green",ls="--")
    ax.set_xlim(cx-r,cx+r); ax.set_ylim(cy-r,cy+r); ax.set_aspect("equal")
    ax.grid(alpha=.3,lw=.4); ax.set_title(nm+"  (red=map에 없는 LiDAR, blue=ego)")
    ax.set_xticks(np.arange(round(cx-r,1),cx+r,0.25)); ax.set_yticks(np.arange(round(cy-r,1),cy+r,0.25))
    ax.tick_params(labelsize=7)
plt.tight_layout(); plt.savefig("shape.png",dpi=105); print("ok")
