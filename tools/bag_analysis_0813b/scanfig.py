#!/usr/bin/env python3
import pickle, sqlite3, sys
import numpy as np
sys.path.insert(0,"/home/miru/IFAC_ws/tools")
from bagviz import dec_grid
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
BAG="/home/miru/IFAC_ws/rosbag_0813/run_0813_221645/run_0813_221645_0.db3"
db=sqlite3.connect(BAG); T={n:i for i,n in db.execute("select id,name from topics")}
res,ox,oy,w,h,grid=dec_grid(db.execute("select data from messages where topic_id=?",(T['/map'],)).fetchone()[0])
P=np.load("scanpts.npy"); D=pickle.load(open("data.pkl","rb")); OB=pickle.load(open("obst.pkl","rb"))
_,gb=D["global_wpnts"][0]; gs,gx,gy,gdr,gdl,gpsi=gb[:,0],gb[:,2],gb[:,3],gb[:,4],gb[:,5],gb[:,6]
nx,ny=-np.sin(gpsi),np.cos(gpsi)
# 2D 히스토그램(밀도)
Z=[("O2 square-right",4.98,2.72),("O6 hairpin",-8.59,2.76),
   ("O7 straight",-2.44,-0.04),("O3 top-right",1.64,4.54),
   ("O4 diagonal",-2.52,2.30),("O1 wall?",4.98,-0.49)]
fig,axs=plt.subplots(2,3,figsize=(19,12))
ext=[ox,ox+w*res,oy,oy+h*res]
for ax,(nm,cx,cy) in zip(axs.ravel(),Z):
    r=1.8
    ax.imshow(np.where(grid<0,40,grid),origin="lower",extent=ext,cmap="Greys",vmin=0,vmax=100,alpha=.45)
    m=(np.abs(P[:,0]-cx)<r)&(np.abs(P[:,1]-cy)<r)
    ax.hexbin(P[m,0],P[m,1],gridsize=90,cmap="inferno_r",mincnt=1,bins="log",alpha=.9)
    ax.plot(gx,gy,lw=1.6,color="tab:red")
    ax.plot(gx+nx*gdl,gy+ny*gdl,lw=1,color="tab:green",ls="--")
    ax.plot(gx-nx*gdr,gy-ny*gdr,lw=1,color="tab:green",ls="--")
    for o in OB:
        ax.add_patch(plt.Circle((o["x"],o["y"]),o["size"]/2,fill=False,ec="tab:blue",lw=2.2))
    ax.set_xlim(cx-r,cx+r); ax.set_ylim(cy-r,cy+r); ax.set_aspect("equal"); ax.grid(alpha=.25)
    ax.set_title(nm)
plt.tight_layout(); plt.savefig("scanzoom.png",dpi=100); print("ok")
