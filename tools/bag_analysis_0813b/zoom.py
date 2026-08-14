#!/usr/bin/env python3
import pickle, sqlite3, sys
import numpy as np
sys.path.insert(0,"/home/miru/IFAC_ws/tools")
from bagviz import dec_grid
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

BAG="/home/miru/IFAC_ws/rosbag_0813/run_0813_221645/run_0813_221645_0.db3"
db=sqlite3.connect(BAG)
tid=db.execute("select id from topics where name='/map'").fetchone()[0]
raw=db.execute("select data from messages where topic_id=?",(tid,)).fetchone()[0]
res,ox,oy,w,h,grid=dec_grid(raw)
D=pickle.load(open("data.pkl","rb")); OB=pickle.load(open("obst.pkl","rb"))
_,gb=D["global_wpnts"][0]
gs,gx,gy,gdr,gdl,gpsi=gb[:,0],gb[:,2],gb[:,3],gb[:,4],gb[:,5],gb[:,6]
pf=D["/pf/pose/odom"]; fre=D["/car_state/frenet/odom"]
nx,ny=-np.sin(gpsi),np.cos(gpsi)

# 근거리 장애물 관측 원시점
pts=[]
for t,obs in D["obs_frenet"]:
    es=np.interp(t,fre[:,0],fre[:,1])
    for o in obs:
        gap=(o["s_center"]-es)%32.07
        if gap>16: gap-=32.07
        if -0.5<gap<3.0: pts.append((o["x"],o["y"]))
pts=np.array(pts)

ZOOM=[("O2_square_right",4.98,2.72,2.2),("O6_hairpin",-8.59,2.76,2.2),
      ("O7_straight",-2.44,-0.04,2.2),("O3_topright",1.64,4.54,2.2)]
fig,axs=plt.subplots(2,2,figsize=(15,13))
ext=[ox,ox+w*res,oy,oy+h*res]
for ax,(nm,cx,cy,r) in zip(axs.ravel(),ZOOM):
    ax.imshow(np.where(grid<0,50,grid),origin="lower",extent=ext,cmap="gray_r",vmin=0,vmax=100,alpha=.8)
    ax.plot(pf[:,1],pf[:,2],lw=.5,color="tab:blue",alpha=.4)
    ax.plot(gx,gy,lw=1.8,color="tab:red")
    ax.plot(gx+nx*gdl,gy+ny*gdl,lw=1,color="tab:green",ls="--")
    ax.plot(gx-nx*gdr,gy-ny*gdr,lw=1,color="tab:green",ls="--")
    ax.scatter(pts[:,0],pts[:,1],s=2,color="tab:orange",alpha=.25)
    for o in OB:
        ax.add_patch(plt.Circle((o["x"],o["y"]),o["size"]/2,fill=False,ec="darkred",lw=2))
        ax.add_patch(plt.Circle((o["x"],o["y"]),o["size"]/2+0.15,fill=False,ec="darkred",lw=.8,ls=":"))
    ax.set_xlim(cx-r,cx+r); ax.set_ylim(cy-r,cy+r); ax.set_aspect("equal"); ax.grid(alpha=.3)
    ax.set_title(nm)
plt.tight_layout(); plt.savefig("zoom.png",dpi=105); print("ok")
