#!/usr/bin/env python3
import pickle, sqlite3, sys
from pathlib import Path
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
print(f"map {w}x{h} res={res} origin=({ox},{oy})")

D=pickle.load(open("data.pkl","rb")); OB=pickle.load(open("obst.pkl","rb"))
_,gb=D["global_wpnts"][0]
gs,gx,gy,gdr,gdl,gpsi=gb[:,0],gb[:,2],gb[:,3],gb[:,4],gb[:,5],gb[:,6]
pf=D["/pf/pose/odom"]

fig,ax=plt.subplots(figsize=(13,11))
ext=[ox,ox+w*res,oy,oy+h*res]
ax.imshow(np.where(grid<0,50,grid),origin="lower",extent=ext,cmap="gray_r",vmin=0,vmax=100,alpha=.75)
ax.plot(pf[:,1],pf[:,2],lw=.35,color="tab:blue",alpha=.35,label="driven path (all laps)")
ax.plot(gx,gy,lw=2,color="tab:red",label="raceline")
# 경계
nx,ny=-np.sin(gpsi),np.cos(gpsi)   # left normal
ax.plot(gx+nx*gdl,gy+ny*gdl,lw=.8,color="tab:green",ls="--")
ax.plot(gx-nx*gdr,gy-ny*gdr,lw=.8,color="tab:green",ls="--")
for o in OB:
    ax.add_patch(plt.Circle((o["x"],o["y"]),o["size"]/2,color="tab:orange",alpha=.85,zorder=5))
    ax.annotate(f"{o['name']}\ns={o['s']:.1f}",(o["x"],o["y"]),xytext=(10,10),
                textcoords="offset points",fontsize=11,color="darkred",weight="bold")
for sm in range(0,32,2):
    i=np.argmin(abs(gs-sm)); ax.annotate(f"s{sm}",(gx[i],gy[i]),fontsize=8,color="navy")
ax.set_aspect("equal"); ax.grid(alpha=.25); ax.legend(loc="upper right", prop={'size':9})
ax.set_title("run_0813_221645  map + raceline + obstacles")
plt.tight_layout(); plt.savefig("map_overview.png",dpi=115)
print("saved map_overview.png")
