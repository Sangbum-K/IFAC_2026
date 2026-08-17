import pickle, sqlite3, sys, numpy as np
from pathlib import Path
sys.path.insert(0,"/home/miru/IFAC_ws/tools")
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from bagviz import CDR, dec_scan
P=Path(__file__).parent
D=pickle.load(open(P/"d124904.pkl","rb"))
BAG="/home/miru/IFAC_ws/rosbag0817/run_0817_124904/run_0817_124904_0.db3"
db=sqlite3.connect(BAG); t0=db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
tid=dict((n,i) for i,n,_ in db.execute("SELECT id,name,type FROM topics"))["/scan"]
SC=[]
for ts,d in db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",(tid,)):
    t=(ts-t0)/1e9
    if 58.0<=t<=62.5:
        ht,amin,ainc,tinc,r=dec_scan(d,1); SC.append((t,amin,ainc,np.asarray(r)))
print("full-res scans:",len(SC))
mp=D["map"]; res=mp["res"]; ox,oy=mp["ox"],mp["oy"]
grid=np.array(mp["data"]).reshape(mp["h"],mp["w"])
occ=grid>50
pf=D["/pf/pose/odom"]; gw=D["global_wpnts"][0][1]; LX=0.28
def pose(t):
    return (np.interp(t,pf[:,0],pf[:,1]), np.interp(t,pf[:,0],pf[:,2]),
            np.arctan2(np.interp(t,pf[:,0],np.sin(pf[:,3])),np.interp(t,pf[:,0],np.cos(pf[:,3]))))
def near_wall(x,y,rad=0.20):
    k=int(rad/res)
    ix=((x-ox)/res).astype(int); iy=((y-oy)/res).astype(int)
    out=np.zeros(len(x),bool)
    for dx in range(-k,k+1):
        for dy in range(-k,k+1):
            jx=np.clip(ix+dx,0,mp["w"]-1); jy=np.clip(iy+dy,0,mp["h"]-1)
            out |= occ[jy,jx]
    return out
def scan_xy(t):
    k=min(range(len(SC)),key=lambda i:abs(SC[i][0]-t))
    ts,amin,ainc,r=SC[k]
    n=len(r); ang=amin+ainc*np.arange(n); ok=(r>0.06)&(r<8)&np.isfinite(r)
    x,y=r[ok]*np.cos(ang[ok]), r[ok]*np.sin(ang[ok])
    px,py,yw=pose(ts); c,s=np.cos(yw),np.sin(yw); bx,by=x+LX,y
    return ts, px+c*bx-s*by, py+s*bx+c*by
TS=[59.4,59.8,60.1,60.4,60.7,61.0,61.3,61.8]
fig,axes=plt.subplots(2,4,figsize=(24,11))
for ax,t in zip(axes.ravel(),TS):
    ax.imshow(grid,origin="lower",extent=[ox,ox+mp["w"]*res,oy,oy+mp["h"]*res],cmap="Greys",vmin=-1,vmax=100,alpha=.45)
    ax.plot(gw[:,2],gw[:,3],"-",c="tab:blue",lw=1,alpha=.7)
    m=(pf[:,0]>=t-2.5)&(pf[:,0]<=t); ax.plot(pf[m,1],pf[m,2],"-",c="tab:green",lw=2)
    ts,sx,sy=scan_xy(t); w=near_wall(sx,sy)
    ax.plot(sx[w],sy[w],".",c="0.45",ms=2)
    ax.plot(sx[~w],sy[~w],".",c="red",ms=4)
    px,py,yw=pose(t); c,s=np.cos(yw),np.sin(yw)
    # car footprint 0.55 x 0.30, rear axle at base_link
    for (lx,ly) in [(-0.1,-0.15),(-0.1,0.15),(0.45,0.15),(0.45,-0.15),(-0.1,-0.15)]:
        pass
    fp=np.array([(-0.1,-0.15),(0.45,-0.15),(0.45,0.15),(-0.1,0.15),(-0.1,-0.15)])
    fx=px+c*fp[:,0]-s*fp[:,1]; fy=py+s*fp[:,0]+c*fp[:,1]
    ax.plot(fx,fy,"-",c="k",lw=2)
    k=min(range(len(D["obs_perc"])),key=lambda i:abs(D["obs_perc"][i][0]-t))
    to,obs=D["obs_perc"][k]
    for o in obs:
        ax.add_patch(Circle((o["x"],o["y"]),max(o["radius"],0.05),fill=False,ec="tab:orange",lw=2))
        ax.text(o["x"],o["y"]+0.16,f"#{o['id']}",color="tab:orange",fontsize=8,ha="center")
    k=min(range(len(D["otwpnts"])),key=lambda i:abs(D["otwpnts"][i][0]-t))
    tw,W=D["otwpnts"][k]
    if len(W): ax.plot(W[:,2],W[:,3],"-",c="magenta",lw=2)
    k=min(range(len(D["local_wpnts"])),key=lambda i:abs(D["local_wpnts"][i][0]-t))
    tl,LW=D["local_wpnts"][k]
    if len(LW): ax.plot(LW[:,2],LW[:,3],"--",c="tab:cyan",lw=1.8)
    ax.set_xlim(px-2.6,px+2.6); ax.set_ylim(py-1.9,py+1.9); ax.set_aspect("equal")
    ax.set_title(f"t={t:.2f}  scan@{ts:.2f}  obs n={len(obs)}",fontsize=10)
axes[0,0].plot([],[],".",c="red",label="scan: 맵에 없음"); axes[0,0].plot([],[],".",c="0.45",label="scan: 맵 벽")
axes[0,0].plot([],[],"-",c="magenta",label="otwpnts(회피선)"); axes[0,0].plot([],[],"--",c="tab:cyan",label="local_wpnts")
axes[0,0].legend(fontsize=8,loc="lower left")
plt.tight_layout(); plt.savefig(P/"collision_fullres.png",dpi=105)
print("saved")
