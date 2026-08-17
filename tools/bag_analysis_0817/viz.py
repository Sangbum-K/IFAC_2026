import pickle, numpy as np
from pathlib import Path
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
P=Path(__file__).parent
D=pickle.load(open(P/"d124904.pkl","rb"))
mp=D["map"]; res=mp["res"]; ox,oy=mp["ox"],mp["oy"]
grid=np.array(mp["data"]).reshape(mp["h"],mp["w"])
pf=D["/pf/pose/odom"]      # t x y yaw v
gw=D["global_wpnts"][0][1] # s d x y d_r d_l psi kappa vx ax
LX=0.28   # base_link -> laser
def pose(t):
    return (np.interp(t,pf[:,0],pf[:,1]), np.interp(t,pf[:,0],pf[:,2]),
            np.arctan2(np.interp(t,pf[:,0],np.sin(pf[:,3])),np.interp(t,pf[:,0],np.cos(pf[:,3]))))
def scan_xy(t):
    k=min(range(len(D["scan"])),key=lambda i:abs(D["scan"][i][0]-t))
    ts,amin,ainc,r=D["scan"][k]
    n=len(r); ang=amin+ainc*np.arange(n); ok=(r>0.06)&(r<10)&np.isfinite(r)
    x,y=r[ok]*np.cos(ang[ok]), r[ok]*np.sin(ang[ok])
    px,py,yw=pose(ts); c,s=np.cos(yw),np.sin(yw)
    bx,by=x+LX,y
    return ts, px+c*bx-s*by, py+s*bx+c*by
TS=[59.5,59.9,60.3,60.6,60.9,61.3]
fig,axes=plt.subplots(2,3,figsize=(19,11))
for ax,t in zip(axes.ravel(),TS):
    ax.imshow(grid,origin="lower",extent=[ox,ox+mp["w"]*res,oy,oy+mp["h"]*res],cmap="Greys",vmin=-1,vmax=100,alpha=.55)
    ax.plot(gw[:,2],gw[:,3],"-",c="tab:blue",lw=1,alpha=.6,label="raceline")
    m=(pf[:,0]>=t-3)&(pf[:,0]<=t)
    ax.plot(pf[m,1],pf[m,2],"-",c="tab:green",lw=2,label="ego path(-3s)")
    ts,sx,sy=scan_xy(t)
    ax.plot(sx,sy,".",c="tab:red",ms=2.5,label=f"scan@{ts:.2f}")
    px,py,yw=pose(t)
    ax.plot([px,px+0.4*np.cos(yw)],[py,py+0.4*np.sin(yw)],"-",c="k",lw=2.5)
    ax.plot(px,py,"o",c="k",ms=7)
    # detected obstacles (map frame) nearest in time
    k=min(range(len(D["obs_perc"])),key=lambda i:abs(D["obs_perc"][i][0]-t))
    to,obs=D["obs_perc"][k]
    for o in obs:
        ax.add_patch(Circle((o["x"],o["y"]),max(o["radius"],0.05),fill=False,ec="tab:orange",lw=2))
        ax.text(o["x"],o["y"]+0.18,f"#{o['id']}",color="tab:orange",fontsize=8,ha="center")
    # avoidance line
    k=min(range(len(D["otwpnts"])),key=lambda i:abs(D["otwpnts"][i][0]-t))
    tw,W=D["otwpnts"][k]
    if len(W): ax.plot(W[:,2],W[:,3],"-",c="magenta",lw=2,label=f"otwpnts n={len(W)}@{tw:.2f}")
    ax.set_xlim(px-3.2,px+3.2); ax.set_ylim(py-2.4,py+2.4); ax.set_aspect("equal")
    ax.set_title(f"t={t:.2f}s  obs@{to:.2f}(n={len(obs)})"); ax.legend(fontsize=7,loc="upper left")
plt.tight_layout(); plt.savefig(P/"collision_60s.png",dpi=110)
print("saved", P/"collision_60s.png")
