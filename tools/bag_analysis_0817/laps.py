import pickle, sqlite3, sys, numpy as np
from pathlib import Path
sys.path.insert(0,"/home/miru/IFAC_ws/tools")
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from bagviz import dec_scan
P=Path(__file__).parent
D=pickle.load(open(P/"d124904.pkl","rb"))
BAG="/home/miru/IFAC_ws/rosbag0817/run_0817_124904/run_0817_124904_0.db3"
db=sqlite3.connect(BAG); t0=db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
tid=dict((n,i) for i,n,_ in db.execute("SELECT id,name,type FROM topics"))["/scan"]
fre=D["/car_state/frenet/odom"]; pf=D["/pf/pose/odom"]
mp=D["map"]; res=mp["res"]; ox,oy=mp["ox"],mp["oy"]
grid=np.array(mp["data"]).reshape(mp["h"],mp["w"]); occ=grid>50
gw=D["global_wpnts"][0][1]; LX=0.28
# find times when ego s in [31,37.5] each lap
s=fre[:,1]; t=fre[:,0]
inwin=(s>=30.5)&(s<=37.4)
segs=[];i=0
while i<len(t):
    if inwin[i]:
        j=i
        while j+1<len(t) and inwin[j+1]: j+=1
        if t[j]-t[i]>0.5: segs.append((t[i],t[j]))
        i=j+1
    else: i+=1
print("straight passes:",[(round(a,1),round(b,1)) for a,b in segs])
SC={}
for ts,d in db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",(tid,)):
    tt=(ts-t0)/1e9
    for a,b in segs:
        if a<=tt<=b:
            ht,amin,ainc,tinc,r=dec_scan(d,2); SC.setdefault((a,b),[]).append((tt,amin,ainc,np.asarray(r))); break
def pose(tt):
    return (np.interp(tt,pf[:,0],pf[:,1]), np.interp(tt,pf[:,0],pf[:,2]),
            np.arctan2(np.interp(tt,pf[:,0],np.sin(pf[:,3])),np.interp(tt,pf[:,0],np.cos(pf[:,3]))))
def near_wall(x,y,rad=0.15):
    k=int(rad/res); ix=((x-ox)/res).astype(int); iy=((y-oy)/res).astype(int); out=np.zeros(len(x),bool)
    for dx in range(-k,k+1):
        for dy in range(-k,k+1):
            out |= occ[np.clip(iy+dy,0,mp["h"]-1),np.clip(ix+dx,0,mp["w"]-1)]
    return out
n=len(segs); fig,axes=plt.subplots(1,n,figsize=(5.2*n,5.0))
if n==1: axes=[axes]
for ax,(a,b) in zip(axes,segs):
    ax.imshow(grid,origin="lower",extent=[ox,ox+mp["w"]*res,oy,oy+mp["h"]*res],cmap="Greys",vmin=-1,vmax=100,alpha=.45)
    ax.plot(gw[:,2],gw[:,3],"-",c="tab:blue",lw=1)
    XS=[];YS=[]
    for tt,amin,ainc,r in SC.get((a,b),[]):
        ang=amin+ainc*np.arange(len(r)); ok=(r>0.06)&(r<6)&np.isfinite(r)
        x,y=r[ok]*np.cos(ang[ok])+LX, r[ok]*np.sin(ang[ok])
        px,py,yw=pose(tt); c,sn=np.cos(yw),np.sin(yw)
        XS.append(px+c*x-sn*y); YS.append(py+sn*x+c*y)
    if XS:
        X=np.concatenate(XS); Y=np.concatenate(YS); w=near_wall(X,Y)
        ax.plot(X[w],Y[w],".",c="0.6",ms=1); ax.plot(X[~w],Y[~w],".",c="red",ms=1.5)
    m=(pf[:,0]>=a)&(pf[:,0]<=b); ax.plot(pf[m,1],pf[m,2],"-",c="tab:green",lw=2)
    ax.set_xlim(-7.5,1.0); ax.set_ylim(-2.2,2.2); ax.set_aspect("equal")
    ax.set_title(f"pass {a:.1f}-{b:.1f}s (s 30.5->37.4)",fontsize=10)
plt.tight_layout(); plt.savefig(P/"straight_laps.png",dpi=110); print("saved")
