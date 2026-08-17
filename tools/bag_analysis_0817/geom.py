import pickle, sqlite3, sys, numpy as np
from pathlib import Path
sys.path.insert(0,"/home/miru/IFAC_ws/tools")
from bagviz import dec_scan
P=Path(__file__).parent
D=pickle.load(open(P/"d124904.pkl","rb"))
BAG="/home/miru/IFAC_ws/rosbag0817/run_0817_124904/run_0817_124904_0.db3"
db=sqlite3.connect(BAG); t0=db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
tid=dict((n,i) for i,n,_ in db.execute("SELECT id,name,type FROM topics"))["/scan"]
pf=D["/pf/pose/odom"]; mp=D["map"]; res=mp["res"]; ox,oy=mp["ox"],mp["oy"]
grid=np.array(mp["data"]).reshape(mp["h"],mp["w"]); occ=grid>50
gw=D["global_wpnts"][0][1]; LX=0.28
def pose(tt):
    return (np.interp(tt,pf[:,0],pf[:,1]), np.interp(tt,pf[:,0],pf[:,2]),
            np.arctan2(np.interp(tt,pf[:,0],np.sin(pf[:,3])),np.interp(tt,pf[:,0],np.cos(pf[:,3]))))
def near_wall(x,y,rad=0.15):
    k=int(rad/res); ix=((x-ox)/res).astype(int); iy=((y-oy)/res).astype(int); out=np.zeros(len(x),bool)
    for dx in range(-k,k+1):
        for dy in range(-k,k+1): out |= occ[np.clip(iy+dy,0,mp["h"]-1),np.clip(ix+dx,0,mp["w"]-1)]
    return out
# frenet projection onto global waypoints
GX,GY,GS,GPSI=gw[:,2],gw[:,3],gw[:,0],gw[:,6]
def to_frenet(X,Y):
    d2=(X[:,None]-GX[None,:])**2+(Y[:,None]-GY[None,:])**2
    j=d2.argmin(1)
    dx=X-GX[j]; dy=Y-GY[j]
    nx,ny=-np.sin(GPSI[j]),np.cos(GPSI[j])
    return GS[j], dx*nx+dy*ny
# accumulate laps 1-4 (obstacles unmoved)
XS=[];YS=[]
for ts,d in db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",(tid,)):
    tt=(ts-t0)/1e9
    if not any(a<=tt<=b for a,b in [(16.1,18.0),(27.1,29.4),(38.6,40.4),(48.8,50.8)]): continue
    ht,amin,ainc,tinc,r=dec_scan(d,2); r=np.asarray(r)
    ang=amin+ainc*np.arange(len(r)); ok=(r>0.06)&(r<5)&np.isfinite(r)
    x,y=r[ok]*np.cos(ang[ok])+LX, r[ok]*np.sin(ang[ok])
    px,py,yw=pose(tt); c,sn=np.cos(yw),np.sin(yw)
    XS.append(px+c*x-sn*y); YS.append(py+sn*x+c*y)
X=np.concatenate(XS); Y=np.concatenate(YS)
keep=~near_wall(X,Y)
X,Y=X[keep],Y[keep]
S,Dd=to_frenet(X,Y)
m=(S>=30.0)&(S<=37.5)&(np.abs(Dd)<1.4)
S,Dd=S[m],Dd[m]
print("off-map points on straight:",len(S))
# histogram in s
for lo in np.arange(30,37.5,0.25):
    sel=(S>=lo)&(S<lo+0.25)
    if sel.sum()>25:
        print(f"  s {lo:5.2f}-{lo+0.25:5.2f}  n={sel.sum():5d}  d [{Dd[sel].min():+5.2f},{Dd[sel].max():+5.2f}]  d_med {np.median(Dd[sel]):+5.2f}")
print("\n--- 트랙 폭 (global wpnts) s 30-37.5 ---")
for r in gw:
    if 30<=r[0]<=37.5 and abs(r[0]%1.0)<0.51:
        print(f"  s={r[0]:5.2f} d_right=-{r[4]:.2f} d_left=+{r[5]:.2f}  width={r[4]+r[5]:.2f}  vx={r[8]:.2f}")
