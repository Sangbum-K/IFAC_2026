import pickle, sqlite3, sys, numpy as np
from pathlib import Path
sys.path.insert(0,"/home/miru/IFAC_ws/tools")
import matplotlib; matplotlib.use("Agg")
matplotlib.rcParams["font.family"]="Noto Sans CJK JP"; matplotlib.rcParams["axes.unicode_minus"]=False
import matplotlib.pyplot as plt
from bagviz import dec_scan
P=Path(__file__).parent
D=pickle.load(open(P/"d124904.pkl","rb"))
BAG="/home/miru/IFAC_ws/rosbag0817/run_0817_124904/run_0817_124904_0.db3"
db=sqlite3.connect(BAG); t0=db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
tid=dict((n,i) for i,n,_ in db.execute("SELECT id,name,type FROM topics"))["/scan"]
pf=D["/pf/pose/odom"]; od=D["/odom"]; imu=D["/sensors/imu/raw"]; gw=D["global_wpnts"][0][1]
mp=D["map"]; res=mp["res"]; ox,oy=mp["ox"],mp["oy"]
grid=np.array(mp["data"]).reshape(mp["h"],mp["w"]); occ=grid>50
LX=0.28
# dead-reckon from last good PF pose at t=60.85 using odom speed + IMU yaw rate
T0=60.85
i0=np.searchsorted(pf[:,0],T0)
x,y=pf[i0,1],pf[i0,2]; yaw=pf[i0,3]
ts=np.arange(T0,62.0,0.005); traj={}
for k in range(1,len(ts)):
    dt=ts[k]-ts[k-1]
    v=np.interp(ts[k],od[:,0],od[:,4])
    wz=np.radians(np.interp(ts[k],imu[:,0],imu[:,4]))
    yaw+=wz*dt; x+=v*np.cos(yaw)*dt; y+=v*np.sin(yaw)*dt
    traj[round(ts[k],3)]=(x,y,yaw)
def dr(t):
    k=min(traj.keys(),key=lambda q:abs(q-t)); return traj[k]
def near_wall(X,Y,rad=0.13):
    k=int(rad/res); ix=((X-ox)/res).astype(int); iy=((Y-oy)/res).astype(int); out=np.zeros(len(X),bool)
    for dx in range(-k,k+1):
        for dy in range(-k,k+1): out |= occ[np.clip(iy+dy,0,mp["h"]-1),np.clip(ix+dx,0,mp["w"]-1)]
    return out
SC=[]
for tsn,d in db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",(tid,)):
    tt=(tsn-t0)/1e9
    if 60.8<=tt<=61.9 or any(a<=tt<=b for a,b in [(16.1,18.0),(38.6,40.4),(48.8,50.8)]):
        ht,amin,ainc,tinc,r=dec_scan(d,2); SC.append((tt,amin,ainc,np.asarray(r)))
fig,ax=plt.subplots(figsize=(13,8))
ax.imshow(grid,origin="lower",extent=[ox,ox+mp["w"]*res,oy,oy+mp["h"]*res],cmap="Greys",vmin=-1,vmax=100,alpha=.45)
ax.plot(gw[:,2],gw[:,3],"-",c="tab:blue",lw=1)
# 이전 랩들의 off-map 점 = 실제 장애물 위치
XS=[];YS=[]
for tt,amin,ainc,r in SC:
    if tt>60: continue
    ang=amin+ainc*np.arange(len(r)); ok=(r>0.06)&(r<5)&np.isfinite(r)
    px,py,yw=np.interp(tt,pf[:,0],pf[:,1]),np.interp(tt,pf[:,0],pf[:,2]),np.arctan2(np.interp(tt,pf[:,0],np.sin(pf[:,3])),np.interp(tt,pf[:,0],np.cos(pf[:,3])))
    c,sn=np.cos(yw),np.sin(yw); bx,by=r[ok]*np.cos(ang[ok])+LX,r[ok]*np.sin(ang[ok])
    XS.append(px+c*bx-sn*by); YS.append(py+sn*bx+c*by)
X=np.concatenate(XS);Y=np.concatenate(YS);k=~near_wall(X,Y)
ax.plot(X[k],Y[k],".",c="0.3",ms=1.5,label="lap1~3 실제 장애물 (맵에 없는 점)")
# 충돌 후 데드레커닝 자세로 투영한 스캔
for tt,amin,ainc,r in SC:
    if tt<60.8: continue
    ang=amin+ainc*np.arange(len(r)); ok=(r>0.06)&(r<5)&np.isfinite(r)
    px,py,yw=dr(tt); c,sn=np.cos(yw),np.sin(yw)
    bx,by=r[ok]*np.cos(ang[ok])+LX,r[ok]*np.sin(ang[ok])
    ax.plot(px+c*bx-sn*by, py+sn*bx+c*by,".",c="tab:red",ms=1.5)
ax.plot([],[],".",c="tab:red",ms=6,label="60.85~61.9s 스캔 (오도메트리 추측항법 자세)")
tr=np.array([traj[k] for k in sorted(traj)])
ax.plot(tr[:,0],tr[:,1],"-",c="tab:orange",lw=2.5,label="충돌 후 추측항법 궤적")
m=(pf[:,0]>=58.5)&(pf[:,0]<=60.85); ax.plot(pf[m,1],pf[m,2],"-",c="tab:green",lw=2.5,label="PF 궤적 58.5~60.85s")
for lab,tt in [("1차 충돌 59.80",59.80),("측위 LOST 60.89",60.89),("정지 61.7",61.70)]:
    if tt<60.85: px,py=np.interp(tt,pf[:,0],pf[:,1]),np.interp(tt,pf[:,0],pf[:,2])
    else: px,py,_=dr(tt)
    ax.plot(px,py,"*",ms=16,c="k"); ax.text(px,py+0.15,lab,fontsize=9,ha="center")
ax.set_aspect("equal"); ax.set_xlim(-7.5,2.0); ax.set_ylim(-2.0,2.0); ax.legend(fontsize=9,loc="lower left")
ax.set_title("1차 충돌(s≈33.2) 이후 2차 접촉까지",fontsize=13)
plt.tight_layout(); plt.savefig(P/"second_impact.png",dpi=115); print("saved")
