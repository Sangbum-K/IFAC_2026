import pickle, sqlite3, sys, numpy as np
from pathlib import Path
sys.path.insert(0,"/home/miru/IFAC_ws/tools")
import matplotlib; matplotlib.use("Agg")
matplotlib.rcParams["font.family"]="Noto Sans CJK JP"
matplotlib.rcParams["axes.unicode_minus"]=False
import matplotlib.pyplot as plt
from bagviz import dec_scan
P=Path(__file__).parent
D=pickle.load(open(P/"d124904.pkl","rb"))
BAG="/home/miru/IFAC_ws/rosbag0817/run_0817_124904/run_0817_124904_0.db3"
db=sqlite3.connect(BAG); t0=db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
tid=dict((n,i) for i,n,_ in db.execute("SELECT id,name,type FROM topics"))["/scan"]
pf=D["/pf/pose/odom"]; fre=D["/car_state/frenet/odom"]; gw=D["global_wpnts"][0][1]
mp=D["map"]; res=mp["res"]; ox,oy=mp["ox"],mp["oy"]
grid=np.array(mp["data"]).reshape(mp["h"],mp["w"]); occ=grid>50
LX=0.28
GX,GY,GS,GPSI=gw[:,2],gw[:,3],gw[:,0],gw[:,6]
def pose(tt):
    return (np.interp(tt,pf[:,0],pf[:,1]), np.interp(tt,pf[:,0],pf[:,2]),
            np.arctan2(np.interp(tt,pf[:,0],np.sin(pf[:,3])),np.interp(tt,pf[:,0],np.cos(pf[:,3]))))
def near_wall(x,y,rad=0.13):
    k=int(rad/res); ix=((x-ox)/res).astype(int); iy=((y-oy)/res).astype(int); out=np.zeros(len(x),bool)
    for dx in range(-k,k+1):
        for dy in range(-k,k+1): out |= occ[np.clip(iy+dy,0,mp["h"]-1),np.clip(ix+dx,0,mp["w"]-1)]
    return out
def to_frenet(X,Y):
    d2=(X[:,None]-GX[None,:])**2+(Y[:,None]-GY[None,:])**2; j=d2.argmin(1)
    nx,ny=-np.sin(GPSI[j]),np.cos(GPSI[j])
    return GS[j], (X-GX[j])*nx+(Y-GY[j])*ny
PASSES=[(16.1,18.0,"lap1"),(27.1,29.4,"lap2"),(38.6,40.4,"lap3"),(48.8,50.8,"lap4"),(59.2,60.9,"lap5(충돌)")]
fig,ax=plt.subplots(figsize=(15,7))
allS=[];allD=[]
for a,b,lab in PASSES[:4]:
    XS=[];YS=[]
    for ts,d in db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",(tid,)):
        tt=(ts-t0)/1e9
        if not (a<=tt<=b): continue
        ht,amin,ainc,tinc,r=dec_scan(d,2); r=np.asarray(r)
        ang=amin+ainc*np.arange(len(r)); ok=(r>0.06)&(r<4.5)&np.isfinite(r)
        x,y=r[ok]*np.cos(ang[ok])+LX, r[ok]*np.sin(ang[ok])
        px,py,yw=pose(tt); c,sn=np.cos(yw),np.sin(yw)
        XS.append(px+c*x-sn*y); YS.append(py+sn*x+c*y)
    X=np.concatenate(XS); Y=np.concatenate(YS); k=~near_wall(X,Y)
    S,Dd=to_frenet(X[k],Y[k]); allS.append(S); allD.append(Dd)
S=np.concatenate(allS); Dd=np.concatenate(allD)
m=(S>=30.5)&(S<=37.5)
ax.plot(S[m],Dd[m],".",c="0.25",ms=2,label="맵에 없는 스캔점 (lap1~4 누적) = 실제 장애물")
g=gw[(gw[:,0]>=30.3)&(gw[:,0]<=37.5)]
ax.plot(g[:,0],+g[:,5],"-",c="tab:blue",lw=2,label="좌 경계 d_left")
ax.plot(g[:,0],-g[:,4],"-",c="tab:blue",lw=2,label="우 경계 d_right")
ax.axhline(0,c="tab:blue",ls=":",lw=1)
cols=["tab:green","tab:olive","tab:cyan","tab:purple","red"]
for (a,b,lab),c in zip(PASSES,cols):
    mm=(fre[:,0]>=a)&(fre[:,0]<=b)&(fre[:,1]>=30.3)&(fre[:,1]<=37.5)
    ax.plot(fre[mm,1],fre[mm,2],"-",c=c,lw=2.5 if lab.startswith("lap5") else 1.6,label=lab)
# perception boxes for lap5
seen=set()
for t,obs in D["obs_frenet"]:
    if not (58.8<=t<=60.6): continue
    for o in obs:
        if o["id"] in seen or not (30<=o["s_center"]<=37.6): continue
        seen.add(o["id"])
        ax.add_patch(plt.Rectangle((o["s_start"],o["d_right"]),o["s_end"]-o["s_start"],o["d_left"]-o["d_right"],
                                   fill=False,ec="tab:orange",lw=2))
        ax.text(o["s_center"],o["d_left"]+0.03,f"#{o['id']}",color="tab:orange",fontsize=9,ha="center")
ax.plot([],[],"-",c="tab:orange",lw=2,label="perception이 발행한 장애물 (lap5)")
ax.set_xlabel("s [m]"); ax.set_ylabel("d [m]  (+왼쪽)"); ax.set_ylim(-0.9,1.5); ax.set_xlim(30.3,37.5)
ax.grid(alpha=.3); ax.legend(fontsize=9,ncol=3,loc="upper left")
ax.set_title("헤어핀 탈출 직선 (s 30.3~37.5): 실제 장애물 vs 인지 vs 주행선",fontsize=13)
plt.tight_layout(); plt.savefig(P/"frenet_straight.png",dpi=115); print("saved")
