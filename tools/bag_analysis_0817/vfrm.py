import pickle, sqlite3, sys, numpy as np
from pathlib import Path
sys.path.insert(0,"/home/miru/IFAC_ws/tools")
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from bagviz import CDR, dec_scan
P=Path(__file__).parent
D=pickle.load(open(P/"d124904.pkl","rb"))
BAG="/home/miru/IFAC_ws/rosbag0817/run_0817_124904/run_0817_124904_0.db3"
db=sqlite3.connect(BAG); t0=db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
tid=dict((n,i) for i,n,_ in db.execute("SELECT id,name,type FROM topics"))["/scan"]
SC=[]
for ts,d in db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",(tid,)):
    t=(ts-t0)/1e9
    if 58.5<=t<=62.0:
        ht,amin,ainc,tinc,r=dec_scan(d,1); SC.append((t,amin,ainc,np.asarray(r)))
fre=D["/car_state/frenet/odom"]; cmd=D["/autonomy/ackermann_cmd"]
def at(a,t,c): return np.interp(t,a[:,0],a[:,c])
TS=np.arange(59.3,61.61,0.2)
fig,axes=plt.subplots(3,4,figsize=(22,14))
for ax,t in zip(axes.ravel(),TS):
    k=min(range(len(SC)),key=lambda i:abs(SC[i][0]-t)); ts,amin,ainc,r=SC[k]
    n=len(r); ang=amin+ainc*np.arange(n); ok=(r>0.06)&(r<6)&np.isfinite(r)
    x,y=r[ok]*np.cos(ang[ok])+0.28, r[ok]*np.sin(ang[ok])   # base_link frame
    ax.plot(y,x,".",c="tab:red",ms=3)           # y left, x forward  (plot: left-right = y)
    fp=np.array([(-0.1,-0.15),(0.45,-0.15),(0.45,0.15),(-0.1,0.15),(-0.1,-0.15)])
    ax.plot(fp[:,1],fp[:,0],"-",c="k",lw=2)
    st=at(cmd,ts,1); v=at(fre,ts,4)
    L=1.2
    ax.plot([0,-L*np.sin(st)],[0,L*np.cos(st)],"-",c="tab:blue",lw=2)
    ax.axvline(0,c="0.8",lw=.5); ax.axhline(0,c="0.8",lw=.5)
    ax.set_xlim(1.8,-1.8); ax.set_ylim(-0.6,3.2); ax.set_aspect("equal")
    ax.set_title(f"t={ts:.2f} v={v:.2f} cmd_v={at(cmd,ts,2):.2f} steer={np.degrees(st):+.1f}d={at(fre,ts,2):+.2f}",fontsize=9)
    ax.set_xlabel("y (left+) [m]"); ax.set_ylabel("x fwd [m]")
plt.tight_layout(); plt.savefig(P/"vehicle_frame.png",dpi=100); print("saved")
