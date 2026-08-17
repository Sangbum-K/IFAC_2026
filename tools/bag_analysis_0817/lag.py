import sqlite3, sys
sys.path.insert(0,"/home/miru/IFAC_ws/tools")
import numpy as np
from bagviz import CDR
BAG="/home/miru/IFAC_ws/rosbag0817/run_0817_124904/run_0817_124904_0.db3"
db=sqlite3.connect(BAG)
topics={n:(tid,ty) for tid,n,ty in db.execute("SELECT id,name,type FROM topics")}
t0=db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
def rows(n):
    return db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",(topics[n][0],)).fetchall()
for name in ("/car_state/frenet/odom","/scan","/autonomy/ackermann_cmd","/pf/pose/odom","/perception/obstacles_frenet","/planner/avoidance/otwpnts","/local_waypoints"):
    if name not in topics: continue
    lags=[]
    for ts,d in rows(name):
        c=CDR(d); ht,_=c.header()
        lags.append(((ts-t0)/1e9, (ts-t0)/1e9-(ht-t0/1e9) if False else ((ts/1e9)-ht)))
    L=np.array(lags)
    print(f"{name:35s} n={len(L)} lag ms: med={np.median(L[:,1])*1e3:7.1f} p95={np.percentile(L[:,1],95)*1e3:8.1f} max={L[:,1].max()*1e3:9.1f}")
    m=(L[:,0]>55)&(L[:,0]<64)
    if m.any(): print(f"{'':35s}   window55-64: med={np.median(L[m,1])*1e3:7.1f} max={L[m,1].max()*1e3:9.1f}")
