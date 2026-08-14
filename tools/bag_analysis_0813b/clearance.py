#!/usr/bin/env python3
"""PF 무관 실측 여유: 차체 옆(±)으로 가장 가까운 LiDAR 반사까지의 횡거리."""
import pickle, sqlite3, sys
import numpy as np
sys.path.insert(0,"/home/miru/IFAC_ws/tools")
from bagviz import dec_scan
BAG="/home/miru/IFAC_ws/rosbag_0813/run_0813_221645/run_0813_221645_0.db3"
db=sqlite3.connect(BAG); T={n:i for i,n in db.execute("select id,name from topics")}
t0=db.execute("select min(timestamp) from messages").fetchone()[0]
D=pickle.load(open("data.pkl","rb"))
fre=D["/car_state/frenet/odom"]
rows=db.execute("select timestamp,data from messages where topic_id=? order by timestamp",(T['/scan'],)).fetchall()
out=[]
# base_link 기준 차체 x∈[-0.14,0.41] -> laser(=base+0.28) x∈[-0.42,0.13]
for ts,d in rows:
    t=(ts-t0)/1e9
    ht,amin,ainc,ti,r=dec_scan(d,1)
    n=len(r); ang=amin+ainc*np.arange(n)
    m=(r>0.05)&(r<3.0)
    x=r[m]*np.cos(ang[m]); y=r[m]*np.sin(ang[m])
    side=(x>-0.45)&(x<0.16)
    yl=y[side&(y>0)]; yr=-y[side&(y<0)]
    # 전방 1m 이내 최근접(정면 충돌 위험)
    fwd=(np.abs(y)<0.18)&(x>0.13)&(x<1.2)
    out.append((t, yl.min() if len(yl) else np.nan, yr.min() if len(yr) else np.nan,
                x[fwd].min() if fwd.sum() else np.nan))
C=np.array(out); np.save("clearance.npy",C)
s=np.interp(C[:,0],fre[:,0],fre[:,1])
print(f"{len(C)} scans. 좌여유 p1={np.nanpercentile(C[:,1],1):.3f} 우여유 p1={np.nanpercentile(C[:,2],1):.3f}")
print("차체 반폭 0.15 기준 → 여유<0.15 는 접촉")
tot=np.nanmin(C[:,1:3],axis=1)
bad=np.where(tot<0.16)[0]
print(f"여유<0.16 스캔 {len(bad)}개 ({100*len(bad)/len(C):.2f}%)")
import collections
h=collections.Counter(np.round(s[bad]).astype(int))
print("s별 근접 발생:", sorted(h.items()))
