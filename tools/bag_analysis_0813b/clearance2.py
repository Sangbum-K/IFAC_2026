#!/usr/bin/env python3
"""여유 재측정: 인접 3빔 이상 일치하는 반사만 인정(스퍼리어스 제거)."""
import pickle, sqlite3, sys
import numpy as np
sys.path.insert(0,"/home/miru/IFAC_ws/tools")
from bagviz import dec_scan
BAG="/home/miru/IFAC_ws/rosbag_0813/run_0813_221645/run_0813_221645_0.db3"
db=sqlite3.connect(BAG); T={n:i for i,n in db.execute("select id,name from topics")}
t0=db.execute("select min(timestamp) from messages").fetchone()[0]
D=pickle.load(open("data.pkl","rb")); fre=D["/car_state/frenet/odom"]
rows=db.execute("select timestamp,data from messages where topic_id=? order by timestamp",(T['/scan'],)).fetchall()
out=[]
for ts,d in rows:
    t=(ts-t0)/1e9
    ht,amin,ainc,ti,r=dec_scan(d,1)
    n=len(r); ang=amin+ainc*np.arange(n)
    ok=r>0.05
    # 이웃 2빔과 0.06m 이내로 이어지는 점만 (고립 스퍼리어스 제거)
    rr=np.where(ok,r,np.nan)
    d1=np.abs(np.diff(rr,prepend=np.nan)); d2=np.abs(np.diff(rr,append=np.nan))
    solid=ok&((d1<0.06)|(d2<0.06))
    x=rr*np.cos(ang); y=rr*np.sin(ang)
    side=solid&(x>-0.45)&(x<0.16)&(np.abs(y)<1.2)
    yl=y[side&(y>0)]; yr=-y[side&(y<0)]
    out.append((t, np.nanmin(yl) if np.any(np.isfinite(yl)) else np.nan,
                np.nanmin(yr) if np.any(np.isfinite(yr)) else np.nan))
C=np.array(out); np.save("clearance2.npy",C)
s=np.interp(C[:,0],fre[:,0],fre[:,1])
mn=np.nanmin(C[:,1:3],axis=1)
print(f"좌여유 p1={np.nanpercentile(C[:,1],1):.3f} 우여유 p1={np.nanpercentile(C[:,2],1):.3f} "
      f"최소여유 p1={np.nanpercentile(mn,1):.3f} p5={np.nanpercentile(mn,5):.3f}")
for th in (0.15,0.20,0.25):
    bad=np.where(mn<th)[0]
    import collections
    h=collections.Counter(np.round(s[bad]).astype(int))
    print(f"\n여유<{th}: {len(bad)}스캔 ({100*len(bad)/len(C):.1f}%)  s분포 {sorted(h.items())}")
