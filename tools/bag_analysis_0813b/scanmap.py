#!/usr/bin/env python3
import pickle, sqlite3, sys
import numpy as np
sys.path.insert(0,"/home/miru/IFAC_ws/tools")
from bagviz import dec_grid, dec_scan, dec_pose_stamped, dec_tf_static
BAG="/home/miru/IFAC_ws/rosbag_0813/run_0813_221645/run_0813_221645_0.db3"
db=sqlite3.connect(BAG)
T={n:i for i,n in db.execute("select id,name from topics")}
t0=db.execute("select min(timestamp) from messages").fetchone()[0]
for ts,d in db.execute("select timestamp,data from messages where topic_id=?",(T['/tf_static'],)):
    for r in dec_tf_static(d): print("TF_STATIC",r)
pz=[]
for ts,d in db.execute("select timestamp,data from messages where topic_id=? order by timestamp",(T['/pf/viz/inferred_pose'],)):
    x,y,yaw=dec_pose_stamped(d); pz.append(((ts-t0)/1e9,x,y,yaw))
pz=np.array(pz); print("poses",pz.shape)
LX=0.0
import os
LX=float(os.environ.get("LX","0.0"))
pts=[]
rows=db.execute("select timestamp,data from messages where topic_id=? order by timestamp",(T['/scan'],)).fetchall()
for ts,d in rows[::3]:
    t=(ts-t0)/1e9
    i=np.searchsorted(pz[:,0],t)
    if i<=0 or i>=len(pz) or abs(pz[i,0]-t)>0.06: continue
    ex,ey,eyaw=pz[i,1],pz[i,2],pz[i,3]
    lx=ex+LX*np.cos(eyaw); ly=ey+LX*np.sin(eyaw)
    ht,amin,ainc,ti,r=dec_scan(d,2)
    n=len(r); ang=amin+ainc*np.arange(n)
    m=(r>0.1)&(r<6.0)
    a=ang[m]+eyaw; rr=r[m]
    pts.append(np.stack([lx+rr*np.cos(a),ly+rr*np.sin(a)],1))
P=np.vstack(pts); print("points",P.shape)
np.save("scanpts.npy",P)
