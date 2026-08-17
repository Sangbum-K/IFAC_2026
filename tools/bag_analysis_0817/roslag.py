import sqlite3, sys, re
sys.path.insert(0,"/home/miru/IFAC_ws/tools")
import numpy as np
from bagviz import CDR
BAG="/home/miru/IFAC_ws/rosbag0817/run_0817_124904/run_0817_124904_0.db3"
db=sqlite3.connect(BAG)
topics={n:(tid,ty) for tid,n,ty in db.execute("SELECT id,name,type FROM topics")}
t0=db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
r=db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",(topics["/rosout"][0],)).fetchall()
print("rosout msgs:",len(r))
def dec(b):
    c=CDR(b); sec=c.i32(); nsec=c.u32(); lvl=c.u8(); name=c.string(); msg=c.string()
    return sec+nsec*1e-9, lvl, name, msg
out=[]
for ts,d in r:
    try:
        st,lvl,name,msg=dec(d); out.append(((ts-t0)/1e9, st-t0/1e9, lvl,name,msg))
    except Exception as e: pass
A=np.array([(o[0],o[1]) for o in out])
print("rosout lag(rec-stamp) ms: med %.1f p95 %.1f max %.1f"%tuple(np.percentile(A[:,0]-A[:,1],[50,95,99.9])*1e3))
print("\n--- trace lines with true stamp ---")
for rec,st,lvl,name,msg in out:
    m=re.match(r"\[trace\] s=([\d.-]+) v=([\d.-]+)",msg)
    if m and 58<=st<=63.5:
        print(f"  stamp={st:6.2f} rec={rec:6.2f} lag={rec-st:5.2f}  {msg[:150]}")
