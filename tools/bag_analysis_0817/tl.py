import sqlite3, sys, re
sys.path.insert(0,"/home/miru/IFAC_ws/tools")
import numpy as np
from bagviz import CDR, dec_string
BAG="/home/miru/IFAC_ws/rosbag0817/run_0817_124904/run_0817_124904_0.db3"
db=sqlite3.connect(BAG)
topics={n:(tid,ty) for tid,n,ty in db.execute("SELECT id,name,type FROM topics")}
t0=db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]; T0=t0/1e9
def rows(n): return db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",(topics[n][0],)).fetchall()
def declog(b):
    c=CDR(b); sec=c.i32(); nsec=c.u32(); lvl=c.u8(); name=c.string(); msg=c.string()
    return sec+nsec*1e-9-T0, lvl, name, msg
LOG=[]
for ts,d in rows("/rosout"):
    try: LOG.append(declog(d))
    except Exception: pass
LOG.sort()
EV=[]
for ts,d in rows("/pitwall/events"):
    s=dec_string(d)
    m=re.match(r"\[(\d+\.\d+)\]\[([^\]]+)\]\s*(.*)",s)
    if m: EV.append((float(m.group(1))-T0, m.group(2), m.group(3), (ts-t0)/1e9))
    else: EV.append(((ts-t0)/1e9, "?", s, (ts-t0)/1e9))
EV.sort()
LO,HI=float(sys.argv[1]) if len(sys.argv)>1 else 55.0, float(sys.argv[2]) if len(sys.argv)>2 else 63.0
print(f"=== pitwall events (참시각) {LO}-{HI} ===")
for t,node,msg,rec in EV:
    if LO<=t<=HI: print(f"{t:7.2f} (rec+{rec-t:4.1f}) {msg}")
print(f"\n=== rosout (참시각) {LO}-{HI} ===")
for t,lvl,name,msg in LOG:
    if LO<=t<=HI and name not in ("particle_filter","monitor_node","planner_visualizer_node"):
        print(f"{t:7.2f} L{lvl} [{name}] {msg[:190]}")
