import sqlite3, sys, pickle
sys.path.insert(0,"/home/sangbum/IFAC_ws/tools")
from bagviz import CDR
def dec_log2(b):
    c=CDR(b); t=c.time(); lvl=c.u8(); name=c.string(); msg=c.string()
    return t, lvl, name, msg
db=sqlite3.connect("/home/sangbum/IFAC_ws/run_0820_235246/run_0820_235246_0.db3")
tid=db.execute("SELECT id FROM topics WHERE name='/rosout'").fetchone()[0]
t0=db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]/1e9
out=[]
for ts,d in db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",(tid,)):
    t,lvl,name,msg=dec_log2(d)
    out.append((t-t0,lvl,name,msg))
out.sort()
pickle.dump(out,open("/home/sangbum/IFAC_ws/tools/bag_analysis_0821/rosout2.pkl","wb"))
print(len(out), "%.2f .. %.2f"%(out[0][0], out[-1][0]))
