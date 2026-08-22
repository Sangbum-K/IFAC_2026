import sqlite3,sys,pickle
sys.path.insert(0,"/home/miru/IFAC_ws/tools")
from bagviz import CDR
def dec_log2(b):
    c=CDR(b); t=c.time(); lvl=c.u8(); name=c.string(); msg=c.string()
    return t,lvl,name,msg
bag,out=sys.argv[1],sys.argv[2]
db=sqlite3.connect(bag)
tid=db.execute("SELECT id FROM topics WHERE name='/rosout'").fetchone()[0]
t0=db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]/1e9
res=[]
for ts,d in db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",(tid,)):
    t,lvl,name,msg=dec_log2(d); res.append((t-t0,lvl,name,msg))
res.sort(); pickle.dump(res,open(out,"wb"))
print(len(res),"%.2f .. %.2f"%(res[0][0],res[-1][0]))
