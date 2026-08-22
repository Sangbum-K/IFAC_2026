import pickle,sqlite3,sys,numpy as np
sys.path.insert(0,"/home/miru/IFAC_ws/tools")
from bagviz import dec_bool
pkl,bag=sys.argv[1],sys.argv[2]
D=pickle.load(open(pkl,"rb"))
db=sqlite3.connect(bag)
topics={n:(tid,ty) for tid,n,ty in db.execute("SELECT id,name,type FROM topics")}
t0=db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
for n,(tid,ty) in topics.items():
    if ty.endswith("Bool") and n not in D:
        r=db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",(tid,)).fetchall()
        D[n]=np.array([((ts-t0)/1e9,1.0 if dec_bool(d) else 0.0) for ts,d in r]) if r else np.zeros((0,2))
        print("added",n,len(D[n]))
pickle.dump(D,open(pkl,"wb"))
