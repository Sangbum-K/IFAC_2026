import sqlite3, sys, re
sys.path.insert(0,"/home/miru/IFAC_ws/tools")
import numpy as np
from bagviz import CDR
BAG="/home/miru/IFAC_ws/rosbag0817/run_0817_124904/run_0817_124904_0.db3"
db=sqlite3.connect(BAG); T0=db.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]/1e9
topics={n:tid for tid,n,ty in db.execute("SELECT id,name,type FROM topics")}
def declog(b):
    c=CDR(b); sec=c.i32(); nsec=c.u32(); lvl=c.u8(); name=c.string(); msg=c.string()
    return sec+nsec*1e-9-T0, lvl, name, msg
L=[]
for ts,d in db.execute("SELECT timestamp,data FROM messages WHERE topic_id=?",(topics["/rosout"],)):
    try: L.append(declog(d))
    except Exception: pass
L.sort()
print("=== 초대형 클러스터 거부 (전 구간, 참시각) ===")
for t,lvl,name,msg in L:
    if "초대형" in msg: print(f"  {t:7.2f}  {msg}")
print("\n=== rosout 노드별 생존 개수 (총 %d) ==="%len(L))
from collections import Counter
for k,v in Counter(n for _,_,n,_ in L).most_common(): print(f"  {v:5d}  {k}")
