#!/usr/bin/env python3
"""재생 결과를 원본과 비교 (정렬은 otwpnts path-start-s 상호상관)."""
import sqlite3, glob, sys, pickle
import numpy as np
sys.path.insert(0,"/home/miru/IFAC_ws/tools")
from bagviz import CDR
REP = glob.glob(sys.argv[1] + "/*.db3")[0]
D = pickle.load(open("/home/miru/IFAC_ws/tools/bag_analysis_0822c/d26.pkl","rb")); L=34.44
def dec(b):
    c=CDR(b); c.header(); n=c.u32()
    if n==0: return np.zeros((0,10))
    a=np.empty((n,10))
    for i in range(n):
        c.i32(); a[i]=c.f64n(10)
    return a
db=sqlite3.connect(REP)
tid={n:i for i,n,_ in db.execute("SELECT id,name,type FROM topics")}["/planner/avoidance/otwpnts"]
rows=db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",(tid,)).fetchall()
rts=np.array([ts for ts,_ in rows],float)/1e9; rts-=rts[0]
na=[dec(d) for _,d in rows]
rs=np.array([a[0,0] if len(a) else np.nan for a in na])
old=D["otwpnts"]; ot=np.array([t for t,_ in old]); oa=[a for _,a in old]
osv=np.array([a[0,0] if len(a) else np.nan for a in oa])
grid=np.arange(10,205,0.05)
def rsmp(t,s,off=0.0):
    m=np.isfinite(s); return np.interp(grid,t[m]+off,s[m])
best=(1e9,0)
for off in np.arange(-5,25,0.01):
    d=np.abs(((rsmp(rts,rs,off)-rsmp(ot,osv))+L/2)%L-L/2); e=np.median(d)
    if e<best[0]: best=(e,off)
err,OFF=best; nt=rts+OFF
print(f"정렬 offset={OFF:.2f}s 오차={err:.3f}m")
assert err<0.3, "정렬 실패"
def pick(ta,al,tq):
    i=int(np.clip(np.searchsorted(ta,tq),0,len(ta)-1)); a=al[i]; return a if len(a) else None
def at_s(a,s):
    if a is None: return None
    k=np.argmin(np.abs(((a[:,0]-s+L/2)%L)-L/2)); return a[k,8]
print(f"\n{'t':>7} {'구간':<5} | {'구 @전면':>8} | {'신 @전면':>8}  변화")
tot=[]
for tq,so,tag in ((100.6,1.83,"1:40"),(101.0,1.83,"1:40"),(101.4,1.83,"1:40"),(101.8,1.83,"1:40"),
                  (111.7,1.83,"1:52"),(112.1,1.83,"1:52"),(112.5,1.83,"1:52"),
                  (167.5,21.61,"2:48"),(167.8,21.61,"2:48"),(168.0,21.61,"2:48"),(168.2,21.61,"2:48")):
    fo_=at_s(pick(ot,oa,tq),so); fn=at_s(pick(nt,na,tq),so)
    f=lambda v:f"{v:8.2f}" if v is not None else "       -"
    dl=f"{fn-fo_:+.2f}" if (fo_ and fn is not None) else "-"
    if fo_ and fn is not None: tot.append(fn-fo_)
    print(f"{tq:7.1f} {tag:<5} | {f(fo_)} | {f(fn)}  {dl}")
print(f"\n무더기 전면 계획속도 평균 변화: {np.mean(tot):+.2f} m/s")
