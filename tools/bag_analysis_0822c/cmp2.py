#!/usr/bin/env python3
"""재생 시각을 원본 시각에 상호상관으로 정렬한 뒤 프로파일 비교."""
import sqlite3, glob, sys, pickle
import numpy as np
sys.path.insert(0,"/home/miru/IFAC_ws/tools")
from bagviz import CDR

REP = glob.glob("/tmp/claude-1000/-home-miru-IFAC-ws/d4e0c27a-90b8-4a7a-aad6-c81329524e48/scratchpad/rep_new/*.db3")[0]
D = pickle.load(open("d26.pkl","rb")); L = 34.44
fo = D["/car_state/frenet/odom"]

def dec(b):
    c = CDR(b); c.header(); n = c.u32()
    if n==0: return np.zeros((0,10))
    a = np.empty((n,10))
    for i in range(n):
        c.i32(); a[i] = c.f64n(10)
    return a

db = sqlite3.connect(REP)
tid = {n:i for i,n,_ in db.execute("SELECT id,name,type FROM topics")}["/planner/avoidance/otwpnts"]
rows = db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",(tid,)).fetchall()
rts = np.array([ts for ts,_ in rows], dtype=float)/1e9
rts -= rts[0]
arrs = [dec(d) for _,d in rows]
rs = np.array([a[0,0] if len(a) else np.nan for a in arrs])

# 원본 ego s(t)
ot = fo[:,0]; os_ = fo[:,1]
def ego_s(t): return np.interp(t, ot, np.unwrap(os_*2*np.pi/L)*L/(2*np.pi)) % L

best=(1e9,None)
for off in np.arange(0.0, 25.0, 0.02):
    tq = rts + off
    m = (tq>5)&(tq<210)&np.isfinite(rs)
    if m.sum()<500: continue
    diff = np.abs(((rs[m]-ego_s(tq[m]))+L/2)%L - L/2)
    err = np.median(diff)
    if err<best[0]: best=(err,off)
err,off = best
print(f"정렬: offset={off:.2f}s, 중앙 s오차={err:.3f}m  (재생 {len(rows)} / 원본 {len(D['otwpnts'])} 사이클)")
if err>0.5:
    print("⚠️ 정렬 실패 — 비교 무효"); sys.exit(1)
nt = rts+off
old = D["otwpnts"]; old_t=np.array([t for t,_ in old])

def prof(lst_t, lst_a, tq):
    i=int(np.clip(np.searchsorted(lst_t,tq),0,len(lst_t)-1))
    a = lst_a[i]
    return (a if len(a) else None)

print(f"\n{'t':>7} | {'구 선두':>7} {'구 최저':>7} {'구 @전면':>8} | {'신 선두':>7} {'신 최저':>7} {'신 @전면':>8}")
def at_s(a, s_obs):
    if a is None or len(a)==0: return None
    k=np.argmin(np.abs(((a[:,0]-s_obs+L/2)%L)-L/2)); return a[k,8]
for tq,s_obs in ((100.6,1.83),(101.0,1.83),(101.4,1.83),(101.8,1.83),
                 (111.7,1.83),(112.1,1.83),(112.5,1.83),
                 (167.5,21.61),(167.8,21.61),(168.0,21.61),(168.2,21.61),(168.6,21.61)):
    ao = prof(old_t,[a for _,a in old],tq); an = prof(nt,arrs,tq)
    g=lambda a,f: (f"{f(a):7.2f}" if a is not None and len(a) else "      -")
    h=lambda a: a[0,8]; lo=lambda a: a[:,8].min()
    fo_=lambda a: at_s(a,s_obs)
    print(f"{tq:7.1f} | {g(ao,h)} {g(ao,lo)} {g(ao,fo_):>8} | {g(an,h)} {g(an,lo)} {g(an,fo_):>8}")
