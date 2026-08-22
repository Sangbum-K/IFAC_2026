#!/usr/bin/env python3
"""재생(신규 코드) vs 백 원본(구 코드) — 임계 시점의 명령 프로파일 비교."""
import sqlite3, glob, sys, pickle
import numpy as np
sys.path.insert(0, "/home/miru/IFAC_ws/tools")
from bagviz import CDR, dec_log

REP = glob.glob("/tmp/claude-1000/-home-miru-IFAC-ws/d4e0c27a-90b8-4a7a-aad6-c81329524e48/scratchpad/rep_new/*.db3")[0]
D = pickle.load(open("d26.pkl","rb"))
T0 = D["t0_ns"]

def dec_wpnts(b):
    c = CDR(b); ht,_ = c.header(); n = c.u32()
    if n == 0: return ht, np.zeros((0,10))
    a = np.empty((n,10))
    for i in range(n):
        c.i32(); a[i] = c.f64n(10)
    return ht, a

db = sqlite3.connect(REP)
topics = {n:(i,t) for i,n,t in db.execute("SELECT id,name,type FROM topics")}
print("재생 토픽:", [n for n in topics if "otwpnt" in n or "rosout" in n])

rows = db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
                  (topics["/planner/avoidance/otwpnts"][0],)).fetchall()
new = []
for ts,d in rows:
    ht,a = dec_wpnts(d)
    new.append(((ht-T0)/1e9, a))
print(f"재생 otwpnts: {len(new)}개, t {new[0][0]:.1f}..{new[-1][0]:.1f}")

old = D["otwpnts"]
old_t = np.array([t for t,_ in old]); new_t = np.array([t for t,_ in new])

def head_at(arr_list, tarr, tq):
    i = int(np.clip(np.searchsorted(tarr, tq), 0, len(tarr)-1))
    t, a = arr_list[i]
    if len(a)==0: return t, None, None, None
    return t, a[0,8], a[:,8].min(), a

print("\n=== 명령 프로파일 선두속도 / 최저속도 (구 → 신) ===")
print(f"{'t':>7} | {'구 선두':>7} {'구 최저':>7} | {'신 선두':>7} {'신 최저':>7}")
for tq in (100.6, 101.0, 101.4, 101.8, 111.7, 112.1, 112.5,
           167.5, 167.8, 168.0, 168.2, 168.6):
    to, ho, mo, ao = head_at(old, old_t, tq)
    tn, hn, mn, an = head_at(new, new_t, tq)
    f = lambda v: f"{v:7.2f}" if v is not None else "      -"
    print(f"{tq:7.1f} | {f(ho)} {f(mo)} | {f(hn)} {f(mn)}")

# 장애물 전면(s) 지점의 계획 속도
print("\n=== 무더기 전면 통과 계획속도 ===")
for label, tq, s_obs in (("case1 s=1.83", 101.0, 1.83), ("case1 s=1.83", 101.4, 1.83),
                          ("case3 s=21.61", 167.8, 21.61), ("case3 s=21.61", 168.2, 21.61)):
    for nm, lst, tarr in (("구", old, old_t), ("신", new, new_t)):
        t,_,_,a = head_at(lst, tarr, tq)
        if a is None or len(a)==0: print(f"  {label} t={tq} {nm}: 빈 경로"); continue
        k = np.argmin(np.abs(((a[:,0]-s_obs+17.22)%34.44)-17.22))
        print(f"  {label} t={tq} {nm}: s={a[k,0]:.2f} 계획 v={a[k,8]:.2f}")

print("\n=== 신규 로그 (제동 값) ===")
r = db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
               (topics["/rosout"][0],)).fetchall()
cnt = 0
for ts,d in r:
    try: lvl,name,msg = dec_log(d)
    except Exception: continue
    if "제동" in msg and "m/s" in msg:
        cnt += 1
        if cnt <= 12: print(f"  {msg[:120]}")
print(f"  ... 총 {cnt}건")
