#!/usr/bin/env python3
"""회피 판단 흐름: 장애물별·랩별 mode/reject 시퀀스."""
import pickle, re, sys
from collections import Counter, defaultdict
import numpy as np
D=pickle.load(open("data.pkl","rb"))
TL=32.07
TR=re.compile(r"\[trace\] s=([-\d.]+) v=([-\d.]+) d=([-\d.]+) \| obs id=([-\d]+) "
  r"dist=([-\d.]+) d\[([-\d.]+),([-\d.]+)\] static=([-\d]+) nobs=([-\d]+) \| "
  r"evade=(\d) sector=(\d) commit=(\d) \| (\w+) \| (.*)")
TR2=[]
for t,l,n,m in D["rosout"]:
    g=TR.search(m)
    if g:
        s,v,d,oid,dist,dr,dl,st,nobs,ev,sec,cm,mode,rej=g.groups()
        TR2.append(dict(t=t,s=float(s),v=float(v),d=float(d),oid=int(oid),dist=float(dist),
                        dr=float(dr),dl=float(dl),st=int(st),nobs=int(nobs),ev=int(ev),
                        sec=int(sec),cm=int(cm),mode=mode,
                        rej=[r for r in rej.split(",") if r and r!="-"]))
print(f"trace {len(TR2)}  ({TR2[0]['t']:.1f}~{TR2[-1]['t']:.1f}s)")
pickle.dump(TR2,open("trace.pkl","wb"))

print("\n=== 전체 mode 분포 ===")
for k,v in Counter(x["mode"] for x in TR2).most_common():
    print(f"  {k:16s} {v:5d} ({100*v/len(TR2):4.1f}%)")
print("\n=== 장애물이 잡힌 사이클(dist>=0)만 ===")
W=[x for x in TR2 if x["dist"]>=0 and x["oid"]!=0]
print(f"  n={len(W)}")
for k,v in Counter(x["mode"] for x in W).most_common():
    print(f"  {k:16s} {v:5d} ({100*v/len(W):4.1f}%)")
print("\n=== reject 사유 빈도 (장애물 사이클 기준) ===")
c=Counter()
for x in W:
    for r in set(x["rej"]): c[r]+=1
for k,v in c.most_common(30): print(f"  {k:42s} {v:5d} ({100*v/len(W):4.1f}%)")

print("\n=== s 구간별 mode (장애물 있는 사이클) ===")
BIN=[(0,3,"S/F직선"),(3,5,"네모코너 진입"),(5,8,"네모코너 중앙(O2)"),(8,11.5,"네모 탈출/상단(O3)"),
     (11.5,14,"우코너"),(14,17,"대각(O4)"),(17,20,"헤어핀 진입"),(20,24,"헤어핀(O6)"),
     (24,27,"헤어핀 탈출"),(27,32.1,"직선(O7)")]
hdr=sorted({x["mode"] for x in W})
print(f"{'구간':22s} {'n':>5s} "+" ".join(f"{h[:11]:>11s}" for h in hdr))
for a,b,nm in BIN:
    ws=[x for x in W if a<=x["s"]<b]
    if not ws: continue
    cc=Counter(x["mode"] for x in ws)
    print(f"{nm:22s} {len(ws):5d} "+" ".join(f"{100*cc[h]/len(ws):10.0f}%" for h in hdr))

print("\n\n=== 구간별 reject 사유 (상위 8) ===")
for a,b,nm in BIN:
    ws=[x for x in W if a<=x["s"]<b]
    if len(ws)<30: continue
    c=Counter()
    for x in ws:
        for r in set(x["rej"]): c[r]+=1
    print(f"\n--- {nm}  (n={len(ws)}) ---")
    for k,v in c.most_common(8):
        print(f"    {k:40s} {v:5d} ({100*v/len(ws):4.0f}%)")
