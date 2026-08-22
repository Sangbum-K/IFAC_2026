#!/usr/bin/env python3
"""run_0822_172617 [trace] 종합: 모드 분포 / 기각 사유 / 거리별 성공률"""
import pickle,re,sys,collections,numpy as np
L=pickle.load(open(sys.argv[1],"rb"))
T0,T1=(float(sys.argv[2]),float(sys.argv[3])) if len(sys.argv)>3 else (4.0,170.0)
pat=re.compile(r"s=([-\d.]+) v=([-\d.]+) d=([-\d.]+) \| obs id=(-?\d+) dist=([-\d.]+) d\[([-\d.]+),([-\d.]+)\] static=(\d) nobs=(\d+) \| evade=(\d) sector=(\d) commit=(\d) \| (\w+) \| (.*)$")
R=[]
for t,lvl,n,m in L:
    if "[trace]" not in m or not (T0<=t<=T1): continue
    g=pat.search(m)
    if not g: continue
    R.append(dict(t=t,s=float(g[1]),v=float(g[2]),d=float(g[3]),oid=int(g[4]),
        dist=float(g[5]),dl=float(g[6]),dr=float(g[7]),static=g[8]=="1",
        nobs=int(g[9]),evade=g[10]=="1",sector=g[11]=="1",commit=g[12]=="1",
        mode=g[13],tags=g[14]))
print("trace %d (%.0f~%.0f s)"%(len(R),T0,T1))
print("정적 %d / 동적 %d"%(sum(r["static"] for r in R),sum(not r["static"] for r in R)))
S=[r for r in R if r["static"]]
print("\n[모드 분포 — 정적만]")
c=collections.Counter(r["mode"] for r in S)
for k,v in c.most_common(): print("  %-14s %5d  %5.1f%%"%(k,v,100*v/len(S)))
print("\n[커밋 상태]"); print("  commit=1 %d (%.1f%%)  evade=1 %d (%.1f%%)"%(
    sum(r["commit"] for r in S),100*np.mean([r["commit"] for r in S]),
    sum(r["evade"] for r in S),100*np.mean([r["evade"] for r in S])))
print("\n[기각 사유 빈도 — 정적 사이클]")
c=collections.Counter()
for r in S:
    for tag in r["tags"].split(","):
        tag=tag.strip()
        if tag and not tag.startswith("+"): c[tag]+=1
for k,v in c.most_common(20): print("  %-42s %5d  %5.1f%%"%(k,v,100*v/len(S)))
print("\n[거리 구간별 모드 — 정적]")
bins=[(0,0.5),(0.5,1.0),(1.0,1.5),(1.5,2.0),(2.0,2.5),(2.5,3.0),(3.0,4.0),(4.0,6.0),(6.0,12.0)]
print("  dist        n | lane_sw  spline  trailing escape  | v중앙")
for a,b in bins:
    sub=[r for r in S if a<=r["dist"]<b]
    if not sub: continue
    cc=collections.Counter(r["mode"] for r in sub)
    print("  %4.1f-%4.1f %4d | %5.1f%% %6.1f%% %7.1f%% %6.1f%% | %.2f"%(a,b,len(sub),
        100*cc["lane_switch"]/len(sub),100*cc["free_spline"]/len(sub),
        100*cc["trailing"]/len(sub),100*cc["escape_arc"]/len(sub),
        np.median([r["v"] for r in sub])))
print("\n[최초 관측 거리 — 트랙 id 별 첫 trace]")
first={}
for r in S:
    if r["oid"] not in first: first[r["oid"]]=r
dd=np.array([r["dist"] for r in first.values()])
print("  트랙 %d개  최초거리 p10 %.2f p50 %.2f p90 %.2f max %.2f"%(
    len(dd),np.percentile(dd,10),np.median(dd),np.percentile(dd,90),dd.max()))
