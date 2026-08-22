#!/usr/bin/env python3
import pickle,re,collections,numpy as np
L=pickle.load(open("log17.pkl","rb"))
pat=re.compile(r"s=([-\d.]+) v=([-\d.]+) d=([-\d.]+) \| obs id=(-?\d+) dist=([-\d.]+) d\[([-\d.]+),([-\d.]+)\] static=(\d) nobs=(\d+) \| evade=(\d) sector=(\d) commit=(\d) \| (\w+) \| (.*)$")
R=[]
for t,lvl,n,m in L:
    if "[trace]" not in m or not(4<=t<=170): continue
    g=pat.search(m)
    if g: R.append(dict(t=t,s=float(g[1]),v=float(g[2]),dist=float(g[5]),
        nobs=int(g[9]),commit=g[12]=="1",mode=g[13],
        tags=[x.strip() for x in g[14].split(",") if x.strip() and not x.strip().startswith("+")]))
tr=[r for r in R if r["mode"]=="trailing"]
print("trailing 사이클 %d — 그 사이클의 기각 사유"%len(tr))
c=collections.Counter()
for r in tr:
    for x in r["tags"]: c[x]+=1
for k,v in c.most_common(14): print("  %-40s %4d  %5.1f%%"%(k,v,100*v/len(tr)))
print("\ntrailing 사이클의 nobs(관측수) 중앙 %d | dist 중앙 %.2f | v 중앙 %.2f"%(
    int(np.median([r["nobs"] for r in tr])),np.median([r["dist"] for r in tr]),
    np.median([r["v"] for r in tr])))
print("\n[커밋 전(commit=0) vs 커밋 중 사이클의 모드]")
for cm in (False,True):
    g=[r for r in R if r["commit"]==cm]
    cc=collections.Counter(r["mode"] for r in g)
    print("  commit=%d n=%4d : %s"%(cm,len(g),dict(cc)))
print("\n[nobs 별 모드] — 커밋에 필요한 최소 관측수 확인")
for lo,hi in ((0,1),(1,3),(3,6),(6,12),(12,999)):
    g=[r for r in R if lo<=r["nobs"]<hi]
    if not g: continue
    cc=collections.Counter(r["mode"] for r in g)
    print("  nobs %2d-%3d n=%4d : %s"%(lo,hi-1,len(g),dict(cc)))
# 첫 조우(트랙 등장)부터 커밋까지 걸린 시간
print("\n[조우 시퀀스] 연속 trace 블록 = 한 번의 조우")
R.sort(key=lambda r:r["t"])
blocks=[];cur=[R[0]]
for a,b in zip(R,R[1:]):
    if b["t"]-a["t"]<0.3: cur.append(b)
    else: blocks.append(cur); cur=[b]
blocks.append(cur)
print("  조우 %d회, 길이 중앙 %.2fs"%(len(blocks),np.median([b[-1]["t"]-b[0]["t"] for b in blocks])))
d2c=[];ok=0
for b in blocks:
    cm=[r for r in b if r["commit"]]
    if cm:
        ok+=1; d2c.append((b[0]["dist"],cm[0]["dist"],cm[0]["t"]-b[0]["t"],b[0]["v"]))
    else: d2c.append((b[0]["dist"],np.nan,np.nan,b[0]["v"]))
A=np.array(d2c,float)
print("  커밋에 도달한 조우 %d/%d (%.0f%%)"%(ok,len(blocks),100*ok/len(blocks)))
m=~np.isnan(A[:,1])
print("  최초 감지거리 중앙 %.2f m (v %.2f) -> 커밋 시점 거리 중앙 %.2f m, 지연 %.2f s"%(
    np.median(A[:,0]),np.median(A[:,3]),np.median(A[m,1]),np.median(A[m,2])))
