import pickle,re,numpy as np
R=pickle.load(open("rosout2.pkl","rb"))
pat=re.compile(r"\[trace\] s=([-\d.]+) v=([-\d.]+) d=([-\d.]+) \| obs id=(-?\d+) dist=([-\d.]+) d\[([-\d.]+),([-\d.]+)\] static=(\d) nobs=(-?\d+) \| evade=(\d) sector=(\d) commit=(\d) \| (\w+) \| (.*)")
rows=[]
for t,l,name,msg in R:
    m=pat.search(msg)
    if m:
        g=m.groups()
        rows.append((t,float(g[0]),float(g[1]),float(g[2]),int(g[3]),float(g[4]),float(g[5]),float(g[6]),int(g[7]),int(g[8]),int(g[9]),int(g[10]),int(g[11]),g[12],g[13]))
print("traces",len(rows))
import collections
mode=collections.Counter(r[13] for r in rows)
print("mode:",dict(mode))
# 장애물이 있는 사이클만
has=[r for r in rows if r[4]!=0 and r[5]<50]
print("obs 있는 trace",len(has))
st=collections.Counter(r[8] for r in has)
print("static flag:",dict(st))
# 동적/정적별 dist 분포
for s in (0,1):
    d=np.array([r[5] for r in has if r[8]==s])
    if len(d): print(f"static={s} n={len(d)} dist p10 {np.percentile(d,10):.2f} p50 {np.median(d):.2f} p90 {np.percentile(d,90):.2f}")
# 모드별
print("\n모드 x static:")
c=collections.Counter((r[13],r[8]) for r in has)
for k,v in sorted(c.items(),key=lambda x:-x[1]): print(f"  {k}: {v}")
pickle.dump(rows,open("trace.pkl","wb"))
