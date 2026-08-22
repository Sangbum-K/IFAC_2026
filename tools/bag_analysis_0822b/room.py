#!/usr/bin/env python3
"""lane_wall_no_room 의 기하: 장애물 d 범위 vs 트랙 경계, 남는 틈"""
import pickle,re,numpy as np,collections
D=pickle.load(open("d17.pkl","rb")); L=pickle.load(open("log17.pkl","rb"))
gw=D["global_wpnts"][0][1]; TL=gw[-1,0]
pat=re.compile(r"s=([-\d.]+) v=([-\d.]+) d=([-\d.]+) \| obs id=(-?\d+) dist=([-\d.]+) d\[([-\d.]+),([-\d.]+)\] static=(\d) nobs=(\d+) \| evade=(\d) sector=(\d) commit=(\d) \| (\w+) \| (.*)$")
R=[]
for t,lvl,n,m in L:
    if "[trace]" not in m or not(4<=t<=170): continue
    g=pat.search(m)
    if g: R.append(dict(t=t,s=float(g[1]),v=float(g[2]),d=float(g[3]),oid=int(g[4]),
        dist=float(g[5]),o_lo=float(g[6]),o_hi=float(g[7]),mode=g[13],tags=g[14]))
def bounds(s):
    i=np.argmin(np.abs(gw[:,0]-(s%TL))); return gw[i,4],gw[i,5]   # d_right, d_left
MARG=0.25   # RequiredBoundaryMargin (차반폭0.15+추적여유0.10)
CLR=0.20    # path_check_margin
print("장애물 d 범위와 좌/우 통과 틈 (틈 = 경계여유 이후 남는 폭 - 필요 0.20)")
print("모드/사유별:")
for name,sel in (("전체",lambda r:True),
                 ("lane_wall_no_room 포함",lambda r:"lane_wall_no_room" in r["tags"]),
                 ("trailing 모드",lambda r:r["mode"]=="trailing"),
                 ("lane_switch 성공",lambda r:r["mode"]=="lane_switch")):
    g=[r for r in R if sel(r)]
    if not g: continue
    left=[];right=[];w=[]
    for r in g:
        # 장애물 s 위치의 경계. dist 만큼 앞 -> s+dist
        dr,dl=bounds(r["s"]+max(r["dist"],0))
        # 좌측(더 큰 d) 통과: obs_hi ~ +dl
        left.append((dl-MARG)-r["o_hi"]-CLR)
        right.append(r["o_lo"]-(-dr+MARG)-CLR)
        w.append(dl+dr)
    left=np.array(left);right=np.array(right)
    best=np.maximum(left,right)
    print("  %-24s n=%4d | 좌틈 중앙 %+.2f 우틈 중앙 %+.2f | 넓은쪽 중앙 %+.2f | 양쪽 다 음수 %.0f%% | 트랙폭 중앙 %.2f"%(
        name,len(g),np.median(left),np.median(right),np.median(best),100*np.mean(best<0),np.median(w)))
print()
print("[lane_wall_no_room 이 뜬 s 위치 분포]")
g=[r for r in R if "lane_wall_no_room" in r["tags"]]
h=collections.Counter(int(r["s"]) for r in g)
for s,c in sorted(h.items()):
    if c>=10:
        dr,dl=bounds(s)
        print("  s=%2d  %4d 사이클  트랙폭 %.2f (dl %.2f dr %.2f)"%(s,c,dl+dr,dl,dr))
print()
print("[장애물 폭] o_hi-o_lo 중앙 %.2f p90 %.2f  (반경 0.25 고정원통 -> 폭 0.50 예상)"%(
    np.median([r["o_hi"]-r["o_lo"] for r in R]),np.percentile([r["o_hi"]-r["o_lo"] for r in R],90)))
