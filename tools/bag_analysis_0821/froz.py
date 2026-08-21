import pickle,numpy as np,collections
D=pickle.load(open("d.pkl","rb"))
L=36.833
otw=D["otwpnts"]
rows=pickle.load(open("trace.pkl","rb"))
rt=np.array([r[0] for r in rows]); rst=np.array([r[8] for r in rows]); rid=np.array([r[4] for r in rows])
def lab(t):
    k=np.searchsorted(rt,t); k=min(max(k,0),len(rt)-1)
    if abs(rt[k]-t)>0.25: return None
    return rst[k],rid[k]
# 동결 구간: (d,kappa,vx) 배열이 비트 단위 동일
runs=[];cur=None;prev=None
for t,w in otw:
    if len(w)==0:
        if cur and cur[1]-cur[0]>0.0: runs.append(cur)
        cur=None;prev=None;continue
    key=w[:,[1,7,8]].tobytes()
    if prev is not None and key==prev:
        cur=(cur[0],t,cur[2]+1)
    else:
        if cur and cur[1]-cur[0]>0.0: runs.append(cur)
        cur=(t,t,1)
    prev=key
if cur: runs.append(cur)
runs=[r for r in runs if r[1]-r[0]>0.15]
dur=np.array([r[1]-r[0] for r in runs])
print(f"otwpnts 동결 구간 {len(runs)}개, 총 {dur.sum():.1f}s (백 663s 중 {dur.sum()/663*100:.0f}%)")
print(f"  길이 p50 {np.median(dur):.2f}s p90 {np.percentile(dur,90):.2f}s max {dur.max():.2f}s")
# 동적/정적 라벨
tracks=collections.defaultdict(list)
for t,o in D["obs_frenet"]:
    for x in o: tracks[x["id"]].append((t,x["d_center"],x["s_center"],x["vs"]))
def dmove(i,t0,t1):
    v=tracks.get(i)
    if not v: return None
    a=np.array(v); m=(a[:,0]>=t0)&(a[:,0]<=t1)
    if m.sum()<3: return None
    return a[m,1].max()-a[m,1].min()
dyn=[];sta=[];mv=[]
for r in runs:
    l=lab(r[0])
    if l is None: continue
    if l[0]==0:
        dyn.append(r[1]-r[0])
        d=dmove(l[1],r[0],r[1])
        if d is not None: mv.append((r[1]-r[0],d))
    else: sta.append(r[1]-r[0])
print(f"  동적 라벨 동결 {len(dyn)}개 총 {sum(dyn):.1f}s (p50 {np.median(dyn):.2f} p90 {np.percentile(dyn,90):.2f} max {max(dyn):.2f})")
print(f"  정적 라벨 동결 {len(sta)}개 총 {sum(sta):.1f}s (p50 {np.median(sta):.2f} p90 {np.percentile(sta,90):.2f})")
mv=np.array(mv)
print(f"\n동적 동결 구간 {len(mv)}개에서 그 사이 상대의 d 이동폭:")
print(f"  p50 {np.median(mv[:,1]):.2f} p90 {np.percentile(mv[:,1],90):.2f} max {mv[:,1].max():.2f} m")
long=mv[mv[:,0]>0.5]
print(f"  0.5s 넘게 동결된 {len(long)}개만: d 이동폭 p50 {np.median(long[:,1]):.2f} p90 {np.percentile(long[:,1],90):.2f} max {long[:,1].max():.2f} m")
print(f"  그중 상대가 0.35m(=evasion_dist) 넘게 움직인 비율 {np.mean(long[:,1]>0.35)*100:.0f}%")
