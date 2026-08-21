import pickle,re,numpy as np,collections
R=pickle.load(open("rosout2.pkl","rb"))
rows=pickle.load(open("trace.pkl","rb"))
# committed_break 사유
br=collections.Counter()
for t,l,n,m in R:
    mm=re.search(r"committed_break pos=([\d.]+)/([\d.]+) stable_obs=(\d+) reason=(\w+)",m)
    if mm: br[mm.group(4)]+=1
print("committed_break 사유:",dict(br), "합계",sum(br.values()))
# 에피소드 내 closest id 변화
eps=[];cur=[]
for r in rows:
    if cur and r[0]-cur[-1][0]>0.7: eps.append(cur);cur=[]
    cur.append(r)
eps.append(cur)
ch=[];dur=[]
for e in eps:
    if e[-1][0]-e[0][0]<1.0: continue
    ids=[r[4] for r in e if r[4]>0]
    if not ids: continue
    sw=sum(1 for a,b in zip(ids,ids[1:]) if a!=b)
    ch.append(sw); dur.append(e[-1][0]-e[0][0])
ch=np.array(ch);dur=np.array(dur)
print(f"\n에피소드({len(ch)}개) 내 '최근접 장애물 id' 전환: 합 {ch.sum()}, 초당 {ch.sum()/dur.sum():.2f}회, 중앙 {np.median(ch):.0f}")
# 분류 전환 시점 = 첫 관측 후 얼마?
first={};turn={}
for r in rows:
    i=r[4]
    if i<=0: continue
    if i not in first: first[i]=(r[0],r[5],r[8])
    if i not in turn and r[8]==0 and first[i][2]==1: turn[i]=r[0]
lat=[(turn[i]-first[i][0], first[i][1]) for i in turn]
if lat:
    a=np.array(lat)
    print(f"'정적으로 태어나 동적이 되기까지' {len(lat)}건: 지연 p50 {np.median(a[:,0]):.2f}s p90 {np.percentile(a[:,0],90):.2f}s / 그동안 최초거리 p50 {np.median(a[:,1]):.2f}m")
# static 으로 시작한 트랙 비율
s0=sum(1 for i in first if first[i][2]==1); print(f"trace 최근접으로 처음 등장할 때 static=1 인 id: {s0}/{len(first)}")
# 동적 사이클에서 예측튜브 존재?
D=pickle.load(open("d.pkl","rb"))
P=D["obs_pred"]
pt=np.array([[t,len(o)] for t,o in P])
F=D["obs_frenet"]
dyn_t=np.array([t for t,o in F if any(not x["is_static"] for x in o)])
print(f"\nfrenet 프레임 {len(F)} 중 동적 플래그 포함 {len(dyn_t)} ({len(dyn_t)/len(F)*100:.0f}%)")
# 그 시각에 예측이 비어있지 않았나
idx=np.searchsorted(pt[:,0],dyn_t); idx=np.clip(idx,0,len(pt)-1)
ok=(pt[idx,1]>0)&(np.abs(pt[idx,0]-dyn_t)<0.3)
print(f"동적 프레임 중 같은 시각 예측튜브 존재: {ok.mean()*100:.1f}%")
