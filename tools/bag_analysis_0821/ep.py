import pickle,numpy as np,collections
rows=pickle.load(open("trace.pkl","rb"))
D=pickle.load(open("d.pkl","rb"))
ack=D["/ackermann_cmd"]
def cmd(t):
    i=np.searchsorted(ack[:,0],t); i=min(max(i,0),len(ack)-1); return ack[i,2]
# 에피소드: trace 가 0.5s 이상 끊기면 분할
eps=[]; cur=[]
for r in rows:
    if cur and r[0]-cur[-1][0]>0.7:
        eps.append(cur); cur=[]
    cur.append(r)
if cur: eps.append(cur)
print(f"에피소드 {len(eps)}개 (trace 연속 구간)")
print(f"{'t0':>7} {'dur':>5} {'dyn%':>5} {'dist0':>6} {'v0':>5} {'minD':>5} {'minV':>5} {'meanV':>5} | 모드 분포")
out=[]
for e in eps:
    dur=e[-1][0]-e[0][0]
    if dur<1.0: continue
    dyn=1-np.mean([r[8] for r in e])
    dist=np.array([r[5] for r in e]); v=np.array([r[2] for r in e])
    c=collections.Counter(r[13] for r in e)
    tot=len(e)
    ms=" ".join(f"{k[:5]}{v2*100//tot}%" for k,v2 in c.most_common())
    out.append((e[0][0],dur,dyn,e[0][5],e[0][2],dist.min(),v.min(),v.mean(),ms,e))
    print(f"{e[0][0]:7.1f} {dur:5.1f} {dyn*100:5.0f} {e[0][5]:6.2f} {e[0][2]:5.2f} {dist.min():5.2f} {v.min():5.2f} {v.mean():5.2f} | {ms}")
pickle.dump([o[:9] for o in out],open("eps.pkl","wb"))
