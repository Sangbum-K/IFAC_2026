import pickle,numpy as np,collections
D=pickle.load(open("d.pkl","rb"))
rows=pickle.load(open("trace.pkl","rb"))
rt=np.array([r[0] for r in rows]); rst=np.array([r[8] for r in rows])
otw=[(t,w) for t,w in D["otwpnts"] if len(w)>0]
def lab(t):
    k=np.searchsorted(rt,t); k=min(max(k,0),len(rt)-1)
    return rst[k] if abs(rt[k]-t)<0.25 else None
# 에피소드 단위 side flip
eps=[];cur=[]
prev=None
for t,w in otw:
    if prev is not None and t-prev>0.7: eps.append(cur);cur=[]
    cur.append((t,w)); prev=t
eps.append(cur)
def apex(w):
    i=np.argmax(np.abs(w[:,1])); return w[i,1]
tot={0:[0,0,0.0],1:[0,0,0.0]}
for e in eps:
    if e[-1][0]-e[0][0]<1.5: continue
    ls=[lab(t) for t,w in e]
    ls=[x for x in ls if x is not None]
    if not ls: continue
    l=1 if np.mean(ls)>0.5 else 0
    ap=np.array([apex(w) for t,w in e])
    sig=np.sign(ap)
    sig=sig[np.abs(ap)>0.12]
    if len(sig)<5: continue
    fl=int(np.sum(np.abs(np.diff(sig))>1))
    tot[l][0]+=fl; tot[l][1]+=1; tot[l][2]+=e[-1][0]-e[0][0]
for l,n in ((0,"동적"),(1,"정적")):
    f,c,d=tot[l]
    if c: print(f"{n} 에피소드 {c}개 ({d:.0f}s): 회피 좌우 뒤집힘 {f}회 = 초당 {f/d:.2f}회, 에피소드당 {f/c:.1f}회")
# 진폭 변화율
for l,n in ((0,"동적"),(1,"정적")):
    rates=[]
    for e in eps:
        ls=[lab(t) for t,w in e]; ls=[x for x in ls if x is not None]
        if not ls or (1 if np.mean(ls)>0.5 else 0)!=l: continue
        ap=np.array([apex(w) for t,w in e]); ts=np.array([t for t,w in e])
        if len(ap)<5: continue
        rates.append(np.abs(np.diff(ap))/np.maximum(np.diff(ts),1e-3))
    if rates:
        r=np.concatenate(rates)
        print(f"{n}: 계획 apex 변화율 p50 {np.median(r):.2f} p90 {np.percentile(r,90):.2f} m/s")
