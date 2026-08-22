import pickle,sys,numpy as np,collections
D=pickle.load(open(sys.argv[1],'rb'))
pf=D['/pf/pose/odom']; sc=D['scan']; OF=D['obs_frenet']
ot=np.array([x[0] for x in OF])
def pose(t):
    i=int(np.searchsorted(pf[:,0],t)); i=min(max(i,0),len(pf)-1); return pf[i,1],pf[i,2],pf[i,3]
# 가시 정적 트랙별 중심 (평균)
cen=collections.defaultdict(list)
for t,obs in OF:
    for o in obs:
        if o['is_static'] and o['is_visible']: cen[o['id']].append((t,o['x'],o['y']))
res=[]
for oid,L in cen.items():
    if len(L)<15: continue
    ox,oy=np.mean([a[1] for a in L]),np.mean([a[2] for a in L])
    ts=set(round(a[0],2) for a in L)
    W=[]
    for t,amin,ainc,rr in sc:
        if round(t,2) not in ts: continue
        px,py,yaw=pose(t)
        d_car=np.hypot(ox-px,oy-py)
        if not (0.7<d_car<2.5): continue          # 근거리 프레임만
        rr=np.asarray(rr,float); ang=amin+ainc*np.arange(len(rr))
        m=np.isfinite(rr)&(rr>0.05)&(rr<12)
        X=px+rr[m]*np.cos(yaw+ang[m]); Y=py+rr[m]*np.sin(yaw+ang[m])
        sel=np.hypot(X-ox,Y-oy)<0.35              # 벽 오염 줄이려 반경 축소
        if sel.sum()<4: continue
        P=np.c_[X[sel],Y[sel]]
        # 시선 수직 방향(=회랑을 막는 방향) 폭
        u=np.array([ox-px,oy-py]); u/=np.linalg.norm(u)
        perp=np.array([-u[1],u[0]])
        proj=P@perp
        W.append(proj.max()-proj.min())
    if len(W)>=10:
        W=np.array(W); res.append((oid,len(W),np.median(W),np.percentile(W,90),W.max()))
print(f"{sys.argv[1]}  가시 정적 트랙 {len(res)}개 — 시선수직 실측폭 [m]")
for oid,n,md,p90,mx in sorted(res):
    print(f"  id {oid:4d} n={n:3d}  median {md:.3f}  p90 {p90:.3f}  max {mx:.3f}   (half {md/2:.3f})")
if res:
    a=np.array([r[2] for r in res])
    print(f"  --> 트랙별 median 폭의 중앙값 {np.median(a):.3f} m  (half {np.median(a)/2:.3f}),  p90 {np.percentile(a,90):.3f}")
