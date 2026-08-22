import pickle, numpy as np
D=pickle.load(open('d.pkl','rb'))
gw=D['global_wpnts'][0][1]
S,X,Y,DR,DL,PSI=gw[:,0],gw[:,2],gw[:,3],gw[:,4],gw[:,5],gw[:,6]
pf=D['/pf/pose/odom']
def xy2sd(px,py):
    i=int(np.argmin((X-px)**2+(Y-py)**2)); dx,dy=px-X[i],py-Y[i]; ps=PSI[i]
    return S[i]+dx*np.cos(ps)+dy*np.sin(ps), -dx*np.sin(ps)+dy*np.cos(ps)
for t in (23.3,23.6,23.9,24.2):
    ts,amin,ainc,rr=min(D['scan'],key=lambda r:abs(r[0]-t))
    i=np.argmin(np.abs(pf[:,0]-ts)); ex,ey,eyaw=pf[i,1],pf[i,2],pf[i,3]
    ang=amin+ainc*np.arange(len(rr)); ok=np.isfinite(rr)&(rr>0.1)&(rr<12)
    px=ex+rr[ok]*np.cos(eyaw+ang[ok]); py=ey+rr[ok]*np.sin(eyaw+ang[ok])
    sd=np.array([xy2sd(a,b) for a,b in zip(px,py)])
    m=(sd[:,0]>6.4)&(sd[:,0]<8.2)
    pts=sd[m]; pts=pts[np.argsort(pts[:,1])]
    es,ed=xy2sd(ex,ey)
    # cluster in d
    cl=[];cur=[pts[0]]
    for p in pts[1:]:
        if p[1]-cur[-1][1]<0.10: cur.append(p)
        else: cl.append(np.array(cur)); cur=[p]
    cl.append(np.array(cur))
    print(f"\n=== t={ts:.2f} ego s={es:.2f} d={ed:+.2f} | s-window 6.4~8.2, {len(pts)} returns ===")
    prev_hi=None
    for c in cl:
        if len(c)<2: continue
        lo,hi=c[:,1].min(),c[:,1].max()
        gap=f"  <-- 빈틈 {lo-prev_hi:.2f} m" if prev_hi is not None else ""
        print(f"   d[{lo:+.2f},{hi:+.2f}] w={hi-lo:.2f} s[{c[:,0].min():.2f},{c[:,0].max():.2f}] n={len(c)}{gap}")
        prev_hi=hi
