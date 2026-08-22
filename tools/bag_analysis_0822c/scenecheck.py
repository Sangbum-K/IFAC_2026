import pickle, numpy as np
D=pickle.load(open('d.pkl','rb'))
gw=D['global_wpnts'][0][1]   # s d x y d_r d_l psi kappa vx ax
S,X,Y,DR,DL,PSI=gw[:,0],gw[:,2],gw[:,3],gw[:,4],gw[:,5],gw[:,6]
track_len=S[-1]+ (S[1]-S[0])
pf=D['/pf/pose/odom']
def xy2sd(px,py):
    d2=(X-px)**2+(Y-py)**2; i=int(np.argmin(d2))
    # local frame
    dx,dy=px-X[i],py-Y[i]
    ps=PSI[i]
    ds= dx*np.cos(ps)+dy*np.sin(ps)
    dd=-dx*np.sin(ps)+dy*np.cos(ps)
    return S[i]+ds, dd
def scan_at(t):
    best=min(D['scan'],key=lambda r:abs(r[0]-t)); return best
# map occupancy
M=D['map']; res,ox,oy,w,h=M['res'],M['ox'],M['oy'],M['w'],M['h']
grid=M['data'].reshape(h,w)
def occ(px,py):
    c=int((px-ox)/res); r=int((py-oy)/res)
    if 0<=r<h and 0<=c<w: return grid[r,c]
    return -1
for t in (23.30, 23.62, 23.90):
    ts,amin,ainc,rr=scan_at(t)
    i=np.argmin(np.abs(pf[:,0]-ts)); ex,ey,eyaw=pf[i,1],pf[i,2],pf[i,3]
    ang=amin+ainc*np.arange(len(rr))
    ok=np.isfinite(rr)&(rr>0.1)&(rr<12)
    # laser frame -> base_link offset ~0.27? use tf_static
    px=ex+rr[ok]*np.cos(eyaw+ang[ok]); py=ey+rr[ok]*np.sin(eyaw+ang[ok])
    sd=np.array([xy2sd(a,b) for a,b in zip(px,py)])
    es,ed=xy2sd(ex,ey)
    print(f"\n=== t={ts:.2f} ego s={es:.2f} d={ed:+.2f} ===")
    for sc in np.arange(6.6,8.4,0.2):
        m=(sd[:,0]>=sc)&(sd[:,0]<sc+0.2)
        j=np.argmin(np.abs(S-(sc+0.1)))
        if m.sum()==0:
            print(f" s={sc:.1f}-{sc+0.2:.1f}: no returns  (wall d {-DR[j]:+.2f}..{DL[j]:+.2f})"); continue
        dv=np.sort(sd[m,1])
        # free intervals between -DR and DL
        lo,hi=-DR[j],DL[j]
        pts=[p for p in dv if lo-0.3<p<hi+0.3]
        gaps=[]
        prev=lo
        for p in pts+[hi]:
            if p-prev>0.12: gaps.append((prev,p))
            prev=max(prev,p)
        gs=" ".join(f"[{a:+.2f},{b:+.2f}]w={b-a:.2f}" for a,b in gaps)
        print(f" s={sc:.1f}-{sc+0.2:.1f}: n={m.sum():3d} wall[{lo:+.2f},{hi:+.2f}] free {gs}")
