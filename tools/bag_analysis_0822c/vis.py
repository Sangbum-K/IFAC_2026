import lib2, numpy as np, pickle
d=lib2.D(); g=lib2.gw(d)
S,X,Y,PSI=g[:,0],g[:,2],g[:,3],g[:,6]
pf=d['/pf/pose/odom']
def xy2sd(px,py):
    i=int(np.argmin((X-px)**2+(Y-py)**2)); dx,dy=px-X[i],py-Y[i]; ps=PSI[i]
    return S[i]+dx*np.cos(ps)+dy*np.sin(ps), -dx*np.sin(ps)+dy*np.cos(ps)
print(" t     ego_s  ego_v  거리   장애물영역 반사점수")
for ts,amin,ainc,rr in d['scan']:
    if not (20.5<=ts<=23.6): continue
    i=np.argmin(np.abs(pf[:,0]-ts)); ex,ey,eyaw,ev=pf[i,1],pf[i,2],pf[i,3],pf[i,4]
    ang=amin+ainc*np.arange(len(rr)); ok=np.isfinite(rr)&(rr>0.1)&(rr<12)
    px=ex+rr[ok]*np.cos(eyaw+ang[ok]); py=ey+rr[ok]*np.sin(eyaw+ang[ok])
    sd=np.array([xy2sd(a,b) for a,b in zip(px,py)])
    m=(sd[:,0]>6.7)&(sd[:,0]<7.8)&(sd[:,1]>-1.2)&(sd[:,1]<0.25)
    es,ed=xy2sd(ex,ey)
    dist=(6.9-es)%37.0
    print(f"{ts:6.2f} {es:6.2f} {ev:5.2f} {dist:5.2f}   n={m.sum():3d}")
