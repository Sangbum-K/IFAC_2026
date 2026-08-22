import lib2, numpy as np
d=lib2.D(); m=lib2.Map(d); g=lib2.gw(d)
s,x,y,dr,dl,psi=g[:,0],g[:,2],g[:,3],g[:,4],g[:,5],g[:,6]
nx,ny=-np.sin(psi),np.cos(psi)
def room(i,sign,maxd=3.0):
    st=np.arange(0,maxd,0.01)
    px=x[i]+sign*st*nx[i]; py=y[i]+sign*st*ny[i]
    dist=m.dist(px,py); bad=np.where(dist<=0.001)[0]
    return st[bad[0]] if len(bad) else maxd
print(" s   d_l(plan) 실제좌 | d_r(plan) 실제우 | 실폭  선언폭")
for tgt in np.arange(6.4,8.5,0.2):
    i=int(np.argmin(np.abs(s-tgt)))
    tl=room(i,+1); tr=room(i,-1)
    print(f"{s[i]:5.2f}   {dl[i]:.2f}   {tl:.2f}  |  {dr[i]:.2f}   {tr:.2f}  | {tl+tr:.2f}  {dl[i]+dr[i]:.2f}")
print()
kap=g[:,7]
for tgt in np.arange(1.0,8.5,0.5):
    i=int(np.argmin(np.abs(s-tgt)))
    print(f"s={s[i]:5.2f} kappa={kap[i]:+.3f} R={1/abs(kap[i]) if abs(kap[i])>1e-3 else 999:.1f} vx={g[i,8]:.2f}")
