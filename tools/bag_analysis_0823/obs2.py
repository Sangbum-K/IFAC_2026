import pickle, numpy as np, sys
tag=sys.argv[3] if len(sys.argv)>3 else 'd43.pkl'
D=pickle.load(open(tag,'rb'))
gw=D['global_wpnts'][0][1]; L=gw[-1,0]+(gw[-1,0]-gw[-2,0])
fr=D['/car_state/frenet/odom']
def at(a,t):
    i=np.searchsorted(a[:,0],t); return a[min(max(i,0),len(a)-1)]
def wrap(x):
    x=x%L
    return x-L if x>L/2 else x
def bounds(s):
    i=int(np.argmin(np.abs(((gw[:,0]-s+L/2)%L)-L/2)))
    return gw[i,4],gw[i,5],gw[i,7]   # d_right,d_left,kappa
t0,t1=float(sys.argv[1]),float(sys.argv[2])
lw=D['local_wpnts']; ot=D['otwpnts']
ots=np.array([x[0] for x in ot]); lws=np.array([x[0] for x in lw])
prev=None
for t,obs in D['obs_frenet']:
    if not(t0<=t<=t1): continue
    f=at(fr,t)
    parts=[]
    for o in obs:
        parts.append("id%d s[%.2f,%.2f] d[%+.2f,%+.2f]%s Δs%+.2f"%(
            o['id'],o['s_start'],o['s_end'],o['d_right'],o['d_left'],
            'S' if o['is_static'] else 'D', wrap(o['s_center']-f[1])))
    dr,dl,kap=bounds(f[1])
    i=min(np.searchsorted(lws,t),len(lw)-1); W=lw[i][1]
    ds=np.array([wrap(w-f[1]) for w in W[:,0]])
    sel=np.argsort(np.abs(ds))
    # d at +1,2,3,4 m ahead
    def dat(x):
        j=int(np.argmin(np.abs(ds-x))); return W[j,1],W[j,8]
    j=min(np.searchsorted(ots,t),len(ot)-1)
    otn=len(ot[j][1]) if abs(ot[j][0]-t)<0.2 else 0
    print("%6.2f s=%6.2f d=%+.3f v=%.2f | 트랙d[%+.2f,%+.2f] κ=%+.2f | lw_d@1/2/3/4=%+.2f/%+.2f/%+.2f/%+.2f v@2=%.2f ot_n=%d | %s"%(
        t,f[1],f[2],f[4],dr,dl,kap,dat(1)[0],dat(2)[0],dat(3)[0],dat(4)[0],dat(2)[1],otn,
        " ; ".join(parts) if parts else "-"))
