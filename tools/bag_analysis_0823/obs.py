import pickle, numpy as np, sys
D=pickle.load(open(sys.argv[3] if len(sys.argv)>3 else 'd43.pkl','rb'))
gw=D['global_wpnts'][0][1]; L=gw[-1,0]+(gw[-1,0]-gw[-2,0])
fr=D['/car_state/frenet/odom']
def at(a,t):
    i=np.searchsorted(a[:,0],t); return a[min(max(i,0),len(a)-1)]
def wrap(x):
    x=x%L
    return x-L if x>L/2 else x
t0,t1=float(sys.argv[1]),float(sys.argv[2])
lw=D['local_wpnts']; ot=D['otwpnts']
for t,obs in D['obs_frenet']:
    if not(t0<=t<=t1): continue
    f=at(fr,t)
    parts=[]
    for o in obs:
        gap=wrap(o['s_center']-f[1])
        parts.append("id%d s[%.2f,%.2f]d[%+.2f,%+.2f]%s%s Δs%+.2f"%(
            o['id'],o['s_start'],o['s_end'],o['d_right'],o['d_left'],
            'S' if o['is_static'] else 'D', 'v' if o['is_visible'] else '-', gap))
    # local wpnt d 프로파일 앞 4m
    i=np.searchsorted([x[0] for x in lw],t); i=min(max(i,0),len(lw)-1)
    W=lw[i][1]
    ds=np.array([wrap(w-f[1]) for w in W[:,0]])
    sel=(ds>=0)&(ds<=4.0)
    prof=" ".join("%+.2f"%v for v in W[sel][::3,1]) if sel.any() else "-"
    print("%6.2f s=%6.2f d=%+.3f v=%.2f | %s"%(t,f[1],f[2],f[4]," ; ".join(parts) if parts else "(none)"))
    print("        lw_d(0~4m): %s"%prof)
