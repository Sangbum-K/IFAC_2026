import pickle,sys,numpy as np
D=pickle.load(open(sys.argv[1],'rb'))
t0,t1=float(sys.argv[2]),float(sys.argv[3])
fr=D['/car_state/frenet/odom']; ac=D['/ackermann_cmd']; la=D['lookahead']
le=D['/pitwall/lat_err']; sv=D['/commands/servo/position']
lw=D['local_wpnts']; ot=D['otwpnts']
def at(A,t,c=1):
    i=int(np.argmin(np.abs(A[:,0]-t))); return A[i,c]
OF=D['obs_frenet']; ot_t=np.array([x[0] for x in OF])
print("  t     s      d     v   |  steer  servo | lat_err | 가까운장애물 d[..] dist | local d(0..3m)")
for t in np.arange(t0,t1,0.1):
    i=int(np.argmin(np.abs(fr[:,0]-t)))
    s,d=fr[i,1],fr[i,2]
    v=at(ac,t,2); st=at(ac,t,1); sp=at(sv,t); e=at(le,t)
    j=int(np.argmin(np.abs(ot_t-t)))
    ob=""
    L=34.3366476
    best=None
    for o in OF[j][1]:
        ds=(o['s_center']-s)%L
        if ds<6 and (best is None or ds<best[0]): best=(ds,o)
    if best: ob=f"id{best[1]['id']} d[{best[1]['d_right']:+.2f},{best[1]['d_left']:+.2f}] {best[0]:.2f}m"
    # local path lateral at ~1,2,3m ahead
    k=int(np.argmin(np.abs(np.array([x[0] for x in lw])-t)))
    W=lw[k][1]; prof=""
    if len(W):
        for dd in (1.0,2.0,3.0):
            idx=int(np.argmin(np.abs(((W[:,0]-s)%L)-dd)))
            prof+=f"{W[idx,1]:+.2f} "
    print(f"{t:6.2f} {s:6.2f} {d:+.2f} {v:5.2f} | {st:+.3f} {sp:.3f} | {e:+.3f} | {ob:28s} | {prof}")
