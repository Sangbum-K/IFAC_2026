import pickle,sys,numpy as np
D=pickle.load(open(sys.argv[1],'rb'))
fr=D['/car_state/frenet/odom']  # t,x=s,y=d,yaw,v ?
print("frenet cols sample:",fr[:3])
print("dur",fr[-1,0])
# reject counters cumulative?
for k in sorted(D):
    if k.startswith('/pitwall/reject') or k.startswith('/pitwall/outcome'):
        a=D[k]
        if len(a): print(f"{k:60s} n={len(a):4d} first={a[0,1]:.0f} last={a[-1,1]:.0f}")
