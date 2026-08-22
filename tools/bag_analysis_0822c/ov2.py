import pickle, numpy as np
D=pickle.load(open('d.pkl','rb'))
fo=D['obs_frenet']
fr=D['/car_state/frenet/odom']
def ego(t):
    i=np.argmin(np.abs(fr[:,0]-t)); return fr[i,1],fr[i,2],fr[i,4]
last=-9
for ts,o in fo:
    if 23.0<=ts<=31.0 and ts-last>0.28:
        last=ts
        s,d,v=ego(ts)
        print(f"t={ts:6.2f} ego s={s:6.2f} d={d:+.2f} v={v:.2f} n={len(o)}")
        for x in sorted(o,key=lambda x:-x['d_left']):
            print(f"     id{x['id']:4d} s[{x['s_start']:6.2f},{x['s_end']:6.2f}] d[{x['d_right']:+.2f},{x['d_left']:+.2f}] vis={int(x['is_visible'])} st={int(x['is_static'])}")
