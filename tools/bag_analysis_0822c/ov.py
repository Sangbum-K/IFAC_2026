import pickle, numpy as np
D=pickle.load(open('d.pkl','rb'))
def near(lst,t,w=0.3):
    return [(ts,v) for ts,v in lst if abs(ts-t)<w]
fo=D['obs_frenet']
print("== obs_frenet around 23.7 ==")
for ts,o in fo:
    if 22.5<=ts<=25.5:
        pass
# print at ~0.5s steps
last=-9
for ts,o in fo:
    if 20.0<=ts<=28.0 and ts-last>0.4:
        last=ts
        print(f"t={ts:6.2f} n={len(o)}")
        for x in o:
            print(f"   id{x['id']:4d} s[{x['s_start']:6.2f},{x['s_end']:6.2f}] d[{x['d_right']:+.2f},{x['d_left']:+.2f}] "
                  f"vs={x['vs']:+.2f} vd={x['vd']:+.2f} static={int(x['is_static'])} vis={int(x['is_visible'])} size={x['size']:.2f}")
