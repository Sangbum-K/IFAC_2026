import pickle, numpy as np
from pathlib import Path
D=pickle.load(open(Path(__file__).parent/"d124904.pkl","rb"))
fre=D["/car_state/frenet/odom"]
def at(t,c): return np.interp(t,fre[:,0],fre[:,c])
print("=== /perception/obstacles_frenet 58.5-62.0 ===")
print("  t     ego_s  ego_d | id  s_c    d_c   [s_st,s_en] [d_r,d_l]  size  stat vis  gap")
for t,obs in D["obs_frenet"]:
    if not (58.5<=t<=62.0): continue
    es,ed=at(t,1),at(t,2)
    line=f"  {t:5.2f} {es:6.2f} {ed:+5.2f} |"
    if not obs: print(line+" (none)"); continue
    parts=[]
    for o in sorted(obs,key=lambda o:(o['s_start']-es)%37.5):
        gap=(o['s_start']-es)%37.5
        if gap>18: gap-=37.5
        parts.append(f"#{o['id']} s{o['s_center']:6.2f} d{o['d_center']:+5.2f} [{o['s_start']:5.2f},{o['s_end']:5.2f}] d[{o['d_right']:+5.2f},{o['d_left']:+5.2f}] sz{o['size']:.2f} {'S' if o['is_static'] else 'D'}{'V' if o['is_visible'] else '-'} g{gap:+5.2f}")
    print(line+"  ".join(parts[:3]))
