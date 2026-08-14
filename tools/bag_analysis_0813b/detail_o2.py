#!/usr/bin/env python3
"""O2 통과 한 랩의 사이클 단위 상세."""
import pickle, numpy as np, sys
D=pickle.load(open("data.pkl","rb")); TR=pickle.load(open("trace.pkl","rb"))
TL=32.07
fre=D["/car_state/frenet/odom"]
lat=D["/pitwall/lat_err"]; spd=D["/monitor/current_speed"]; st=D["/monitor/steering_input"]
ack=D["/ackermann_cmd"]
def at(a,t,c=1):
    if not len(a): return np.nan
    i=np.searchsorted(a[:,0],t); i=min(max(i,0),len(a)-1); return a[i,c]
T0,T1=(float(sys.argv[1]),float(sys.argv[2])) if len(sys.argv)>2 else (211.0,216.5)
print(f"=== t {T0}~{T1} ===")
print("    t     s   v_ego  cmd_v cmd_δ°  lat_err | 최근접장애물 dist  d[dr,dl] st nobs | mode        | rejects")
for x in TR:
    if not (T0<=x["t"]<=T1): continue
    t=x["t"]
    print(f" {t:7.2f} {x['s']:6.2f} {x['v']:5.2f} {at(ack,t,2):6.2f} "
          f"{np.degrees(at(ack,t,1)):+6.1f} {at(lat,t):+7.2f} | "
          f"id{x['oid']:>4} {x['dist']:5.2f} [{x['dr']:+.2f},{x['dl']:+.2f}] "
          f"{x['st']:>2} {x['nobs']:>3} | {x['mode']:<14}| {','.join(x['rej'])[:78]}")
print("\n--- 같은 창의 전체 장애물 목록 (obs_frenet) ---")
for t,obs in D["obs_frenet"]:
    if not (T0<=t<=T1): continue
    if len(obs)==0: continue
    es=np.interp(t,fre[:,0],fre[:,1])
    ss=" | ".join(f"id{o['id']} s[{o['s_start']:.2f},{o['s_end']:.2f}] d[{o['d_right']:+.2f},{o['d_left']:+.2f}] "
                  f"sz{o['size']:.2f} {'S' if o['is_static'] else 'D'}" for o in obs)
    print(f" {t:7.2f} ego_s={es:5.2f} n={len(obs)}: {ss[:190]}")
