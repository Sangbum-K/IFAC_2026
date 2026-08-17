import pickle, numpy as np
from pathlib import Path
D=pickle.load(open(Path(__file__).parent/"d124904.pkl","rb"))
fre=D["/car_state/frenet/odom"]
LAPS=[(14.5,18.5,"lap0"),(25.5,29.6,"lap1"),(37.0,41.0,"lap2"),(47.5,51.0,"lap3"),(57.5,60.5,"lap4 충돌")]
print("  랩         인지 최초발행 t / ego_s / 거리 | 회피선 t / ego_s | 반응지연")
for a,b,lab in LAPS:
    fp=None
    for t,obs in D["obs_frenet"]:
        if not (a<=t<=b): continue
        for o in obs:
            if 32.4<=o["s_center"]<=34.6:
                es=np.interp(t,fre[:,0],fre[:,1])
                if es<32.0: fp=(t,es,o["s_start"]-es); break
        if fp: break
    hits=[tt for tt,W in D["otwpnts"] if a<=tt<=b and len(W)>=5
          and len(W[(W[:,0]>=33.0)&(W[:,0]<=34.0)]) and W[(W[:,0]>=33.0)&(W[:,0]<=34.0)][:,1].mean()>0.35]
    if fp and hits:
        t1=hits[0]; es1=np.interp(t1,fre[:,0],fre[:,1])
        print(f"  {lab:10s} t={fp[0]:6.2f} s={fp[1]:5.2f} 거리={fp[2]:4.2f}m | t={t1:6.2f} s={es1:5.2f} | {t1-fp[0]:4.2f}s / {es1-fp[1]:4.2f}m")
    else:
        print(f"  {lab:10s} fp={fp} hits={len(hits)}")
