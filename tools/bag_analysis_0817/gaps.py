import pickle, numpy as np
from pathlib import Path
D=pickle.load(open(Path(__file__).parent/"d124904.pkl","rb"))
for key,name in (("obs_perc","/perception/obstacles"),("obs_frenet","/perception/obstacles_frenet"),
                 ("otwpnts","/planner/avoidance/otwpnts"),("local_wpnts","/local_waypoints")):
    T=np.array([t for t,_ in D[key]])
    dt=np.diff(T)
    print(f"{name:34s} n={len(T)} rate={len(T)/(T[-1]-T[0]):.1f}Hz  dt med={np.median(dt)*1e3:.0f}ms p95={np.percentile(dt,95)*1e3:.0f} max={dt.max()*1e3:.0f}")
    big=np.where(dt>0.15)[0]
    print("   gaps>150ms:", " ".join(f"{T[i]:.2f}(+{dt[i]*1e3:.0f}ms)" for i in big[:25]))
sc=np.array([t for t,_,_,_ in D["scan"]])
print(f"/scan n={len(sc)} rate={len(sc)/(sc[-1]-sc[0]):.1f}Hz max gap {np.diff(sc).max()*1e3:.0f}ms")
