import pickle
from pathlib import Path
import numpy as np
D=pickle.load(open(Path(__file__).parent/"d124904.pkl","rb"))
fre=D["/car_state/frenet/odom"]; pf=D["/pf/pose/odom"]; od=D["/odom"]
dt=np.diff(fre[:,0])
print("frenet dt: med %.4f p99 %.4f max %.4f  n=%d"%(np.median(dt),np.percentile(dt,99),dt.max(),len(fre)))
big=np.where(dt>0.08)[0]
print("gaps>80ms:", [(round(fre[i,0],2),round(dt[i],3)) for i in big][:40])
m=(fre[:,0]>=59.0)&(fre[:,0]<=62.2)
print("\n  t      s      d     yaw    v      |  pf_v")
for r in fre[m]:
    print(f"  {r[0]:6.3f} {r[1]:6.2f} {r[2]:+5.2f} {r[3]:+6.3f} {r[4]:5.2f}  | {np.interp(r[0],pf[:,0],pf[:,4]):5.2f}")
