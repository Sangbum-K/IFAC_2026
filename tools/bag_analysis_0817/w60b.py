import pickle, re
from pathlib import Path
import numpy as np
D = pickle.load(open(Path(__file__).parent/"d124904.pkl","rb"))
fre=D["/car_state/frenet/odom"]   # t s d yaw v
def at(a,t,c): return np.interp(t,a[:,0],a[:,c])
print("=== trace log lag check: trace s/v vs ego frenet at same wall time ===")
for t,lvl,name,msg in D["rosout"]:
    m=re.match(r"\[trace\] s=([\d.-]+) v=([\d.-]+) d=([\d.-]+)",msg)
    if m and 53<=t<=63:
        ts,tv=float(m.group(1)),float(m.group(2))
        print(f"  t={t:6.2f} trace s={ts:6.2f} v={tv:4.2f} | ego s={at(fre,t,1):6.2f} v={at(fre,t,4):4.2f} d={at(fre,t,2):+5.2f}")
