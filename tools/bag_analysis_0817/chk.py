import pickle, numpy as np
from pathlib import Path
D=pickle.load(open(Path(__file__).parent/"d124904.pkl","rb"))
print("keys with map/tf:", [k for k in D if k in ("map","tf_static")])
print("tf_static:", D.get("tf_static"))
m=D.get("map")
if m: print("map res",m["res"],"origin",m["ox"],m["oy"],"size",m["w"],m["h"])
gw=D["global_wpnts"]
print("global_wpnts msgs:",len(gw))
if gw:
    a=gw[0][1]; print("cols=s,d,x,y,d_r,d_l,psi,kappa,vx,ax  n=",len(a))
    print(a[330:380:5])
