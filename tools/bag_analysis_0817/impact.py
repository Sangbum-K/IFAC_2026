import pickle, numpy as np
from pathlib import Path
D=pickle.load(open(Path(__file__).parent/"d124904.pkl","rb"))
fre=D["/car_state/frenet/odom"]; cmd=D["/autonomy/ackermann_cmd"]; ak=D["/ackermann_cmd"]
imu=D["/sensors/imu/raw"]; core=D["/sensors/core"]; pf=D["/pf/pose/odom"]
srv=D.get("/commands/servo/position"); brk=D.get("/commands/motor/brake")
# scan front clearance
rows=[]
for t,amin,ainc,r in D["scan"]:
    if not (59.0<=t<=62.5): continue
    n=len(r); ang=amin+ainc*np.arange(n); ok=(r>0.05)&np.isfinite(r)
    xs=r*np.cos(ang); ys=r*np.sin(ang)
    # narrow corridor ahead (|y|<0.20), and full front sector
    c=ok&(xs>0)&(np.abs(ys)<0.20)
    c2=ok&(xs>0)&(np.abs(ys)<0.45)
    rows.append((t, xs[c].min() if c.any() else np.nan, xs[c2].min() if c2.any() else np.nan,
                 r[ok].min() if ok.any() else np.nan))
S=np.array(rows)
def at(a,t,c): return np.interp(t,a[:,0],a[:,c])
print("  t      s     d      v    | cmd_v  steer  | ax    ay     wz   | imot  erpm  | fwd0.2 fwd0.45 rmin")
for tt in np.arange(59.6,62.31,0.06):
    print(f"  {tt:5.2f} {at(fre,tt,1):5.2f} {at(fre,tt,2):+5.2f} {at(fre,tt,4):5.2f} | "
          f"{at(cmd,tt,2):5.2f} {np.degrees(at(cmd,tt,1)):+6.1f} | "
          f"{at(imu,tt,5):+5.2f} {at(imu,tt,6):+5.2f} {at(imu,tt,4):+7.1f} | "
          f"{at(core,tt,2):+6.1f} {at(core,tt,3):7.0f} | "
          f"{at(S,tt,1):5.2f} {at(S,tt,2):5.2f} {at(S,tt,3):5.2f}")
