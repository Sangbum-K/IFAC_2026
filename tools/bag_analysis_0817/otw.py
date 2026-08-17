import pickle, numpy as np
from pathlib import Path
D=pickle.load(open(Path(__file__).parent/"d124904.pkl","rb"))
fre=D["/car_state/frenet/odom"]
def at(t,c): return np.interp(t,fre[:,0],fre[:,c])
def dump(a,b,lab):
    print(f"\n=== {lab} : otwpnts(회피선) ===")
    print("   t    ego_s  ego_d  v   | n   s범위        d@33.5  d@34.0  d_max  d_min")
    for t,W in D["otwpnts"]:
        if not (a<=t<=b): continue
        es=at(t,1)
        if not len(W):
            print(f"  {t:5.2f} {es:6.2f} {at(t,2):+5.2f} {at(t,4):4.2f} | (빈 회피선)"); continue
        s=W[:,0]; d=W[:,1]
        def q(x):
            if s.min()<=x<=s.max(): return f"{np.interp(x,s,d):+6.2f}"
            return "   -  "
        print(f"  {t:5.2f} {es:6.2f} {at(t,2):+5.2f} {at(t,4):4.2f} | {len(W):3d} [{s.min():5.2f},{s.max():5.2f}] {q(33.5)} {q(34.0)} {d.max():+6.2f} {d.min():+6.2f}")
dump(58.85,60.35,"lap4 충돌랩")
dump(15.6,16.9,"lap0 성공랩")
