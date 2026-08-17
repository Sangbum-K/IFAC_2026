import pickle, sys
from pathlib import Path
import numpy as np
D = pickle.load(open(Path(__file__).parent/"d124904.pkl","rb"))
np.set_printoptions(suppress=True, precision=3)
T0,T1 = 54.0, 63.5

fre=D["/car_state/frenet/odom"]; cmd=D["/autonomy/ackermann_cmd"]
print("=== pitwall events 54-63.5 ===")
for t,m in D["events"]:
    if T0<=t<=T1: print(f"{t:7.2f}  {m.split(']')[-1].strip()}")

print("\n=== rosout (spliner/control/perception) 54-63.5 ===")
for t,lvl,name,msg in D["rosout"]:
    if T0<=t<=T1 and lvl>=20:
        print(f"{t:7.2f} L{lvl} [{name}] {msg[:170]}")
