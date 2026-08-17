import pickle, numpy as np
from pathlib import Path
D=pickle.load(open(Path(__file__).parent/"d124904.pkl","rb"))
def seg(a,name):
    if not len(a): print(name,"empty"); return
    ch=[]; prev=None
    for t,v in a:
        if v!=prev: ch.append((t,v)); prev=v
    print(f"--- {name} changes ---")
    for t,v in ch: print(f"   {t:7.2f}  {v:.0f}")
seg(D["/pf/localization_ready"],"/pf/localization_ready")
seg(D["/safety/emergency_stop"],"/safety/emergency_stop")
seg(D["/planner/escape_active"],"/planner/escape_active")
print("\n--- pf_quality (변화만) ---")
prev=None
for t,s in D["pf_quality"]:
    if s!=prev: print(f"   {t:7.2f}  {s[:120]}"); prev=s
print("\n--- IMU |a| 58-63 (0.05s 그리드) ---")
imu=D["/sensors/imu/raw"]   # t yaw wx wy wz ax ay az
m=(imu[:,0]>=58)&(imu[:,0]<=63.5)
sub=imu[m]
for i in range(0,len(sub),3):
    r=sub[i]
    print(f"   {r[0]:6.2f} ax={r[5]:+7.2f} ay={r[6]:+7.2f} az={r[7]:+7.2f} wz={r[4]:+7.2f}")
