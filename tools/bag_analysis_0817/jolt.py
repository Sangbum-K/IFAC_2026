import pickle, numpy as np
from pathlib import Path
D=pickle.load(open(Path(__file__).parent/"d124904.pkl","rb"))
imu=D["/sensors/imu/raw"]; fre=D["/car_state/frenet/odom"]; od=D["/odom"]; cmd=D["/autonomy/ackermann_cmd"]
m=(imu[:,0]>=59.5)&(imu[:,0]<=60.2)
print(" t      ax(g)   ay(g)   az(g)   wz(deg/s)")
for r in imu[m]: print(f" {r[0]:6.3f} {r[5]:+7.3f} {r[6]:+7.3f} {r[7]:+7.3f} {r[4]:+9.1f}")
print("\n--- 전 구간 |ax| 상위 12 (감속 피크) ---")
i=np.argsort(-np.abs(imu[:,5]))[:40]
seen=[]
for k in sorted(i, key=lambda k: imu[k,0]):
    t=imu[k,0]
    if any(abs(t-p)<0.5 for p in seen): continue
    seen.append(t)
    print(f"  t={t:6.2f} ax={imu[k,5]:+6.2f}g ay={imu[k,6]:+6.2f}g wz={imu[k,4]:+7.1f}  v={np.interp(t,fre[:,0],fre[:,4]):.2f} cmd_v={np.interp(t,cmd[:,0],cmd[:,2]):.2f} s={np.interp(t,fre[:,0],fre[:,1]):.1f}")
print("\n--- /odom vs pf 속도 (59.5-60.2) ---")
for tt in np.arange(59.5,60.21,0.05):
    print(f"  {tt:5.2f} odom_v={np.interp(tt,od[:,0],od[:,4]):5.2f} pf_v={np.interp(tt,fre[:,0],fre[:,4]):5.2f}")
