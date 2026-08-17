import pickle, numpy as np
from pathlib import Path
D=pickle.load(open(Path(__file__).parent/"d124904.pkl","rb"))
fre=D["/car_state/frenet/odom"]; cmd=D["/autonomy/ackermann_cmd"]
def at(a,t,c): return np.interp(t,a[:,0],a[:,c])
print("=== 59.3~60.0 트레일링 라인의 속도/횡 프로파일 ===")
for t,W in D["otwpnts"]:
    if not (59.33<=t<=60.0) or not len(W): continue
    print(f"  t={t:5.2f} ego_s={at(fre,t,1):6.2f} v={at(fre,t,4):4.2f} cmd_v={at(cmd,t,2):4.2f} | n={len(W)} s[{W[0,0]:5.2f},{W[-1,0]:5.2f}] d[{W[:,1].min():+.2f},{W[:,1].max():+.2f}] vx[{W[:,8].min():.2f},{W[:,8].max():.2f}] vx머리={W[0,8]:.2f} vx꼬리={W[-1,8]:.2f}")
print("\n=== 같은 구간 local_waypoints (컨트롤러가 실제로 따르는 것) ===")
for t,W in D["local_wpnts"]:
    if not (59.33<=t<=60.0) or not len(W): continue
    if int(t*40)%4: continue
    print(f"  t={t:5.2f} n={len(W)} s[{W[0,0]:5.2f},{W[-1,0]:5.2f}] d머리={W[0,1]:+.2f} d@+1m={np.interp(W[0,0]+1.0,W[:,0],W[:,1]):+.2f} d@+2m={np.interp(W[0,0]+2.0,W[:,0],W[:,1]):+.2f} vx머리={W[0,8]:.2f} vx최소={W[:,8].min():.2f}")
