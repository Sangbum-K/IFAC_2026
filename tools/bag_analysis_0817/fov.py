import pickle, numpy as np
from pathlib import Path
D=pickle.load(open(Path(__file__).parent/"d124904.pkl","rb"))
pf=D["/pf/pose/odom"]; gw=D["global_wpnts"][0][1]
# 장애물 A 중심 (map): s≈33.4 d≈-0.05 -> 좌표
i=np.argmin(np.abs(gw[:,0]-33.4)); psi=gw[i,6]
OX=gw[i,2]+(-np.sin(psi))*(-0.05); OY=gw[i,3]+np.cos(psi)*(-0.05)
print(f"장애물 A 중심 map=({OX:.2f},{OY:.2f})")
print("   t     ego_s   거리   방위각(deg, 0=정면)  |  헤어핀 요레이트")
imu=D["/sensors/imu/raw"]; fre=D["/car_state/frenet/odom"]
for tt in np.arange(57.4,59.4,0.1):
    px=np.interp(tt,pf[:,0],pf[:,1]); py=np.interp(tt,pf[:,0],pf[:,2])
    yaw=np.arctan2(np.interp(tt,pf[:,0],np.sin(pf[:,3])),np.interp(tt,pf[:,0],np.cos(pf[:,3])))
    dx,dy=OX-px,OY-py; rng=np.hypot(dx,dy)
    br=np.degrees(np.arctan2(dy,dx)-yaw); br=(br+180)%360-180
    print(f"  {tt:5.2f} {np.interp(tt,fre[:,0],fre[:,1]):6.2f} {rng:6.2f} {br:+8.1f}{'   <-- FOV(±80) 밖' if abs(br)>80 else ''}   {np.interp(tt,imu[:,0],imu[:,4]):+7.1f}")
