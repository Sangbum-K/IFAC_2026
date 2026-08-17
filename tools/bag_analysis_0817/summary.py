import pickle, numpy as np
from pathlib import Path
D=pickle.load(open(Path(__file__).parent/"d124904.pkl","rb"))
fre=D["/car_state/frenet/odom"]
def vat_s(a,b,s0):
    m=(fre[:,0]>=a)&(fre[:,0]<=b)
    sub=fre[m]; i=np.argmin(np.abs(sub[:,1]-s0))
    return sub[i,4], sub[i,2], sub[i,0]
LAPS=[(14.5,18.5,"lap0"),(25.5,29.6,"lap1"),(37.0,41.0,"lap2"),(47.5,51.0,"lap3"),(57.5,60.5,"lap4 충돌")]
print("장애물 A(s_start≈33.2) 진입 조건")
print("  랩         s=30.5 통과 v   d      | 회피선 최초 채택 t   그때 ego_s  v     남은거리 L  필요 A   6·A·v²/L²")
for a,b,lab in LAPS:
    v,d,t=vat_s(a,b,30.5)
    # 회피선 최초 채택: d_max>0.3인 첫 otwpnts
    first=None
    for tt,W in D["otwpnts"]:
        if not (a<=tt<=b) or not len(W): continue
        if W[:,1].max()>0.35:
            es=np.interp(tt,fre[:,0],fre[:,1])
            if es<34.0: first=(tt,es,np.interp(tt,fre[:,0],fre[:,4]),np.interp(tt,fre[:,0],fre[:,2]),W[:,1].max()); break
    if first:
        tt,es,vv,dd,dmax=first; L=33.2-es; A=dmax-dd
        alat=6*A*vv*vv/max(L,0.01)**2
        print(f"  {lab:10s} {v:4.2f} m/s d={d:+5.2f} | t={tt:6.2f} s={es:5.2f} v={vv:4.2f} L={L:4.2f}m A={A:4.2f}m  a_lat필요={alat:5.1f} m/s²")
    else:
        print(f"  {lab:10s} {v:4.2f} m/s d={d:+5.2f} | 회피선 없음")
print("\n제동만으로 가능한가 (A_DEC=1.75 m/s²)")
for a,b,lab in LAPS:
    v,d,t=vat_s(a,b,29.6)
    print(f"  {lab:10s} s=29.6에서 v={v:4.2f} -> 정지거리 {v*v/(2*1.75):4.2f} m (장애물까지 3.6m)")
