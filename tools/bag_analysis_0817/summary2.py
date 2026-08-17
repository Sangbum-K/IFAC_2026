import pickle, numpy as np
from pathlib import Path
D=pickle.load(open(Path(__file__).parent/"d124904.pkl","rb"))
fre=D["/car_state/frenet/odom"]
LAPS=[(14.5,18.5,"lap0"),(25.5,29.6,"lap1"),(37.0,41.0,"lap2"),(47.5,51.0,"lap3"),(57.5,60.5,"lap4 충돌")]
print("장애물 A(s 33.2~33.9) 를 겨냥한 회피선이 처음 나온 시점")
print("  랩        t      ego_s  ego_d  v     남은L  A     필요a_lat  이후 유지시간")
for a,b,lab in LAPS:
    hits=[]
    for tt,W in D["otwpnts"]:
        if not (a<=tt<=b) or len(W)<5: continue
        sel=W[(W[:,0]>=33.0)&(W[:,0]<=34.0)]
        if len(sel) and sel[:,1].mean()>0.35: hits.append(tt)
    if not hits: print(f"  {lab:10s} 없음"); continue
    t0=hits[0]
    # 연속 유지 길이
    hold=0.0
    for i in range(1,len(hits)):
        if hits[i]-hits[i-1]>0.12: break
        hold=hits[i]-t0
    es=np.interp(t0,fre[:,0],fre[:,1]); ed=np.interp(t0,fre[:,0],fre[:,2]); v=np.interp(t0,fre[:,0],fre[:,4])
    L=33.2-es; A=0.58-ed
    print(f"  {lab:10s} {t0:6.2f} {es:6.2f} {ed:+5.2f} {v:4.2f}  {L:5.2f} {A:5.2f}  {6*A*v*v/max(L,.01)**2:6.1f}   {hold:4.2f}s ({len(hits)}프레임)")
