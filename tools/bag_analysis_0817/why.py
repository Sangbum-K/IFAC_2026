import pickle, numpy as np
from pathlib import Path
D=pickle.load(open(Path(__file__).parent/"d124904.pkl","rb"))
fre=D["/car_state/frenet/odom"]; cmd=D["/autonomy/ackermann_cmd"]
def at(a,t,c): return np.interp(t,a[:,0],a[:,c])
S_OBS=33.20; D_TGT=0.58; BUDGET=8.83
print("사이클별: 남은거리 L, 실제속도 v, 회피선 머리속도, 명령속도, 필요 a_lat")
print("   t     ego_s  ego_d   v    회피선머리v  명령v  | L     A     필요a_lat(실v)  필요a_lat(계획v)  발행내용")
for t,W in D["otwpnts"]:
    if not (59.15<=t<=59.55): continue
    es=at(fre,t,1); ed=at(fre,t,2); v=at(fre,t,4); cv=at(cmd,t,2)
    L=S_OBS-es; A=D_TGT-ed
    head=W[0,8] if len(W) else float("nan")
    a_real=6*A*v*v/max(L,0.01)**2
    a_plan=6*A*head*head/max(L,0.01)**2 if len(W) else float("nan")
    if len(W):
        kind = "회피선(+%.2f)"%W[:,1].max() if W[:,1].max()>0.35 else "횡유지(d=%+.2f)"%W[:,1].mean()
    else: kind="빈 경로"
    print(f"  {t:5.2f} {es:6.2f} {ed:+5.2f} {v:5.2f}   {head:5.2f}    {cv:5.2f} | {L:4.2f} {A:5.2f}   {a_real:6.1f}          {a_plan:6.1f}      {kind}")
print(f"\n예산 blend_lat_acc_max = {BUDGET}")
print("\n4.80 m/s 에서 회피선 머리속도(2.2)까지 줄이려면 필요한 감속:")
for L in (2.6,2.2,2.0):
    print(f"  {L:.1f} m 안에 4.80->2.23 : {(4.80**2-2.23**2)/(2*L):.2f} m/s²   (차 실측 제동 ~1.75, trailing 설정 1.0)")
