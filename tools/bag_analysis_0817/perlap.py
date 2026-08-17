import pickle, numpy as np
from pathlib import Path
D=pickle.load(open(Path(__file__).parent/"d124904.pkl","rb"))
fre=D["/car_state/frenet/odom"]
def at(t,c): return np.interp(t,fre[:,0],fre[:,c])
LAPS=[(0,18.3,"lap0"),(18.3,29.6,"lap1"),(29.6,40.6,"lap2"),(40.6,51.0,"lap3"),(51.0,62.0,"lap4(충돌)")]
print("장애물 A(s≈33.2~33.9) 랩별 최초 발행")
for a,b,lab in LAPS:
    first=None
    for t,obs in D["obs_frenet"]:
        if not (a<=t<=b): continue
        for o in obs:
            if 32.9<=o["s_center"]<=34.2 and o["s_end"]-o["s_start"]>0.2:
                es=at(t,1); gap=(o["s_start"]-es)%37.51
                if gap>18: continue
                if first is None: first=(t,o,es,gap)
        if first: break
    if first:
        t,o,es,gap=first
        print(f"  {lab:11s} t={t:6.2f} ego_s={es:6.2f} d={at(t,2):+5.2f} v={at(t,4):4.2f} | 최초발행 거리 {gap:4.2f}m  obs d[{o['d_right']:+.2f},{o['d_left']:+.2f}] static={int(o['is_static'])}")
    else: print(f"  {lab:11s} 없음")
print("\n각 랩 s=31~34 구간 자차 d 궤적")
for a,b,lab in LAPS:
    m=(fre[:,0]>=a)&(fre[:,0]<=b)&(fre[:,1]>=30.5)&(fre[:,1]<=34.5)
    sub=fre[m]
    if not len(sub): continue
    pts=" ".join(f"s{r[1]:.1f}:d{r[2]:+.2f}" for r in sub[::6])
    print(f"  {lab:11s} {pts}")
print("\n각 랩 s=31~34 통과 시 otwpnts(회피선) d 목표")
ot=D["otwpnts"]
for a,b,lab in LAPS:
    out=[]
    for t,W in ot:
        if not (a<=t<=b) or not len(W): continue
        es=at(t,1)
        if not (30.5<=es<=34.0): continue
        sel=W[(W[:,0]>=33.0)&(W[:,0]<=34.0)]
        if len(sel): out.append((t,es,sel[:,1].mean()))
    if out:
        print(f"  {lab:11s} " + " ".join(f"t{t:.1f}(s{es:.1f}) d*={d:+.2f}" for t,es,d in out[::5]))
    else:
        print(f"  {lab:11s} (s33~34 구간을 덮는 회피선 없음)")
