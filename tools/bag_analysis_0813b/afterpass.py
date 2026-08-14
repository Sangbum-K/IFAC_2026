#!/usr/bin/env python3
"""콘 옆을 스쳐 지나가는 그 순간 전후에 무슨 일이 일어나는가 (전 랩 정렬)."""
import pickle, re, numpy as np
D=pickle.load(open("data.pkl","rb")); TR=pickle.load(open("trace.pkl","rb"))
TL=32.07
fre=D["/car_state/frenet/odom"]; t_e,s_e,d_e,v_e=fre[:,0],fre[:,1],fre[:,2],fre[:,4]
ack=D["/ackermann_cmd"]; lat=D["/pitwall/lat_err"]; sp=D["/pitwall/spliced_wpnts"]
LW=D["local_wpnts"]; lw_t=np.array([t for t,_ in LW])
OT=D["otwpnts"];  ot_t=np.array([t for t,_ in OT])
pf=D["/pf/pose/odom"]
tr_t=np.array([x["t"] for x in TR])
wrap=np.where(np.diff(s_e)<-TL*0.5)[0]; lapb=np.concatenate([[t_e[0]],t_e[wrap],[t_e[-1]]])
bk=[(t,re.search(r"경로오차 ([\d.]+)m / 조향포화 ([\d.]+)% -> 속도 ([\d.]+) -> ([\d.]+)",m).groups())
    for t,l,n,m in D["rosout"] if "추종 백스톱" in m]
brk=[(t,re.search(r"reason=(\w+)",m).group(1)) for t,l,n,m in D["rosout"] if "committed_break" in m]

def local_d_at_car(t):
    i=int(np.searchsorted(lw_t,t)); i=min(max(i,0),len(LW)-1); a=LW[i][1]
    if not len(a): return np.nan
    ex=np.interp(t,pf[:,0],pf[:,1]); ey=np.interp(t,pf[:,0],pf[:,2])
    k=int(np.argmin(np.hypot(a[:,2]-ex,a[:,3]-ey)))
    return a[k,1]
def ot_head_d(t):
    j=int(np.searchsorted(ot_t,t)); j=min(max(j,0),len(OT)-1); b=OT[j][1]
    return (b[0,1] if len(b) else np.nan), len(b)

SO=28.86; NAME="O7 직선"
print(f"=== {NAME} (콘 s={SO}) — 최근접 순간 t0 기준 정렬 ===\n")
rows=[]
for li in range(1,len(lapb)-2):
    m=(t_e>=lapb[li])&(t_e<lapb[li+1])
    ss,tt=s_e[m],t_e[m]
    if not len(ss): continue
    j=int(np.argmin(np.abs(ss-SO)))
    if abs(ss[j]-SO)>0.5: continue
    t0=tt[j]
    # 이벤트 상대시각
    cb=[(t-t0,r) for t,r in brk if -1.0<=t-t0<=1.5]
    bs=[(t-t0,g) for t,g in bk if -0.2<=t-t0<=2.0]
    md=[]
    prev=None
    for x in TR:
        if not (-0.8<=x["t"]-t0<=1.5): continue
        if x["mode"]!=prev: md.append((round(x["t"]-t0,2),x["mode"])); prev=x["mode"]
    rows.append((li,t0,cb,bs,md))
    seq=" ".join(f"{a:+.2f}:{b[:10]}" for a,b in md)
    cbs=" ".join(f"{a:+.2f}:{r}" for a,r in cb)
    bss=" ".join(f"{a:+.2f}:{g[2]}→{g[3]}" for a,g in bs)
    print(f"lap{li:2d} t0={t0:7.2f}  mode: {seq}")
    if cbs: print(f"           커밋파기: {cbs}")
    if bss: print(f"           백스톱  : {bss}")
print("\n=== 최근접 순간 전후 평균 거동 (전 랩 정렬, 0.1s 간격) ===")
print("  Δt     ego_d   따라갈경로_d   |차이|   otwpnts머리_d  ot점수  실속   명령속도")
for dt in np.arange(-0.6,1.51,0.15):
    ed=[];ld=[];od=[];on=[];vv=[];cv=[]
    for li,t0,_,_,_ in rows:
        t=t0+dt
        ed.append(np.interp(t,t_e,d_e)); ld.append(local_d_at_car(t))
        a,b=ot_head_d(t); od.append(a); on.append(b)
        vv.append(np.interp(t,t_e,v_e)); cv.append(np.interp(t,ack[:,0],ack[:,2]))
    ed=np.array(ed);ld=np.array(ld)
    print(f" {dt:+5.2f} {np.nanmedian(ed):+8.2f} {np.nanmedian(ld):+11.2f} "
          f"{np.nanmedian(np.abs(ed-ld)):8.2f} {np.nanmedian(od):+12.2f} {np.median(on):6.0f} "
          f"{np.median(vv):6.2f} {np.median(cv):8.2f}")
