#!/usr/bin/env python3
"""속도 분석: 구간별 실측 vs 계획, 손실 원인 분해."""
import pickle, re, numpy as np
from collections import Counter
D=pickle.load(open("data.pkl","rb")); TR=pickle.load(open("trace.pkl","rb"))
TL=32.07
fre=D["/car_state/frenet/odom"]; t_e,s_e,d_e,v_e=fre[:,0],fre[:,1],fre[:,2],fre[:,4]
_,gb=D["global_wpnts"][0]; gs,gvx,gkap=gb[:,0],gb[:,8],gb[:,7]
VS=0.96
wrap=np.where(np.diff(s_e)<-TL*0.5)[0]; lapb=np.concatenate([[t_e[0]],t_e[wrap],[t_e[-1]]])
laptimes=np.diff(lapb)
print("=== 랩타임 ===")
for i,lt in enumerate(laptimes):
    tag=" (콘 O2 있음)" if i<=14 else " (O2 콘 없음)"
    print(f"  lap{i:2d}  {lt:6.2f}s{tag}")
A=laptimes[1:15]; B=laptimes[15:19]
print(f"\n  laps1-14(O2 콘 有) 평균 {A.mean():.2f}s  최속 {A.min():.2f}s")
print(f"  laps15-18(O2 콘 無) 평균 {B.mean():.2f}s  최속 {B.min():.2f}s   → 차이 {A.mean()-B.mean():+.2f}s")

print("\n=== 0.5m 구간별 실측/계획 속도 (전체 vs 랩15+) ===")
print("   s    v_plan  v_all  비율   v_lap15+ 비율  |kappa|")
bins=np.arange(0,TL,1.0)
for b in bins:
    m=(s_e>=b)&(s_e<b+1.0)
    if m.sum()<10: continue
    m2=m&(t_e>=lapb[15])
    vp=np.median(np.interp(s_e[m],gs,gvx))*VS
    va=np.median(v_e[m]); vb=np.median(v_e[m2]) if m2.sum()>5 else np.nan
    k=np.interp(b+0.5,gs,np.abs(gkap))
    mark=""
    for nm,so in {"O2":6.63,"O3":10.45,"O4":14.99,"O6":22.11,"O7":28.86}.items():
        if b<=so<b+1.0: mark=" <-- "+nm
    print(f" {b:5.1f} {vp:6.2f} {va:6.2f} {100*va/vp:5.0f}% {vb:8.2f} {100*vb/vp:5.0f}% {k:7.3f}{mark}")

print("\n=== 속도를 깎은 메커니즘 (로그 발동 횟수/영향) ===")
pat={"트레일링 v_target=0":r"Trailing fallback: .* v_target=0\.0",
     "트레일링 v_target>0":r"Trailing fallback: .* v_target=(?!0\.0)",
     "제어 추종 백스톱":r"추종 백스톱",
     "회피경로 상실 홀드":r"회피 경로 상실",
     "LaneSwitch 속도캡":r"LaneSwitch 속도 캡",
     "MAP 조향레이트 클립":r"MAP steering rate clipped"}
for nm,p in pat.items():
    n=sum(1 for t,l,nn,m in D["rosout"] if re.search(p,m))
    print(f"  {nm:24s} {n:5d}")
bk=[ (t,m) for t,l,n,m in D["rosout"] if "추종 백스톱" in m]
vals=[re.search(r"경로오차 ([\d.]+)m / 조향포화 ([\d.]+)% -> 속도 ([\d.]+) -> ([\d.]+)",m).groups() for _,m in bk]
V=np.array([[float(a) for a in v] for v in vals])
print(f"\n  추종 백스톱 {len(V)}회: 경로오차 p50 {np.median(V[:,0]):.2f}m, 조향포화 p50 {np.median(V[:,1]):.0f}%, "
      f"속도 {np.median(V[:,2]):.2f}->{np.median(V[:,3]):.2f} (p50 감속 {np.median(V[:,2]-V[:,3]):.2f} m/s)")
print(f"  그중 1.50 으로 바닥친 것 {np.sum(V[:,3]<=1.51)}회 ({100*np.mean(V[:,3]<=1.51):.0f}%)")
ts=np.array([t for t,_ in bk])
import collections
h=collections.Counter(np.round(np.interp(ts,t_e,s_e)).astype(int))
print("  백스톱 발생 s 분포:",sorted(h.items()))
