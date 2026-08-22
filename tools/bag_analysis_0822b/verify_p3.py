#!/usr/bin/env python3
"""P3 재검증: max_disp 적산을 gap>1.0 m 프레임으로 제한했을 때 ever_moved 발동 변화"""
import pickle,numpy as np
D=pickle.load(open("d17.pkl","rb"))
gw=D["global_wpnts"][0][1]; TL=gw[-1,0]; fre=D["/car_state/frenet/odom"]
def egos(t): return fre[np.argmin(np.abs(fre[:,0]-t)),1]
GAP=1.0; THR=0.5
tr={}
for t,o in D["obs_frenet"]:
    if not(4<=t<=170): continue
    for ob in o: tr.setdefault(ob["id"],[]).append((t,ob["s_center"],ob["s_start"]))
old={};new={}
for i,v in tr.items():
    ref=v[0][1]
    mo=mn=0.0
    for t,sc,ss in v:
        disp=abs((((sc-ref+TL/2)%TL)-TL/2))
        mo=max(mo,disp)
        gap=(ss-egos(t))%TL
        if GAP<gap<TL/2: mn=max(mn,disp)
    old[i]=mo; new[i]=mn
O=[i for i in old if old[i]>=THR]; N=[i for i in new if new[i]>=THR]
print("트랙 %d개 | ever_moved 발동: 종전 %d개 %s -> 신규 %d개 %s"%(
    len(old),len(O),sorted(O),len(N),sorted(N)))
print("\n종전 발동 트랙의 재계산:")
for i in sorted(O):
    print("  id=%-4d  max_disp 전체 %.2f -> gap>%.1fm 만 %.2f  %s"%(
        i,old[i],GAP,new[i],"여전히 발동" if new[i]>=THR else "**해제**"))
a=np.array(list(new.values()))
print("\n신규 max_disp 분포: p50 %.2f p90 %.2f max %.2f (문턱 %.1f 까지 여유 %.2f)"%(
    np.median(a),np.percentile(a,90),a.max(),THR,THR-a.max()))
