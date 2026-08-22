#!/usr/bin/env python3
"""P2 재검증: '지나침 + 횡 여유 확보' 필터가 저속 트레일링 사이클을 얼마나 방출하나"""
import pickle,re,numpy as np
D=pickle.load(open("d17.pkl","rb")); L=pickle.load(open("log17.pkl","rb"))
gw=D["global_wpnts"][0][1]; TL=gw[-1,0]
fre=D["/car_state/frenet/odom"]; ack=D["/ackermann_cmd"]
NEED=0.25   # RequiredObstacleClearance: max(path_check_margin .20, 반폭 .15) + sigma(0) + 0
def near(a,t): return a[np.argmin(np.abs(a[:,0]-t))]
GATE=[(122.59,123.32),(168.89,169.99)]
def ingate(t): return any(a<=t<=b for a,b in GATE)
rel=0;keep=0;relc=[];keepc=[]
for t,o in D["obs_frenet"]:
    if not(4<=t<=170) or ingate(t) or not o: continue
    f=near(fre,t); s,d=f[1],f[2]
    stat=[ob for ob in o if ob["is_static"]]
    if not stat: continue
    cand=[]
    for ob in stat:
        ahead=(ob["s_start"]-s)%TL
        behind=ahead>TL/2
        lat_clear=(d>=ob["d_left"]+NEED) or (d<=ob["d_right"]-NEED)
        cand.append((ahead if not behind else ahead-TL,behind and lat_clear))
    # closest = s_end 기준 최근접 (코드와 동일)
    order=sorted(range(len(stat)),key=lambda i:(stat[i]["s_end"]-s)%TL)
    ci=order[0]
    c=near(ack,t)[2]
    if cand[ci][1]: rel+=1; relc.append(c)
    else: keep+=1; keepc.append(c)
print("정적 closest 가 있는 사이클 %d"%(rel+keep))
print("  방출(지나침+횡여유) %d (%.1f%%)  그때 명령속도 p50 %.2f | <1.5 %.0f%%"%(
    rel,100*rel/(rel+keep),np.median(relc) if relc else -1,
    100*np.mean(np.array(relc)<1.5) if relc else -1))
print("  유지                %d (%.1f%%)  그때 명령속도 p50 %.2f | <1.5 %.0f%%"%(
    keep,100*keep/(rel+keep),np.median(keepc),100*np.mean(np.array(keepc)<1.5)))
