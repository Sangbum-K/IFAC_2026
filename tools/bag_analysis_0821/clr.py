import pickle,numpy as np,collections
D=pickle.load(open("d.pkl","rb"))
L=36.833
# 장기 트랙(동적 상대)만: 관측 시계열에서 10프레임 std_d / std_s 재현
tracks=collections.defaultdict(list)
for t,o in D["obs_frenet"]:
    for x in o:
        tracks[x["id"]].append((t,x["s_center"],x["d_center"],x["vs"],x["vd"],x["is_static"],x["d_left"],x["d_right"]))
long=[i for i,v in tracks.items() if len(v)>=100]
print(f"관측 100+ 트랙 {len(long)}개")
allsd=[];allss=[];allw=[]
for i in long:
    a=np.array([(r[1],r[2],r[5],r[6],r[7]) for r in tracks[i]])
    d=a[:,1]
    for k in range(10,len(d)):
        w=d[k-10:k]
        allsd.append(w.std())
    s=a[:,0]
    for k in range(10,len(s)):
        w=np.unwrap(s[k-10:k],period=L)
        allss.append(w.std())
    allw.append(np.median(a[:,3]-a[:,4]))
allsd=np.array(allsd);allss=np.array(allss)
print(f"동적 트랙 10프레임 창 std_d: p50 {np.median(allsd):.3f} p90 {np.percentile(allsd,90):.3f} m")
print(f"                     std_s: p50 {np.median(allss):.3f} p90 {np.percentile(allss,90):.3f} m")
sig=3.0
need=np.maximum(0.15,0.15)+sig*np.maximum(allsd,0.05)
print(f"-> 요구 장애물 간격 need = 0.15 + 3σ_d : p50 {np.median(need):.2f} p90 {np.percentile(need,90):.2f} m  (evasion_dist 목표 0.35)")
print(f"   need > 0.35 인 비율 {np.mean(need>0.35)*100:.0f}%   need > 0.50 {np.mean(need>0.5)*100:.0f}%")
print(f"관측된 상대 박스 폭(d_left-d_right) 중앙 {np.median(allw):.2f} m")
# 트랙 폭
gw=D["global_wpnts"][0][1]
w=gw[:,4]+gw[:,5]
print(f"\n트랙 폭(d_r+d_l): p10 {np.percentile(w,10):.2f} p50 {np.median(w):.2f} p90 {np.percentile(w,90):.2f} m")
# 상대 폭 + 필요간격x1 + 경계여유 vs 트랙폭
print(f"필요 총폭 = 상대폭 {np.median(allw):.2f} + need {np.median(need):.2f} + 차반폭0.15 + 경계여유0.25 = {np.median(allw)+np.median(need)+0.15+0.25:.2f} m")
