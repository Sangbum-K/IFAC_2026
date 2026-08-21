import pickle,numpy as np,collections
D=pickle.load(open("d.pkl","rb"))
L=36.833
tracks=collections.defaultdict(list)
for t,o in D["obs_frenet"]:
    for x in o: tracks[x["id"]].append((t,x["s_center"],x["d_center"],x["vs"],x["is_static"]))
# 동적(장기) 트랙만 모아 s 빈별 d 분포
S=[];Dd=[]
for i,v in tracks.items():
    a=np.array(v)
    if len(a)<80: continue
    if a[:,4].mean()>0.4: continue   # 정적 위주 제외
    S.append(a[:,1]); Dd.append(a[:,2])
S=np.concatenate(S); Dd=np.concatenate(Dd)
print(f"동적 상대 관측 {len(S)}점 ({len(set())})")
print(f"{'s':>8} {'n':>5} {'d 중앙':>7} {'d p10':>7} {'d p90':>7} {'산포(p90-p10)':>12}")
sp=[]
for lo in np.arange(0,L,2.0):
    m=(S>=lo)&(S<lo+2.0)
    if m.sum()<30: continue
    d=Dd[m]
    sp.append(np.percentile(d,90)-np.percentile(d,10))
    print(f"{lo:4.0f}~{lo+2:3.0f} {m.sum():5d} {np.median(d):+7.2f} {np.percentile(d,10):+7.2f} {np.percentile(d,90):+7.2f} {np.percentile(d,90)-np.percentile(d,10):12.2f}")
print(f"\n구간별 d 산포(p90-p10) 중앙 {np.median(sp):.2f} m — 트랙 폭 중앙 1.99 m 대비 {np.median(sp)/1.99*100:.0f}%")
# 시간 손실
fo=D["/car_state/frenet/odom"]
gw=D["global_wpnts"][0][1]
vr=np.interp(fo[:,1]%L,gw[:,0],gw[:,8])
rows=pickle.load(open("trace.pkl","rb"))
rt=np.array([r[0] for r in rows]); rd=np.array([r[5] for r in rows]); rs=np.array([r[8] for r in rows])
k=np.searchsorted(rt,fo[:,0]); k=np.clip(k,0,len(rt)-1)
near=(np.abs(rt[k]-fo[:,0])<0.2)
dynnear=near&(rs[k]==0)&(rd[k]<5.0)
print(f"\n동적 상대가 5m 안에 있던 시간 {dynnear.sum()*0.0265:.0f}s / 663s ({dynnear.mean()*100:.0f}%)")
print(f"  그 구간 실속도/레이스라인 = {np.median(fo[dynnear,4]/vr[dynnear])*100:.0f}%   (자유주행 {np.median(fo[~near,4]/vr[~near])*100:.0f}%)")
