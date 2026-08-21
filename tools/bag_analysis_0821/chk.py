import pickle,re,numpy as np,collections
R=pickle.load(open("rosout2.pkl","rb"))
# 트랙별 분류 전이 시퀀스
seq=collections.defaultdict(list)
for t,l,n,m in R:
    x=re.search(r"obstacle (\d+) classified (STATIC|DYNAMIC)",m)
    if x: seq[int(x.group(1))].append((t,x.group(2)))
born_static=[i for i,v in seq.items() if v[0][1]=="STATIC"]
became_dyn=[i for i in born_static if any(y[1]=="DYNAMIC" for y in seq[i])]
lat=[]
for i in became_dyn:
    t0=seq[i][0][0]
    t1=[y[0] for y in seq[i] if y[1]=="DYNAMIC"][0]
    lat.append(t1-t0)
print(f"분류 로그가 있는 트랙 {len(seq)}개")
print(f"  첫 분류가 STATIC: {len(born_static)}개 ({len(born_static)/len(seq)*100:.0f}%)")
print(f"  그중 나중에 DYNAMIC 이 된 것(=진짜 상대차): {len(became_dyn)}개")
print(f"  STATIC->DYNAMIC 지연: p50 {np.median(lat):.2f}s p90 {np.percentile(lat,90):.2f}s max {max(lat):.2f}s")
print(f"  첫 분류가 DYNAMIC: {len(seq)-len(born_static)}개")
# 트레일링 제동 포락선
print("\n트레일링 제동이 요구하는 최소 여유거리 (decel_dyn=1.5, lag=0.3, gap=1.5):")
for v in (3,4,5,6):
    for vt in (1.0,1.5,2.0):
        req=(v*v-vt*vt)/(2*1.5)+v*0.3+1.5
        print(f"  v={v} 상대={vt}: {req:5.2f} m", end="")
    print()
print("  실측 최초 탐지거리 중앙 4.1 m (코너 2.9 / 직선 7.9)")
