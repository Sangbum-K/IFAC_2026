import pickle,numpy as np,collections
D=pickle.load(open("d.pkl","rb"))
L=36.833
tracks=collections.defaultdict(list)
for t,o in D["obs_frenet"]:
    for x in o: tracks[x["id"]].append((t,x["s_center"],x["d_center"],x["vs"],x["is_static"]))
# 확실히 정지한 물체 = 전체 관측에서 s 이동폭 < 0.25 m 이고 관측 40+ (= 콘)
still=[]
for i,v in tracks.items():
    a=np.array(v)
    if len(a)<40: continue
    s=np.unwrap(a[:,1],period=L)
    if s.max()-s.min()<0.25: still.append((i,a))
print(f"확실히 정지한 트랙(s 이동폭 <0.25 m, 관측 40+): {len(still)}개")
allm=[]
for i,a in still:
    vs=np.abs(a[:,3])
    m=np.array([vs[max(0,k-9):k+1].mean() for k in range(9,len(vs))])
    allm.append(m)
m=np.concatenate(allm)
print(f"  10프레임 |vs| 이동평균: p10 {np.percentile(m,10):.3f} p50 {np.median(m):.3f} p90 {np.percentile(m,90):.3f} max {m.max():.3f} m/s")
print(f"  cls_vs_reset(0.10) 미만인 프레임 비율: {np.mean(m<0.10)*100:.1f}%")
print(f"  10프레임 **연속**(cls_vs_hold) 으로 0.10 미만인 구간을 만든 트랙:")
n_ok=0
for i,a in still:
    vs=np.abs(a[:,3])
    mm=np.array([vs[max(0,k-9):k+1].mean() for k in range(9,len(vs))])
    lo=mm<0.10
    run=0;best=0
    for x in lo:
        run=run+1 if x else 0; best=max(best,run)
    if best>=10: n_ok+=1
print(f"    {n_ok}/{len(still)}개 ({n_ok/len(still)*100:.0f}%)")
for thr in (0.15,0.20,0.25,0.30):
    n=0
    for i,a in still:
        vs=np.abs(a[:,3])
        mm=np.array([vs[max(0,k-9):k+1].mean() for k in range(9,len(vs))])
        lo=mm<thr; run=0;best=0
        for x in lo:
            run=run+1 if x else 0; best=max(best,run)
        if best>=10: n+=1
    print(f"    문턱 {thr:.2f} 이면 {n}/{len(still)} ({n/len(still)*100:.0f}%)")

# by_vote 지연: 장수 동적 트랙이 멈췄다고 가정
print("\nby_vote 경로: static_count >= (total_count+1)/2")
print("  장수 동적 트랙은 total_count 가 이미 크고 static_count=0 이다.")
for n0 in (100,300,500,700):
    k=n0+1   # s = (n0+s+1)/2 -> s = n0+1
    print(f"    이미 {n0} 프레임 동적이었다면 뒤집기까지 {k} 프레임 더 = {k/30.0:.1f}s @30Hz")
# 실제 장수 트랙의 관측 수
lens=[len(v) for v in tracks.values() if len(v)>=100]
print(f"  이 백의 장수 트랙 관측 수: p50 {np.median(lens):.0f} max {max(lens)} 프레임")
