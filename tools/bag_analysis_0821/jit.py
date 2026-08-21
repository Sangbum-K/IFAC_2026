import pickle,numpy as np,collections
D=pickle.load(open("d.pkl","rb"))
L=36.833
fo=D["/car_state/frenet/odom"]
def ego(t):
    i=np.searchsorted(fo[:,0],t); i=min(max(i,0),len(fo)-1); return fo[i,1],fo[i,4]
tracks=collections.defaultdict(list)
for t,o in D["obs_frenet"]:
    for x in o:
        s,v=ego(t)
        gap=(x["s_start"]-s)%L
        if gap>L/2: gap-=L
        tracks[x["id"]].append((t,gap,x["s_start"],x["s_end"],x["is_static"],x["size"],v))
res=[];wid=[]
for i,vv in tracks.items():
    a=np.array(vv)
    if len(a)<60 or a[:,4].mean()>0.4: continue
    t=a[:,0]; g=a[:,1]
    # 0.5s 이동중앙값으로 추세 제거 -> 잔차 = 측정 지터
    tr=np.array([np.median(g[max(0,k-7):k+8]) for k in range(len(g))])
    res.append(g-tr)
    wid.append(((a[:,3]-a[:,2])%L))
res=np.concatenate(res)
print(f"동적 상대 '간격(dist)' 측정 지터 (0.5s 이동중앙값 대비 잔차, {len(res)}표본):")
print(f"  std {res.std():.3f} m,  |잔차| p50 {np.median(np.abs(res)):.3f} p90 {np.percentile(np.abs(res),90):.3f} max {np.abs(res).max():.2f} m")
w=np.concatenate(wid)
print(f"관측 클러스터 종방향 길이(s_end-s_start): p10 {np.percentile(w,10):.2f} p50 {np.median(w):.2f} p90 {np.percentile(w,90):.2f} m  (실제 RC 차 길이 ~0.35 m)")
# 프레임간 s_start 점프
jmp=[]
for i,vv in tracks.items():
    a=np.array(vv)
    if len(a)<60 or a[:,4].mean()>0.4: continue
    ds=np.diff(np.unwrap(a[:,2],period=L)); dt=np.diff(a[:,0])
    m=dt>0
    jmp.append(np.abs(ds[m]))
jmp=np.concatenate(jmp)
print(f"프레임간 s_start 점프: p50 {np.median(jmp):.3f} p90 {np.percentile(jmp,90):.3f} p99 {np.percentile(jmp,99):.3f} m")
