import pickle,numpy as np,collections
D=pickle.load(open("d.pkl","rb"))
L=36.833
fo=D["/car_state/frenet/odom"]
def ego(t):
    i=np.searchsorted(fo[:,0],t); i=min(max(i,0),len(fo)-1); return fo[i,1],fo[i,2],fo[i,4]
tracks=collections.defaultdict(list)
for t,o in D["obs_frenet"]:
    for x in o:
        s,d,v=ego(t)
        ds=(x["s_center"]-s)%L
        if ds>L/2: ds-=L
        tracks[x["id"]].append((t,ds,x["d_center"],d,x["d_right"],x["d_left"],x["vs"],v,x["is_static"]))
ev=[]
for i,v in tracks.items():
    a=np.array(v)
    if len(a)<20: continue
    dynfrac=1-a[:,8].mean()
    # 종방향 중첩 순간
    m=np.abs(a[:,1])<0.45
    if m.sum()<2: continue
    lat=np.abs(a[m,2]-a[m,3])
    # 차체 여유 = 횡거리 - (상대 반폭 + 차 반폭 0.15)
    halfw=(a[m,5]-a[m,4])/2.0
    slack=lat-halfw-0.15
    ev.append((i,a[m,0].min(),dynfrac,lat.min(),slack.min(),a[:,7].max()))
ev.sort(key=lambda x:x[1])
print("종방향으로 나란히 선(|Δs|<0.45m) 순간의 실제 횡 여유")
print(f"{'id':>6} {'t':>7} {'dyn%':>5} {'횡거리':>7} {'차체여유':>8}")
for e in ev: print(f"{e[0]:6d} {e[1]:7.1f} {e[2]*100:5.0f} {e[3]:7.2f} {e[4]:8.2f}")
a=np.array([(e[2],e[4]) for e in ev])
dy=a[a[:,0]>0.5][:,1]; st=a[a[:,0]<=0.5][:,1]
print(f"\n동적({len(dy)}회): 차체여유 p50 {np.median(dy):.2f} min {dy.min():.2f} / 0 이하(접촉) {int((dy<=0).sum())}회")
if len(st): print(f"정적({len(st)}회): 차체여유 p50 {np.median(st):.2f} min {st.min():.2f} / 0 이하 {int((st<=0).sum())}회")
