import pickle,numpy as np,collections
D=pickle.load(open("d.pkl","rb"))
L=36.833
gw=D["global_wpnts"][0][1]     # s d x y d_r d_l psi kappa vx ax
gs=gw[:,0]; dr=gw[:,4]; dl=gw[:,5]
fo=D["/car_state/frenet/odom"]
def ego(t):
    i=np.searchsorted(fo[:,0],t); i=min(max(i,0),len(fo)-1); return fo[i,1],fo[i,4]
NEED=0.30+0.25    # RequiredObstacleClearance p50 + 경계여유
hits=[]
for t,o in D["obs_frenet"]:
    s,v=ego(t)
    ah=[]
    for x in o:
        ds=(x["s_start"]-s)%L
        if ds>L/2: ds-=L
        if -0.5<ds<8.0: ah.append((ds,x))
    if len(ah)<2: continue
    ah.sort()
    for i in range(len(ah)):
        for j in range(i+1,len(ah)):
            a=ah[i][1]; b=ah[j][1]
            if abs(ah[j][0]-ah[i][0])>1.2: continue
            # 정적 + 동적 조합만
            if a["is_static"]==b["is_static"]: continue
            sc=(a["s_center"]+b["s_center"])/2 % L
            wl=np.interp(sc,gs,dl); wr=np.interp(sc,gs,dr)
            dleft=max(a["d_left"],b["d_left"]); dright=min(a["d_right"],b["d_right"])
            lg=wl-dleft; rg=wr+dright
            if lg<NEED and rg<NEED:
                hits.append((t,ah[i][0],a["id"],b["id"],lg,rg,v,
                             a["vs"] if not a["is_static"] else b["vs"]))
            break
        break
print(f"정적+동적이 1.2 m 안에 나란히 있고 양쪽 다 통과 불가(<{NEED:.2f} m)인 프레임: {len(hits)}")
if hits:
    h=np.array([(x[0],x[1],x[4],x[5],x[6],x[7]) for x in hits])
    # 시간 군집
    grp=[[h[0]]]
    for x in h[1:]:
        if x[0]-grp[-1][-1][0]>0.5: grp.append([])
        grp[-1].append(x)
    print(f"  시간 군집 {len(grp)}개:")
    for g in grp:
        g=np.array(g)
        print(f"    t={g[0,0]:7.2f}~{g[-1,0]:7.2f} ({g[-1,0]-g[0,0]:.2f}s) 거리 {g[:,1].min():.2f}~{g[0,1]:.2f} m, "
              f"좌여유 {np.median(g[:,2]):+.2f} 우여유 {np.median(g[:,3]):+.2f}, 자차 {np.median(g[:,4]):.2f} m/s, 동적 vs {np.median(g[:,5]):+.2f}")
