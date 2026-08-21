import pickle,numpy as np,collections
D=pickle.load(open("d.pkl","rb"))
L=36.833
# 1) 상대의 실제 감속 능력 (관측 vs 시계열)
tracks=collections.defaultdict(list)
for t,o in D["obs_frenet"]:
    for x in o: tracks[x["id"]].append((t,x["s_center"],x["d_center"],x["vs"],x["is_static"]))
dec=[]
for i,v in tracks.items():
    a=np.array(v)
    if len(a)<60 or a[:,4].mean()>0.4: continue
    t=a[:,0]; vs=a[:,3]
    for k in range(len(t)):
        j=np.searchsorted(t,t[k]+0.5)
        if j>=len(t): break
        dt=t[j]-t[k]
        if dt<0.4: continue
        dec.append((vs[k]-vs[j])/dt)
dec=np.array(dec)
print(f"상대 종방향 감속도(0.5s 창, {len(dec)}표본): p50 {np.median(dec):+.2f} p90 {np.percentile(dec,90):+.2f} p99 {np.percentile(dec,99):+.2f} max {dec.max():+.2f} m/s^2")

# 2) 자차 실제 감속 능력 (명령 하강 후 PF 속도)
pf=D["/pf/pose/odom"]; ack=D["/ackermann_cmd"]
tv=pf[:,0]; v=pf[:,4]
ed=[]
for i in range(len(tv)):
    j=np.searchsorted(tv,tv[i]+0.5)
    if j>=len(tv): break
    if v[i]<1.0: continue
    ed.append((v[i]-v[j])/(tv[j]-tv[i]))
ed=np.array(ed)
print(f"자차 종방향 감속도(0.5s 창): p90 {np.percentile(ed,90):.2f} p99 {np.percentile(ed,99):.2f} max {ed.max():.2f} m/s^2")

# 3) 트레일링 중 간격 유지 정확도
rows=pickle.load(open("trace.pkl","rb"))
tr=[r for r in rows if r[13]=="trailing" and r[8]==0]
seg=[];cur=[]
for r in tr:
    if cur and r[0]-cur[-1][0]>0.2: seg.append(cur);cur=[]
    cur.append(r)
seg.append(cur)
rates=[]
for s in seg:
    if len(s)<15: continue
    d=np.array([r[5] for r in s]); t=np.array([r[0] for r in s])
    rates.append(np.abs(np.diff(d))/np.maximum(np.diff(t),1e-3))
r=np.concatenate(rates)
print(f"\n트레일링 중 dist 변화율(=상대 접근속도) p50 {np.median(r):.2f} p90 {np.percentile(r,90):.2f} m/s")
print(f"  |상대속도| > 0.5 m/s 인 사이클 {np.mean(r>0.5)*100:.0f}%,  > 1.0 {np.mean(r>1.0)*100:.0f}%")

# 4) 0.30 m 꽁지 붙기의 여유: 상대가 p90/p99 로 밟을 때 접촉까지
print("\n꽁지 0.30 m 에서 상대 급제동 시 (자차는 지연 0.3s 뒤 동일 감속 가정):")
for ad in (np.percentile(dec,90), np.percentile(dec,99), dec.max()):
    # 자차 감속 능력 상한
    ae=np.percentile(ed,90)
    # 상대 감속 ad, 자차는 0.3s 지연 후 ae. 상대속도 0 시작
    dt=0.01; g=0.30; dv=0.0; t=0.0; hit=None
    while t<3.0:
        ao = ad
        ai = ae if t>0.3 else 0.0
        dv += (ao-ai)*dt      # 접근 상대속도 증가
        g -= dv*dt
        t+=dt
        if g<=0: hit=(t,dv); break
        if dv<=0: break
    print(f"  상대 감속 {ad:4.1f} m/s^2 -> " + (f"{hit[0]:.2f}s 뒤 접촉, 충돌 상대속도 {hit[1]:.2f} m/s" if hit else "접촉 없음"))
