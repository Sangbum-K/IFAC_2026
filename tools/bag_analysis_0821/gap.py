import pickle,numpy as np,collections
D=pickle.load(open("d.pkl","rb"))
rows=pickle.load(open("trace.pkl","rb"))
L=36.833
ack=D["/ackermann_cmd"]; fo=D["/car_state/frenet/odom"]
def at(a,t,c):
    i=np.searchsorted(a[:,0],t); i=min(max(i,0),len(a)-1); return a[i,c]
# 장애물 vs (관측) 매핑
F={}
for t,o in D["obs_frenet"]:
    F[t]={x["id"]:x for x in o}
ft=np.array(sorted(F))
def obs_vs(t,i):
    k=np.searchsorted(ft,t); k=min(max(k,0),len(ft)-1)
    if abs(ft[k]-t)>0.3: return None
    return F[ft[k]].get(i)
tr=[r for r in rows if r[13]=="trailing" and r[8]==0 and r[4]>0]
print(f"동적 트레일링 사이클 {len(tr)}")
g=[];cv=[];ov=[];ev=[]
for r in tr:
    o=obs_vs(r[0],r[4])
    if o is None: continue
    g.append(r[5]); cv.append(at(ack,r[0],2)); ov.append(o["vs"]); ev.append(r[2])
g=np.array(g);cv=np.array(cv);ov=np.array(ov);ev=np.array(ev)
print(f"간격 dist: p10 {np.percentile(g,10):.2f} p50 {np.median(g):.2f} p90 {np.percentile(g,90):.2f} m")
print(f"명령속도  : p10 {np.percentile(cv,10):.2f} p50 {np.median(cv):.2f} p90 {np.percentile(cv,90):.2f} m/s")
print(f"실측속도  : p50 {np.median(ev):.2f}")
print(f"상대속도vs: p10 {np.percentile(ov,10):.2f} p50 {np.median(ov):.2f} p90 {np.percentile(ov,90):.2f} m/s")
print(f"ego 가 상대보다 느린 사이클: {(ev<ov).mean()*100:.0f}%   명령이 상대보다 느린: {(cv<ov).mean()*100:.0f}%")
# 거리 구간별
print("\n거리대별 (동적 트레일링):")
print(f"{'dist':>10} {'n':>5} {'cmd':>6} {'ego':>6} {'opp':>6}")
for lo,hi in [(-1,0.5),(0.5,1.0),(1.0,1.5),(1.5,2.0),(2.0,3.0),(3.0,4.0),(4.0,6.0),(6.0,20)]:
    m=(g>=lo)&(g<hi)
    if m.sum()<5: continue
    print(f"{lo:4.1f}~{hi:4.1f} {m.sum():5d} {np.median(cv[m]):6.2f} {np.median(ev[m]):6.2f} {np.median(ov[m]):6.2f}")
# otwpnts 진폭
print("\notwpnts 진폭(|d| 최대):")
otw=D["otwpnts"]
amp=[];tt=[]
for t,w in otw:
    if len(w)==0: continue
    amp.append(np.abs(w[:,1]).max()); tt.append(t)
amp=np.array(amp);tt=np.array(tt)
tri=set()
# 동적/정적 구간 라벨: 가장 가까운 trace
rt=np.array([r[0] for r in rows]); rs=np.array([r[8] for r in rows])
k=np.searchsorted(rt,tt); k=np.clip(k,0,len(rt)-1)
ok=np.abs(rt[k]-tt)<0.2
for lab,name in ((0,"동적"),(1,"정적")):
    m=ok&(rs[k]==lab)
    if m.sum(): print(f"  {name}: n={m.sum()} amp p50 {np.median(amp[m]):.2f} p90 {np.percentile(amp[m],90):.2f} max {amp[m].max():.2f}  / amp<0.15 인 비율 {(amp[m]<0.15).mean()*100:.0f}%")
