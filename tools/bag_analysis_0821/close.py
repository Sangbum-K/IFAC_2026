import pickle,numpy as np,collections
D=pickle.load(open("d.pkl","rb"))
rows=pickle.load(open("trace.pkl","rb"))
L=36.833
F={}
for t,o in D["obs_frenet"]:
    F[t]={x["id"]:x for x in o}
ft=np.array(sorted(F))
def obs_at(t,i):
    k=np.searchsorted(ft,t); k=min(max(k,0),len(ft)-1)
    return F[ft[k]].get(i) if abs(ft[k]-t)<0.3 else None
QS=0.6; HALF=0.275; A_LAT=8.83
n=0; below=0; unlock=[]
for r in rows:
    if r[8]!=0 or r[4]<=0: continue          # 동적만
    o=obs_at(r[0],r[4])
    if o is None: continue
    v=r[2]; vo=o["vs"]; closing=v-vo
    n+=1
    if closing<=QS: below+=1
    raw=max(r[5]-HALF,0.0)
    cur = raw if closing<=QS else min(raw*v/closing, max(12.0,raw))
    fix = min(raw*v/max(closing,QS), max(12.0,raw)) if v>0 else raw
    # 횡 0.45 m 이동에 필요한 활주로
    need=v*np.sqrt(6*0.45/A_LAT)
    unlock.append((cur>=need, fix>=need, closing, raw, v))
u=np.array(unlock,dtype=float)
print(f"동적 사이클 {n}개 중 접근속도 <= {QS} m/s 인 것: {below} ({below/n*100:.0f}%)")
print(f"  = 그 사이클에서 entry_avail 보정이 통째로 꺼진다")
print(f"\n횡 0.45 m 이동에 필요한 활주로 v*sqrt(6A/a_lat) 대비:")
print(f"  현재 코드로 충족: {u[:,0].mean()*100:.0f}%")
print(f"  closing 을 max(closing,{QS}) 로 바닥만 깔면: {u[:,1].mean()*100:.0f}%")
m=u[:,2]<=QS
print(f"\n접근속도<={QS} 인 {int(m.sum())}사이클만 보면: 현재 {u[m,0].mean()*100:.0f}% -> 수정 {u[m,1].mean()*100:.0f}%")
print(f"  그 구간 중앙: 간격 {np.median(u[m,3])+HALF:.2f} m, 자차 {np.median(u[m,4]):.2f} m/s, 접근속도 {np.median(u[m,2]):+.2f} m/s")
