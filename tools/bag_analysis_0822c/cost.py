import lib2, numpy as np
d=lib2.D(); g=lib2.gw(d); S=g[:,0]; TL=S[-1]+(S[1]-S[0])
def mod(x): return x%TL
obs_t=np.array([t for t,_ in d['obs_frenet']])
def obs_at(t):
    i=int(np.clip(np.searchsorted(obs_t,t)-1,0,len(obs_t)-1)); return d['obs_frenet'][i][1]
def inside(sv,a): return mod(sv-a['s_start'])<=mod(a['s_end']-a['s_start'])+1e-9
fr=d['/car_state/frenet/odom']
def vel(t):
    i=np.argmin(np.abs(fr[:,0]-t)); return fr[i,4]
CLR=0.32
print("=== 0.64 m 이상 사이통로를 실제로 지나던 프레임 (A안이 잃는 것) ===")
rows=[]
for t,w in d['local_wpnts']:
    if len(w)==0: continue
    obs=obs_at(t); best=None
    for row in w[:60]:
        sv,dv=row[0],row[1]
        above=[];below=[]
        for a in obs:
            if not np.isfinite(a['s_start']) or not inside(sv,a): continue
            if a['d_right']<=dv<=a['d_left']: above=below=[]; break
            if a['d_right']>dv: above.append((a['d_right'],a['id']))
            else: below.append((a['d_left'],a['id']))
        if above and below:
            corr=min(above)[0]-max(below)[0]
            if best is None or corr<best[0]: best=(corr,sv,min(above)[1],max(below)[1])
    if best and best[0]>=2*CLR:
        rows.append((t,)+best)
for t,c,sv,i1,i2 in rows:
    print(f"  t={t:7.2f}  v={vel(t):4.2f} m/s  사이통로 {c:.2f} m  (id {i2}/{i1})  경로 s={sv:.2f}")
print(f"  총 {len(rows)} 프레임")
print()
print("=== 0.64 m 미만 사이통로를 지나던 프레임 (A안이 막는 것) ===")
rows2=[]
for t,w in d['local_wpnts']:
    if len(w)==0: continue
    obs=obs_at(t); best=None
    for row in w[:60]:
        sv,dv=row[0],row[1]
        above=[];below=[]
        for a in obs:
            if not np.isfinite(a['s_start']) or not inside(sv,a): continue
            if a['d_right']>dv: above.append((a['d_right'],a['id']))
            elif a['d_left']<dv: below.append((a['d_left'],a['id']))
        if above and below:
            corr=min(above)[0]-max(below)[0]
            if best is None or corr<best[0]: best=(corr,sv,min(above)[1],max(below)[1])
    if best and best[0]<2*CLR:
        rows2.append((t,)+best)
cs=[r[1] for r in rows2]
print(f"  총 {len(rows2)} 프레임, t={rows2[0][0]:.2f}~{rows2[-1][0]:.2f}")
print(f"  사이통로 폭: 최소 {min(cs):.2f} 중앙 {np.median(cs):.2f} 최대 {max(cs):.2f} m")
print(f"  그때 속도: {vel(rows2[0][0]):.2f} -> {vel(rows2[-1][0]):.2f} m/s")
