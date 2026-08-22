import lib2, numpy as np
d=lib2.D(); g=lib2.gw(d); S=g[:,0]; TL=S[-1]+(S[1]-S[0])
def mod(x): return x%TL
obs_t=np.array([t for t,_ in d['obs_frenet']])
def obs_at(t):
    i=int(np.clip(np.searchsorted(obs_t,t)-1,0,len(obs_t)-1)); return d['obs_frenet'][i][1]
def inside(sv,a): return mod(sv-a['s_start'])<=mod(a['s_end']-a['s_start'])+1e-9
ev=[]
for t,w in d['local_wpnts']:
    if len(w)==0: continue
    obs=obs_at(t); worst=None
    for row in w[:60]:
        sv,dv=row[0],row[1]
        for a in obs:
            if not np.isfinite(a['s_start']) or not inside(sv,a): continue
            if a['d_right']<=dv<=a['d_left']:
                pen=min(dv-a['d_right'],a['d_left']-dv)
                if worst is None or pen>worst[0]: worst=(pen,sv,a['id'],round(mod(sv-0),2))
    if worst: ev.append((t,)+worst)
print(f"관통 프레임 {len(ev)}")
# episodes
eps=[]; cur=[ev[0]]
for e in ev[1:]:
    if e[0]-cur[-1][0]<0.5: cur.append(e)
    else: eps.append(cur); cur=[e]
eps.append(cur)
print(f"에피소드 {len(eps)}")
for c in eps:
    pens=[x[1] for x in c]
    print(f"  t={c[0][0]:6.2f}~{c[-1][0]:6.2f} ({len(c)}프레임) 최대 침투 {max(pens):.2f} m  obs id {sorted(set(x[3] for x in c))}")
