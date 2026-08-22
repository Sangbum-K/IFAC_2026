import lib2, numpy as np
d=lib2.D(); g=lib2.gw(d)
S=g[:,0]; TL=S[-1]+(S[1]-S[0])
def mod(x): return x%TL
CLR=0.32
obs_t=np.array([t for t,_ in d['obs_frenet']])
def obs_at(t):
    i=int(np.clip(np.searchsorted(obs_t,t)-1,0,len(obs_t)-1)); return d['obs_frenet'][i][1]
def inside(sv,a):
    return mod(sv-a['s_start'])<=mod(a['s_end']-a['s_start'])+1e-9

for key,label in (('local_wpnts','발행 경로 /local_waypoints'),('otwpnts','회피 스플라인 /otwpnts')):
    thread_narrow=0; thread_wide=0; pierce=0; tot=0; frames_bad=[]
    for t,w in d[key]:
        if len(w)==0: continue
        obs=obs_at(t)
        if len(obs)<1: continue
        tot+=1
        bad=False; badw=False; pc=False
        for row in w[:60]:
            sv,dv=row[0],row[1]
            above=[]; below=[]
            for a in obs:
                if not np.isfinite(a['s_start']) or not inside(sv,a): continue
                if a['d_right']<=dv<=a['d_left']: pc=True
                elif a['d_right']>dv: above.append(a['d_right'])   # 왼쪽(더 큰 d)
                else: below.append(a['d_left'])
            if above and below:
                corr=min(above)-max(below)
                if corr < 2*CLR: bad=True
                else: badw=True
        if pc: pierce+=1
        if bad: thread_narrow+=1; frames_bad.append(round(t,2))
        elif badw: thread_wide+=1
    print(f"{label}: 프레임 {tot}")
    print(f"   장애물 사이를 지나는 경로 — 통로<0.64m: {thread_narrow} ({100*thread_narrow/max(tot,1):.2f}%)")
    print(f"   장애물 사이를 지나는 경로 — 통로>=0.64m: {thread_wide} ({100*thread_wide/max(tot,1):.2f}%)")
    print(f"   장애물 박스를 관통하는 경로: {pierce} ({100*pierce/max(tot,1):.2f}%)")
    if frames_bad: print(f"   좁은 사이통과 시각: {frames_bad[:25]} ... 총 {len(frames_bad)}")
