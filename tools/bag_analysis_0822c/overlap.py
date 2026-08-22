import lib2, numpy as np
d=lib2.D(); g=lib2.gw(d); S=g[:,0]; TL=S[-1]+(S[1]-S[0])
def mod(x): return x%TL
obs_t=np.array([t for t,_ in d['obs_frenet']])
def obs_at(t):
    i=int(np.clip(np.searchsorted(obs_t,t)-1,0,len(obs_t)-1)); return d['obs_frenet'][i][1]
def inside(sv,a): return mod(sv-a['s_start'])<=mod(a['s_end']-a['s_start'])+1e-9
CLR=0.32
tot=0; ov=0; det=[]
for t,w in d['local_wpnts']:
    if len(w)==0: continue
    obs=obs_at(t); worst=None
    for row in w[:60]:
        sv,dv=row[0],row[1]
        ab=[];be=[]
        for a in obs:
            if not np.isfinite(a['s_start']) or not inside(sv,a): continue
            if a['d_right']>dv: ab.append((a['d_right'],a))
            elif a['d_left']<dv: be.append((a['d_left'],a))
        if ab and be:
            A=min(ab,key=lambda z:z[0]); B=max(be,key=lambda z:z[0])
            corr=A[0]-B[0]
            if worst is None or corr<worst[0]: worst=(corr,A[1],B[1])
    if worst and worst[0]<2*CLR:
        tot+=1
        a,b=worst[1],worst[2]
        ai=mod(a['s_start']-mod(0)); bi=ai+mod(a['s_end']-a['s_start'])
        aj=mod(b['s_start']); bj=aj+mod(b['s_end']-b['s_start'])
        # s overlap test in a common unwrap
        ref=mod(min(a['s_start'],b['s_start'])-5)
        ai=mod(a['s_start']-ref); bi=ai+mod(a['s_end']-a['s_start'])
        aj=mod(b['s_start']-ref); bj=aj+mod(b['s_end']-b['s_start'])
        sgap=max(ai,aj)-min(bi,bj)
        if sgap<=0: ov+=1
        det.append((round(t,2),round(worst[0],2),round(sgap,2),a['id'],b['id']))
print(f"좁은 사이통로를 지난 프레임 {tot}, 그중 s 구간이 겹치는 쌍 {ov} ({100*ov/tot:.0f}%)")
for x in det: print("  t=%6.2f corr=%.2f s_gap=%+.2f id %d/%d"%x)
