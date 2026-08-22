import lib2, numpy as np, copy
d=lib2.D(); g=lib2.gw(d); S=g[:,0]; TL=S[-1]+(S[1]-S[0])
def mod(x): return x%TL

def seal(obs, need):
    """spliner_node.cpp ClusterObstacles 말미의 새 봉쇄 패스와 같은 식."""
    o=[dict(x) for x in obs]; n=len(o); sealed=[]
    for i in range(n):
        for j in range(i+1,n):
            a,b=o[i],o[j]
            ref=mod(min(a['s_start'],b['s_start'])-15.0)
            ai=mod(a['s_start']-ref); bi=ai+mod(a['s_end']-a['s_start'])
            aj=mod(b['s_start']-ref); bj=aj+mod(b['s_end']-b['s_start'])
            if max(ai,aj)-min(bi,bj) > 0.0: continue          # s 안 겹침
            top=max(a['d_right'],b['d_right']); bot=min(a['d_left'],b['d_left'])
            w=top-bot
            if not (w>0.0): continue                           # 구멍 없음
            if w>=need: continue                               # 진짜 통과 가능
            mid=0.5*(top+bot)
            if a['d_right']>b['d_right']: a['d_right']=mid; b['d_left']=mid
            else: b['d_right']=mid; a['d_left']=mid
            sealed.append((a['id'],b['id'],round(w,2)))
    return o, sealed

obs_t=np.array([t for t,_ in d['obs_frenet']])
def obs_at(t):
    i=int(np.clip(np.searchsorted(obs_t,t)-1,0,len(obs_t)-1)); return d['obs_frenet'][i][1]
def inside(sv,a): return mod(sv-a['s_start'])<=mod(a['s_end']-a['s_start'])+1e-9

for need,lab in ((0.40,'need 0.40 (하한)'),(0.54,'need 0.54 (로그 실측 0.27x2)'),(0.64,'need 0.64 (상한)')):
    thin=0; wide=0; nseal=0; frames_sealed=set()
    for t,w in d['local_wpnts']:
        if len(w)==0: continue
        raw=obs_at(t)
        new,sealed=seal(raw,need)
        if sealed: nseal+=len(sealed); frames_sealed.add(round(t,2))
        # 봉쇄 후에도 좁은 사이통로를 지나는가?
        bad=False; ok=False
        for row in w[:60]:
            sv,dv=row[0],row[1]
            ab=[];be=[]
            for a in new:
                if not np.isfinite(a['s_start']) or not inside(sv,a): continue
                if a['d_right']<=dv<=a['d_left']: ab=be=[]; break
                if a['d_right']>dv: ab.append(a['d_right'])
                else: be.append(a['d_left'])
            if ab and be:
                corr=min(ab)-max(be)
                if corr<0.64: bad=True
                else: ok=True
        if bad: thin+=1
        elif ok: wide+=1
    print(f"{lab}: 봉쇄 발동 {nseal}쌍 / {len(frames_sealed)}프레임 | "
          f"봉쇄 후 좁은 사이통과 {thin}  정상 사이통과 {wide}")
