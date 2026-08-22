import lib2, numpy as np
d=lib2.D(); g=lib2.gw(d); S=g[:,0]; TL=S[-1]+(S[1]-S[0])
def mod(x): return x%TL
need=0.64
grow=[]; tail=[]; span_before=[]; span_after=[]
for t,obs in d['obs_frenet']:
    o=[dict(x) for x in obs]; n=len(o)
    for i in range(n):
        for j in range(i+1,n):
            a,b=o[i],o[j]
            ref=mod(min(a['s_start'],b['s_start'])-15.0)
            ai=mod(a['s_start']-ref); bi=ai+mod(a['s_end']-a['s_start'])
            aj=mod(b['s_start']-ref); bj=aj+mod(b['s_end']-b['s_start'])
            if max(ai,aj)-min(bi,bj)>0.0: continue
            top=max(a['d_right'],b['d_right']); bot=min(a['d_left'],b['d_left'])
            w=top-bot
            if not(0.0<w<need): continue
            mid=0.5*(top+bot)
            un_lo,un_hi=min(ai,aj),max(bi,bj); ov=min(bi,bj)-max(ai,aj)
            tail.append((un_hi-un_lo)-ov)
            span_before.append(max(a['d_left'],b['d_left'])-min(a['d_right'],b['d_right']))
            if a['d_right']>b['d_right']: grow += [a['d_right']-mid, mid-b['d_left']]
            else: grow += [b['d_right']-mid, mid-a['d_left']]
            if a['d_right']>b['d_right']: a['d_right']=mid; b['d_left']=mid
            else: b['d_right']=mid; a['d_left']=mid
            span_after.append(max(a['d_left'],b['d_left'])-min(a['d_right'],b['d_right']))
grow=np.array(grow); tail=np.array(tail)
print(f"봉쇄 쌍 {len(tail)}")
print(f"  박스 하나가 d 로 커지는 양: p50 {np.median(grow):.3f}  p95 {np.percentile(grow,95):.3f}  max {grow.max():.3f} m")
print(f"  그 확장이 새는 s 꼬리 길이: p50 {np.median(tail):.2f}  p95 {np.percentile(tail,95):.2f}  max {tail.max():.2f} m")
print(f"  쌍 union 의 d 폭: 봉쇄 전 p50 {np.median(span_before):.2f} -> 후 p50 {np.median(span_after):.2f} (같아야 정상)")
