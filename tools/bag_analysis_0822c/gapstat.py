import lib2, numpy as np
d=lib2.D(); g=lib2.gw(d)
S=g[:,0]; DR=g[:,4]; DL=g[:,5]
TL=S[-1]+(S[1]-S[0])
def mod(x): return x%TL
CLR=0.32   # RequiredObstacleClearance 상한 (0.20 + 캡 0.12)
CLR_LO=0.20
BND=0.20
def idx(s): return int(np.argmin(np.abs(S-mod(s))))

frames=d['obs_frenet']
pairs_tot=0; pairs_overlap=0
narrow=0; narrow_lo=0; wide=0
fr_with_narrow=set()
per_frame=[]
for t,obs in frames:
    o=[x for x in obs if np.isfinite(x['s_start'])]
    n=len(o)
    fnarrow=0
    for i in range(n):
        for j in range(i+1,n):
            a,b=o[i],o[j]
            # s overlap?
            ai=mod(a['s_start']); bi=ai+mod(a['s_end']-a['s_start'])
            aj=mod(b['s_start']); bj=aj+mod(b['s_end']-b['s_start'])
            # unwrap relative
            sg=max(ai,aj)-min(bi,bj)
            if sg>2.0: continue
            pairs_tot+=1
            if sg<0: pairs_overlap+=1
            lat=max(a['d_right'],b['d_right'])-min(a['d_left'],b['d_left'])
            if lat<0: continue      # d 겹침 = 사이 통로 없음
            if lat < 2*CLR_LO: narrow_lo+=1
            if lat < 2*CLR:
                narrow+=1; fnarrow+=1
            else: wide+=1
    if fnarrow: fr_with_narrow.add(round(t,2))
    per_frame.append((t,fnarrow))
print(f"프레임 {len(frames)}, s 인접(간격<2m) 쌍 {pairs_tot} (그중 s 겹침 {pairs_overlap})")
print(f"  사이 통로 폭 < 0.64m (=2x0.32 요구) : {narrow}  ({100*narrow/max(pairs_tot,1):.1f}%)")
print(f"  사이 통로 폭 < 0.40m (=2x0.20 최소): {narrow_lo}")
print(f"  사이 통로 폭 >= 0.64m (진짜 통과가능): {wide}  ({100*wide/max(pairs_tot,1):.1f}%)")
print(f"  좁은 쌍이 있는 프레임: {len(fr_with_narrow)} / {len(frames)} ({100*len(fr_with_narrow)/len(frames):.1f}%)")
