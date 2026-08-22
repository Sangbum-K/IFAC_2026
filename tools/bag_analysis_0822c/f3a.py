# 3' 인지 박스 폭 플리커 — 시간 필터의 득실
import lib2, numpy as np
from collections import defaultdict, deque
d=lib2.D(); g=lib2.gw(d); S=g[:,0]; DR=g[:,4]; DL=g[:,5]; TL=S[-1]+(S[1]-S[0])
def mod(x): return x%TL
def bidx(s): return int(np.argmin(np.abs(S-mod(s))))
CLR=0.27; BND=0.20; NEED=CLR+BND        # 벽-장애물 통로 필요폭

hist=defaultdict(lambda: deque(maxlen=3))
spike=0; tot=0; flip=0; flip_t=[]
grow_persist=0; grow_events=0
prev_w={}; run=defaultdict(int)
for t,obs in d['obs_frenet']:
    for o in obs:
        i=o['id']; w=o['d_left']-o['d_right']
        h=hist[i]
        if len(h)>=2:
            tot+=1
            med=float(np.median(list(h)+[w]))
            if w>med+0.15:
                spike+=1
                # 이 스파이크가 통로 판정을 뒤집었나? (필터 적용 = 폭을 med 로)
                c=0.5*(o['d_left']+o['d_right'])
                bi=bidx(o['s_center']); wl,wr=DL[bi],-DR[bi]
                def open_sides(width):
                    l=o['d_left']  if width is None else c+width/2
                    r=o['d_right'] if width is None else c-width/2
                    return (wl-l)>=NEED, (r-wr)>=NEED
                rawL,rawR=open_sides(None); filL,filR=open_sides(med)
                if (rawL or rawR)!=(filL or filR) or (rawL!=filL) or (rawR!=filR):
                    flip+=1; flip_t.append(round(t,2))
        # 진짜 성장(3프레임 이상 유지)인가
        if i in prev_w and w>prev_w[i]+0.15:
            run[i]=1; grow_events+=1
        elif i in prev_w and run[i]>0:
            if w>=prev_w[i]-0.05: run[i]+=1
            else:
                if run[i]>=3: grow_persist+=1
                run[i]=0
        prev_w[i]=w
        h.append(w)
print("=== 3' 폭 플리커 ===")
print(f"판정 가능한 관측 {tot}건")
print(f"  중앙값3 대비 +0.15 m 이상 튄 프레임(스파이크): {spike} ({100*spike/tot:.1f}%)")
print(f"  그중 좌/우 통로 '열림/막힘' 판정을 뒤집은 것: {flip} ({100*flip/tot:.2f}% of all)")
print(f"  뒤집힌 시각(앞 20개): {flip_t[:20]}")
print(f"  폭이 0.15 m 이상 커진 사건 {grow_events}건, 그중 3프레임 이상 유지된 '진짜 성장' {grow_persist}건")
print(f"  -> 중앙값3 필터가 늦추는 것은 '진짜 성장' {grow_persist}건 x 약 1프레임(33ms)")
