import lib2, numpy as np
d=lib2.D(); g=lib2.gw(d)
S,DR,DL=g[:,0],g[:,4],g[:,5]; TL=S[-1]+(S[1]-S[0])
def mod(x): return x%TL
def bidx(s): return int(np.argmin(np.abs(S-mod(s))))
CLR=0.27; BND=0.20; NEED=CLR+BND

def groups(obs):
    """s 가 겹치는 것끼리 묶어 union 박스로 (플래너가 실제로 마주하는 단위)"""
    o=[x for x in obs if np.isfinite(x['s_start'])]
    if not o: return []
    ref=mod(min(x['s_start'] for x in o)-15.0)
    iv=[(mod(x['s_start']-ref), mod(x['s_start']-ref)+mod(x['s_end']-x['s_start']), x) for x in o]
    iv.sort()
    out=[]; cur=[iv[0]]
    for a,b,x in iv[1:]:
        if a<=max(c[1] for c in cur): cur.append((a,b,x))
        else: out.append(cur); cur=[(a,b,x)]
    out.append(cur)
    return [(min(c[0] for c in gp)+ref, max(c[1] for c in gp)+ref,
             max(c[2]['d_left'] for c in gp), min(c[2]['d_right'] for c in gp), len(gp))
            for gp in out]

print("=== 3\" : 경계를 Δ 넓혔을 때 '양쪽 다 막힘' 클러스터 ===")
print(" Δ[m]   양쪽막힘   (감소)   그중 2개 이상 뭉친 것")
base=None
for D_ in (0.0,0.02,0.04,0.06,0.08,0.10):
    blk=0; blk_multi=0; tot=0
    for t,obs in d['obs_frenet']:
        for s0,s1,dl,dr,n in groups(obs):
            bi=bidx(0.5*(s0+s1)); wl,wr=DL[bi]+D_,-(DR[bi]+D_)
            tot+=1
            if (wl-dl)<NEED and (dr-wr)<NEED:
                blk+=1
                if n>1: blk_multi+=1
    if base is None: base=blk
    print(f" {D_:.2f}   {blk:6d}   ({blk-base:+5d})     {blk_multi:5d}   (전체 클러스터 {tot})")

print("\n=== 사고 구간(23.1~24.8) 좌/우 통로 여유 (Δ=0) ===")
for t,obs in d['obs_frenet']:
    if not (23.1<=t<=24.8): continue
    for s0,s1,dl,dr,n in groups(obs):
        bi=bidx(0.5*(s0+s1))
        gl=DL[bi]-dl; gr=dr+DR[bi]
        if n>1:
            print(f"  t={t:6.2f} s[{mod(s0):.2f},{mod(s1):.2f}] d[{dr:+.2f},{dl:+.2f}] n={n} "
                  f"| 좌여유 {gl:+.2f} 우여유 {gr:+.2f} (필요 {NEED:.2f}) "
                  f"{'좌부족 %.2f'%(NEED-gl) if gl<NEED else '좌OK'} / {'우부족 %.2f'%(NEED-gr) if gr<NEED else '우OK'}")
            break
