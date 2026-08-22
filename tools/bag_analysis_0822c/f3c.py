import lib2, numpy as np
d=lib2.D(); g=lib2.gw(d)
S,DR,DL=g[:,0],g[:,4],g[:,5]; TL=S[-1]+(S[1]-S[0])
def bidx(s): return int(np.argmin(np.abs(S-(s%TL))))
CLR=0.27; BND=0.20; NEED=CLR+BND
print(" Δ[m]  양쪽 다 막힘  (감소)   |  통로가 '아깝게' 막힌 건(부족<Δ)")
base=None
for D_ in (0.0,0.02,0.04,0.06,0.08,0.10):
    blocked=0; tot=0; near=0
    for t,obs in d['obs_frenet']:
        for o in obs:
            if not np.isfinite(o['s_center']): continue
            bi=bidx(o['s_center']); wl,wr=DL[bi]+D_,-(DR[bi]+D_)
            tot+=1
            gl=wl-o['d_left']; gr=o['d_right']-wr
            if gl<NEED and gr<NEED:
                blocked+=1
                if max(gl,gr) > NEED-0.10: near+=1
    if base is None: base=blocked
    print(f" {D_:.2f}   {blocked:5d}      ({blocked-base:+5d})   |  {near:5d}   (전체 {tot})")
