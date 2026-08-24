#!/usr/bin/env python3
"""ApexFor / lane_wall_no_room 재구성 — 파라미터 A/B."""
import pickle, numpy as np, sys, math

P = dict(path_check_margin=0.15, half_w=0.15, max_unc=0.12, sigma=3.0,
         static_t=0.3, evasion=0.35, apex_eps=0.05, bound_mindist=0.20,
         trk_m=0.10, trk_max=0.22, trk_gain=0.08, trk_ref=3.5,
         arr_decel=2.0, v_floor=1.5, lookahead=12.0)

class Rec:
    def __init__(self, tag):
        D = pickle.load(open(tag+'.pkl','rb'))
        self.obs = pickle.load(open(('o43' if tag=='d43' else 'o47')+'.pkl','rb'))
        gw = D['global_wpnts'][0][1]
        self.gw = gw
        self.L = gw[-1,0] + (gw[-1,0]-gw[-2,0])
        self.step = gw[1,0]-gw[0,0]
        self.dr = gw[:,4]; self.dl = gw[:,5]
        self.fr = D['/car_state/frenet/odom']
        self.D = D
    def idx(self, s): return int((s % self.L)/self.step) % len(self.gw)
    def minwall(self, s0, s1):
        span = (s1-s0) % self.L
        n = int(span/self.step)+2          # C++ 와 같게 끝점 포함
        i0 = self.idx(s0)
        ii = [(i0+k) % len(self.gw) for k in range(max(n,1))]
        return self.dl[ii].min(), self.dr[ii].min()
    def at(self, a, t):
        i = np.searchsorted(a[:,0], t); return a[min(max(i,0),len(a)-1)]

def need_obs(o, p):
    sig = p['sigma']*math.sqrt(max(o['d_var'],0)+ p['static_t']**2*max(o['vd_var'],0)) \
        if o['is_static'] else p['sigma']*math.sqrt(max(o['d_var'],0)+0.25*max(o['vd_var'],0))
    return max(p['path_check_margin'], p['half_w']) + min(sig, p['max_unc'])

def apex(side, o, R, cur_s, v0, p):
    dist = ((o['s_start']-0.5 - cur_s) % R.L)
    if not np.isfinite(dist) or dist > p['lookahead']: dist = 0.0
    a = p['arr_decel']
    varr = max(math.sqrt(max(v0*v0-2*a*dist,0.0)), p['v_floor']) if (a>0 and dist>0) else v0
    trk = min(p['trk_m'] + p['trk_gain']*max(varr-p['trk_ref'],0.0), p['trk_max'])
    bm = max(p['bound_mindist'], p['half_w']+trk)
    wl, wr = R.minwall((o['s_start']-0.5) % R.L, (o['s_end']+0.5) % R.L)
    need = need_obs(o,p) + p['apex_eps']
    dmin = need; dtar = max(p['evasion'], need)
    if side=='left':
        corr = wl - o['d_left']; sur = max(corr-dmin-bm, 0.0)
        bal = o['d_left']+dmin+sur/2; tgt = o['d_left']+dtar
        return min(max(min(tgt,bal), 0.0), max(wl-bm,0.0)), bm, need
    corr = wr + o['d_right']; sur = max(corr-dmin-bm, 0.0)
    bal = o['d_right']-dmin-sur/2; tgt = o['d_right']-dtar
    return max(min(max(tgt,bal), 0.0), -max(wr-bm,0.0)), bm, need

def run(tag, p, verbose=None):
    R = Rec(tag)
    nf=0; okL=0; okR=0; okAny=0; details=[]
    for t, obs in R.obs:
        f = R.at(R.fr, t)
        cur_s, cur_d, v0 = f[1], f[2], max(f[4], p['v_floor'])
        cand = [o for o in obs if o['is_static'] and
                0.0 <= ((o['s_start']-cur_s) % R.L) <= p['lookahead']]
        if not cand: continue
        o = min(cand, key=lambda o: (o['s_start']-cur_s) % R.L)
        nf += 1
        aL,bmL,ndL = apex('left', o, R, cur_s, v0, p)
        aR,bmR,ndR = apex('right', o, R, cur_s, v0, p)
        n0 = need_obs(o,p)                 # 게이트는 apex_eps 를 안 쓴다
        cL = aL >= o['d_left'] + n0
        cR = aR <= o['d_right'] - n0
        okL += cL; okR += cR; okAny += (cL or cR)
        if verbose and verbose[0] <= t <= verbose[1]:
            wl,wr = R.minwall((o['s_start']-0.5)%R.L, (o['s_end']+0.5)%R.L)
            details.append((t,cur_s,cur_d,v0,o['id'],o['d_right'],o['d_left'],
                            wl,wr,aL,cL,aR,cR,bmL,n0))
    return dict(n=nf, L=okL, R=okR, any=okAny), details

if __name__=='__main__':
    for tag in ('d43','d47'):
        s,_ = run(tag, P)
        print("%s  판정 프레임 %d | 좌 가능 %d(%.1f%%) 우 가능 %d(%.1f%%) 한쪽이상 %d(%.1f%%)"%(
            tag, s['n'], s['L'],100*s['L']/s['n'], s['R'],100*s['R']/s['n'], s['any'],100*s['any']/s['n']))

def detail(tag, t0, t1, p=P):
    s,det = run(tag, p, verbose=(t0,t1))
    print("  t     s      d     v  | id  obs_d[R,L]  wallL wallR | apexL ok  apexR ok | bm   need | 좌필요 좌한계")
    for (t,cs,cd,v,oid,dr,dl,wl,wr,aL,cL,aR,cR,bm,nd) in det:
        print("%6.2f %6.2f %+5.2f %4.1f | %3d [%+.2f,%+.2f] %.2f %.2f | %+.2f %s %+.2f %s | %.2f %.2f | %+.2f  %+.2f"%(
            t,cs,cd,v,oid,dr,dl,wl,wr,aL,'O' if cL else 'X',aR,'O' if cR else 'X',bm,nd, dl+nd, wl-bm))
