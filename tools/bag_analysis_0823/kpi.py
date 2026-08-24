"""에피소드 단위 KPI: 회피가 '실행 가능한 시점'에 열렸는가."""
import recon, numpy as np, math, collections
BASE=recon.P
KMAX=1.16   # SteerKappaMax (로그 확인값)

def runway_needed(dd, kappa):
    spare = 0.8*max(KMAX-abs(kappa), 0.15*KMAX)
    return math.sqrt(6.0*abs(dd)/spare) if spare>1e-6 else 1e9

def episodes(tag, p):
    R = recon.Rec(tag)
    ev = []
    cur = None
    for t, obs in R.obs:
        f = R.at(R.fr, t)
        cur_s, cur_d, v0 = f[1], f[2], max(f[4], p['v_floor'])
        cand=[o for o in obs if o['is_static'] and 0.0<=((o['s_start']-cur_s)%R.L)<=p['lookahead']]
        if not cand:
            if cur: ev.append(cur); cur=None
            continue
        o=min(cand,key=lambda o:(o['s_start']-cur_s)%R.L)
        dist=(o['s_start']-cur_s)%R.L
        if cur is None or cur['oid']!=o['id']:
            if cur: ev.append(cur)
            cur=dict(oid=o['id'],rows=[])
        aL,bm,_=recon.apex('left',o,R,cur_s,v0,p)
        aR,_,_ =recon.apex('right',o,R,cur_s,v0,p)
        n0=recon.need_obs(o,p)
        kap=R.gw[R.idx(cur_s),7]
        ok=[]
        for a,c in ((aL, aL>=o['d_left']+n0),(aR, aR<=o['d_right']-n0)):
            if c: ok.append(runway_needed(a-cur_d, kap)<=dist)
        cur['rows'].append((dist, any(ok), any([aL>=o['d_left']+n0, aR<=o['d_right']-n0])))
    if cur: ev.append(cur)
    return [e for e in ev if len(e['rows'])>=8]

def score(tag, p):
    ev=episodes(tag,p)
    firsts=[]; nev=0; nnever=0
    for e in ev:
        nev+=1
        d=[r[0] for r in e['rows'] if r[1]]
        if d: firsts.append(max(d))
        else: nnever+=1
    return nev, nnever, np.array(firsts)

cases=[("현행",{}),
       ("A 감속: 진입속도 상한 4.0",{"_force_varr":4.0}),
       ("A' 감속 크레딧 2.0→3.0",{"arr_decel":3.0}),
       ("B1 벽예비 0.22→0.15",{"trk_max":0.15}),
       ("B2 불확실성 0.12→0.06",{"max_unc":0.06}),
       ("C  RSS 합성(벽+장애물)",{"_rss":True}),
       ("B1+B2",{"trk_max":0.15,"max_unc":0.06}),
       ("A'+B2",{"arr_decel":3.0,"max_unc":0.06}),
      ]
orig=recon.apex
def apex2(side,o,R,cur_s,v0,p):
    if "_force_varr" in p: v0=min(v0,p["_force_varr"])
    return orig(side,o,R,cur_s,v0,p)
recon.apex=apex2

for tag in ("d43","d47"):
    print("\n=== %s"%tag)
    for name,ov in cases:
        p=dict(BASE); p.update(ov)
        if ov.get("_rss"):
            # 벽예비와 불확실성을 제곱합근으로 합성 -> 등가 unc/trk 축소
            p=dict(BASE); p['_rss']=True
            # 근사: 두 여유의 합 대신 RSS. unc 0.12, trk r -> 합 0.12+r => RSS sqrt(.12²+r²)
            # 구현: max_unc 를 0 으로 두고 trk 에 RSS 를 넣는다
            def apex3(side,o,R,cur_s,v0,pp,_o=orig):
                a,bm,nd=_o(side,o,R,cur_s,v0,dict(pp,max_unc=0.0))
                return a,bm,nd
            pass
        nev,nnever,f=score(tag,p)
        print("  %-26s 에피소드 %2d | '한 번도 실행가능 안 됨' %2d (%4.0f%%) | 처음 실행가능해진 거리 p50 %.2f m / 최대 %.2f m"%(
            name,nev,nnever,100*nnever/nev, np.median(f) if len(f) else 0, f.max() if len(f) else 0))
