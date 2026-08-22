import pickle,sys,numpy as np
from tr import load
D,T=load(sys.argv[1])
gb=D['global_wpnts'][0][1]
S=gb[:,0]; DR=gb[:,4]; DL=gb[:,5]
L=S[-1]+ (S[1]-S[0]); step=S[1]-S[0]; n=len(S)
def idx(s): return int((s%L)/step)%n
def minwall(s0,s1):
    i0=idx(s0); i1=idx(s1)
    k=(i1-i0)%n
    ii=[(i0+j)%n for j in range(k+1)]
    return DL[ii].min(), DR[ii].min()
# obstacle lookup by (t,id)
OF=D['obs_frenet']
ot=np.array([x[0] for x in OF])
def obs_at(t,oid):
    j=int(np.argmin(np.abs(ot-t)))
    for o in OF[j][1]:
        if o['id']==oid: return o
    return None
PCM=0.15; HW=0.15; SIG=3.0; CAP=0.12; EPS=0.05; BM_REQ=0.20
TR_M=0.10; TR_GAIN=0.08; TR_REF=3.5; TR_MAX=0.22; ADEC=2.0; VFLOOR=1.5
def need_clear(o, cap=CAP):
    # 브릿지 분산 하한(pos 0.01 / vel 0.25)이 상수라 sigma_d=3*0.180=0.54 -> 항상 cap.
    return max(PCM,HW)+cap
def bmargin(v0,dist, tr_m=TR_M, tr_gain=TR_GAIN, tr_max=TR_MAX, adec=ADEC, req=BM_REQ):
    v0=max(abs(v0),VFLOOR)
    va=max(np.sqrt(max(v0*v0-2*adec*max(dist,0),0.0)),VFLOOR) if adec>0 and dist>0 else v0
    tr=min(tr_m+tr_gain*max(va-tr_ref_g,0.0),tr_max)
    return max(req, HW+tr), va
tr_ref_g=TR_REF
rows=[]
for x in T:
    if x['oid']<0: continue
    o=obs_at(x['t'],x['oid'])
    if o is None: continue
    s0=(o['s_start']-0.5)%L; s1=(o['s_end']+0.5)%L
    wl,wr=minwall(s0,s1)
    dist_hold=(s0-x['s'])%L
    if dist_hold>12.0: dist_hold=0.0
    nc=need_clear(o); need=nc+EPS
    bm,va=bmargin(x['v'],dist_hold)
    left_max=max(wl-bm,0.0); right_min=-max(wr-bm,0.0)
    defL=(o['d_left']+nc)-left_max     # >0 = blocked
    defR=right_min-(o['d_right']-nc)
    rows.append(dict(t=x['t'],s=x['s'],d=x['d'],v=x['v'],oid=x['oid'],dist=x['dist'],
        dl=o['d_left'],dr=o['d_right'],wl=wl,wr=wr,nc=nc,bm=bm,va=va,
        defL=defL,defR=defR,outcome=x['outcome'],rej=x['rej'],dist_hold=dist_hold,
        static=o['is_static']))
import pickle as pk
pk.dump(rows,open(sys.argv[2],'wb'))
R=rows
print(f"n={len(R)}")
bl=[r for r in R if r['defL']>0 and r['defR']>0]
print(f"양쪽 다 막힘: {len(bl)} ({100*len(bl)/len(R):.1f}%)")
import statistics as st
m=[min(r['defL'],r['defR']) for r in bl]
print("가까운 쪽 부족량(m): median %.3f p25 %.3f p75 %.3f  <=0.05:%d  <=0.10:%d  <=0.15:%d"%(
  st.median(m),np.percentile(m,25),np.percentile(m,75),
  sum(1 for v in m if v<=0.05),sum(1 for v in m if v<=0.10),sum(1 for v in m if v<=0.15)))
