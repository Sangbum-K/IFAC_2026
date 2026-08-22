import pickle,sys,numpy as np,statistics as st
from tr import load
D,T=load(sys.argv[1])
gb=D['global_wpnts'][0][1]; S=gb[:,0]; DR=gb[:,4]; DL=gb[:,5]
step=S[1]-S[0]; n=len(S); L=S[-1]+step
def idx(s): return int((s%L)/step)%n
def minwall(s0,s1):
    i0=idx(s0); k=(idx(s1)-i0)%n
    ii=[(i0+j)%n for j in range(k+1)]
    return DL[ii].min(), DR[ii].min()
OF=D['obs_frenet']; ot=np.array([x[0] for x in OF])
def obs_at(t,oid):
    j=int(np.argmin(np.abs(ot-t)))
    for o in OF[j][1]:
        if o['id']==oid: return o
    return None
HW=0.15; PCM=0.15; VF=1.5
def scen(v,dist,cap,tr_m,tr_gain,tr_max,adec,req,vcap):
    v0=max(min(abs(v),vcap),VF)
    va=max(np.sqrt(max(v0*v0-2*adec*max(dist,0),0.0)),VF) if adec>0 and dist>0 else v0
    tr=min(tr_m+tr_gain*max(va-3.5,0.0),tr_max)
    return max(PCM,HW)+cap, max(req, HW+tr)
CASES=[
 ("현행 (base)",              dict(cap=.12,tr_m=.10,tr_gain=.08,tr_max=.22,adec=2.0,req=.20,vcap=99)),
 ("A 벽예비 속도항 제거 gain0", dict(cap=.12,tr_m=.10,tr_gain=.00,tr_max=.22,adec=2.0,req=.20,vcap=99)),
 ("A' 벽예비 상한 0.22->0.15", dict(cap=.12,tr_m=.10,tr_gain=.08,tr_max=.15,adec=2.0,req=.20,vcap=99)),
 ("B 불확실캡 0.12->0.06",     dict(cap=.06,tr_m=.10,tr_gain=.08,tr_max=.22,adec=2.0,req=.20,vcap=99)),
 ("B' 불확실캡 0.12->0.09",    dict(cap=.09,tr_m=.10,tr_gain=.08,tr_max=.22,adec=2.0,req=.20,vcap=99)),
 ("C 코너 속도캡 3.5",         dict(cap=.12,tr_m=.10,tr_gain=.08,tr_max=.22,adec=2.0,req=.20,vcap=3.5)),
 ("C' 코너 속도캡 4.5",        dict(cap=.12,tr_m=.10,tr_gain=.08,tr_max=.22,adec=2.0,req=.20,vcap=4.5)),
 ("A+B'",                     dict(cap=.09,tr_m=.10,tr_gain=.00,tr_max=.22,adec=2.0,req=.20,vcap=99)),
 ("물리한계(여유0)",           dict(cap=.00,tr_m=.00,tr_gain=.00,tr_max=.00,adec=0.0,req=.15,vcap=99)),
]
rows=[]
for x in T:
    if x['oid']<0: continue
    o=obs_at(x['t'],x['oid'])
    if o is None: continue
    s0=(o['s_start']-0.5)%L; s1=(o['s_end']+0.5)%L
    wl,wr=minwall(s0,s1)
    dh=(s0-x['s'])%L
    if dh>12.0: dh=0.0
    rows.append((x,o,wl,wr,dh))
print(f"{'시나리오':26s} {'양쪽막힘%':>9s} {'왼쪽열림%':>9s} {'오른열림%':>9s} {'실여유중앙(m)':>12s}")
base=None
for name,kw in CASES:
    blocked=0; okL=0; okR=0; slacks=[]
    for x,o,wl,wr,dh in rows:
        nc,bm=scen(x['v'],dh,**kw)
        lmax=max(wl-bm,0.0); rmin=-max(wr-bm,0.0)
        dL=(o['d_left']+nc)-lmax; dR=rmin-(o['d_right']-nc)
        if dL<=0: okL+=1
        if dR<=0: okR+=1
        if dL>0 and dR>0: blocked+=1
        else:
            # 실제 확보되는 물리 여유(콘 가장자리~차체) = nc - 0.15
            slacks.append(nc-HW)
    N=len(rows)
    print(f"{name:26s} {100*blocked/N:8.1f}% {100*okL/N:8.1f}% {100*okR/N:8.1f}% {st.median(slacks) if slacks else float('nan'):11.3f}")
    if base is None: base=blocked
print(f"총 사이클 {len(rows)}")
