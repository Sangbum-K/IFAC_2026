"""반영된 실제 값으로 좌/우 통로 성립률을 다시 계산한다."""
import pickle,sys,numpy as np,statistics as st
from tr import load
BRK=np.array([1.5,2.0,2.5,3.0,3.5,4.0,4.5,5.0,5.5,6.0,6.5,7.0])
ACC=np.array([2.27,3.63,5.26,6.72,8.14,9.69,9.60,8.27,6.41,6.00,5.49,4.93])
KIN=np.tan(0.3578)/0.33
def cap_for(k):
    k=abs(k)
    if k<1e-4: return np.inf
    if k>KIN: return 0.0
    best=0.0; v=0.02
    while v<=BRK[-1]:
        if k*v*v>np.interp(v,BRK,ACC): break
        best=v; v+=0.02
    return best if best<BRK[-1]-1e-9 else np.sqrt(ACC[-1]/k)
D,T=load(sys.argv[1])
gb=D['global_wpnts'][0][1]; S=gb[:,0]; DR=gb[:,4]; DL=gb[:,5]; KA=np.abs(gb[:,7])
step=S[1]-S[0]; n=len(S); L=S[-1]+step
vcap_wp=np.array([cap_for(k) for k in KA])       # 레이스라인 조향 속도캡
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
HW=0.15; PCM=0.15
def bm(v,dist,gain,tr_m=0.10,tr_max=0.22,adec=2.0,req=0.20):
    v0=max(abs(v),1.5)
    va=max(np.sqrt(max(v0*v0-2*adec*max(dist,0),0.0)),1.5) if dist>0 else v0
    return max(req, HW+min(tr_m+gain*max(va-3.5,0.0),tr_max))
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
def run(name, cap_u, gain, shrink, use_vcap):
    nc=max(PCM,HW)+cap_u
    blocked=okL=okR=0
    for x,o,wl,wr,dh in rows:
        dl=o['d_left']-shrink; dr=o['d_right']+shrink
        v=x['v']
        if use_vcap:
            v=min(v, vcap_wp[idx(x['s'])])       # 코너 속도캡 반영
        m=bm(v,dh,gain)
        if (dl+nc)-max(wl-m,0.0)<=0: okL+=1
        elif -max(wr-m,0.0)-(dr-nc)<=0: okR+=1
        else: blocked+=1; continue
        if -max(wr-m,0.0)-(dr-nc)<=0 and (dl+nc)-max(wl-m,0.0)>0: pass
    N=len(rows)
    print(f"  {name:38s} 양쪽막힘 {100*blocked/N:5.1f}%   콘표면~차체 실여유 {cap_u:.2f} m")
    return blocked/N
print(f"=== {sys.argv[1]}  ({len(rows)} 계획 사이클) ===")
run("변경 전 (0823 런 그대로)",                0.12, 0.08, 0.00, False)
run("불확실성 캡 0.12->0.09",                  0.09, 0.08, 0.00, False)
run("+ 벽예비 속도항 제거  = 현재 반영값",       0.09, 0.00, 0.00, False)
run("  (참고) + 장애물 폭 0.60->0.50 이었다면", 0.09, 0.00, 0.05, False)
