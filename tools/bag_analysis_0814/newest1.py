import pickle,numpy as np,re
D=pickle.load(open('data.pkl','rb'))
ot=D['otwpnts']; fr=D['/car_state/frenet/odom']; gw=D['global_wpnts'][-1][1]
ft=fr[:,0]; fv=fr[:,4]; fs=fr[:,1]; gs=gw[:,0]; gv=gw[:,8]; L=39.45
rows=[]
for t,l,n,m in D['rosout']:
    if n=='spliner_node' and '[trace]' in m:
        mm=re.search(r'\| ([a-z_]+) \|',m)
        if mm: rows.append((t,mm.group(1)))
rt=np.array([r[0] for r in rows]); rm=np.array([r[1] for r in rows])
# 동결이 실제로 발목을 잡는 사이클: 머리 < 0.9 x 현재 차속
otts=np.array([x[0] for x in ot])
frozen_bind=[]
for t,a in ot:
    if len(a)<20 or t<40 or t>478: continue
    j=int(np.argmin(abs(rt-t)))
    if abs(rt[j]-t)>0.1 or rm[j]!='committed_hold': continue
    v0=float(np.interp(t,ft,fv))
    frozen_bind.append((t, a[0,8] < 0.9*max(v0,.1)))
fb=np.array(frozen_bind)
print("committed_hold 사이클 %d개 중 동결이 발목 잡는 것 %d개 (%.1f%%)"
      % (len(fb), int(fb[:,1].sum()), 100*fb[:,1].mean()))
# 손실시간 귀속
m=(ft>40)&(ft<478); t=ft[m]; s=fs[m]; v=fv[m]
dt=np.clip(np.r_[np.diff(t),0],0,0.15)
vref=np.interp(s%L,gs,gv); lost=dt*np.clip(1-v/np.maximum(vref,0.1),0,1)
idx=np.searchsorted(rt,t).clip(0,len(rt)-1)
mode=np.where(np.abs(rt[idx]-t)<0.1, rm[idx], 'none')
bi=np.searchsorted(fb[:,0],t).clip(0,len(fb)-1)
bind=(np.abs(fb[bi,0]-t)<0.1)&(fb[bi,1]>0.5)
ch=(mode=='committed_hold')
print()
print("committed_hold 총 손실 %.1fs (랩당 %.2fs)" % (lost[ch].sum(), lost[ch].sum()/34))
print("  그중 동결이 발목 잡던 사이클 %.1fs (랩당 %.2fs)" % (lost[ch&bind].sum(), lost[ch&bind].sum()/34))
print("  나머지(경로 자체가 느린 것)   %.1fs (랩당 %.2fs)" % (lost[ch&~bind].sum(), lost[ch&~bind].sum()/34))
