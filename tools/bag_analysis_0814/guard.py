import pickle,numpy as np,re
D=pickle.load(open('data.pkl','rb'))
rows=[]
for t,l,n,m in D['rosout']:
    if n=='spliner_node' and '[trace]' in m:
        tail=m.split('|')[-1]; md=re.search(r'\| ([a-z_]+) \|',m); v=re.search(r'v=([-\d.]+)',m)
        if md and v: rows.append((t, md.group(1), float(v.group(1)),
                                  'trailing_d_goal_zeroed' in tail or 'trailing_d_goal_shrunk' in tail))
rt=np.array([r[0] for r in rows]); rmode=np.array([r[1] for r in rows])
rdeg=np.array([r[3] for r in rows]); rv=np.array([r[2] for r in rows])
tr=rmode=='trailing'
print("trailing 사이클 %d — 횡계획 온전 %d (%.0f%%) / 깎임 %d (%.0f%%)"
      % (tr.sum(), (tr&~rdeg).sum(), 100*(tr&~rdeg).mean()/tr.mean(),
         (tr&rdeg).sum(), 100*(tr&rdeg).mean()/tr.mean()))
print("  온전한 사이클 중 차속 2.5 미만: %d (%.0f%%)  ← 바닥 2.5 가 들어올릴 대상"
      % (((tr&~rdeg)&(rv<2.5)).sum(), 100*((tr&~rdeg)&(rv<2.5)).sum()/max((tr&~rdeg).sum(),1)))
# 접촉 17건이 어느 상태였나
fr=D['/car_state/frenet/odom']; obs=D['obs_frenet']
ft=fr[:,0]; fs=fr[:,1]; fd=fr[:,2]; L=39.45
hits=[]
for t,ol in obs:
    if t<40: continue
    cs=float(np.interp(t,ft,fs)); cd=float(np.interp(t,ft,fd))
    for o in ol:
        ds=((o['s_center']-cs+L/2)%L)-L/2
        if abs(ds)>0.35: continue
        lat=max(o['d_right']-cd, cd-o['d_left'], 0.0)
        if lat<0.15: hits.append(t)
hits=np.array(hits)
br=np.where(np.diff(hits)>1.0)[0]
eps=[e[0] for e in np.split(hits,br+1)]
print()
print("접촉 수준(여유<0.15m) 에피소드 %d건 — 그때 플래너 상태:" % len(eps))
import collections
c=collections.Counter(); dg=0
for e in eps:
    j=int(np.argmin(np.abs(rt-e)))
    if abs(rt[j]-e)>0.2: c['(trace 없음)']+=1; continue
    c[rmode[j]]+=1
    if rmode[j]=='trailing' and rdeg[j]: dg+=1
for k,v in c.most_common(): print("   %-16s %d건" % (k,v))
print("   그중 trailing이면서 횡계획 깎임(=가드로 1.5 유지): %d건" % dg)
