import pickle,numpy as np,re
D=pickle.load(open('data.pkl','rb'))
ot=D['otwpnts']; fr=D['/car_state/frenet/odom']; gw=D['global_wpnts'][-1][1]
ft=fr[:,0]; fv=fr[:,4]; gs=gw[:,0]; gv=gw[:,8]; L=39.45
rows=[]
for t,l,n,m in D['rosout']:
    if n=='spliner_node' and '[trace]' in m:
        md=re.search(r'\| ([a-z_]+) \|',m)
        if md: rows.append((t,md.group(1)))
rt=np.array([r[0] for r in rows]); rmode=np.array([r[1] for r in rows])
res=[]
for t,a in ot:
    if len(a)<20 or t<40 or t>478: continue
    j=int(np.argmin(np.abs(rt-t)))
    if abs(rt[j]-t)>0.1 or rmode[j]!='committed_hold': continue
    v0=float(np.interp(t,ft,fv)); frozen=float(a[0,8])
    base=float(np.interp(a[0,0]%L,gs,gv))
    res.append((frozen, v0, base))
r=np.array(res)
gain=np.clip(np.minimum(r[:,1],r[:,2])-r[:,0],0,None)
print("committed_hold 사이클 %d개" % len(r))
print("  동결 머리 %.2f  /  현재 차속 %.2f  /  레이스라인 %.2f  (중앙값)"
      % (np.median(r[:,0]), np.median(r[:,1]), np.median(r[:,2])))
print()
print("갱신으로 회복 가능한 머리 속도 (기하 상한이 허용하는 한, 상한 추정)")
print("  p50 = +%.2f m/s   p75 = +%.2f   p90 = +%.2f" % tuple(np.percentile(gain,[50,75,90])))
print("  회복 여지가 0.3 m/s 이상인 사이클: %.1f%%" % (100*np.mean(gain>0.3)))
print("  회복 여지가 1.0 m/s 이상인 사이클: %.1f%%" % (100*np.mean(gain>1.0)))
