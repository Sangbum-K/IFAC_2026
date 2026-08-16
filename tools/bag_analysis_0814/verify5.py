import pickle,numpy as np,re
D=pickle.load(open('data.pkl','rb'))
ot=D['otwpnts']; obs=D['obs_frenet']; fr=D['/car_state/frenet/odom']
ft=fr[:,0]; fs=fr[:,1]; fd=fr[:,2]; L=39.45
ots=np.array([x[0] for x in ot]); obt=np.array([x[0] for x in obs])
rows=[]
for t,l,n,m in D['rosout']:
    if n=='spliner_node' and '[trace]' in m:
        md=re.search(r'\| ([a-z_]+) \|',m)
        if md: rows.append((t,md.group(1)))
rt=np.array([r[0] for r in rows]); rmode=np.array([r[1] for r in rows])

def path_hits(t, clear):
    """발행된 경로 점이 장애물 스팬 안(±clear)으로 들어가는가."""
    i=int(np.argmin(np.abs(ots-t))); a=ot[i][1]
    if len(a)<3: return False
    j=int(np.argmin(np.abs(obt-t)))
    for o in obs[j][1]:
        s0,s1=o['s_start'],o['s_end']
        span=(s1-s0)%L
        if span<1e-3: span=max(o['size'],0.1)
        for w in a[::2]:
            ds=(w[0]-s0)%L
            if ds>span+0.15: continue
            if o['d_right']-clear <= w[1] <= o['d_left']+clear: return True
    return False

# 접촉 에피소드
hits=[]
for t,ol in obs:
    if t<40: continue
    cs=float(np.interp(t,ft,fs)); cd=float(np.interp(t,ft,fd))
    for o in ol:
        d=((o['s_center']-cs+L/2)%L)-L/2
        if abs(d)>0.35: continue
        if max(o['d_right']-cd, cd-o['d_left'], 0.0)<0.15: hits.append(t)
hits=np.array(hits); br=np.where(np.diff(hits)>1.0)[0]
eps=[e[0] for e in np.split(hits,br+1)]
tr_idx=[i for i,(t,a) in enumerate(ot) if 40<t<478 and
        (lambda j: abs(rt[j]-t)<0.1 and rmode[j]=='trailing')(int(np.argmin(np.abs(rt-t))))]
print("접촉 에피소드 %d건 / 트레일링 사이클 %d개" % (len(eps), len(tr_idx)))
print()
print("clear   접촉 에피소드 사전 검출   트레일링 사이클 중 발동률")
for clear in (0.10,0.15,0.20,0.25):
    # 접촉 0.3초 전에 이미 경로가 장애물 안에 있었나
    caught=sum(1 for e in eps if any(path_hits(e-dt,clear) for dt in (0.0,0.15,0.3)))
    fire=np.mean([path_hits(ot[i][0],clear) for i in tr_idx[::3]])
    print("  %.2f      %2d / %2d  (%3.0f%%)          %5.1f%%" % (clear,caught,len(eps),100*caught/len(eps),100*fire))
