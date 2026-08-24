import pickle,re,numpy as np,collections
RX=re.compile(r"\[trace\] s=([-\d.]+) v=([-\d.]+) d=([-\d.]+) \| obs id=(\d+) dist=([-\d.]+) d\[([-\d.]+),([-\d.]+)\] static=(\d) nobs=(\d+) \| evade=(\d) sector=(\d) commit=(\d) \| (\w+) \| (.*)$")
for tag in ('d43','d47'):
    D=pickle.load(open(tag+'.pkl','rb'))
    rows=[]
    for t,lvl,name,msg in D['rosout']:
        m=RX.search(msg)
        if m:
            g=m.groups(); rows.append(dict(t=t,s=float(g[0]),v=float(g[1]),d=float(g[2]),
                oid=int(g[3]),dist=float(g[4]),dr=float(g[5]),dl=float(g[6]),
                static=g[7]=='1',mode=g[12],rj=g[13].split(',')))
    print("\n=== %s"%tag)
    for key in ('lane_wall_no_room','lane_geom_floor','lane_speed_profile'):
        d=np.array([r['dist'] for r in rows if key in r['rj']])
        v=np.array([r['v'] for r in rows if key in r['rj']])
        if len(d): print("  %-20s n=%4d  거리 p10/p50/p90 = %.2f/%.2f/%.2f m   속도 p50 %.2f"%(
            key,len(d),*np.percentile(d,[10,50,90]),np.median(v)))
    # 에피소드: 같은 oid 연속 구간
    eps=[]; cur=None
    for r in rows:
        if cur is None or r['oid']!=cur['oid'] or r['t']-cur['t1']>0.5:
            if cur: eps.append(cur)
            cur=dict(oid=r['oid'],t0=r['t'],t1=r['t'],rs=[])
        cur['t1']=r['t']; cur['rs'].append(r)
    if cur: eps.append(cur)
    eps=[e for e in eps if len(e['rs'])>=5]
    print("  회피 에피소드 %d개"%len(eps))
    nfirst=collections.Counter(); nlast=collections.Counter(); ever=0
    trans=0
    for e in eps:
        far=[r for r in e['rs'] if r['dist']>=2.5]
        near=[r for r in e['rs'] if 0.3<=r['dist']<2.5]
        if far: nfirst[[x for x in far[0]['rj'] if x.startswith('lane_')][:1] and [x for x in far[0]['rj'] if x.startswith('lane_')][0] or 'ok']+=1
        if near: nlast[[x for x in near[-1]['rj'] if x.startswith('lane_')][:1] and [x for x in near[-1]['rj'] if x.startswith('lane_')][0] or 'ok']+=1
        if far and near:
            f=[x for x in far[0]['rj'] if x.startswith('lane_')]
            n=[x for x in near[-1]['rj'] if x.startswith('lane_')]
            if f and f[0]=='lane_wall_no_room' and n and n[0] in ('lane_geom_floor','lane_speed_profile'): trans+=1
        if any('lane_' not in ''.join(r['rj']) for r in e['rs']): ever+=1
    print("   원거리(>=2.5m) 첫 판정:", dict(nfirst))
    print("   근거리(<2.5m) 마지막 판정:", dict(nlast))
    print("   '멀리선 벽부족 -> 가까이선 기하/속도불가' 전이 에피소드: %d / %d"%(trans,len(eps)))
