import pickle, numpy as np, re, collections
for tag in ('d43','d47'):
    D=pickle.load(open(tag+'.pkl','rb'))
    fr=D['/car_state/frenet/odom']; lw=D['local_wpnts']
    gw=D['global_wpnts'][0][1]; L=gw[-1,0]+(gw[-1,0]-gw[-2,0])
    def at(a,t):
        i=np.searchsorted(a[:,0],t); return a[min(max(i,0),len(a)-1)]
    # 목표 d = 앞 2m 지점의 local wpnt d
    ts=[];tg=[]
    for t,W in lw:
        if len(W)<3: continue
        f=at(fr,t); ds=(W[:,0]-f[1])%L; ds=np.where(ds>L/2,ds-L,ds)
        j=int(np.argmin(np.abs(ds-2.0)))
        ts.append(t); tg.append(W[j,1])
    ts=np.array(ts); tg=np.array(tg)
    # 회피 활성 구간만 (|목표|>0.05)
    act=np.abs(tg)>0.05
    sg=np.sign(tg)
    flips=int((((sg[1:]*sg[:-1])<0)&(np.abs(tg[1:])>0.10)&(np.abs(tg[:-1])>0.10)).sum())
    print("=== %s  local 목표 샘플 %d, |목표|>0.10 부호반전 %d회 (%.2f회/초)"%(
        tag,len(tg),flips,flips/(ts[-1]-ts[0])))
    # 사이클간 목표 변화량
    dd=np.abs(np.diff(tg))
    print("   사이클간 목표 d 변화 p50/p90/p99/max = %.3f/%.3f/%.3f/%.3f m"%tuple(np.percentile(dd,[50,90,99,100])))
    print("   0.20 m 이상 급변 %d회 (%.1f%%)"%((dd>0.20).sum(),100*(dd>0.20).mean()))
    # 트레일링 횡목표 이벤트
    cnt=collections.Counter()
    for t,lvl,name,msg in D['rosout']:
        for k in ('Trailing 횡목표 포기','Trailing 횡목표 축소','차체 여유 최대 후보로 이동',
                  'LaneSwitch 사이드 잠금 양보','LaneSwitch 사이드 전환 거부','최소오프셋 완화'):
            if k in msg: cnt[k]+=1
    print("   ",dict(cnt))
