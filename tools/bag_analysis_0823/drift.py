import pickle, numpy as np, collections
for tag in ('o43','o47'):
    R=pickle.load(open(tag+'.pkl','rb'))
    hist=collections.defaultdict(list)
    for t,obs in R:
        for o in obs:
            if o['is_static']: hist[o['id']].append((t,o['d_left'],o['d_right']))
    out={h:[] for h in (0.3,0.5,1.0)}
    for k,v in hist.items():
        a=np.array(v)
        if len(a)<10: continue
        for h in out:
            for i in range(len(a)):
                j=np.searchsorted(a[:,0], a[i,0]+h)
                if j<len(a) and abs(a[j,0]-a[i,0]-h)<0.15:
                    out[h].append(abs(a[j,1]-a[i,1])); out[h].append(abs(a[j,2]-a[i,2]))
    print("=== %s  정적 장애물 경계 드리프트"%tag)
    for h in (0.3,0.5,1.0):
        d=np.array(out[h])
        if len(d): print("   Δt=%.1fs: p50 %.3f p90 %.3f p99 %.3f max %.3f (n=%d)"%(h,*np.percentile(d,[50,90,99,100]),len(d)))
