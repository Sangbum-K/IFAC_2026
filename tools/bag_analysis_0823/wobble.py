import pickle, numpy as np, collections
for tag in ('o43','o47'):
    R=pickle.load(open(tag+'.pkl','rb'))
    hist=collections.defaultdict(list)
    for t,obs in R:
        for o in obs:
            if o['is_static']: hist[o['id']].append((t,o['d_left'],o['d_right'],o['s_start'],o['s_end']))
    jl=[];jr=[];w=[]
    for k,v in hist.items():
        if len(v)<5: continue
        a=np.array(v)
        jl += list(np.abs(np.diff(a[:,1])))
        jr += list(np.abs(np.diff(a[:,2])))
        w  += list(a[:,1]-a[:,2])
    jl=np.array(jl); jr=np.array(jr); w=np.array(w)
    print("=== %s  정적 트랙 %d개, 샘플 %d"%(tag,len(hist),len(jl)))
    print("  d_left 프레임간 점프 p50/p90/p99/max = %.3f/%.3f/%.3f/%.3f m"%tuple(np.percentile(jl,[50,90,99,100])))
    print("  d_right                              = %.3f/%.3f/%.3f/%.3f m"%tuple(np.percentile(jr,[50,90,99,100])))
    print("  박스 폭 p10/p50/p90/max = %.2f/%.2f/%.2f/%.2f m"%tuple(np.percentile(w,[10,50,90,100])))
    print("  점프 >=0.10 m 비율 좌 %.1f%% 우 %.1f%%"%(100*(jl>=0.10).mean(),100*(jr>=0.10).mean()))
