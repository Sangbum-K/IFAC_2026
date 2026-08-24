import pickle, numpy as np
for tag in ('d43','d47'):
    D=pickle.load(open(tag+'.pkl','rb'))
    ak=D['/ackermann_cmd']; au=D['/autonomy/ackermann_cmd']; tel=D['/teleop']
    print("===",tag)
    print(" teleop n=%d steer range %+.3f..%+.3f"%(len(tel), tel[:,1].min() if len(tel) else 0, tel[:,1].max() if len(tel) else 0))
    m=ak[:,1]<=-0.33
    print(" /ackermann_cmd <=-0.33 at t:", np.round(ak[m,0],1)[:20], "n=",m.sum())
    # autonomy saturation episodes
    for lim,side in ((0.3950,'좌'),(-0.3220,'우')):
        mm = au[:,1]>=lim-1e-4 if lim>0 else au[:,1]<=lim+1e-4
        print("  %s 포화 %d cycles (%.2f%%)"%(side,mm.sum(),100*mm.sum()/len(au)))
