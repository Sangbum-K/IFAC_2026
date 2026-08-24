import pickle, numpy as np
for tag in ('d43','d47'):
    D=pickle.load(open(tag+'.pkl','rb'))
    sv=D['/commands/servo/position']; ak=D['/ackermann_cmd']; au=D['/autonomy/ackermann_cmd']
    fr=D['/car_state/frenet/odom']
    mv=fr[:,4] if fr.shape[1]>4 else None
    print("=== %s  dur=%.1f  servo n=%d"%(tag, sv[-1,0], len(sv)))
    print("  servo min/max = %.4f / %.4f"%(sv[:,1].min(), sv[:,1].max()))
    for lo,hi,name in ((0.05,0.0501,'servo_min(좌 최대)'),(0.6999,0.70,'servo_max(우 최대)')):
        n=((sv[:,1]>=lo)&(sv[:,1]<=hi)).sum(); print("   %-20s %5d (%.2f%%)"%(name,n,100*n/len(sv)))
    # ackermann cmd steering distribution (col2 = steering?)
    print("  ackermann_cmd cols", ak.shape)
    st=ak[:,1]; sp=ak[:,2]
    # check which col is steering (range ~ +-0.4)
    if abs(st).max()>2: st,sp=ak[:,2],ak[:,1]
    print("  cmd steer min/max = %+.4f / %+.4f"%(st.min(), st.max()))
    for th,name in ((0.3210,'우 |δ|>=0.321 (한계)'),):
        nR=(st<=-th).sum(); nL=(st>=th).sum()
        print("   좌 δ>=+0.321: %d (%.2f%%)   우 δ<=-0.321: %d (%.2f%%)"%(nL,100*nL/len(st),nR,100*nR/len(st)))
    nL394=(st>=0.394).sum(); print("   좌 δ>=0.3949(제어 한계): %d"%nL394)
