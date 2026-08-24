import pickle, numpy as np
for tag in ('d43','d47'):
    D=pickle.load(open(tag+'.pkl','rb'))
    le=D['/pitwall/lat_err']; fr=D['/car_state/frenet/odom']
    gw=D['global_wpnts'][0][1]; L=gw[-1,0]+(gw[-1,0]-gw[-2,0]); step=gw[1,0]-gw[0,0]
    v=np.interp(le[:,0], fr[:,0], fr[:,4])
    e=np.abs(le[:,1])
    print("=== %s  lat_err n=%d"%(tag,len(le)))
    for lo,hi in ((0,2),(2,3.5),(3.5,5),(5,7)):
        m=(v>=lo)&(v<hi)
        if m.sum()>20:
            print("   v %3.1f~%3.1f m/s: |lat_err| p50 %.3f p90 %.3f p99 %.3f max %.3f  (n=%d)"%(
                lo,hi,*np.percentile(e[m],[50,90,99,100]),m.sum()))
    # 실제 벽 여유 (글로벌 폭 기준)
    idx=((fr[:,1]%L)/step).astype(int)%len(gw)
    d=fr[:,2]
    clr=np.where(d>=0, gw[idx,5]-d, gw[idx,4]+d) - 0.15
    print("   실제 벽 여유(차체 가장자리) p1/p5/p50 = %.3f/%.3f/%.3f m, 음수 %.2f%%"%(
        *np.percentile(clr,[1,5,50]), 100*(clr<0).mean()))
    print("   |d| p90 = %.2f, 트랙 반폭 p50 좌 %.2f 우 %.2f"%(np.percentile(abs(d),90), np.median(gw[:,5]), np.median(gw[:,4])))
