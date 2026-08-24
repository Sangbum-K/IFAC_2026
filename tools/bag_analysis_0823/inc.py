import pickle,numpy as np,recon
D=pickle.load(open('d43.pkl','rb'))
au=D['/autonomy/ackermann_cmd']; fr=D['/car_state/frenet/odom']
imu=D['/sensors/imu/raw']
R=recon.Rec('d43')
EV=[(81.07,-0.57),(89.60,-0.57),(98.53,-0.52),(106.53,-0.50),(114.64,-0.50),
    (124.13,+0.24),(131.97,-0.52),(142.33,-0.49),(151.07,-0.50)]
print("t        진행방향 | 좌통로 우통로 | 선택 | 조향포화(±0.5s) 좌/우 | 최대|δ| | 최대 yawrate")
for t,goal in EV:
    obs=[o for tt,o in R.obs if abs(tt-t)<0.06]
    o=None
    if obs:
        f=R.at(fr,t)
        cand=[x for x in obs[0] if x['is_static'] and 0<=((x['s_start']-f[1])%R.L)<=12]
        if cand: o=min(cand,key=lambda x:(x['s_start']-f[1])%R.L)
    m=(au[:,0]>t-0.5)&(au[:,0]<t+0.7)
    st=au[m,1]
    mi=(imu[:,0]>t-0.5)&(imu[:,0]<t+0.7)
    wz=np.abs(imu[mi,4]).max() if mi.sum() else 0
    if o is None:
        print("%6.2f  (장애물 미검출)"%t); continue
    wl,wr=R.minwall((o['s_start']-0.5)%R.L,(o['s_end']+0.5)%R.L)
    lg=wl-o['d_left']; rg=wr+o['d_right']
    print("%6.2f  %s | %.2f  %.2f | %s | 좌%d 우%d /%d | %+.3f | %.0f deg/s"%(
        t, "우로" if goal<0 else "좌로", lg, rg,
        "넓은쪽" if ((goal<0)==(rg>lg)) else "**좁은쪽**",
        (st>=0.3949).sum(),(st<=-0.3219).sum(),len(st), st[np.argmax(np.abs(st))], wz))
