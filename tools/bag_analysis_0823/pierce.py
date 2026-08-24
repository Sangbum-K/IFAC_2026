import pickle,numpy as np,recon
HW=0.15
for tag in ('d43','d47'):
    D=pickle.load(open(tag+'.pkl','rb'))
    R=recon.Rec(tag)
    lw=D['local_wpnts']; fr=D['/car_state/frenet/odom']
    O={t:o for t,o in R.obs}
    ots=np.array(sorted(O))
    nfr=0; npr=0; worst=0; worstt=0; ego_hit=0; negs=[]
    for t,W in lw:
        if len(W)<3: continue
        j=int(np.argmin(np.abs(ots-t)))
        if abs(ots[j]-t)>0.1: continue
        obs=[o for o in O[ots[j]] if o['is_static']]
        if not obs: continue
        f=R.at(fr,t); nfr+=1
        pen=0
        for o in obs:
            ss,se=o['s_start'],o['s_end']
            for s_,d_ in W[:,:2]:
                if ((s_-ss)%R.L)<=((se-ss)%R.L):
                    p=min(o['d_left']-(d_-HW), (d_+HW)-o['d_right'])
                    if p>0: pen=max(pen,p)
        if pen>0:
            npr+=1
            if pen>worst: worst,worstt=pen,t
        # 자차 실제 침투
        for o in obs:
            if ((f[1]-o['s_start'])%R.L)<=((o['s_end']-o['s_start'])%R.L):
                p=min(o['d_left']-(f[2]-HW),(f[2]+HW)-o['d_right'])
                if p>0: ego_hit+=1; negs.append((t,p,f[4]))
    print("=== %s  정적장애물 동반 프레임 %d"%(tag,nfr))
    print("   발행 경로가 장애물 박스를 관통: %d (%.1f%%), 최대 침투 %.2f m (t=%.2f)"%(npr,100*npr/nfr,worst,worstt))
    print("   자차 차체가 박스 안: %d 프레임"%ego_hit)
    if negs:
        a=np.array(negs)
        # 접촉 에피소드
        eps=[]; cur=[negs[0]]
        for r in negs[1:]:
            if r[0]-cur[-1][0]>1.0: eps.append(cur); cur=[r]
            else: cur.append(r)
        eps.append(cur)
        print("   접촉 에피소드 %d개:"%len(eps))
        for e in eps:
            a=np.array(e); print("      t=%.1f~%.1f  최대침투 %.2f m  접촉속도 %.2f m/s"%(a[0,0],a[-1,0],a[:,1].max(),a[:,2].max()))
