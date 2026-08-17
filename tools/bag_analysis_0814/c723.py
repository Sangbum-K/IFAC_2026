import pickle,numpy as np
D=pickle.load(open('data.pkl','rb'))
fr=D['/car_state/frenet/odom']; obs=D['obs_frenet']; gw=D['global_wpnts'][-1][1]
ft=fr[:,0]; fs=fr[:,1]; fd=fr[:,2]; fvv=fr[:,4]; L=39.45
ots=np.array([x[0] for x in obs])
print("t      차s     차d    v   |  전방 8m 장애물 (거리, d스팬)                     | 통과가능 틈")
for t in np.arange(441.0,445.4,0.25):
    j=int(np.argmin(abs(ft-t))); i=int(np.argmin(abs(ots-t)))
    cs,cd,cv=fs[j],fd[j],fvv[j]
    ol=[]
    for o in obs[i][1]:
        g=(o['s_center']-cs)%L
        if g<8: ol.append((g,o['d_right'],o['d_left'],o['id']))
    ol.sort()
    txt=' ; '.join("+%.1f d[%+.2f,%+.2f]"%(g,a,b) for g,a,b in [(x[0],x[1],x[2]) for x in ol])
    # 가장 가까운 장애물 기준 좌우 여유 (트랙 벽 사용)
    gap=''
    if ol:
        gi=int(round((cs+ol[0][0])/0.1001))%len(gw)
        wl=abs(gw[gi,5]); wr=abs(gw[gi,4])
        near=[x for x in ol if x[0]<ol[0][0]+1.5]
        lo=min(x[1] for x in near); hi=max(x[2] for x in near)
        # 같은 s 근처 장애물이 2개면 사이 틈도
        left=wl-hi; right=wr+lo
        gap="L:%.2f R:%.2f"%(left,right)
        if len(near)>1:
            ds=sorted(near,key=lambda x:x[1])
            between=ds[1][1]-ds[0][2]
            gap+=" 사이:%.2f"%between
    print("%6.2f %6.2f %+.2f %4.2f | %-48s | %s"%(t,cs,cd,cv,txt[:48],gap))
