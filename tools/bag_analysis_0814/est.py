import pickle,numpy as np
D=pickle.load(open('data.pkl','rb'))
ot=D['otwpnts']; fr=D['/car_state/frenet/odom']
ft=fr[:,0]; fv=fr[:,4]
ev=[]; prev=-9
for t,a in ot:
    if len(a)<20 or t<40 or t>478: continue
    v0=float(np.interp(t,ft,fv)); head=float(a[:5,8].min())
    if v0>2.5 and head<0.7*v0 and t-prev>0.7:
        ev.append((t,v0,head)); prev=t
print("머리 낙폭 이벤트 (차속>2.5, 머리<70퍼): %d회" % len(ev))
cost=[]
for t,v0,head in ev:
    m=(ft>=t)&(ft<=t+0.8)
    if m.sum()<5: continue
    vv=fv[m]; tt=ft[m]; dts=np.r_[np.diff(tt),0]
    cost.append((np.sum(dts*np.clip(1-vv/v0,0,1)), float(vv.min()), v0))
c=np.array(cost)
print("  이벤트당 손실 중앙 %.3fs / 총 %.1fs / 랩당 %.2fs" % (np.median(c[:,0]), c[:,0].sum(), c[:,0].sum()/34))
print("  속도 저점 중앙 %.2f  (진입 차속 중앙 %.2f)" % (np.median(c[:,1]), np.median(c[:,2])))
gw=D['global_wpnts'][-1][1]; gs=gw[:,0]; gv=gw[:,8]; L=39.45
m=(ft>40)&(ft<478); t=ft[m]; s=fr[m,1]; v=fv[m]
dt=np.clip(np.r_[np.diff(t),0],0,0.15)
vref=np.interp(s%L,gs,gv); lost=dt*np.clip(1-v/np.maximum(vref,0.1),0,1)
zone=(s>11)&(s<18)
wrap=np.where(np.diff(s)<-20)[0]; bnd=np.r_[0,wrap+1,len(s)]
multi={4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,27,31}
lm=[];ln=[]
for k in range(len(bnd)-1):
    sl=slice(bnd[k],bnd[k+1]); z=zone[sl]
    if z.sum()<10: continue
    (lm if k in multi else ln).append(lost[sl][z].sum())
print()
print("s[11,18] 랩당 손실시간:  다중검출 랩 %d개 중앙 %.2fs  /  아닌 랩 %d개 중앙 %.2fs" % (len(lm),np.median(lm),len(ln),np.median(ln)))
