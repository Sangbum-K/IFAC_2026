import pickle,numpy as np
D=pickle.load(open('data.pkl','rb'))
ot=D['otwpnts']; fr=D['/car_state/frenet/odom']; gw=D['global_wpnts'][-1][1]
ft=fr[:,0]; fv=fr[:,4]; gs=gw[:,0]; gv=gw[:,8]; L=39.45
ALAT=8.83; A_ACC=3.0
rows=[]
for t,a in ot:
    if len(a)<20 or t<40 or t>478: continue
    v0=float(np.interp(t,ft,fv)); head=float(a[0,8])
    d=np.cumsum(np.r_[0,np.hypot(np.diff(a[:,2]),np.diff(a[:,3]))])
    base=np.interp(a[:,0]%L,gs,gv)
    kap=np.abs(a[:,7])
    cap_k=np.sqrt(ALAT/np.maximum(kap[0],1e-4))
    # 경로 전체가 head 에서 출발한 가속 envelope 인가?
    env=np.sqrt(head*head+2*A_ACC*d)
    m=d<=4.0
    fit=float(np.median(np.abs(a[m,8]-env[m])/np.maximum(env[m],.1)))
    rows.append((v0,head,base[0],cap_k,fit,float(a[:,8].min()),float(np.argmin(a[:,8]))))
r=np.array(rows)
dip=r[:,1]<0.7*np.maximum(r[:,0],.1)
print("전체 %d · 머리 낙폭 사이클 %d (%.1f%%)" % (len(r),dip.sum(),100*dip.mean()))
print()
print("낙폭 사이클에서 머리 속도 vs 각 후보 (중앙값)")
print("  실제 머리 속도            %.2f" % np.median(r[dip,1]))
print("  차속 cur_vs               %.2f" % np.median(r[dip,0]))
print("  레이스라인 base           %.2f" % np.median(r[dip,2]))
print("  머리 곡률캡 sqrt(a/k)     %.2f" % np.median(r[dip,3]))
print()
print("  머리가 경로 최소값인 비율        %.1f%%" % (100*np.mean(r[dip,6]<3)))
print("  경로가 head 기준 가속 envelope 와 ±10%% 일치  %.1f%%" % (100*np.mean(r[dip,4]<0.10)))
print()
print("낙폭 아닌 사이클: 머리 %.2f / 차속 %.2f / envelope 일치 %.1f%%"
      % (np.median(r[~dip,1]),np.median(r[~dip,0]),100*np.mean(r[~dip,4]<0.10)))
