import pickle,numpy as np
D=pickle.load(open('data.pkl','rb'))
ot=D['otwpnts']; fr=D['/car_state/frenet/odom']
ft=fr[:,0]; fv=fr[:,4]
K=np.tan(0.36)/0.33; REJ=K*1.15; ALAT=8.83
def kg(x,y,i,w):
    h0=np.arctan2(y[i]-y[i-w], x[i]-x[i-w]); h1=np.arctan2(y[i+w]-y[i], x[i+w]-x[i])
    b=0.5*np.hypot(x[i+w]-x[i-w], y[i+w]-y[i-w])
    d=(h1-h0+np.pi)%(2*np.pi)-np.pi
    return d/max(b,1e-6)
old_k=[];new_k=[];old_v=[];new_v=[];rej_o=0;rej_n=0;N=0
for t,a in ot:
    if len(a)<7 or t<40 or t>478: continue
    x,y=a[:,2],a[:,3]; N+=1
    ko=abs(kg(x,y,3,3))                       # 예전: idx3 값을 0,1,2 에 복사
    kn=max(abs(kg(x,y,1,1)),abs(kg(x,y,2,2))) # 이후: 좁힌 윈도 (머리 최대)
    old_k.append(ko); new_k.append(kn)
    old_v.append(np.sqrt(ALAT/max(ko,1e-4))); new_v.append(np.sqrt(ALAT/max(kn,1e-4)))
    rej_o += ko>REJ+1e-3; rej_n += kn>REJ+1e-3
o=np.array(old_k);n_=np.array(new_k);ov=np.array(old_v);nv=np.array(new_v)
print("경로 %d개 · 조향한계 %.3f · 거부문턱 %.3f rad/m" % (N,K,REJ))
print()
print("머리 3점 곡률 |kappa|      p50      p90      p99      max")
print("  변경 전 (idx3 복사)   %7.3f  %7.3f  %7.3f  %7.3f" % tuple(np.percentile(o,[50,90,99,100])))
print("  변경 후 (좁힌 윈도)   %7.3f  %7.3f  %7.3f  %7.3f" % tuple(np.percentile(n_,[50,90,99,100])))
print()
print("머리 횡가속 캡 sqrt(a/k) [m/s]   p10      p25      p50")
print("  변경 전                     %7.2f  %7.2f  %7.2f" % tuple(np.percentile(ov,[10,25,50])))
print("  변경 후                     %7.2f  %7.2f  %7.2f" % tuple(np.percentile(nv,[10,25,50])))
print()
print("머리가 거부 문턱을 넘어 경로 전체가 버려지는 경우:  전 %d회 -> 후 %d회" % (rej_o,rej_n))
print("곡률이 낮아진 경로 %.1f%% / 높아진 경로 %.1f%% (더 국소적이라 커질 수도 있음)"
      % (100*np.mean(n_<o), 100*np.mean(n_>o)))
print("낮아진 경우 캡 상승 중앙값 +%.2f m/s" % np.median((nv-ov)[n_<o]))
