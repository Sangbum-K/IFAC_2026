import pickle, numpy as np
D=pickle.load(open('d43.pkl','rb'))
fr=D['/car_state/frenet/odom']
t,v=fr[:,0],fr[:,4]
dv=np.gradient(v,t)
# 장애물 접근 감속 구간: 감속 중 & v>2
m=(dv<-0.5)&(v>2.0)
print("전 구간 감속률 p50/p90/max = %.2f/%.2f/%.2f m/s²"%tuple(-np.percentile(dv[m],[50,10,0])))
for t0,t1,lab in ((88.8,90.4,'89.6 충돌 접근'),(123.4,124.6,'124 접근'),(131.5,133.2,'132 접근'),(150.6,151.8,'151 접근')):
    s=(t>=t0)&(t<=t1)
    if s.sum()>3:
        a=(v[s][-1]-v[s][0])/(t[s][-1]-t[s][0])
        print("  %-14s v %.2f -> %.2f, 평균 %.2f m/s², 최대 %.2f"%(lab,v[s][0],v[s][-1],a,-dv[s].min()))
