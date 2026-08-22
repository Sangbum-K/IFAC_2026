import pickle, numpy as np
D=pickle.load(open('d.pkl','rb'))
gw=D['global_wpnts'][0][1]
print("gw cols s d x y d_r d_l psi kappa vx ax ; n=",len(gw))
s=gw[:,0]
for target in (6.0,6.5,7.0,7.5,8.0,8.5):
    i=np.argmin(np.abs(s-target))
    print(f"s={gw[i,0]:5.2f} d_r={gw[i,4]:.2f} d_l={gw[i,5]:.2f} width={gw[i,4]+gw[i,5]:.2f} raceline d={gw[i,1]:+.2f} vx={gw[i,8]:.2f}")
fr=D['/car_state/frenet/odom']  # t,x=s,y=d,yaw,v
print("\n== ego frenet (s,d) ==")
m=(fr[:,0]>22.0)&(fr[:,0]<28.5)
for r in fr[m][::10]:
    print(f"t={r[0]:6.2f} s={r[1]:6.2f} d={r[2]:+.2f} v={r[4]:.2f}")
