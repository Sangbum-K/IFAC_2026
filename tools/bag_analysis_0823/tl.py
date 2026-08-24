import pickle, numpy as np, sys
D=pickle.load(open('d43.pkl','rb'))
gw=D['global_wpnts'][0][1]; L=gw[-1,0]+(gw[-1,0]-gw[-2,0])
fr=D['/car_state/frenet/odom']  # t, s, d, yaw?, v
au=D['/autonomy/ackermann_cmd']
imu=D['/sensors/imu/raw']  # t,yaw,wx,wy,wz,ax,ay,az
core=D['/sensors/core']
def at(a,t):
    i=np.searchsorted(a[:,0],t); return a[min(max(i,0),len(a)-1)]
t0,t1=float(sys.argv[1]),float(sys.argv[2])
print(" t      s      d     v   cmd_v  cmd_st  |  ax    ay    wz  | erpm")
for t in np.arange(t0,t1,0.1):
    f=at(fr,t); a=at(au,t); im=at(imu,t); c=at(core,t)
    print("%6.2f %6.2f %+6.3f %5.2f %5.2f %+7.4f | %+5.2f %+5.2f %+5.2f | %7.0f"%(
        t,f[1],f[2],f[4],a[2],a[1],im[5],im[6],im[4],c[3]))
