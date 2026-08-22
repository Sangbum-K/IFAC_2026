import pickle,sys,numpy as np
D=pickle.load(open(sys.argv[1],"rb"))
gw=D["global_wpnts"][0][1]   # s d x y d_r d_l psi kappa vx ax
print("global wpnts n=%d  s %.2f..%.2f"%(len(gw),gw[0,0],gw[-1,0]))
for s in [float(x) for x in sys.argv[2:]]:
    i=np.argmin(np.abs(gw[:,0]-s))
    w=gw[i]
    print("s=%6.2f  d_r=%.3f d_l=%.3f  width=%.3f  kappa=%+.3f vx=%.2f  xy=(%.2f,%.2f)"%(
        w[0],w[4],w[5],w[4]+w[5],w[7],w[8],w[2],w[3]))
