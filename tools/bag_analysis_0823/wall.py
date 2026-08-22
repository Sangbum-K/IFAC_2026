import pickle,sys,numpy as np
D=pickle.load(open(sys.argv[1],'rb'))
gb=D['global_wpnts'][0][1]   # s d x y d_r d_l psi kappa vx ax
s=gb[:,0]; dr=gb[:,4]; dl=gb[:,5]; kap=gb[:,7]; vx=gb[:,8]
print("track len", s[-1], "n", len(s))
print(" s     d_r    d_l   width  kappa  vx")
lo,hi=float(sys.argv[2]),float(sys.argv[3])
for i in range(len(s)):
    if lo<=s[i]<=hi:
        print(f"{s[i]:6.2f} {dr[i]:6.2f} {dl[i]:6.2f} {dr[i]+dl[i]:6.2f} {kap[i]:6.2f} {vx[i]:5.2f}")
