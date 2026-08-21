import pickle, numpy as np
D=pickle.load(open("d.pkl","rb"))
print("keys sample:", [k for k in D if k.startswith("/pitwall")][:5])
# laps from frenet odom s wrap
fo=D["/car_state/frenet/odom"]   # t,x=s,y=d,yaw,v
print("frenet odom", fo.shape)
s=fo[:,1]; t=fo[:,0]
wrap=np.where(np.diff(s)<-5)[0]
print("laps:",len(wrap))
lt=np.diff(t[wrap])
print("lap times:", " ".join(f"{x:.2f}" for x in lt))
print("track_len approx", s.max())
# outcome counters final
for k in sorted(D):
    if k.startswith("/pitwall/outcome") or k.startswith("/pitwall/reject"):
        a=D[k]
        if len(a) and a[-1,1]>0: print(f"{k:56s} {a[-1,1]:.0f}")
