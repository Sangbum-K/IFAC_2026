import pickle,numpy as np,collections
D=pickle.load(open("d.pkl","rb"))
L=36.833
tracks=collections.defaultdict(list)
for t,o in D["obs_frenet"]:
    for x in o:
        tracks[x["id"]].append((t,x["s_center"],x["d_center"],x["vs"],x["vd"],x["is_static"]))
rep_d=[];fd_d=[];rep_s=[];fd_s=[]
for i,v in tracks.items():
    if len(v)<40: continue
    a=np.array(v)
    t=a[:,0]; s=np.unwrap(a[:,1],period=L); d=a[:,2]; vs=a[:,3]; vd=a[:,4]
    # 0.3s 중심차분
    for k in range(len(t)):
        lo=np.searchsorted(t,t[k]-0.15); hi=np.searchsorted(t,t[k]+0.15)
        if hi-lo<4 or hi>=len(t): continue
        dt=t[hi]-t[lo]
        if dt<0.2: continue
        fd_d.append((d[hi]-d[lo])/dt); rep_d.append(vd[k])
        fd_s.append((s[hi]-s[lo])/dt); rep_s.append(vs[k])
rep_d=np.array(rep_d);fd_d=np.array(fd_d);rep_s=np.array(rep_s);fd_s=np.array(fd_s)
print(f"표본 {len(rep_d)} (관측 40+ 트랙)")
def rep(name,r,f):
    print(f"{name}: 보고 |v| p50 {np.median(np.abs(r)):.2f} p90 {np.percentile(np.abs(r),90):.2f} / 실제(0.3s 차분) |v| p50 {np.median(np.abs(f)):.2f} p90 {np.percentile(np.abs(f),90):.2f}")
    m=np.abs(f)>0.3
    if m.sum():
        print(f"   실제 |v|>0.3 인 {m.sum()}표본에서 보고/실제 비율 중앙 {np.median(np.abs(r[m])/np.abs(f[m])):.2f}, 부호 일치 {np.mean(np.sign(r[m])==np.sign(f[m]))*100:.0f}%")
    print(f"   상관 {np.corrcoef(r,f)[0,1]:.2f}")
rep("vd(횡)",rep_d,fd_d)
rep("vs(종)",rep_s,fd_s)
# 횡 이동량: 1초 동안 상대 d 가 얼마나 변하나
print("\n상대 d 의 1초 변화량:")
ch=[]
for i,v in tracks.items():
    if len(v)<60: continue
    a=np.array(v); t=a[:,0]; d=a[:,2]
    for k in range(len(t)):
        hi=np.searchsorted(t,t[k]+1.0)
        if hi>=len(t): break
        ch.append(abs(d[hi]-d[k]))
ch=np.array(ch)
print(f"  |Δd(1s)| p50 {np.median(ch):.2f} p90 {np.percentile(ch,90):.2f} max {ch.max():.2f} m   (evasion_dist 0.35)")
print(f"  1초 안에 0.35m 이상 횡이동한 비율 {np.mean(ch>0.35)*100:.0f}%")
