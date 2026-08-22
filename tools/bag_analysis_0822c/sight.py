import lib2, numpy as np
d=lib2.D(); m=lib2.Map(d); g=lib2.gw(d)
S,X,Y,VX=g[:,0],g[:,2],g[:,3],g[:,8]
n=len(S); TL=S[-1]+(S[1]-S[0])
def blocked(x0,y0,x1,y1):
    L=np.hypot(x1-x0,y1-y0); k=max(int(L/0.02),2)
    t=np.linspace(0,1,k)
    return np.any(m.dist(x0+t*(x1-x0), y0+t*(y1-y0))<=0.001)
sight=np.zeros(n)
for i in range(n):
    dd=0.0
    for k in range(1,n):
        j=(i+k)%n
        if blocked(X[i],Y[i],X[j],Y[j]): break
        dd=(S[j]-S[i])%TL
        if dd>15: break
    sight[i]=dd
np.save('sight.npy',np.c_[S,sight,VX])
A=3.5   # 실측 감속 (아래 참조)
LAT=0.15
vsafe=np.sqrt(np.maximum(A*A*LAT*LAT+2*A*sight,0))-A*LAT
print(" s     시야[m]  v_safe  raceline vx   초과")
worst=[]
for i in range(n):
    ex=VX[i]-vsafe[i]
    worst.append((ex,S[i],sight[i],vsafe[i],VX[i]))
for ex,s,si,vs,vx in sorted(worst,reverse=True)[:18]:
    print(f"{s:6.2f}  {si:5.2f}   {vs:5.2f}    {vx:5.2f}     {ex:+5.2f}")
print()
print("전 구간: 시야 p5 %.2f p50 %.2f  | vx>v_safe 인 웨이포인트 %.1f%%  (평균 초과 %.2f m/s)"%(
    np.percentile(sight,5),np.median(sight),100*np.mean(VX>vsafe),np.mean(np.maximum(VX-vsafe,0))))
for tgt in (2.0,3.0,4.0,5.0,6.0,7.0):
    i=int(np.argmin(np.abs(S-tgt)))
    print(f"  s={S[i]:5.2f} 시야 {sight[i]:5.2f} m -> v_safe {vsafe[i]:5.2f} / 계획 {VX[i]:5.2f}")
