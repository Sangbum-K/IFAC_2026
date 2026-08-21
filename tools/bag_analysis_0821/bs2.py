import pickle,re,numpy as np
R=pickle.load(open("rosout2.pkl","rb"))
D=pickle.load(open("d.pkl","rb"))
p=re.compile(r"추종 백스톱: 경로오차 ([\d.]+)m / 조향포화 ([\d.]+)% -> 속도 ([\d.]+) -> ([\d.]+)")
a=np.array([(t,float(x.group(1)),float(x.group(2)),float(x.group(3)),float(x.group(4)))
            for t,l,n,m in R if (x:=p.search(m))])
print("경로오차 구간별 백스톱 발화:")
print(f"{'오차[m]':>12} {'n':>4} {'속도 전':>7} {'속도 후':>7} {'1.50 클램프':>11}")
for lo,hi in ((0.12,0.20),(0.20,0.25),(0.25,0.35),(0.35,0.50),(0.50,1.0)):
    m=(a[:,1]>=lo)&(a[:,1]<hi)
    if m.sum()==0: continue
    cl=np.abs(a[m,4]-1.5)<0.02
    print(f"  {lo:.2f}~{hi:.2f} {m.sum():4d} {np.median(a[m,3]):7.2f} {np.median(a[m,4]):7.2f} {cl.sum():5d} ({cl.mean()*100:3.0f}%)")

# 제안: 오차 의존 바닥  floor(e) = 1.5 * clamp((E2-e)/(E2-E1),0,1),  E1=0.35 E2=0.70
E1,E2=0.35,0.70
fl=1.5*np.clip((E2-a[:,1])/(E2-E1),0,1)
ok=(a[:,3]-1.5)>0.05
over=np.zeros(len(a)); over[ok]=(a[ok,3]-a[ok,4])/(a[ok,3]-1.5)
newcap=np.maximum(a[:,3]-over*(a[:,3]-fl), fl)
print(f"\n제안(오차 {E1}m 까지 1.5 유지 -> {E2}m 에서 0):")
print(f"  발화 {len(a)}건 중 값이 바뀌는 것 {int((np.abs(newcap-a[:,4])>0.02).sum())}건 ({np.mean(np.abs(newcap-a[:,4])>0.02)*100:.0f}%)")
m=np.abs(newcap-a[:,4])>0.02
if m.sum():
    print(f"  그 건들의 경로오차 p50 {np.median(a[m,1]):.2f} m, 속도 {np.median(a[m,4]):.2f} -> {np.median(newcap[m]):.2f}")
print(f"  0.5 m/s 미만이 되는 비율 {np.mean(newcap<0.5)*100:.0f}% (하한 폐기 시 40%)")

# BlendGeomFloor 교차속도
import math
for kt,name in ((0.0,"직선"),(0.40,"코너 kappa 0.4")):
    ksm=1.14; m_=0.8
    spare=m_*max(ksm-kt, 0.15*ksm)
    v=math.sqrt(8.83/spare)
    print(f"\n{name}: 기하하한 sqrt(6A/{spare:.2f}) vs 그립 v*sqrt(6A/8.83) 교차 v = {v:.2f} m/s")
    print(f"   -> 그 아래로 감속해도 필요 조주거리는 안 줄어든다")

# trailing 이 정확히 1.5 를 발행한 비율
tr=pickle.load(open("trace.pkl","rb"))
ack=D["/ackermann_cmd"]
def cmd(t):
    i=np.searchsorted(ack[:,0],t); i=min(max(i,0),len(ack)-1); return ack[i,2]
c=np.array([cmd(r[0]) for r in tr if r[13]=="trailing"])
print(f"\n[참고] trailing 사이클 {len(c)}개 중 명령이 정확히 1.50±0.02 : {int((np.abs(c-1.5)<0.02).sum())} ({np.mean(np.abs(c-1.5)<0.02)*100:.0f}%)")
print(f"        정확히 2.50±0.02 (trailing_retry_speed) : {int((np.abs(c-2.5)<0.02).sum())} ({np.mean(np.abs(c-2.5)<0.02)*100:.0f}%)")
