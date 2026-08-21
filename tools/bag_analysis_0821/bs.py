import pickle,re,numpy as np
R=pickle.load(open("rosout2.pkl","rb"))
p=re.compile(r"추종 백스톱: 경로오차 ([\d.]+)m / 조향포화 ([\d.]+)% -> 속도 ([\d.]+) -> ([\d.]+)")
rows=[]
for t,l,n,m in R:
    x=p.search(m)
    if x: rows.append((t,float(x.group(1)),float(x.group(2)),float(x.group(3)),float(x.group(4))))
a=np.array(rows)
print(f"백스톱 발화 로그 {len(a)}건 (1초 throttle — 실제 사이클 수는 더 많음)")
print(f"  경로오차 p50 {np.median(a[:,1]):.2f} p90 {np.percentile(a[:,1],90):.2f} max {a[:,1].max():.2f} m  (문턱 0.25)")
print(f"  조향포화 p50 {np.median(a[:,2]):.0f}% (문턱 85%)")
print(f"  속도 {np.median(a[:,3]):.2f} -> {np.median(a[:,4]):.2f} (중앙), 낙폭 p50 {np.median(a[:,3]-a[:,4]):.2f} p90 {np.percentile(a[:,3]-a[:,4],90):.2f} m/s")
clamp=np.abs(a[:,4]-1.5)<0.02
print(f"  결과가 정확히 1.50 = 하한 클램프가 물린 것: {clamp.sum()}건 ({clamp.mean()*100:.0f}%)")
# 어떤 분기?
err_only=(a[:,1]>0.25)&(a[:,2]<=85)
sat_only=(a[:,1]<=0.25)&(a[:,2]>85)
both=(a[:,1]>0.25)&(a[:,2]>85)
print(f"  분기: 경로오차만 {err_only.sum()} / 조향포화만 {sat_only.sum()} / 둘 다 {both.sum()}")
# 하한이 없었다면 어디까지 내려갔을까 (램프 목표를 0 으로 두고 재계산)
# cap = v - over*(v - 1.5)  ->  over = (v-cap)/(v-1.5)
ok=(a[:,3]-1.5)>0.05
over=(a[ok,3]-a[ok,4])/(a[ok,3]-1.5)
cap0=a[ok,3]*(1-over)
print(f"\n  하한을 0 으로 두면 같은 램프가 낼 속도: p50 {np.median(cap0):.2f} p10 {np.percentile(cap0,10):.2f} min {cap0.min():.2f} m/s")
print(f"  그중 0.5 m/s 미만이 되는 비율 {np.mean(cap0<0.5)*100:.0f}%,  1.0 미만 {np.mean(cap0<1.0)*100:.0f}%")
# 접촉 시각 부근
for tc in (475.1,231.0,648.6,68.2):
    m=np.abs(a[:,0]-tc)<2.5
    print(f"  t={tc}: 부근 백스톱 {m.sum()}건" + (f", 최근 {a[m][-1,3]:.2f}->{a[m][-1,4]:.2f} (오차 {a[m][-1,1]:.2f})" if m.sum() else ""))
