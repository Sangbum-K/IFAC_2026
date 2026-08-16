import pickle,numpy as np,re
D=pickle.load(open('data.pkl','rb'))
rows=[]
for t,l,n,m in D['rosout']:
    if n=='spliner_node' and '[trace]' in m:
        tail=m.split('|')[-1]
        v=re.search(r'v=([-\d.]+)',m); d=re.search(r'd=([-\d.]+) \|',m)
        rows.append((t,'lane_side_switch_blocked_mid_evasion' in tail,
                     float(v.group(1)) if v else -1, float(d.group(1)) if d else 0))
rt=np.array([r[0] for r in rows]); blk=np.array([r[1] for r in rows])
print("봉쇄 태그가 뜬 사이클: %d / %d" % (blk.sum(), len(rows)))
ch=np.diff(blk.astype(int)); st=rt[1:][ch==1]; en=rt[1:][ch==-1]
if len(en) and len(st) and en[0]<st[0]: en=en[1:]
n=min(len(st),len(en)); dur=en[:n]-st[:n]
print("연속 봉쇄 구간: %d개  p50=%.2fs p90=%.2fs max=%.2fs  총 %.1fs" %
      (n, np.median(dur), np.percentile(dur,90), dur.max(), dur.sum()))
print("  1초 이상: %d개, 합 %.1fs" % ((dur>1).sum(), dur[dur>1].sum()))
print()
print("7:23 창(442.0~444.6) 봉쇄 구간:")
for a,b in zip(st[:n],en[:n]):
    if 441.5<a<444.8: print("   %.2f ~ %.2f  (%.2fs)" % (a,b,b-a))
print()
print("7:42 창(461.5~467.2) 봉쇄 구간:")
for a,b in zip(st[:n],en[:n]):
    if 461.0<a<467.5: print("   %.2f ~ %.2f  (%.2fs)" % (a,b,b-a))
# 봉쇄 중 이동거리
fr=D['/car_state/frenet/odom']; ft=fr[:,0]; fs=fr[:,1]
tot=0
for a,b in zip(st[:n],en[:n]):
    sa=np.interp(a,ft,fs); sb=np.interp(b,ft,fs); ds=(sb-sa)%39.45
    if ds<20: tot+=ds
print()
print("봉쇄 상태로 흘려보낸 총 종방향 거리: %.1f m (구간당 평균 %.2f m)" % (tot, tot/n))
