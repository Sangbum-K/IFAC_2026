import pickle,numpy as np,re
D=pickle.load(open('data.pkl','rb'))
fr=D['/car_state/frenet/odom']; gw=D['global_wpnts'][-1][1]
ft=fr[:,0]; fs=fr[:,1]; fd=fr[:,2]; fvv=fr[:,4]
gsx=gw[:,0]; dR=np.abs(gw[:,4]); dL=np.abs(gw[:,5]); L=39.45
rows=[]
for t,l,n,m in D['rosout']:
    if n=='spliner_node' and '[trace]' in m:
        tail=m.split('|')[-1]; md=re.search(r'\| ([a-z_]+) \|',m)
        if md and md.group(1)=='trailing':
            rows.append((t,'trailing_d_goal_zeroed' in tail))
r=np.array(rows)
z=r[:,1]>0.5
def wall(ts):
    s=np.interp(ts,ft,fs)%L; d=np.interp(ts,ft,fd)
    i=np.clip((s/0.1001).astype(int),0,len(gw)-1)
    # 차 중심에서 가까운 쪽 벽까지 (반폭 0.15 차감 전)
    return np.minimum(dL[i]-d, dR[i]+d)
for lab,sel in (("횡목표 포기(hold 후보)",z),("그 외 트레일링",~z)):
    w=wall(r[sel,0]); v=np.interp(r[sel,0],ft,fvv)
    print("%-22s n=%4d  벽여유 p10=%.3f p50=%.3f  차반폭0.15 미만 %.1f%%  차속 p50=%.2f"
          % (lab,sel.sum(),np.percentile(w,10),np.percentile(w,50),100*np.mean(w<0.15),np.median(v)))
w=wall(r[z,0])
print()
print("횡목표 포기 사이클 중 차 중심이 이미 트랙 경계 **밖**(여유<0): %.1f%%" % (100*np.mean(w<0)))
print("여유 0.15~0.30 (반폭은 들어가나 마진 없음)           : %.1f%%" % (100*np.mean((w>=0.15)&(w<0.30))))
