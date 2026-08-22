#!/usr/bin/env python3
"""새 게이트로 발진하는 사이클의 실제 노출: 전방 + 횡중첩 장애물까지의 간격"""
import pickle,re,sys,numpy as np
L=pickle.load(open(sys.argv[1],"rb"))
HALF=0.15   # 차체 반폭 (30 cm 차)
pat=re.compile(r"s=([-\d.]+) v=([-\d.]+) d=([-\d.]+) \| obs id=(-?\d+) dist=([-\d.]+) d\[([-\d.]+),([-\d.]+)\].*?\| (\w+) \| (.*)$")
fwd=[];lat_clear=0;tot=0
for t,lvl,n,m in L:
    if "[trace]" not in m: continue
    g=pat.search(m)
    if not g: continue
    v=float(g.group(2)); ego_d=float(g.group(3)); dist=float(g.group(5))
    odr,odl=float(g.group(6)),float(g.group(7)); mode=g.group(8); tags=g.group(9)
    if abs(v)>=0.10: continue
    if not (mode=="trailing" and "trailing_retry_clamp_raised" in tags): continue
    tot+=1
    overlap = (ego_d-HALF) < odl and (ego_d+HALF) > odr
    if not overlap: lat_clear+=1; continue
    if dist>0: fwd.append(dist)
print("새로 발진하는 정지 사이클 %d"%tot)
print("  횡으로 이미 비켜 있음(정면 아님): %d (%.1f%%)"%(lat_clear,100*lat_clear/tot))
print("  정면(횡중첩) + 전방: %d (%.1f%%)"%(len(fwd),100*len(fwd)/tot))
if fwd:
    f=np.array(fwd)
    print("    간격 p5 %.2f p25 %.2f p50 %.2f p90 %.2f m"%(
        np.percentile(f,5),np.percentile(f,25),np.median(f),np.percentile(f,90)))
    for th in (0.78,0.50,0.30):
        print("    < %.2f m: %d (%.1f%% of 전체 발진 사이클)"%(th,int((f<th).sum()),100*(f<th).sum()/tot))
