#!/usr/bin/env python3
"""[trace] 파싱 -> 모드/정지권한/거리"""
import pickle,re,sys,numpy as np
L=pickle.load(open(sys.argv[1],"rb"))
rows=[]
pat=re.compile(r"s=([-\d.]+) v=([-\d.]+) d=([-\d.]+) \| obs id=(-?\d+) dist=([-\d.]+).*?static=(\d).*?\| (\w+) \| (.*)$")
for t,lvl,n,m in L:
    if "[trace]" not in m: continue
    g=pat.search(m)
    if not g: continue
    rows.append(dict(t=t,s=float(g.group(1)),v=float(g.group(2)),d=float(g.group(3)),
        oid=int(g.group(4)),dist=float(g.group(5)),static=g.group(6)=="1",
        mode=g.group(7),tags=g.group(8)))
print("trace %d"%len(rows))
tr=[r for r in rows if r["mode"]=="trailing"]
st=[r for r in tr if abs(r["v"])<0.10]
raised=[r for r in st if "trailing_retry_clamp_raised" in r["tags"]]
print("trailing %d | 그중 정지중(v<0.1) %d | 그중 정지권한 박탈(clamp_raised) %d (%.1f%%)"%(
    len(tr),len(st),len(raised),100*len(raised)/max(len(st),1)))
ever=[r for r in raised if "trailing_stop_denied_ever_dynamic" in r["tags"]]
print("  그중 '달리다 멈춘 상대'(ever_dynamic) %d — 진짜 정지 의도 후보"%len(ever))
if raised:
    dd=np.array([r["dist"] for r in raised])
    print("  clamp_raised 정지 사이클의 장애물 거리: p5 %.2f p25 %.2f p50 %.2f p75 %.2f max %.2f"%(
        np.percentile(dd,5),np.percentile(dd,25),np.median(dd),np.percentile(dd,75),dd.max()))
    print("  거리 < 0.78 m (1.27 m/s 정지거리) 인 사이클: %d (%.1f%%)"%(
        int((dd<0.78).sum()),100*np.mean(dd<0.78)))
    print("  거리 < 0.30 m 인 사이클: %d (%.1f%%)"%(int((dd<0.30).sum()),100*np.mean(dd<0.30)))
notraised=[r for r in st if "trailing_retry_clamp_raised" not in r["tags"]]
print("  정지권한 유지(게이트 계속 닫힘) %d 사이클"%len(notraised))
