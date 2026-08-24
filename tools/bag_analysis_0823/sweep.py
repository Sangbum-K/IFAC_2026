import recon, copy, numpy as np
BASE = recon.P
cases = [
 ("현행 (기준)", {}),
 ("A: 코너 감속만 (도착속도 3.5 상한)", {"_force_varr":3.5}),
 ("B1: 벽 추적예비 상한 0.22→0.15", {"trk_max":0.15}),
 ("B2: 불확실성 상한 0.12→0.06", {"max_unc":0.06}),
 ("B3: 벽 최소 0.20→0.17", {"bound_mindist":0.17}),
 ("B1+B2", {"trk_max":0.15,"max_unc":0.06}),
 ("B1+B2+B3", {"trk_max":0.15,"max_unc":0.06,"bound_mindist":0.17}),
 ("A+B1+B2", {"_force_varr":3.5,"trk_max":0.15,"max_unc":0.06}),
]
# patch apex to honour _force_varr
orig_apex = recon.apex
import math
def apex2(side,o,R,cur_s,v0,p):
    if "_force_varr" in p: v0=min(v0,p["_force_varr"])
    return orig_apex(side,o,R,cur_s,v0,p)
recon.apex = apex2

for tag in ("d43","d47"):
    print("\n=== %s"%tag)
    base=None
    for name,ov in cases:
        p=dict(BASE); p.update(ov)
        s,_=recon.run(tag,p)
        if base is None: base=s
        print("  %-34s 한쪽이상 가능 %4d/%4d = %5.1f%%  (좌 %5.1f%% 우 %5.1f%%)  Δ=%+.1f%%p"%(
            name,s['any'],s['n'],100*s['any']/s['n'],100*s['L']/s['n'],100*s['R']/s['n'],
            100*(s['any']-base['any'])/s['n']))
