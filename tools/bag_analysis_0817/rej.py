import pickle, numpy as np
from pathlib import Path
D=pickle.load(open(Path(__file__).parent/"d124904.pkl","rb"))
def val(k,t):
    a=D.get(k)
    if a is None or not len(a): return None
    i=np.searchsorted(a[:,0],t)-1
    return a[max(i,0),1]
keys=[k for k in D if isinstance(k,str) and k.startswith("/pitwall/reject")]
print("리젝트 카운터 증분")
for W in [(56.0,58.9),(58.9,60.1),(60.1,61.6)]:
    print(f"\n--- {W[0]}~{W[1]}s ---")
    rows=[]
    for k in keys:
        a,b=val(k,W[0]),val(k,W[1])
        if a is not None and b is not None and b-a>0: rows.append((b-a,k))
    for d,k in sorted(rows,reverse=True): print(f"   +{d:5.0f}  {k.replace('/pitwall/reject_','')}")
print("\n/pitwall/evasion_d_apex")
a=D.get("/pitwall/evasion_d_apex")
for t,v in a:
    if 50<=t<=62: print(f"   {t:6.2f}  {v:+.3f}")
print("\n/pitwall/avoidance_loss_hold (55-62)")
a=D.get("/pitwall/avoidance_loss_hold")
for t,v in a:
    if 55<=t<=62: print(f"   {t:6.2f}  {v:.2f}")
