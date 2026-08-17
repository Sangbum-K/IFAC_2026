import pickle, numpy as np
from pathlib import Path
D=pickle.load(open(Path(__file__).parent/"d124904.pkl","rb"))
def show(tt,lab):
    k=min(range(len(D["otwpnts"])),key=lambda i:abs(D["otwpnts"][i][0]-tt))
    t,W=D["otwpnts"][k]
    print(f"\n=== {lab}  t={t:.2f}  n={len(W)} ===")
    print("  s      d      vx")
    for r in W[::max(1,len(W)//45)]:
        print(f"  {r[0]:6.2f} {r[1]:+6.2f} {r[8]:5.2f}")
show(59.25,"lap4 회피선 채택 순간 (93점)")
show(15.95,"lap0 회피선 (97점)")
