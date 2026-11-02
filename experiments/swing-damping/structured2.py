import sys, json, os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import swing_eval as E, scan
from design_search import agg
cands = [("none", None), ("preload only", dict(sr=1e-9, sb=3.0))]
for lim in (8.0, 12.0):
  for sr in (0.3, 0.6, 1.0, 2.0):
    cands.append((f"hip sr={sr} lim{lim}", dict(sr=sr, sh=1.0, sb=3.0, lim=lim)))
for sr in (0.3, 1.0):
    cands.append((f"hip+ankle sr={sr}", dict(sr=sr, sh=1.0, sa=1.0, sb=3.0, lim=8.0)))
    cands.append((f"hip sr={sr} +se0.3", dict(sr=sr, se=0.3, sh=1.0, sb=3.0, lim=8.0)))
    cands.append((f"hip sr={sr} sb0", dict(sr=sr, sh=1.0, sb=0.0, lim=8.0)))
SEEDS = 6; KDS = (0.03, 0.05, 0.07, 0.10)
jobs, meta = [], []
for name, sw in cands:
    for kd in KDS:
        for st in "RL":
            for s in range(SEEDS):
                jobs.append((E.cal(seed=s, stance=st), dict(kd=kd), sw)); meta.append((name, kd))
res = scan.pmap(jobs)
for name, sw in cands:
    line = []
    for kd in KDS:
        rr = [r for r, m in zip(res, meta) if m == (name, kd)]
        g, e, clr, gap, bad = agg(rr)
        line.append(f"kd{kd}: {g.round(1).tolist()} e{e:.0f} clr{clr:.0f} bad{bad}")
    print(f"{name:20s}", " | ".join(line), flush=True)
