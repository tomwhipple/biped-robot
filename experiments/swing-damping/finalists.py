import sys, json, os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import swing_eval as E, scan
from design_search import agg
tab = json.load(open(sys.argv[1]))
cands = [("none", None), ("preload2 only", dict(sr=1e-9, sb=2.0)), ("preload4 only", dict(sr=1e-9, sb=4.0))] + [(f"top{i}", t[1]) for i, t in enumerate(tab[:int(sys.argv[2])])]
SEEDS = 6
jobs, meta = [], []
for name, sw in cands:
    for kd in (0.03, 0.05, 0.07):
        for st in "RL":
            for s in range(SEEDS):
                jobs.append((E.cal(seed=s, stance=st), dict(kd=kd), sw)); meta.append((name, kd))
res = scan.pmap(jobs)
for name, sw in cands:
    line = []
    for kd in (0.03, 0.05, 0.07):
        rr = [r for r, m in zip(res, meta) if m == (name, kd)]
        g, e, clr, gap, bad = agg(rr)
        line.append(f"kd{kd}: rms {g.round(2).tolist()} errmax {e:.1f} clr {clr:.1f} unst {sum(r['err_peak']>6 for r in rr)}/{len(rr)}")
    print(name, sw if name.startswith("top") else "", "\n   ", "\n    ".join(line), flush=True)
