"""Structured one-term-at-a-time scan of the swing law with the swing leg preloaded (sb 3 deg), against preload alone."""
import sys, json, os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import swing_eval as E, scan
from design_search import agg
cands = [("preload only", dict(sr=1e-9, sb=3.0))]
for joint, (sh, sa) in (("hip", (1.0, 0.0)), ("ankle", (0.0, 1.0))):
    for term, vals in (("sr", (-0.3, -0.1, -0.03, 0.03, 0.1, 0.3)), ("se", (-3, -1, -0.3, 0.3, 1, 3)), ("si", (-10, -3, -1, 1, 3, 10))):
        for v in vals:
            cands.append((f"{joint} {term}={v}", {term: v, "sh": sh, "sa": sa, "sb": 3.0, "tau_w": 1.0, "lim": 8.0}))
SEEDS = 6; KDS = (0.03, 0.05, 0.07)
jobs, meta = [], []
for name, sw in cands:
    for kd in KDS:
        for st in "RL":
            for s in range(SEEDS):
                jobs.append((E.cal(seed=s, stance=st), dict(kd=kd), sw)); meta.append((name, kd))
res = scan.pmap(jobs)
out = {}
for name, sw in cands:
    line = []
    for kd in KDS:
        rr = [r for r, m in zip(res, meta) if m == (name, kd)]
        g, e, clr, gap, bad = agg(rr)
        out[f"{name}|{kd}"] = dict(rms=g.round(2).tolist(), err=e, clr=clr, gap=gap, unst=sum(r['err_peak'] > 6 for r in rr), usw=max(r['usw_max'] for r in rr))
        line.append(f"kd{kd}: {g.round(2).tolist()} e{e:.1f} unst{sum(r['err_peak']>6 for r in rr)} usw{max(r['usw_max'] for r in rr):.1f}")
    print(f"{name:18s}", " | ".join(line), flush=True)
json.dump(out, open("/home/claw/code/robot-swing-damp/build/swing/structured.json", "w"), indent=1)
