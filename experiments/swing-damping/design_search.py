"""Random search over the swing-leg law on the calibrated plant (right + left stance, kd 0.03 and the hardware-like
kd 0.05 rocking), objective = rate rms after the lift-off burst plus penalties for err, clearance, contact."""
import sys, json, os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import swing_eval as E, scan

def agg(res):
    rm = np.array([r["rms"] for r in res]); g = np.exp(np.log(np.maximum(rm, .05)).mean(0))
    return g, max(r["err_peak"] for r in res), min(r["clr_min"] for r in res), min(r["gap_min"] for r in res), sum(r["fell"] or r["tripped"] for r in res)

def objective(g, e, clr, gap, bad):
    j = g[0] + 2 * g[1] + 4 * g[2]
    j += 3 * max(0, e - 3.0) + 0.5 * max(0, 11.0 - clr) + 100 * bad
    return float(j)

def main(n, seed, out):
    rng = np.random.default_rng(seed)
    S = []
    for _ in range(n):
        sw = dict(sh=float(rng.choice([1.0, 1.0, 0.0, 1.0])), sa=float(rng.choice([0.0, 0.0, 1.0, -1.0, 0.5])),
                  sr=float(rng.choice([0.0, 0.0, 0.05, -0.05, 0.1, -0.1, 0.2, -0.2, 0.4, -0.4])),
                  se=float(rng.choice([0.0, 0.0, 0.1, -0.1, 0.3, -0.3, 0.6, -0.6, 1.0, -1.0, 2.0])),
                  si=float(rng.choice([0.0, 0.0, 0.3, -0.3, 1.0, -1.0, 3.0, -3.0])), tau_w=float(rng.choice([0.3, 1.0, 3.0])),
                  sb=float(rng.choice([0.0, 2.0, 3.0, 4.0])), sba=float(rng.choice([0.0, 0.0, 2.0])), lim=float(rng.choice([4.0, 8.0])))
        if sw["sh"] == 0 and sw["sa"] == 0: sw["sh"] = 1.0
        if not (sw["sr"] or sw["se"] or sw["si"]): sw["sr"] = 0.1
        S.append(sw)
    jobs, meta = [], []
    for i, sw in enumerate(S):
        for kd in (0.03, 0.05):
            for st in "RL":
                jobs.append((E.cal(seed=i % 3, stance=st), dict(kd=kd), sw)); meta.append((i, kd, st))
    res = scan.pmap(jobs)
    table = []
    for i, sw in enumerate(S):
        js, det = 0.0, {}
        for kd in (0.03, 0.05):
            rr = [r for r, m in zip(res, meta) if m[0] == i and m[1] == kd]
            g, e, clr, gap, bad = agg(rr); js += objective(g, e, clr, gap, bad)
            det[kd] = dict(rms=g.round(2).tolist(), err=round(e, 2), clr=round(clr, 1), gap=round(gap, 2), bad=bad)
        table.append((js, sw, det))
    table.sort(key=lambda x: x[0])
    json.dump(table, open(out, "w"), indent=1)
    for js, sw, det in table[:12]: print(round(js, 2), sw, det, flush=True)

if __name__ == "__main__":
    main(int(sys.argv[1]), int(sys.argv[2]), sys.argv[3])
