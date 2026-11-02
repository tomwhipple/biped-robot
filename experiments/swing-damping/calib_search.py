"""Calibrate the sim to the 2026-10-25 hardware balance logs (right stance, as-built shift 1.3x, straight 30 mm
lift over 4 s, controller on).  Random search over the physically plausible knobs; score against the three
runs that are on the same bench path: kd 0.03 (baseline), kd 0.05, kd 0.07 (unstable).

    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/swing-damping/calib_search.py search --n 240 --json build/swing/calib.json
"""
import argparse, json, math, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import scan, hwlog

HW = {0.03: dict(rms=[4.04, 2.00, 0.75], err=2.35), 0.05: dict(rms=[3.10, 2.94, 0.47], err=2.36)}
SPACE = dict(
    delay=[1, 2, 3, 4, 5, 6], roll_lash=[1.0, 1.9, 2.6, 3.4, 4.2], play=[0.0, 1.0, 2.0, 3.0], lash=[0.5, 0.7, 1.0, 1.5],
    kp_ankle=[1.0, 1.7, 2.8], kp_knee=[1.0, 2.8], kp_hip=[0.5, 1.0, 1.5], kd_mult=[0.0, 0.1, 0.3, 0.6], jdamp=[0.0, 0.02, 0.05, 0.1], mu=[0.5, 0.7, 1.0],
    pad_solref=[None, (0.02, 0.3), (0.04, 0.15)], jfric=[0.0, 0.03, 0.1], lag=[0.0, 6.0, 3.0])
KDS = (0.03, 0.05, 0.07)


def score(rs):
    """rs: {kd: [runs over seeds]} -> (score, detail)"""
    s, det = 0.0, {}
    for kd in (0.03, 0.05):
        rr = [r for r in rs[kd] if not r["fell"]]
        if len(rr) < len(rs[kd]):
            s += 10.0 * (len(rs[kd]) - len(rr)); det[kd] = "fell"; continue
        rms = np.exp(np.mean([np.log(np.maximum(r["rms"], 0.05)) for r in rr], axis=0))
        err = np.mean([r["err_peak"] for r in rr])
        e = np.log(rms / np.array(HW[kd]["rms"]))
        s += float(np.sum(e ** 2)) + float((math.log(err / HW[kd]["err"])) ** 2)
        det[kd] = dict(rms=[round(float(x), 2) for x in rms], err=round(float(err), 2), f=round(float(np.median([r["f_hz"] for r in rr])), 2))
    # kd 0.07 must go unstable in most seeds: late-window rate growth or err > 6
    unst = [(r["fell"] or r["err_peak"] > 6.0 or (r["rms"][1] > 15)) for r in rs[0.07]]
    s += 3.0 * (len(unst) - sum(unst)) / len(unst)
    det[0.07] = f"unstable {sum(unst)}/{len(unst)}"
    # frequency of the baseline rocking (hardware: 2.0-2.4 Hz)
    f = det[0.03]["f"] if isinstance(det[0.03], dict) else 0
    s += (max(0.0, abs(f - 2.15) - 0.3) / 0.4) ** 2
    return s, det


def evaluate(samples, seeds=3, procs=12):
    jobs = [(dict(cfg, seed=sd), dict(kd=kd), None) for cfg in samples for kd in KDS for sd in range(seeds)]
    res = scan.pmap(jobs, procs)
    out = []
    i = 0
    for cfg in samples:
        rs = {kd: [] for kd in KDS}
        for kd in KDS:
            for sd in range(seeds):
                rs[kd].append(res[i]); i += 1
        bad = [r for k in rs.values() for r in k if "error" in r]
        if bad:
            out.append((1e9, {"error": bad[0]["error"]}, cfg)); continue
        sc, det = score(rs)
        out.append((sc, det, cfg))
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("mode"); ap.add_argument("--n", type=int, default=120)
    ap.add_argument("--seed", type=int, default=0); ap.add_argument("--json", default=None)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    samples = [{k: v[rng.integers(len(v))] for k, v in SPACE.items()} for _ in range(a.n)]
    out = sorted(evaluate(samples), key=lambda x: x[0])
    for sc, det, cfg in out[:15]:
        print(round(sc, 2), det, {k: v for k, v in cfg.items()})
    if a.json:
        os.makedirs(os.path.dirname(os.path.abspath(a.json)), exist_ok=True)
        json.dump([(sc, {str(k): v for k, v in det.items()}, cfg) for sc, det, cfg in out], open(a.json, "w"), indent=1, default=str)
