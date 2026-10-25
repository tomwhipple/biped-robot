"""Stage 3: local random refinement around the best stage-2 candidates, incl. the fitted lift-off kick size."""
import json, sys, os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import calib_search as C
src = json.load(open(sys.argv[1])); nb = int(sys.argv[2]); n = int(sys.argv[3]); seed = int(sys.argv[4])
rng = np.random.default_rng(seed)
base = []
for sc, det, c in src[:nb]:
    c = dict(c); c["kick_dir"] = tuple(c.get("kick_dir", (0, 0, 1.0)))
    if isinstance(c.get("pad_solref"), list): c["pad_solref"] = tuple(c["pad_solref"])
    base.append(c)
samples = []
for i in range(n):
    c = dict(base[i % nb])
    c["kick"] = float(np.exp(rng.uniform(np.log(0.1), np.log(3.0))))
    c["kick_dir"] = [(0, 1., 0), (0, -1., 0), (0, 0, 1.)][rng.integers(3)]
    c["jdamp"] = float(rng.choice([0.0, 0.02, 0.05, 0.1]))
    c["kd_mult"] = float(rng.choice([0.0, 0.1, 0.3]))
    c["delay"] = int(np.clip(c["delay"] + rng.integers(-1, 2), 1, 6))
    c["roll_lash"] = float(np.clip(c["roll_lash"] + rng.choice([-0.8, 0, 0.8]), 0.5, 5))
    c["lag"] = float(rng.choice([0.0, 3.0, 6.0, 10.0]))
    samples.append(c)
out = sorted(C.evaluate(samples), key=lambda x: x[0])
for sc, det, cfg in out[:10]:
    print(round(sc, 2), det, {k: v for k, v in cfg.items()}, flush=True)
json.dump([(sc, {str(k): v for k, v in det.items()}, cfg) for sc, det, cfg in out], open(sys.argv[5], "w"), indent=1, default=str)
