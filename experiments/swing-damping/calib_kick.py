"""Stage 2: take the best plant candidates of calib_search and add the lift-off disturbance (fitted)."""
import json, sys, os, itertools
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import calib_search as C
src = json.load(open(sys.argv[1]))
top = [c for sc, det, c in src[:int(sys.argv[2])]]
for c in top:
    if isinstance(c.get("pad_solref"), list): c["pad_solref"] = tuple(c["pad_solref"])
samples = []
for c in top:
    for J, dr in itertools.product((0.15, 0.4, 0.8), ((0, 1., 0), (0, -1., 0), (0, 0, 1.))):
        samples.append({**c, "kick": J, "kick_dir": dr})
out = sorted(C.evaluate(samples), key=lambda x: x[0])
for sc, det, cfg in out[:12]:
    print(round(sc, 2), det, {k: v for k, v in cfg.items()}, flush=True)
json.dump([(sc, {str(k): v for k, v in det.items()}, cfg) for sc, det, cfg in out], open(sys.argv[3], "w"), indent=1, default=str)
