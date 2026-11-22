#!/usr/bin/env python3
"""Re-grade a get-up run on the supply sag at a given R (#6), CPU referee.

    JAX_PLATFORMS=cpu CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/supply-r/regrade.py \
        --src /tmp/i6n/runs/robot_getup_w19k --ohm 0.25 --tag a \
        --scenarios recover_supine_ph [--out bus.json]

Copies the run's config.json + params.pkl to sim/runs/<run>_sag<mOhm>[_<tag>] (gitignored) with
`supply_sag_ohm` set, as PR #339 did for its 0.15 / 0.44 ohm readings (same policy, no retraining,
V0 = the run's own supply_voltage), then runs `sim/mjx/eval_precision.py --overload-column` on it
and records the sag's bus current and supply voltage every control tick per scenario.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("CUDA_VISIBLE_DEVICES", "")
os.environ.setdefault("MUJOCO_GL", "disabled")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="the run dir to copy (config.json, params.pkl)")
    ap.add_argument("--ohm", type=float, required=True)
    ap.add_argument("--tag", default="")
    ap.add_argument("--scenarios", required=True)
    ap.add_argument("--episodes", type=int, default=8)
    ap.add_argument("--out", default=None, help="bus-current json (default: in the run copy)")
    a = ap.parse_args()

    src = os.path.abspath(a.src)
    name = f"{os.path.basename(src)}_sag{round(a.ohm * 1000):03d}" + (f"_{a.tag}" if a.tag else "")
    dst = os.path.join(ROOT, "sim", "runs", name)
    os.makedirs(dst, exist_ok=True)
    shutil.copy2(os.path.join(src, "params.pkl"), dst)
    cfg = json.load(open(os.path.join(src, "config.json")))
    if a.ohm > 0:
        cfg["supply_sag_ohm"] = [a.ohm, a.ohm]
    else:
        cfg.pop("supply_sag_ohm", None)
    json.dump(cfg, open(os.path.join(dst, "config.json"), "w"), indent=2)

    sys.path.insert(0, os.path.join(ROOT, "sim", "mjx"))
    sys.path.insert(0, os.path.join(ROOT, "sim"))
    os.chdir(os.path.join(ROOT, "sim", "mjx"))
    import numpy as np
    import walker_env
    import eval_precision as ep

    rec = {}
    step0 = walker_env.BimoWalkerEnv.step

    def step(self, act):
        r = step0(self, act)
        if getattr(self, "_sag_r", None) is not None:
            rec.setdefault(ep._overload["scenario"], []).append(
                (float(self._sag_i), float(self.supply_voltage) * float(self._sag_s_run)))
        return r

    walker_env.BimoWalkerEnv.step = step
    sys.argv = ["eval_precision.py", "--run-name", name, "--overload-column",
                "--episodes", str(a.episodes), "--scenarios", a.scenarios]
    try:
        ep.main()
    finally:
        stats = {}
        for k, v in rec.items():
            x = np.array(v)
            stats[k] = dict(n=len(x), i_median=float(np.median(x[:, 0])),
                            i_p95=float(np.percentile(x[:, 0], 95)), i_max=float(x[:, 0].max()),
                            v_median=float(np.median(x[:, 1])), v_min=float(x[:, 1].min()))
        out = a.out or os.path.join(dst, "bus.json")
        json.dump(dict(ohm=a.ohm, run=name, stats=stats), open(out, "w"), indent=1)
        for k, s in stats.items():
            print(f"BUS {k}: I median {s['i_median']:.2f} p95 {s['i_p95']:.2f} max {s['i_max']:.2f} A; "
                  f"V median {s['v_median']:.2f} min {s['v_min']:.2f} V (n={s['n']})")


if __name__ == "__main__":
    main()
