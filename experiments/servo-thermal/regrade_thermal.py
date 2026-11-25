#!/usr/bin/env python3
"""Grade a run with the servo temperature on (#6), CPU referee: per joint the episode's mean heat and the
minutes to the 70 degC cutoff if that heat were held.

    JAX_PLATFORMS=cpu CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/servo-thermal/regrade_thermal.py \
        --src /tmp/i6n/runs/robot_getup_w19k --start 29 --scenarios recover_prone [--episodes 4]

Copies the run's config.json + params.pkl to sim/runs/<run>_th<T> (gitignored) with `servo_thermal` set
to a fixed start temperature (same policy, no retraining; the run's own plant otherwise), runs
`sim/mjx/eval_precision.py` on it (the referee inherits servo_thermal and writes a "thermal" block per
scenario into the scorecard json: eval_precision.thermal_summary), and prints that block.
"""
from __future__ import annotations

import argparse
import glob
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
    ap.add_argument("--start", type=float, default=29.0, help="every servo's start temperature, degC")
    ap.add_argument("--scenarios", required=True)
    ap.add_argument("--episodes", type=int, default=4)
    ap.add_argument("--out", default=None, help="json of the thermal blocks (default: in the run copy)")
    a = ap.parse_args()

    src = os.path.abspath(a.src)
    name = f"{os.path.basename(src)}_th{round(a.start)}"
    dst = os.path.join(ROOT, "sim", "runs", name)
    os.makedirs(dst, exist_ok=True)
    shutil.copy2(os.path.join(src, "params.pkl"), dst)
    cfg = json.load(open(os.path.join(src, "config.json")))
    cfg["servo_thermal"] = [a.start, a.start]
    json.dump(cfg, open(os.path.join(dst, "config.json"), "w"), indent=2)

    sys.path.insert(0, os.path.join(ROOT, "sim", "mjx"))
    sys.path.insert(0, os.path.join(ROOT, "sim"))
    os.chdir(os.path.join(ROOT, "sim", "mjx"))
    import eval_precision as ep
    sys.argv = ["eval_precision.py", "--run-name", name, "--episodes", str(a.episodes),
                "--scenarios", a.scenarios]
    ep.main()
    card = max(glob.glob(os.path.join(dst, "scorecard*.json")), key=os.path.getmtime)
    sc = json.load(open(card))
    out = {k: dict(successes=v["successes"], n=v["n"], episode_seconds=v["episode_seconds"],
                   thermal=v["thermal"])
           for k, v in sc.items() if isinstance(v, dict) and "thermal" in v}
    json.dump(dict(run=os.path.basename(src), start=a.start, scenarios=out),
              open(a.out or os.path.join(dst, "thermal.json"), "w"), indent=1)
    for k, v in out.items():
        print(f"\n{k}: {v['successes']}/{v['n']}, {v['episode_seconds']} s")
        rows = sorted(v["thermal"].items(), key=lambda kv: -kv[1]["heat_w"])
        for j, r in rows[:6]:
            m = r["min_to_cutoff"]
            print(f"  {j:14s} heat {r['heat_w']:6.2f} W  rise {r['rise_c']:5.2f} C  peak {r['t_peak']:5.1f} C  "
                  f"cut off {r['cut_off']}  held: {'never' if m is None else f'{m:.1f} min'} to 70 C")


if __name__ == "__main__":
    main()
