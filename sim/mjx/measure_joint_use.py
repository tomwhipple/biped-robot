#!/usr/bin/env python3
"""How much of each joint's allowed travel does a policy actually USE?

The plant can permit a range the gait never asks for -- that is exactly what
the 2026-08-02 ROM audit was about, except in reverse: there the limit was too
tight and nothing said so. This reports, per joint, the commanded and achieved
excursion against the range the plant allows, so "the knee barely moves" is a
number instead of an impression.

Run:  JAX_PLATFORMS=cpu .venv/bin/python sim/mjx/measure_joint_use.py \
          --run-name v5body_romfix [--seconds 12] [--cmd 0.4,0,0]
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

import mujoco  # noqa: E402
import eval_precision as EP  # noqa: E402
import joint_rom  # noqa: E402

RUNS = os.path.join(HERE, "..", "runs")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run-name", required=True)
    p.add_argument("--seconds", type=float, default=12.0)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--cmd", default="0.4,0,0", help="vx,vy,wz")
    args = p.parse_args()

    run_dir = os.path.join(RUNS, args.run_name)
    with open(os.path.join(run_dir, "config.json")) as fh:
        cfg = json.load(fh)
    xml = os.path.join(HERE, "..", cfg["xml_path"])

    env = EP.make_env(cfg, args.seconds, nominal=False, xml=xml)
    obs_size = env.observation_space.shape[0]
    act_size = env.action_space.shape[0]
    act = EP.load_policy(run_dir, obs_size, act_size)
    drv = EP.Driver(env, act, args.seed, record=False)

    m = env.model
    names = list(joint_rom.all_sim_ranges())
    jid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in names}
    qadr = {n: m.jnt_qposadr[jid[n]] for n in names}

    cmd = tuple(float(v) for v in args.cmd.split(","))
    q = {n: [] for n in names}
    steps = int(args.seconds / env.control_dt)
    for _ in range(steps):
        drv.step(cmd)
        for n in names:
            q[n].append(float(env.data.qpos[qadr[n]]))
        if drv.fell:
            break

    print(f"run {args.run_name}  cmd={cmd}  {len(q[names[0]])} steps "
          f"({'FELL' if drv.fell else 'survived'})")
    print(f"{'joint':<12} {'allowed (deg)':>18} {'used (deg)':>18} "
          f"{'span':>7} {'% of range':>11}")
    for n in names:
        lo_a, hi_a = joint_rom.all_sim_ranges()[n]
        d = np.degrees(np.array(q[n]))
        lo_u, hi_u = d.min(), d.max()
        span = hi_u - lo_u
        pct = 100.0 * span / (hi_a - lo_a)
        print(f"{n:<12} {lo_a:8.1f}..{hi_a:7.1f} {lo_u:8.1f}..{hi_u:7.1f} "
              f"{span:7.1f} {pct:10.1f}%")


if __name__ == "__main__":
    sys.exit(main())
