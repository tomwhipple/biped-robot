"""Bus-current budget from the sim's calibrated electrical model.

Why this exists: docs/wiring.md sized the harness on a **hand-estimated ~10 A
transient** ("2-3 joints near stall simultaneously"). Nothing ever measured
it. The Waveshare board is rated 5 A and its servo V+ is a bare passthrough
from the barrel jack (schematic, 2026-07-26), so the gap between those two
numbers decides whether the pack can feed the servo bus through the board at
all -- and it is the connectors, not silicon, that set the ceiling.

The env already computes electrical watts every tick with a model calibrated
to the ST3215 stall point (walker_env `_K_CU` = 3.75 W/(N*m)^2: 2.94 N*m
stall -> ~32 W -> 2.7 A at 12 V, matching the vendor's stall-current spec).
Current is just that over the bus voltage. So instead of estimating, run the
referee's scenarios and report the distribution:

  peak   -- the single worst tick (what a 10 A claim is about)
  p99    -- the worst 1 % of ticks
  rms    -- what actually heats a connector (I^2 R over time)
  mean   -- what drains the pack

Per-leg columns matter because the daisy chain means one 3-pin connector
carries its whole leg's current, and the board's two ports are the same
electrical bus -- the split is a routing choice, not an electrical one.

Run:  .venv/bin/python sim/current_budget.py --run loco_v5t [--episodes 4]
"""
from __future__ import annotations
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "mjx"))
RUNS = os.path.join(HERE, "runs")

from eval_precision import load_policy, make_env      # noqa: E402

# (name, vx, wz) -- the referee's command scenarios plus a hard accel, since
# the worst transient is a start-from-rest, not steady-state cruise.
SCENARIOS = [
    ("stand",   0.0,  0.0),
    ("slow",    0.35, 0.0),
    ("walk",    0.6,  0.0),
    ("fast",    1.0,  0.0),
    ("pivot",   0.0,  0.8),
    ("turn",    0.6,  0.6),
]


def leg_masks(env):
    """Split action indices into (left, right). The env already resolves the
    per-leg role maps from the model, so this works on 8- and 10-DOF plants."""
    left = np.array(sorted(i for i in env._legL.values() if i is not None))
    right = np.array(sorted(i for i in env._legR.values() if i is not None))
    return left, right


def rollout_amps(env, act, v, w, seed, volts):
    """One episode -> per-tick total and per-leg bus current (A)."""
    obs, _ = env.reset(seed=seed)
    env.set_command(v, w)
    obs = env._obs()
    li, ri = leg_masks(env)
    tot, lft, rgt, per_joint = [], [], [], []
    for _ in range(env.max_steps):
        a = act(obs)
        obs, _, term, _, info = env.step(a)
        d = env.data
        tau = np.asarray(env._servo_tau if env._servo is not None
                         else d.actuator_force, dtype=float)
        qd = np.asarray(d.qvel[env._jqvel], dtype=float)
        # same formula as walker_env's power_w, kept un-summed
        p_j = np.maximum(tau * qd, 0.0) + env._K_CU * tau ** 2
        i_j = p_j / volts
        tot.append(i_j.sum())
        lft.append(i_j[li].sum())
        rgt.append(i_j[ri].sum())
        per_joint.append(i_j.max())
        if term:
            break
    return (np.asarray(tot), np.asarray(lft), np.asarray(rgt),
            np.asarray(per_joint))


def stats(a):
    return dict(peak=float(a.max()), p99=float(np.percentile(a, 99)),
                rms=float(np.sqrt(np.mean(a ** 2))), mean=float(a.mean()))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run", default="loco_v5t")
    p.add_argument("--episodes", type=int, default=4)
    p.add_argument("--volts", type=float, default=11.1,
                   help="bus voltage (3S nominal; use 10.5 for the landing floor)")
    p.add_argument("--out", default=None)
    args = p.parse_args()

    run_dir = os.path.join(RUNS, args.run)
    with open(os.path.join(run_dir, "config.json")) as f:
        cfg = json.load(f)
    xml = os.path.join(HERE, cfg.get("xml_path", "bimo_biped_v3yaw.xml"))
    env = make_env(cfg, episode_seconds=10.0, nominal=False, xml=xml)
    act = load_policy(run_dir, obs_size=env.observation_space.shape[0],
                      act_size=env.action_space.shape[0])

    print(f"bus-current budget: {args.run}  ({len(env._act_names)} joints, "
          f"{args.volts:.1f} V bus, {args.episodes} eps/scenario, "
          f"hardware-claim DR)")
    print(f"{'scenario':9s} {'peak':>7s} {'p99':>7s} {'rms':>7s} {'mean':>7s} "
          f"{'legpeak':>8s} {'jntpeak':>8s}")

    rows, all_tot, all_leg, all_jnt = {}, [], [], []
    for name, v, w in SCENARIOS:
        t, l, r, j = [], [], [], []
        for i in range(args.episodes):
            tt, ll, rr, jj = rollout_amps(env, act, v, w, seed=100 * i + 7,
                                          volts=args.volts)
            t.append(tt); l.append(ll); r.append(rr); j.append(jj)
        t = np.concatenate(t); leg = np.concatenate(l + r); j = np.concatenate(j)
        all_tot.append(t); all_leg.append(leg); all_jnt.append(j)
        s = stats(t)
        rows[name] = dict(total=s, leg=stats(leg), joint=stats(j))
        print(f"{name:9s} {s['peak']:7.2f} {s['p99']:7.2f} {s['rms']:7.2f} "
              f"{s['mean']:7.2f} {leg.max():8.2f} {j.max():8.2f}")

    t = np.concatenate(all_tot); leg = np.concatenate(all_leg)
    j = np.concatenate(all_jnt)
    s = stats(t)
    print(f"{'ALL':9s} {s['peak']:7.2f} {s['p99']:7.2f} {s['rms']:7.2f} "
          f"{s['mean']:7.2f} {leg.max():8.2f} {j.max():8.2f}")
    rows["ALL"] = dict(total=s, leg=stats(leg), joint=stats(j),
                       volts=args.volts, episodes=args.episodes)

    out = args.out or os.path.join(run_dir, "current_budget.json")
    with open(out, "w") as f:
        json.dump(rows, f, indent=2)
    print(f"\nsaved -> {out}")


if __name__ == "__main__":
    main()
