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

Per-port columns matter because the daisy chain means one 3-pin connector
carries its whole chain's current, and the board's two ports are the same
electrical bus -- the split is a routing choice, not an electrical one.

Two modes (issue #77):

  --run <run>       a trained policy under the referee's six command
                    scenarios (the original mode; needs sim/runs/<run>,
                    gitignored -- for after a policy exists)
  --trace <kind>    scripted traces that need no trained policy:
                      walk   -- the Gate D open-loop walk (static_gait.py)
                                on the committed robot plant (bimo_biped_v6ar)
                      getup  -- the scripted seat-push get-up
                                (getup_v6_shoulder.py seat_push, the chosen
                                variant on the r5_asdrawn_rom120 plant)

Run:  .venv/bin/python sim/current_budget.py --trace walk
      .venv/bin/python sim/current_budget.py --trace getup
      .venv/bin/python sim/current_budget.py --run loco_v5t [--episodes 4]
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

# Proposed 17-servo ID map -> board port (docs/servo-map.md 2.1 and
# firmware/components/obs/include/obs/bus_map.h kBusPort). Port A is H5
# (right leg + neck + right arm, 9 servos), port B is H6 (left leg + left
# arm, 8). PROPOSED, awaiting owner sign-off (issue #77).
PORT_A_IDS = {1, 2, 3, 4, 9, 11, 13, 14, 15}     # R leg, neck, R arm
PORT_B_IDS = {5, 6, 7, 8, 10, 12, 16, 17}        # L leg, L arm

# Sim actuator name -> servo ID under the proposed map. The 12 leg joints
# and the extras appear here exactly once each; the 10-joint prototype is
# the subset {1..10} and resolves by the same names.
NAME_TO_ID = {
    "R_hip_roll": 1, "R_hip_pitch": 2, "R_knee": 3, "R_ankle": 4,
    "L_hip_roll": 5, "L_hip_pitch": 6, "L_knee": 7, "L_ankle": 8,
    "R_hip_yaw": 9, "L_hip_yaw": 10,
    "R_ankle_roll": 11, "L_ankle_roll": 12,
    "neck_yaw": 13, "R_shoulder": 14, "R_elbow": 15,
    "L_shoulder": 16, "L_elbow": 17,
}


def port_masks(env):
    """Split the env's policy actuator indices (per `env._act_names`) into
    (port A, port B) under the proposed ID map. Works on the 10-, 12-, 13-
    and 17-joint plants: any name not in NAME_TO_ID raises (an unnamed
    actuator is a mapper bug, not a defaults case)."""
    ai, bi = [], []
    for i, name in enumerate(env._act_names):
        sid = NAME_TO_ID[name]
        (ai if sid in PORT_A_IDS else bi).append(i)
    return np.array(ai, dtype=int), np.array(bi, dtype=int)


def amps_per_tick(env, volts):
    """Per-joint bus current (A) at the current env state, using walker_env's
    own electrical model (P = max(tau*qd, 0) + K_CU * tau^2). Returns the
    (n_pol,) array indexed by policy-actuator index, aligned with
    `env._act_names` and `env._jqvel`.
    """
    d = env.data
    tau = np.asarray(env._servo_tau if env._servo is not None
                     else d.actuator_force, dtype=float)
    qd = np.asarray(d.qvel[env._jqvel], dtype=float)
    p_j = np.maximum(tau * qd, 0.0) + env._K_CU * tau ** 2
    return p_j / volts


def stats(a):
    return dict(peak=float(a.max()), p99=float(np.percentile(a, 99)),
                rms=float(np.sqrt(np.mean(a ** 2))), mean=float(a.mean()))


def rollout_amps(env, act, v, w, seed, volts):
    """One policy episode -> per-tick total and per-port bus current (A)."""
    obs, _ = env.reset(seed=seed)
    env.set_command(v, w)
    obs = env._obs()
    ai, bi = port_masks(env)
    tot, pa, pb, per_joint = [], [], [], []
    for _ in range(env.max_steps):
        a = act(obs)
        obs, _, term, _, info = env.step(a)
        i_j = amps_per_tick(env, volts)
        tot.append(i_j.sum())
        pa.append(i_j[ai].sum())
        pb.append(i_j[bi].sum())
        per_joint.append(i_j.max())
        if term:
            break
    return (np.asarray(tot), np.asarray(pa), np.asarray(pb),
            np.asarray(per_joint))


# ---------------------------------------------------------------------------
# scripted traces (issue #77): no policy required, both run on the committed
# plants at the deploy servo model.

def trace_walk(volts):
    """Gate D's own walk: the 8-step nominal-cadence gait from
    `static_gait.walk_timeline` on the committed 17-servo plant
    (sim/bimo_biped_v6ar.xml), at the deploy servo model the design uses.
    The arms hang at the walking pose (the plant holds them through the same
    electrical model)."""
    from gen_plant_v6 import DesignParams
    import static_gait as SG

    p = DesignParams()
    xml = os.path.join(HERE, "bimo_biped_v6ar.xml")
    tl, _windows = SG.walk_timeline(p, n_steps=8, step=0.06, lift_h=0.04)
    env = SG.make_env(p, xml)
    env.reset(seed=0)
    ai, bi = port_masks(env)
    dt = env.control_dt
    na = env._nq_act
    d0, hi, lo = env._default, env._hi, env._lo

    def inv(q):
        q = q[:na] if len(q) >= na else np.concatenate(
            [q, np.zeros(na - len(q))])
        return np.clip(
            np.where(q >= d0, (q - d0) / np.maximum(hi - d0, 1e-6),
                     (q - d0) / np.maximum(d0 - lo, 1e-6)), -1, 1)

    tot, pa, pb, per_joint = [], [], [], []
    for k in range(int(tl.T / dt) + 1):
        key = tl.at(k * dt)
        qc = SG.q_of(p, key)
        env.step(inv(qc))
        i_j = amps_per_tick(env, volts)
        tot.append(i_j.sum())
        pa.append(i_j[ai].sum())
        pb.append(i_j[bi].sum())
        per_joint.append(i_j.max())
    return (np.asarray(tot), np.asarray(pa), np.asarray(pb),
            np.asarray(per_joint))


def trace_getup(volts):
    """The scripted seat-push get-up, on the as-drawn robot
    (getup_v6_shoulder.py `r5_asdrawn_rom120`, the plant after the
    shoulder-girdle restyle at the CAD hip ROM -120). The variant is the
    round-5b robust winner: shoulder 90 -> 0 degress, elbow (0, 0), ankle
    -25, push over 2 s (`no3250_getup.txt` -- 12 variants x 6 robustness
    conditions, this and ((60,-60),(0,0),-25) stand 6/6; the (90,-90)
    variant is the one called out in DESIGN.md section 9)."""
    import dataclasses
    import mujoco  # noqa: F401  (imported by getup_v6 consumers)
    from gen_plant_v6 import build_xml
    import getup_v6_shoulder as GUS

    p = dataclasses.replace(GUS.BASE, **GUS.CONFIGS["r5_asdrawn_rom120"])
    xml = os.path.join(os.environ.get("TMPDIR", "/tmp"),
                       f"gu_budget_{os.getpid()}.xml")
    with open(xml, "w") as fh:
        fh.write(build_xml(p))
    seq = GUS.seat_push(False, 90, 0, 2.0, -25, elbow=(0, 0))
    env = GUS.G.make_env(p, xml, mu=0.7, play_deg=3.0)
    env.reset(seed=0)
    GUS.G.EXTRA[:] = [env.model.actuator(i).name
                      for i in range(12, env.model.nu)]
    kp, kd, stall, w0 = env._servo
    na = env._nq_act
    stall_a = np.full(na, stall)
    w0_a = np.full(na, w0)
    kpa = np.full(na, float(kp))
    kda = np.full(na, float(kd))
    for i, n in enumerate(GUS.G.JN):
        if n in GUS.G.PJ_DEFAULT:
            s = GUS.G.SERVOS[GUS.G.PJ_DEFAULT[n]]
            stall_a[i] = s["stall"]
            w0_a[i] = s["w0"]
            kpa[i] *= s["kp_scale"]
            kda[i] *= s["kp_scale"]
    env._servo = np.array([kpa, kda, stall_a, w0_a], dtype=object)
    env.fall_up_z = -2.0
    env.fall_height = -1.0
    ai, bi = port_masks(env)
    d0, hi, lo = env._default, env._hi, env._lo

    def inv(q):
        q = q[:na] if len(q) >= na else np.concatenate(
            [q, np.zeros(na - len(q))])
        return np.clip(
            np.where(q >= d0, (q - d0) / np.maximum(hi - d0, 1e-6),
                     (q - d0) / np.maximum(d0 - lo, 1e-6)), -1, 1)

    q_prev = GUS.G.q_from_offsets(seq[0][1])
    GUS.G.settle_fallen(env, p, "supine", q_prev)
    dt = env.control_dt
    tot, pa, pb, per_joint = [], [], [], []
    for _label, off, move_s, hold_s in seq:
        q_tgt = GUS.G.q_from_offsets(off)
        n = int(move_s / dt)
        for i in range(n + int(hold_s / dt)):
            s = min(1.0, (i + 1) / max(n, 1))
            s = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
            q = q_prev + (q_tgt - q_prev) * s
            env.step(inv(q))
            i_j = amps_per_tick(env, volts)
            tot.append(i_j.sum())
            pa.append(i_j[ai].sum())
            pb.append(i_j[bi].sum())
            per_joint.append(i_j.max())
        q_prev = q_tgt
    # settle_fallen+seq may end with the env terminated (stand check we
    # overrode); read the final state for the caller's verdict.
    d = env.data
    up = float(d.xmat[env._torso_bid].reshape(3, 3)[2, 2])
    pelvis_z = float(d.qpos[2])
    ok = up > 0.9 and pelvis_z > 0.8 * p.z_yaw_above_sole
    return (np.asarray(tot), np.asarray(pa), np.asarray(pb),
            np.asarray(per_joint), ok, up, pelvis_z)


def report(label, volts, t, a, b, j, extra=None):
    s, sa, sb, sj = stats(t), stats(a), stats(b), stats(j)
    print(f"bus-current budget: {label}  ({volts:.1f} V bus)")
    print(f"{'measure':9s} {'peak':>7s} {'p99':>7s} {'rms':>7s} {'mean':>7s}")
    print(f"{'total':9s} {s['peak']:7.2f} {s['p99']:7.2f} {s['rms']:7.2f} "
          f"{s['mean']:7.2f}")
    print(f"{'port A':9s} {sa['peak']:7.2f} {sa['p99']:7.2f} "
          f"{sa['rms']:7.2f} {sa['mean']:7.2f}")
    print(f"{'port B':9s} {sb['peak']:7.2f} {sb['p99']:7.2f} "
          f"{sb['rms']:7.2f} {sb['mean']:7.2f}")
    print(f"{'worst jnt':9s} {sj['peak']:7.2f} {sj['p99']:7.2f} "
          f"{sj['rms']:7.2f} {sj['mean']:7.2f}")
    if extra:
        print(extra)
    row = dict(volts=volts, total=s, port_a=sa, port_b=sb, joint=sj)
    return row


def run_policy_budget(args):
    run_dir = os.path.join(RUNS, args.run)
    with open(os.path.join(run_dir, "config.json")) as f:
        cfg = json.load(f)
    xml = os.path.join(HERE, cfg.get("xml_path", "bimo_biped_v3yaw.xml"))
    env = make_env(cfg, episode_seconds=10.0, nominal=False, xml=xml)
    act = load_policy(run_dir, obs_size=env.observation_space.shape[0],
                      act_size=env.action_space.shape[0])

    print(f"policy run: {args.run}  ({len(env._act_names)} joints, "
          f"{args.volts:.1f} V bus, {args.episodes} eps/scenario, "
          f"hardware-claim DR)")
    print(f"{'scenario':9s} {'peak':>7s} {'p99':>7s} {'rms':>7s} {'mean':>7s} "
          f"{'portApeak':>10s} {'portBpeak':>10s} {'jntpeak':>8s}")

    rows, all_tot, all_a, all_b, all_jnt = {}, [], [], [], []
    for name, v, w in SCENARIOS:
        t, pa, pb, j = [], [], [], []
        for i in range(args.episodes):
            tt, aa, bb, jj = rollout_amps(env, act, v, w, seed=100 * i + 7,
                                          volts=args.volts)
            t.append(tt); pa.append(aa); pb.append(bb); j.append(jj)
        t = np.concatenate(t); pa = np.concatenate(pa); pb = np.concatenate(pb)
        j = np.concatenate(j)
        all_tot.append(t); all_a.append(pa); all_b.append(pb)
        all_jnt.append(j)
        s = stats(t)
        rows[name] = dict(total=s, port_a=stats(pa), port_b=stats(pb),
                          joint=stats(j))
        print(f"{name:9s} {s['peak']:7.2f} {s['p99']:7.2f} {s['rms']:7.2f} "
              f"{s['mean']:7.2f} {pa.max():10.2f} {pb.max():10.2f} "
              f"{j.max():8.2f}")

    t = np.concatenate(all_tot); a = np.concatenate(all_a)
    b = np.concatenate(all_b); j = np.concatenate(all_jnt)
    s = stats(t)
    print(f"{'ALL':9s} {s['peak']:7.2f} {s['p99']:7.2f} {s['rms']:7.2f} "
          f"{s['mean']:7.2f} {a.max():10.2f} {b.max():10.2f} {j.max():8.2f}")
    rows["ALL"] = dict(total=s, port_a=stats(a), port_b=stats(b),
                       joint=stats(j), volts=args.volts,
                       episodes=args.episodes)

    out = args.out or os.path.join(run_dir, "current_budget.json")
    with open(out, "w") as f:
        json.dump(rows, f, indent=2)
    print(f"\nsaved -> {out}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run", default=None,
                   help="trained run under sim/runs/ (policy mode)")
    p.add_argument("--trace", choices=["walk", "getup"], default=None,
                   help="scripted trace: walk (Gate D) or getup (seat push)")
    p.add_argument("--episodes", type=int, default=4)
    p.add_argument("--volts", type=float, default=11.1,
                   help="bus voltage (3S nominal; use 10.5 for the landing floor)")
    p.add_argument("--out", default=None)
    args = p.parse_args()

    if args.run:
        run_policy_budget(args)
        return
    if args.trace == "walk":
        t, a, b, j = trace_walk(args.volts)
        row = report("trace=walk (Gate D, sim/bimo_biped_v6ar.xml, deploy "
                     "servo model)", args.volts, t, a, b, j)
        row["trace"] = "walk"
        out = args.out or os.path.join(HERE, "runs", "current_budget_walk.json")
    elif args.trace == "getup":
        t, a, b, j, ok, up, pelvis_z = trace_getup(args.volts)
        extra = (f"get-up outcome: {'STANDING' if ok else 'NOT standing'}  "
                 f"up_z {up:+.2f}  pelvis_z {pelvis_z:.3f}")
        row = report("trace=getup (seat push, r5_asdrawn_rom120)",
                     args.volts, t, a, b, j, extra=extra)
        row["trace"] = "getup"
        row["stood"] = bool(ok)
        out = args.out or os.path.join(HERE, "runs",
                                       "current_budget_getup.json")
    else:
        p.error("either --run or --trace is required")
        return
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        json.dump(row, f, indent=2)
    print(f"saved -> {out}")


if __name__ == "__main__":
    main()
