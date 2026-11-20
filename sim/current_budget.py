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

Modes (issue #77):

  --trace refs      the campaign's get-up references (getup_ref_overload.py:
                    prone_ikc, supine_ik) on build/v6/sim/bimo_biped_v6ar_meas.xml,
                    STS3250 hip rolls, kp as built, 11.1 V and the 12 V refit
                    (--all-conds: that study's ten robustness conditions)
  --trace dwell     the track player's dwell walk (experiments/roll-balance:
                    dwell.py through the firmware's track + trim in the SIL,
                    sil_gait.run) on the legs-only 'fit' plant, T 6 / 4 / 3.2 s,
                    seven directions (needs `make -C firmware/host sil` and
                    the grade plant's gitignored run, as sil_gait does)
                    Both sum each servo's supply current (both models of
                    sts_servo_model.py, per servo type) per power-lite branch
                    J4-J7, per General Driver port and over the bus.
  --trace walk      the 2026-09 scripted traces, two ports, bridge model only:
  --trace getup       the Gate D open-loop walk (static_gait.py) on
                      bimo_biped_v6ar, and the seat-push get-up
                      (getup_v6_shoulder.py, r6_asdrawn_rom120)
  --run <run>       a trained policy under the referee's six command
                    scenarios (needs sim/runs/<run>, gitignored)

Run:  .venv/bin/python sim/current_budget.py --trace refs [--all-conds]
      .venv/bin/python sim/current_budget.py --trace dwell
      .venv/bin/python sim/current_budget.py --run <run> [--episodes 4]
"""
from __future__ import annotations
import argparse
import json
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import paths  # noqa: E402
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "mjx"))
RUNS = os.path.join(HERE, "runs")

import sts_servo_model as SM                          # noqa: E402

ROOT = os.path.abspath(os.path.join(HERE, ".."))

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

# The 17-servo ID map -> board port (docs/servo-map.md 2.1 and
# firmware/components/obs/include/obs/bus_map.h kBusServoId / kBusPort): the
# v6 legs as assembled (2026-10-23). Port A is H5 (right leg + neck + right
# arm, 9 servos), port B is H6 (left leg + left arm, 8).
PORT_A_IDS = {52, 6, 7, 8, 10, 11, 13, 14, 15}   # R leg, neck, R arm
PORT_B_IDS = {51, 2, 3, 4, 9, 12, 16, 17}        # L leg, L arm

# Sim actuator name -> servo ID under the bus map. The 12 leg joints and the
# extras appear here exactly once each.
NAME_TO_ID = {
    "R_hip_roll": 52, "R_hip_pitch": 6, "R_knee": 7, "R_ankle": 8,
    "L_hip_roll": 51, "L_hip_pitch": 2, "L_knee": 3, "L_ankle": 4,
    "R_hip_yaw": 10, "L_hip_yaw": 9,
    "R_ankle_roll": 11, "L_ankle_roll": 12,
    "neck_yaw": 13, "R_shoulder": 14, "R_elbow": 15,
    "L_shoulder": 16, "L_elbow": 17,
}


def port_masks(env):
    """Split the env's policy actuator indices (per `env._act_names`) into
    (port A, port B) under the bus ID map. Works on the 10-, 12-, 13-
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
    (build/v6/sim/bimo_biped_v6ar.xml), at the deploy servo model the design uses.
    The arms hang at the walking pose (the plant holds them through the same
    electrical model)."""
    from gen_plant_v6 import DesignParams
    import static_gait as SG

    p = DesignParams()
    xml = paths.v6_plant("bimo_biped_v6ar.xml")
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
    (getup_v6_shoulder.py `r6_asdrawn_rom120`: the shoulder girdle, the
    elbow servo on the upper arm, the CAD hip ROM -120). The variant is the
    round-5b robust winner: shoulder 90 -> 0 degress, elbow (0, 0), ankle
    -25, push over 2 s (`no3250_getup.txt` -- 12 variants x 6 robustness
    conditions, this and ((60,-60),(0,0),-25) stand 6/6; the (90,-90)
    variant is the one called out in DESIGN.md section 9)."""
    import dataclasses
    import mujoco  # noqa: F401  (imported by getup_v6 consumers)
    from gen_plant_v6 import build_xml
    import getup_v6_shoulder as GUS

    p = dataclasses.replace(GUS.BASE, **GUS.CONFIGS["r6_asdrawn_rom120"])
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


# ---------------------------------------------------------------------------
# per power-lite branch, on the robot's servo set and the current motions
# (issue #77, 2026-11-16): `--trace refs` (the get-up references) and
# `--trace dwell` (the track player's dwell walk).

# Power lite's four servo branches (pcb/power-lite/README.md, J4-J7) under the
# servo map (docs/servo-map.md 2.1). Each branch's connector carries its whole
# chain; the General Driver's H5 / H6 (PORT_A_IDS / PORT_B_IDS) are J4 + J5
# and J6 + J7.
BRANCHES = {"J4 R leg": frozenset({10, 52, 6, 7, 8, 11}),
            "J5 R upper": frozenset({13, 14, 15}),
            "J6 L leg": frozenset({9, 51, 2, 3, 4, 12}),
            "J7 L upper": frozenset({16, 17})}
STS3250_IDS = frozenset({51, 52})       # the hip rolls (servo-map 2.1)

# Per servo type: (stall current A at 12 V, Kt N*m/A, K_CU W/(N*m)^2).
#   sts3215  the ST-3215-C018 datasheet and sim/sts_servo_model.py's fitted
#            K_CU (2.94 N*m stall -> 2.7 A at 12 V)
#   sts3250  50 kg.cm = 4.90 N*m stall (walker_env SERVO_TYPES); 4.2 A at
#            stall from retailer listings only (docs/bom.md, docs/design-v6/
#            2026-10-06-power-circuit.md): UNVERIFIED, no Feetech sheet. Kt and
#            K_CU FITTED to that stall point the way the STS3215's K_CU is.
#            Its idle and no-load currents are taken as the STS3215's
#            (undocumented).
SERVO_ELEC = {"sts3215": (SM.I_STALL_12V, SM.KT, SM.K_CU),
              "sts3250": (4.2, 4.90 / 4.2, 4.2 * 12.0 / 4.90 ** 2)}
CURRENT_MODELS = ("motor", "bridge")    # upper and lower bound (sts_servo_model)


def servo_type(name):
    return "sts3250" if NAME_TO_ID[name] in STS3250_IDS else "sts3215"


def servo_amps(names, tau, qd, w0, volts, model="motor"):
    """Per-servo supply current (A), shape (ticks, joints), from the servo
    torque tau (N*m) and joint speed qd (rad/s) traces, each servo with its
    own type's constants (SERVO_ELEC). The two models bracket the truth
    (sim/sts_servo_model.py `supply_current`, which they equal on an STS3215):
      motor   I = I_idle + (I_noload - I_idle) |w| / w0 + |tau| / Kt, capped
              at the stall current x V / 12 (the upper bound)
      bridge  I = I_idle + (max(tau w, 0) + K_CU tau^2) / V, K_CU per type
              (the lower bound)
    w0: each servo's effective no-load speed (rad/s), (joints,)."""
    tau = np.atleast_2d(np.asarray(tau, dtype=float))
    qd = np.atleast_2d(np.asarray(qd, dtype=float))
    w0 = np.asarray(w0, dtype=float)
    istall12, kt, k_cu = np.array([SERVO_ELEC[servo_type(n)]
                                   for n in names]).T
    if model == "motor":
        i = (SM.I_IDLE + (SM.I_NOLOAD - SM.I_IDLE)
             * np.clip(np.abs(qd) / w0, 0.0, 1.0) + np.abs(tau) / kt)
        return np.minimum(i, istall12 * volts / 12.0)
    if model == "bridge":
        return SM.I_IDLE + (np.maximum(tau * qd, 0.0) + k_cu * tau ** 2) / volts
    raise ValueError(model)


def branch_amps(names, amps):
    """Sum per-servo currents (ticks, joints; columns in `names` order) per
    power-lite branch: {branch: (ticks,)}, plus "H5 (A)" = J4 + J5,
    "H6 (B)" = J6 + J7 and "bus" = all four. A servo of the map that the plant
    lacks (the neck and arms on the legs-only plant) is counted at its idle
    current on its branch. A name outside the map, or a servo listed twice,
    raises."""
    amps = np.atleast_2d(np.asarray(amps, dtype=float))
    ids = [NAME_TO_ID[n] for n in names]
    if len(set(ids)) != len(ids):
        raise ValueError(f"a servo appears twice in {names}")
    out = {}
    for b, members in BRANCHES.items():
        cols = [k for k, i in enumerate(ids) if i in members]
        absent = len(members - set(ids))
        out[b] = amps[:, cols].sum(axis=1) + absent * SM.I_IDLE
    out["H5 (A)"] = out["J4 R leg"] + out["J5 R upper"]
    out["H6 (B)"] = out["J6 L leg"] + out["J7 L upper"]
    out["bus"] = out["H5 (A)"] + out["H6 (B)"]
    return out


def worst_window(a, n):
    """the largest mean over n consecutive ticks (the whole trace if shorter)"""
    a = np.asarray(a, dtype=float)
    if len(a) <= n:
        return float(a.mean())
    c = np.concatenate([[0.0], np.cumsum(a)])
    return float(((c[n:] - c[:-n]) / n).max())


# what each lead is held against: the bus against the General Driver's 5 A
# servo rating (wiki FAQ), every connector against a ~3 A-class contact
# (wiring.md: KK / 5264-class, unverified)
LIMIT_A = {"bus": 5.0}
LEAD_LIMIT_A = 3.0


def branch_stats(names, tau, qd, w0, volts, dt):
    """{model: {lead: dict(peak, worst2s, rms, mean, over)}} plus, per model,
    the worst single servo's peak ("servo": (name, A)); `over` = seconds
    above the lead's limit (LIMIT_A, else LEAD_LIMIT_A)"""
    out = {}
    n2 = int(round(2.0 / dt))
    for model in CURRENT_MODELS:
        i_j = servo_amps(names, tau, qd, w0, volts, model)
        rows = {b: dict(peak=float(a.max()), worst2s=worst_window(a, n2),
                        rms=float(np.sqrt(np.mean(a ** 2))), mean=float(a.mean()),
                        over=float((a > LIMIT_A.get(b, LEAD_LIMIT_A)).sum() * dt))
                for b, a in branch_amps(names, i_j).items()}
        k = int(np.argmax(i_j.max(axis=0)))
        rows["servo"] = (names[k], float(i_j[:, k].max()))
        out[model] = rows
    return out


def _ref_job(a):
    """one get-up reference replay with its traces (getup_ref_overload.run)"""
    import getup_ref_overload as O
    ref, cond_label, cond, envelope = a
    try:
        r = O.run(ref, cond, envelope, trace=True)
    except Exception as e:      # an unstable rollout is reported, never kills a sweep
        return dict(ref=ref, cond=cond_label, envelope=envelope, err=repr(e))
    st = branch_stats(r["names"], r["tau"], r["qd"], r["w0"], r["volts"], 0.02)
    return dict(ref=ref, cond=cond_label, envelope=envelope, ok=r["ok"],
                stood_t=r["stood_t"], volts=r["volts"], ticks=len(r["tau"]), stats=st)


def trace_refs(all_conds=False, workers=None):
    """The campaign's get-up references (sim/getup_ref_overload.py REFS:
    prone_ikc, the w19k candidate's, and supine_ik) replayed open loop on
    build/v6/sim/bimo_biped_v6ar_meas.xml with STS3250 hip rolls, kp as built, under
    both supply envelopes; the nominal condition, or every robustness
    condition of that study (all_conds)."""
    import getup_ref_overload as O
    conds = O.CONDS if all_conds else O.CONDS[:1]
    jobs = [(ref, lab, c, env) for env in O.ENVELOPES for ref in O.REFS
            for lab, c in conds]
    n = workers or int(os.environ.get("SIM_WORKERS", "8"))
    if n <= 1 or len(jobs) == 1:
        return [_ref_job(j) for j in jobs]
    from multiprocessing import Pool
    with Pool(min(n, len(jobs))) as pool:
        return pool.map(_ref_job, jobs)


DWELL_PLACE_TICKS = 50      # sil_gait.run's place_and_hold(env, q0, 1.0): not part of the motion


def _dwell_job(a):
    """One dwell gait (experiments/roll-balance/dwell.py) through the
    firmware's track player and trim in the SIL (sil_gait.run, model 'dwell',
    the lift gate on as track_host runs it) on the legs-only plant it was
    tuned on (the 'fit' plant: grade's servo chain, per-side roll play),
    recording every control tick's servo torque and joint speed."""
    T_step, direction, plant, n_steps = a
    sys.path.insert(0, os.path.join(ROOT, "experiments", "roll-balance"))
    import sil_gait as SGT
    rec = dict(tau=[], qd=[])
    box = {}
    build0 = SGT.TB.SR.build

    def build(spec, ep_s):
        label, env = build0(spec, ep_s)
        step0 = env.step

        def step(act):
            res = step0(act)
            rec["tau"].append(np.array(env._servo_tau, dtype=float))
            rec["qd"].append(np.array(env.data.qvel[env._sqvel], dtype=float))
            return res
        env.step = step
        box["env"] = env
        return label, env
    SGT.TB.SR.build = build
    try:
        r = SGT.run(T_step, direction, plant, 0.0, n_steps, model="dwell", lift_gate=True)
    except Exception as e:
        return dict(T=T_step, dir=direction, plant=plant, err=repr(e))
    finally:
        SGT.TB.SR.build = build0
    env = box["env"]
    names = list(env._servo_names)
    tau = np.array(rec["tau"])[DWELL_PLACE_TICKS:]
    qd = np.array(rec["qd"])[DWELL_PLACE_TICKS:]
    w0 = (SM._per_joint(env._servo[3], len(names))
          * SM._per_joint(getattr(env, "_w0_prof", 1.0), len(names)))
    volts = float(env.supply_voltage)
    st = branch_stats(names, tau, qd, w0, volts, float(env.control_dt))
    return dict(T=T_step, dir=direction, plant=plant, fell=r["fell"], lifts=r["lifts"],
                n_steps=r["n_steps"], volts=volts, ticks=len(tau), stats=st)


DWELL_T = (6.0, 4.0, 3.2)       # track_host's start, the robot's walks, the SIL floor
DWELL_DIRS = ("march", "fwd", "back", "sideL", "sideR", "turnL", "turnR")


def trace_dwell(T_steps=DWELL_T, dirs=DWELL_DIRS, plant="fit", n_steps=6, workers=None):
    jobs = [(T, d, plant, n_steps) for T in T_steps for d in dirs]
    n = workers or int(os.environ.get("SIM_WORKERS", "8"))
    if n <= 1 or len(jobs) == 1:
        return [_dwell_job(j) for j in jobs]
    from multiprocessing import Pool
    with Pool(min(n, len(jobs))) as pool:
        return pool.map(_dwell_job, jobs)


BRANCH_ROWS = ("bus", "J4 R leg", "J5 R upper", "J6 L leg", "J7 L upper", "H5 (A)", "H6 (B)")


def print_branch_stats(label, st):
    """one case: per branch, peak / worst 2 s / RMS, upper (motor) / lower (bridge)"""
    print(f"  {label}")
    print(f"    {'lead':11s} {'peak A':>13s} {'worst 2 s A':>13s} {'RMS A':>13s} {'s over limit':>13s}"
          "   (upper / lower; limit 5 A bus, 3 A a lead)")
    for b in BRANCH_ROWS:
        u, lo = st["motor"][b], st["bridge"][b]
        print(f"    {b:11s} " + " ".join(f"{u[k]:6.2f}/{lo[k]:<6.2f}" for k in ("peak", "worst2s", "rms", "over")))
    print(f"    worst servo: upper {st['motor']['servo'][0]} {st['motor']['servo'][1]:.2f} A, "
          f"lower {st['bridge']['servo'][0]} {st['bridge']['servo'][1]:.2f} A", flush=True)


def envelope_of(results):
    """per model and branch, the max over the results of each statistic"""
    out = {}
    for model in CURRENT_MODELS:
        out[model] = {b: {k: max(r["stats"][model][b][k] for r in results)
                          for k in ("peak", "worst2s", "rms", "mean", "over")} for b in BRANCH_ROWS}
        out[model]["servo"] = max((r["stats"][model]["servo"] for r in results), key=lambda x: x[1])
    return out


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
    from eval_precision import load_policy, make_env
    run_dir = os.path.join(RUNS, args.run)
    with open(os.path.join(run_dir, "config.json")) as f:
        cfg = json.load(f)
    xml = paths.resolve_plant(cfg.get("xml_path", "bimo_biped_v6ar.xml"))
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


def _jsonable(res):
    return [dict(r, stats={m: {b: (list(v) if b == "servo" else v) for b, v in st.items()}
                           for m, st in r["stats"].items()}) if "stats" in r else r for r in res]


def run_branch_budget(args):
    """--trace refs | dwell: per power-lite branch, both current models"""
    print("bus-current budget per power-lite branch (J4 R leg, J5 R upper, J6 L leg, J7 L upper; "
          "H5 / H6 = the General Driver's ports, J4+J5 / J6+J7)\n"
          "servos: STS3250 hip rolls (51, 52; 4.90 N*m, 4.2 A stall at 12 V, unverified), STS3215 "
          "elsewhere; upper = motor-current model, lower = lossless bridge (sim/sts_servo_model.py)",
          flush=True)
    if args.trace == "refs":
        res = trace_refs(all_conds=args.all_conds)
        for r in res:
            head = f"{r['ref']}  {r['envelope']}  {r['cond']}"
            if "err" in r:
                print(f"  {head}: ERROR {r['err']}")
                continue
            print_branch_stats(f"{head}: {'STANDS' if r['ok'] else 'does NOT stand'} "
                               f"(up at {r['stood_t']} s), {r['volts']:.1f} V, {r['ticks']} ticks",
                               r["stats"])
        good = [r for r in res if "err" not in r]
        if args.all_conds:
            for key in sorted({(r["envelope"], r["ref"]) for r in good}):
                rr = [r for r in good if (r["envelope"], r["ref"]) == key]
                print_branch_stats(f"{key[1]} {key[0]}: the max of each figure over its {len(rr)} conditions",
                                   envelope_of(rr))
        out = args.out or os.path.join(RUNS, "current_budget_refs.json")
    else:
        res = trace_dwell()
        for r in res:
            head = f"dwell T {r['T']:g} s {r['dir']} ({r['plant']})"
            if "err" in r:
                print(f"  {head}: ERROR {r['err']}")
                continue
            print_branch_stats(f"{head}: {'FELL' if r['fell'] else 'walks'}, lifts {r['lifts']}/{r['n_steps']}, "
                               f"{r['volts']:.1f} V, {r['ticks']} ticks", r["stats"])
        good = [r for r in res if "err" not in r and not r["fell"]]
        if good:
            print_branch_stats(f"dwell: the max of each figure over the {len(good)} runs that did not fall",
                               envelope_of(good))
        out = args.out or os.path.join(RUNS, "current_budget_dwell.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        json.dump(_jsonable(res), f, indent=2)
    print(f"saved -> {os.path.relpath(out, ROOT)}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run", default=None,
                   help="trained run under sim/runs/ (policy mode)")
    p.add_argument("--trace", choices=["refs", "dwell", "walk", "getup"], default=None,
                   help="refs: the get-up references, per power-lite branch; dwell: the "
                        "track player's dwell walk, per branch; walk / getup: the 2026-09 "
                        "scripted Gate D walk and seat push (two ports, bridge model)")
    p.add_argument("--all-conds", action="store_true",
                   help="refs: every robustness condition of getup_ref_overload, not only nominal")
    p.add_argument("--episodes", type=int, default=4)
    p.add_argument("--volts", type=float, default=11.1,
                   help="bus voltage (3S nominal; use 10.5 for the landing floor)")
    p.add_argument("--out", default=None)
    args = p.parse_args()

    if args.run:
        run_policy_budget(args)
        return
    if args.trace in ("refs", "dwell"):
        run_branch_budget(args)
        return
    if args.trace == "walk":
        t, a, b, j = trace_walk(args.volts)
        row = report("trace=walk (Gate D, build/v6/sim/bimo_biped_v6ar.xml, deploy "
                     "servo model)", args.volts, t, a, b, j)
        row["trace"] = "walk"
        out = args.out or os.path.join(HERE, "runs", "current_budget_walk.json")
    elif args.trace == "getup":
        t, a, b, j, ok, up, pelvis_z = trace_getup(args.volts)
        extra = (f"get-up outcome: {'STANDING' if ok else 'NOT standing'}  "
                 f"up_z {up:+.2f}  pelvis_z {pelvis_z:.3f}")
        row = report("trace=getup (seat push, r6_asdrawn_rom120)",
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
