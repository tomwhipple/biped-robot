"""Check 2 of the weight-transfer gate (#259): the 2026-11-09 prec16 stride (`stride1_02s`,
flight_20261109T232022Z_prec16) replayed on the walking plant, numbers next to the flight record.

    python check2_replay.py [--plants grade,train] [--terms SPEC] [--seeds 4] [--json out.json] [--md out.md]

Rows (every row is scored by the same metrics() over 0..2 s from the walk command):
  robot            the flight record: encoders (q), the IMU (gyro, fused quaternion)
  closed <plant> record   the prec16 student closed loop, fed the robot's own per-tick command AND
                   gait phase from the record (vx 0.2 at +0; the guard's trip at +0.64 s ramped it
                   to the stand over 0.3 s -- that ramp is in the record, so it is replayed too);
                   firmware chain as grade.py (C2 shaper 8 Hz, 1 s takeover ramp from the stand2
                   arm pose, the robot's reg-58 dq obs); the sim's own guard only counts
  closed <plant> probe    grade.py's `stride1 0.3 robot` probe (the planner's 2.1 cm R lift): a
                   check that this harness reproduces the grade
  goals <plant>    open loop: the robot's WRITTEN goals, tick by tick, into the plant's servo chain
                   (no policy), from the recorded pose 1 s before the walk
  joints           open loop: the robot's MEASURED joint angles imposed by joint equality
                   constraints on the bare plant (experiments/policy/yaw-gap/replay_joints.py's
                   method: no servo, no policy)

Metrics: per-foot lift = the peak rise of each sole above its standing height (sim: the true
L_sole / R_sole geom; robot: unknown -- see `fk`), "fk" = the same sole-height difference from
FORWARD KINEMATICS of the encoders and the IMU attitude (what the robot's sensors can say; blind to
play beyond the encoders, so it is an upper bound on a real lift), pad clearance (sim, lowest pad
above the floor), hip-roll travel (range of the encoder angle), body yaw (end, and the extreme),
peak |gz|, and the flail guard's body-rate EMA peak (the robot tripped at 3.02).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gate_common as GC  # noqa: E402

FREC = os.path.join(GC.HW, "2026-11-09", "flight_20261109T232022Z_prec16.frec")
DT = 0.02
WIN = (0.0, 2.0)            # s from the walk command
T_WALK = 2.0                # sim: the walk command at 2.0 s (grade.py's probes)
_HW = {}


def load_hw():
    """The flight record in policy order; t = 0 at the first tick that carries the walk command."""
    if "r" not in _HW:
        sys.path.insert(0, os.path.join(GC.ROOT, "experiments", "policy", "rca"))
        import rec
        r = rec.load(FREC)
        i0 = int(np.argmax(np.abs(r["cmd"][:, 0]) > 0.01))
        r = dict(r)
        r["i0"] = i0
        r["trel"] = r["t"] - r["t"][i0]
        _HW["r"] = r
    return _HW["r"]


_FK = {}


def fk_soles(q_pol, quat, xml=None):
    """(T, 2) world z of the L_sole and R_sole geoms from joint angles (policy order) and the torso
    attitude (w, x, y, z), torso at a fixed height (only the difference is used)."""
    import mujoco
    import rec
    xml = xml or GC.XML
    if xml not in _FK:
        m = mujoco.MjModel.from_xml_path(xml)
        _FK[xml] = (m, mujoco.MjData(m), [m.jnt_qposadr[m.joint(n).id] for n in rec.POL],
                    m.geom("L_sole").id, m.geom("R_sole").id)
    m, d, adr, gl, gr = _FK[xml]
    out = []
    for q, qt in zip(q_pol, quat):
        d.qpos[:] = m.qpos0
        d.qpos[2] = 0.5
        d.qpos[3:7] = qt
        d.qpos[adr] = q
        mujoco.mj_kinematics(m, d)
        out.append((d.geom_xpos[gl][2], d.geom_xpos[gr][2]))
    return np.array(out)


def metrics(trel, q, gyro, quat, sole_z=None, clr=None, rate_series=None):
    """The gate's numbers over WIN (module doc). trel: s from the walk command. q policy order rad,
    gyro (T,3) body rad/s, quat (T,4) w x y z. sole_z / clr: (T,2) true sole z / pad clearance (sim)."""
    import rec
    trel = np.asarray(trel, float)
    q = np.asarray(q, float)
    w = (trel >= WIN[0]) & (trel <= WIN[1])
    pre = (trel >= -1.0) & (trel < 0.0)
    if not pre.any():
        pre = (trel < 0.0)
    J = {n: i for i, n in enumerate(rec.POL)}
    rng = lambda n: float(np.degrees(q[w, J[n]].max() - q[w, J[n]].min()))
    yaw = np.degrees(np.unwrap([GC.yaw_of(x) for x in quat]))
    y0 = float(np.interp(-DT, trel, yaw))
    yw = yaw[w] - y0
    gz = np.asarray(gyro, float)[w, 2]
    rate = GC.flail_rate(gyro) if rate_series is None else np.asarray(rate_series)
    roll = np.degrees([GC.roll_of(x) for x in quat])
    r0 = float(np.median(roll[pre]))
    fk = fk_soles(q, quat)
    dz = fk[:, 1] - fk[:, 0]                       # R above L
    dz0 = float(np.median(dz[pre]))
    out = dict(hip_roll_L=rng("L_hip_roll"), hip_roll_R=rng("R_hip_roll"),
               hip_pitch_L=rng("L_hip_pitch"), hip_pitch_R=rng("R_hip_pitch"),
               knee_L=rng("L_knee"), knee_R=rng("R_knee"),
               ankle_roll_L=rng("L_ankle_roll"), ankle_roll_R=rng("R_ankle_roll"),
               yaw_end=float(yw[-1]), yaw_min=float(yw.min()), yaw_max=float(yw.max()),
               gz_peak=float(gz[np.argmax(np.abs(gz))]), rate_peak=float(rate[w].max()),
               roll_L=float(-(roll[w] - r0).min()), roll_R=float((roll[w] - r0).max()),
               fk_lift_L=float(1e3 * max(0.0, -(dz[w] - dz0).min())), fk_lift_R=float(1e3 * max(0.0, (dz[w] - dz0).max())))
    if sole_z is not None:
        sz = np.asarray(sole_z, float)
        z0 = np.median(sz[pre], axis=0)
        lift = sz[w] - z0
        out.update(lift_L=float(1e3 * lift[:, 0].max()), lift_R=float(1e3 * lift[:, 1].max()))
    if clr is not None:
        c = np.asarray(clr, float)[w]
        out.update(clear_L=float(1e3 * c[:, 0].max()), clear_R=float(1e3 * c[:, 1].max()),
                   air_L=float(DT * (c[:, 0] > 0.005).sum()), air_R=float(DT * (c[:, 1] > 0.005).sum()))
    return out


def fk_trace(t_lo=0.3, t_hi=1.5, every=2):
    """The robot's own kinematics through the R swing: per tick, the lowest R sole pad minus the
    lowest L pad and the R-L sole-centre difference (mm, vs the stand before the walk), and each
    sole's tilt from level (deg), from the encoders + the fused IMU attitude on the plant's geometry.
    -> list of (t, pad_dz_mm, centre_dz_mm, tilt_L_deg, tilt_R_deg)"""
    import mujoco
    import rec
    hw = load_hw()
    m = mujoco.MjModel.from_xml_path(GC.XML)
    d = mujoco.MjData(m)
    adr = [m.jnt_qposadr[m.joint(n).id] for n in rec.POL]
    nm = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
    pads = {s: [g for g in range(m.ngeom) if nm(g).startswith(f"{s}_pad_")] for s in "LR"}
    sole = {s: m.geom(f"{s}_sole").id for s in "LR"}
    rows = []
    for k, tk in enumerate(hw["trel"]):
        d.qpos[:] = m.qpos0
        d.qpos[2] = 0.5
        d.qpos[3:7] = hw["quat"][k]
        d.qpos[adr] = hw["q"][k]
        mujoco.mj_kinematics(m, d)
        pz = {s: min(d.geom_xpos[g][2] - m.geom_size[g][0] for g in pads[s]) for s in "LR"}
        tilt = {s: math.degrees(math.acos(min(1.0, abs(d.geom_xmat[sole[s]].reshape(3, 3)[2, 2])))) for s in "LR"}
        rows.append([tk, pz["R"] - pz["L"], d.geom_xpos[sole["R"]][2] - d.geom_xpos[sole["L"]][2], tilt["L"], tilt["R"]])
    r = np.array(rows)
    pre = (r[:, 0] >= -1.0) & (r[:, 0] < 0.0)
    r[:, 1:3] = 1e3 * (r[:, 1:3] - np.median(r[pre, 1:3], axis=0))
    w = (r[:, 0] >= t_lo) & (r[:, 0] <= t_hi)
    return [tuple(float(x) for x in row) for row in r[w][::every]]


def robot_row():
    r = load_hw()
    m = metrics(r["trel"], r["q"], r["gyro"], r["quat"])
    m["src"] = os.path.relpath(FREC, GC.MAIN)
    return m


# --------------------------------------------------------------------------------------------
# closed loop: the prec16 student
# --------------------------------------------------------------------------------------------
def _policy_env(plant, terms, ep_s):
    if plant == "grade":
        return GC.grade_env(terms=terms, ep_s=ep_s)
    import eval_precision as EP
    env = GC.make_plant(plant, terms=terms, ep_s=ep_s)
    act = EP.load_policy(GC.run_dir(), int(env.observation_space.shape[0]), int(env.action_space.shape[0]))
    return env, act, np.zeros(env._n_servo)


def closed_loop(job):
    """(plant, schedule, seed, terms) -> metrics (+ the sim's own guard trip)."""
    plant, schedule, seed, terms = job
    import grade as G
    import simh
    import stand_sim as S
    import rec
    run_s = 6.0
    env, act, zoff = _policy_env(plant, terms, run_s + 4.0)
    hz = S.trained_hz(env)
    rng = np.random.default_rng(1000 + seed)
    env.domain_rand = False
    env.push_prob = 0.0
    env.set_dq_obs(G.DQ_ROBOT["window"], G.DQ_ROBOT["quantum"])
    env.reset(seed=seed)
    env._zero_off = zoff.copy()
    ang = math.radians(rng.normal(0.0, 1.0)); ax = rng.normal(size=3); ax /= np.linalg.norm(ax)
    k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    env._imu_R = np.eye(3) + math.sin(ang) * k + (1 - math.cos(ang)) * k @ k
    env._imu_gyro_bias = rng.uniform(-0.01, 0.01, 3)
    env.domain_rand = True
    nj = env._nq_act
    env.set_command(*G.STAND_CMD)
    simh.place_and_hold(env, np.clip(S.arm_pose(), env._lo, env._hi), 1.5)
    env._prev_action[:] = 0.0
    if schedule == "probe":
        sched = G.at_clock(dict(cmd_schedule=G.stride(0.3)), hz)["cmd_schedule"]
    elif schedule == "script":
        sched = G.at_clock(dict(cmd_schedule=G.Stride(0.2, T_WALK, 1.0, -0.05)), hz)["cmd_schedule"]
    else:
        sched = None
    hw = load_hw() if schedule == "record" else None
    off = int(round(T_WALK / DT)) - (hw["i0"] if hw else 0)        # sim tick i <-> robot tick i - off
    env.set_command(*G.STAND_CMD)
    w0 = S.H.pin_gait_clock(env, hz)
    env._gait_freq = 0.0
    env._gait_phase = float(w0)
    clk = dict(ph=float(w0), f=hz, hz=hz, held=False)
    env._obs_hist = []
    first = env._obs()
    env._obs_hist = [first.copy() for _ in range(env.obs_hist_len - 1)]
    obs = np.concatenate([first] + list(env._obs_hist))
    qm = first[:nj].astype(np.float64)
    sh = simh.Shaper(GC.SHAPER_HZ); sh.reset(qm)
    hold = qm.copy(); cur = {"t": hold}
    env._action_to_ctrl = lambda a_: cur["t"]
    fg = G.FlailGuard(env._default)
    pads = GC.pad_gids(env)
    sole = env._sole_gids
    n = int(round(run_s / DT))
    T, Q, GY, QT, SZ, CL = [], [], [], [], [], []
    v = dict(stand_phase=math.pi / 2)
    fell = False
    for i in range(n):
        a = np.clip(act(obs.copy()), -1, 1)
        raw = rec.action_to_angles(a)
        tgt = sh(raw)
        if i < S.RAMP:
            tgt = hold + (i + 1) / S.RAMP * (tgt - hold)
        cur["t"] = tgt
        if hw is not None:
            kk = i + 1 - off
            if 0 <= kk < len(hw["cmd"]):
                env.set_command(*hw["cmd"][kk])
                env._gait_phase = float(hw["phase"][kk])
            else:
                env.set_command(*G.STAND_CMD)
                env._gait_phase = S.advance_clock(clk, "coast", v, env._cmd, DT)
        else:
            for ts, c in sched:
                if int(round(ts / DT)) == i + 1:
                    env.set_command(*c)
            env._gait_phase = S.advance_clock(clk, "coast", v, env._cmd, DT)
        obs, _, term, _, info = env.step(a)
        d = env.data
        gyro = np.array(d.qvel[3:6])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(info["up_z"])))))
        fg(i, raw, tgt, gyro, tilt, S.stand_gate(env._cmd), i >= S.RAMP)
        T.append((i + 1) * DT - T_WALK)
        Q.append(obs[:nj].astype(np.float64)); GY.append(gyro); QT.append(np.array(d.qpos[3:7]))
        SZ.append((d.geom_xpos[sole[0]][2], d.geom_xpos[sole[1]][2]))
        c = GC.clearance(env, pads); CL.append((c["L"], c["R"]))
        if term:
            fell = True
            break
    m = metrics(T, Q, GY, QT, sole_z=SZ, clr=CL)
    # grade_recover.py's own lift (sole z over ticks 75.. vs the median of the first 50), for the
    # harness check against the planner's numbers
    sz = np.array(SZ)
    m["grade_lift_cm"] = (100 * (sz[75:] - np.median(sz[:50], axis=0)).max(0)).round(2).tolist()
    m.update(fell=fell, trip=fg.trip, plant=plant, schedule=schedule, seed=seed)
    return m


# --------------------------------------------------------------------------------------------
# open loop: the robot's goals through the servo chain, and its joints by constraint
# --------------------------------------------------------------------------------------------
def goals_replay(job):
    plant, terms = job
    import simh
    hw = load_hw()
    i0 = hw["i0"]
    ks = max(0, i0 - 50)
    ke = min(len(hw["t"]), i0 + int(round((WIN[1] + 1.0) / DT)))
    if plant == "grade":
        env, act, zoff = GC.grade_env(terms=terms, ep_s=30.0)
    else:
        env = GC.make_plant(plant, terms=terms, ep_s=30.0)
        zoff = np.zeros(env._n_servo)
    env.domain_rand = False
    env.reset(seed=0)
    env._zero_off = zoff.copy()
    simh.place_and_hold(env, np.clip(hw["q"][ks], env._lo, env._hi), 1.5)
    pads = GC.pad_gids(env)
    sole = env._sole_gids
    cur = {"t": hw["goal"][ks]}
    env._action_to_ctrl = lambda a_: cur["t"]
    T, Q, GY, QT, SZ, CL = [], [], [], [], [], []
    a0 = np.zeros(env.action_space.shape[0], np.float32)
    fell = False
    for k in range(ks, ke):
        cur["t"] = hw["goal"][k]
        obs, _, term, _, info = env.step(a0)
        d = env.data
        T.append(hw["trel"][k]); Q.append(obs[:env._nq_act].astype(np.float64))
        GY.append(np.array(d.qvel[3:6])); QT.append(np.array(d.qpos[3:7]))
        SZ.append((d.geom_xpos[sole[0]][2], d.geom_xpos[sole[1]][2]))
        c = GC.clearance(env, pads); CL.append((c["L"], c["R"]))
        if term:
            fell = True
            break
    m = metrics(T, Q, GY, QT, sole_z=SZ, clr=CL)
    m.update(fell=fell, plant=plant, schedule="goals")
    return m


def joints_replay(job):
    """The measured joints imposed by joint equality constraints (replay_joints.py's build)."""
    (mu,) = job
    import mujoco
    import rec
    hw = load_hw()
    i0 = hw["i0"]
    spec = mujoco.MjSpec.from_file(GC.XML)
    for j in rec.POL:
        e = spec.add_equality()
        e.type = mujoco.mjtEq.mjEQ_JOINT
        e.objtype = mujoco.mjtObj.mjOBJ_JOINT
        e.name1 = j
        e.name = "fix_" + j
        e.data[:5] = 0
        e.solref[:2] = [0.004, 1.0]
        e.solimp[:3] = [0.98, 0.999, 0.0005]
    m = spec.compile()
    for a in range(m.nu):
        m.actuator_gainprm[a, 0] = 0
        m.actuator_biasprm[a, :3] = 0
    m.dof_frictionloss[:] = 0
    m.dof_damping[6:] = 0
    nm = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
    pads = {s: [g for g in range(m.ngeom) if nm(g).startswith(f"{s}_pad_")] for s in "LR"}
    if mu is not None:
        for g in pads["L"] + pads["R"] + [m.geom("floor").id]:
            m.geom_friction[g, 0] = mu
    d = mujoco.MjData(m)
    jq = [m.jnt_qposadr[m.joint(j).id] for j in rec.POL]
    eqs = [m.equality("fix_" + j).id for j in rec.POL]
    ks = max(0, i0 - 50)
    q0 = hw["q"][ks]
    for e, x in zip(eqs, q0):
        m.eq_data[e, 0] = x
    d.qpos[2] = 0.5
    d.qpos[jq] = q0
    mujoco.mj_forward(m, d)
    z = min(d.geom_xpos[g, 2] - m.geom_size[g, 0] for g in pads["L"] + pads["R"])
    d.qpos[2] -= z - 0.001
    for _ in range(int(1.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    t, qrec = hw["trel"], hw["q"]
    sole = (m.geom("L_sole").id, m.geom("R_sole").id)
    T, Q, GY, QT, SZ, CL = [], [], [], [], [], []
    t0, t1 = t[ks], min(t[-1], WIN[1] + 1.0)
    nsub = int(round(DT / m.opt.timestep))
    for kk in range(int((t1 - t0) / DT)):
        for s in range(nsub):
            now = t0 + kk * DT + (s + 1) * m.opt.timestep
            for i, e in enumerate(eqs):
                m.eq_data[e, 0] = np.interp(now, t, qrec[:, i])
            mujoco.mj_step(m, d)
        T.append(t0 + (kk + 1) * DT)
        Q.append(np.array(d.qpos[jq])); GY.append(np.array(d.qvel[3:6])); QT.append(np.array(d.qpos[3:7]))
        SZ.append((d.geom_xpos[sole[0]][2], d.geom_xpos[sole[1]][2]))
        CL.append(tuple(min(d.geom_xpos[g][2] - m.geom_size[g][0] for g in pads[s_]) for s_ in "LR"))
    out = metrics(T, Q, GY, QT, sole_z=SZ, clr=CL)
    out.update(plant="bare plant", schedule=f"joints mu {mu if mu is not None else 'xml'}")
    return out


KEYS = [("lift_L", "lift L mm"), ("lift_R", "lift R mm"), ("clear_L", "pad clear L mm"), ("clear_R", "pad clear R mm"),
        ("fk_lift_L", "FK lift L mm"), ("fk_lift_R", "FK lift R mm"),
        ("roll_L", "torso roll to L deg"), ("roll_R", "torso roll to R deg"),
        ("hip_roll_L", "hip roll L deg"), ("hip_roll_R", "hip roll R deg"),
        ("knee_L", "knee L deg"), ("knee_R", "knee R deg"),
        ("yaw_end", "yaw +2 s deg"), ("yaw_min", "yaw min deg"), ("gz_peak", "gz peak rad/s"),
        ("rate_peak", "guard rate peak rad/s")]


def mean_rows(rows):
    out = {}
    for k, _ in KEYS:
        vals = [r[k] for r in rows if r.get(k) is not None]
        if vals:
            out[k] = (float(np.mean(vals)), float(np.min(vals)), float(np.max(vals)))
    out["n"] = len(rows)
    out["falls"] = sum(bool(r.get("fell")) for r in rows)
    out["trips"] = [r.get("trip") for r in rows]
    if rows and "grade_lift_cm" in rows[0]:
        g = np.array([r["grade_lift_cm"] for r in rows])
        out["grade_lift_cm"] = g.mean(0).round(2).tolist()
    return out


def table(rows):
    L = ["| row | " + " | ".join(h for _, h in KEYS) + " | falls / guard trips |",
         "|---|" + "---|" * len(KEYS) + "---|"]
    for name, r in rows:
        cells = []
        for k, _ in KEYS:
            v = r.get(k)
            if v is None:
                cells.append("unknown" if name == "robot" and k in ("lift_L", "lift_R", "clear_L", "clear_R") else "-")
            elif isinstance(v, tuple):
                cells.append(f"{v[0]:+.1f}" + (f" [{v[1]:+.1f}, {v[2]:+.1f}]" if r.get("n", 1) > 1 and abs(v[2] - v[1]) > 0.05 else ""))
            else:
                cells.append(f"{v:+.1f}")
        ft = "-"
        if "n" in r:
            tr = [t for t in r["trips"] if t]
            ft = f"{r['falls']}/{len(tr)} of {r['n']}" + (f" (first {min(t[1] for t in tr) - T_WALK:+.2f} s, {tr[0][0]})" if tr else "")
        elif "fell" in r:
            ft = "fell" if r["fell"] else "0"
        elif name == "robot":
            ft = "guard trip at +0.64 s (rate 3.02)"
        L.append(f"| {name} | " + " | ".join(cells) + f" | {ft} |")
    return "\n".join(L)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--plants", default="grade,train")
    ap.add_argument("--schedules", default="record,probe")
    ap.add_argument("--terms", default="")
    ap.add_argument("--seeds", type=int, default=4)
    ap.add_argument("--no-open", action="store_true", help="skip the open-loop rows")
    ap.add_argument("--fk-trace", action="store_true", help="print the robot's encoder+IMU sole kinematics through the swing and exit")
    ap.add_argument("--json", default=None)
    ap.add_argument("--md", default=None)
    ap.add_argument("--workers", type=int, default=int(os.environ.get("WORKERS", "12")))
    a = ap.parse_args(argv)
    terms = GC.parse_terms(a.terms)
    if a.fk_trace:
        print("| t s | lowest R pad - lowest L pad, mm | R - L sole centre, mm | L sole tilt deg | R sole tilt deg |")
        print("|---|---|---|---|---|")
        for row in fk_trace():
            print("| {:.2f} | {:+.1f} | {:+.1f} | {:.1f} | {:.1f} |".format(*row))
        return None
    plants = [p for p in a.plants.split(",") if p]
    rows = [("robot", robot_row())]
    with ProcessPoolExecutor(max_workers=a.workers) as ex:
        cl = {(p, s): [ex.submit(closed_loop, (p, s, k, terms)) for k in range(a.seeds)]
              for p in plants for s in a.schedules.split(",") if s}
        op = {} if a.no_open else {("goals", p): ex.submit(goals_replay, (p, terms)) for p in plants}
        jt = {} if a.no_open or terms else {("joints", mu): ex.submit(joints_replay, (mu,)) for mu in (None, 0.5)}
        for (p, s), fs in cl.items():
            rows.append((f"closed {p} {s}", mean_rows([f.result() for f in fs])))
        for (_, p), f in op.items():
            rows.append((f"goals {p}", f.result()))
        for (_, mu), f in jt.items():
            r = f.result()
            rows.append((r["schedule"], r))
    md = table(rows)
    hdr = (f"Check 2, the 2026-11-09 prec16 stride on the walking plant ({GC.XML_NAME})"
           + (f", terms {terms}" if terms else "") + f"; metrics over {WIN[0]:g}..{WIN[1]:g} s from the walk command")
    md = hdr + "\n\n" + md
    for name, r in rows:
        if "grade_lift_cm" in r:
            md += f"\n- {name}: grade_recover-style lift L/R {r['grade_lift_cm'][0]:.2f} / {r['grade_lift_cm'][1]:.2f} cm"
    print(md)
    if a.json:
        json.dump(dict(terms=terms, rows=rows), open(a.json, "w"), indent=1, default=str)
    if a.md:
        open(a.md, "w").write(md + "\n")
    return rows


if __name__ == "__main__":
    main()
