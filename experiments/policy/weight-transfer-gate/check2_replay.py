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
# the flight records the replays can run on: name -> (frec, which walk command (0 = the first)).
# GATE_REC (or --record) picks one; 1109 is the check. 1106t1 / 1106t2 are omni5's stride1_03 and
# steps5_03 of 2026-11-06 (experiments/policy/yaw-gap: the sign check of this pipeline)
RECORDS = {"1109": (FREC, 0),
           "1106t1": (os.path.join(GC.HW, "2026-11-06", "flight_20261106T160828Z_omni5.frec"), 0),
           "1106t2": (os.path.join(GC.HW, "2026-11-06", "flight_20261106T160828Z_omni5.frec"), 1),
           # 2026-11-10 (round 3): the 11-09 stride repeated with floor cameras (2026-11-10-robot/README.md)
           "1110": (os.path.join(GC.HW, "2026-11-10", "m271", "flight_snapshot_0035Z.frec"), 0)}


def record_name():
    return os.environ.get("GATE_REC", "1109")
DT = 0.02
WIN = (0.0, 2.0)            # s from the walk command
T_WALK = 2.0                # sim: the walk command at 2.0 s (grade.py's probes)
_HW = {}


def load_hw():
    """The flight record in policy order; t = 0 at the first tick that carries the walk command."""
    name = record_name()
    if name not in _HW:
        import rec
        path, which = RECORDS[name]
        r = rec.load(path)
        mv = np.abs(r["cmd"][:, 0]) > 0.01
        starts = [0] if mv[0] else []
        starts += list(np.where(mv[1:] & ~mv[:-1])[0] + 1)
        i0 = int(starts[which])
        r = dict(r)
        r["i0"] = i0
        r["trel"] = r["t"] - r["t"][i0]
        r["path"] = path
        _HW[name] = r
    return _HW[name]


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
               yaw_04=float(np.interp(0.4, trel[w], yw)), yaw_07=float(np.interp(0.7, trel[w], yw)),
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
        tw = trel[w]
        out.update(clear_L=float(1e3 * c[:, 0].max()), clear_R=float(1e3 * c[:, 1].max()),
                   air_L=float(DT * (c[:, 0] > 0.005).sum()), air_R=float(DT * (c[:, 1] > 0.005).sum()))
        if c.shape[1] >= 6:
            for k, sd in enumerate("LR"):
                end = np.where(c[:, 2 + k] > 0.003)[0]          # one end of the sole 3 mm up
                off = np.where(c[:, k] > 0.001)[0]              # every pad 1 mm up
                out[f"t_end_{sd}"] = float(tw[end[0]]) if len(end) else None
                out[f"t_off_{sd}"] = float(tw[off[0]]) if len(off) else None
                out[f"tilt_off_{sd}"] = float(c[off[0], 4 + k]) if len(off) else None
                out[f"tilt_end_{sd}"] = float(c[end[0], 4 + k]) if len(end) else None
    up = np.array([GC.up_of(x) for x in quat])
    tiltd = np.degrees(np.arccos(np.clip(up[:, 2], -1, 1)))
    for tt in (0.3, 0.5, 0.6):
        out[f"torso_tilt_{int(tt * 10):02d}"] = float(np.interp(tt, trel, tiltd))
    out["up_x_06"] = float(np.interp(0.6, trel, up[:, 0]))
    out["up_y_06"] = float(np.interp(0.6, trel, up[:, 1]))
    ww = (trel >= 0.0) & (trel <= 0.7)
    out["torso_tilt_max07"] = float(tiltd[ww].max())
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
    m["src"] = os.path.relpath(load_hw()["path"], GC.MAIN)
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
        CL.append(GC.foot_state(env.model, env.data, pads, env._sole_gids))
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
        CL.append(GC.foot_state(env.model, env.data, pads, env._sole_gids))
        if term:
            fell = True
            break
    m = metrics(T, Q, GY, QT, sole_z=SZ, clr=CL)
    m.update(fell=fell, plant=plant, schedule="goals")
    return m


JOINT_VARIANTS = {
    # item 2 of round 2 (README): perturbations that might keep the R foot loaded on 11-09
    "plant": {},
    "CoM 10 mm toward R": dict(com_y=-0.010),
    "CoM 20 mm toward R": dict(com_y=-0.020),
    "CoM 30 mm toward R": dict(com_y=-0.030),
    "torso x0.6, legs x1.2": dict(torso=0.6, legs=1.2),
    "pelvis (hip rolls) 4 deg toward R": dict(hip_roll_off=4.0),
    "pelvis (hip rolls) 8 deg toward R": dict(hip_roll_off=8.0),
    "whole body (ankle rolls) 4 deg toward R": dict(ankle_roll_off=4.0),
    "whole body (ankle rolls) 8 deg toward R": dict(ankle_roll_off=8.0),
    "whole body (ankle rolls) 12 deg toward R": dict(ankle_roll_off=12.0),
    "CoM 50 mm toward R": dict(com_y=-0.050),
    # item 3: the series springs fitted to the 10-24 clamp (fit_series.py), as real sprung joints
    "series spring, hip rolls 14.2 N m/rad": dict(ser_hip=14.2),
    "series spring, ankle rolls 27.8 N m/rad": dict(ser_ankle=27.8),
    "series spring, both 40.1 N m/rad": dict(ser_hip=40.1, ser_ankle=40.1),
}


def joints_replay(job):
    """The measured joints imposed by joint equality constraints (replay_joints.py's build).
    job = (mu,) or (mu, variant): variant keys (all optional)
      com_y            torso CoM lateral offset, m (+ = robot's left)
      torso, legs      mass and inertia x of the torso / of every other body
      hip_roll_off     deg added to BOTH hip-roll targets toward the robot's right: a constant
                       pelvis roll toward R relative to both legs (the hip-roll axes are +x, so
                       toward R is -offset in the joint frame)
      ankle_roll_off   the same at both ankle rolls: the whole body above the feet leans toward R
      ser_hip, ser_ankle  N m/rad: a passive sprung hinge in series with each hip roll / ankle
                       roll on the output side (a second joint in the same body, same axis); the
                       equality constraint holds the SERVO joint at the recorded encoder angle"""
    mu = job[0]
    var = dict(job[1]) if len(job) > 1 else {}
    import mujoco
    import rec
    hw = load_hw()
    i0 = hw["i0"]
    spec = mujoco.MjSpec.from_file(GC.XML)
    for side in "LR":
        for jn, key in (("hip_roll", "ser_hip"), ("ankle_roll", "ser_ankle")):
            if var.get(key):
                GC.add_series_joint(spec, f"{side}_{jn}", float(var[key]))
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
    ser = [m.jnt_dofadr[j] for j in range(m.njnt) if m.joint(j).name.endswith("_ser")]
    keep = {a: m.dof_damping[a] for a in ser}
    m.dof_damping[6:] = 0
    for a, v in keep.items():
        m.dof_damping[a] = v
    tb = m.body("torso").id
    if "torso" in var:
        m.body_mass[tb] *= var["torso"]
        m.body_inertia[tb] *= var["torso"]
    if "legs" in var:
        for b in range(1, m.nbody):
            if b != tb:
                m.body_mass[b] *= var["legs"]
                m.body_inertia[b] *= var["legs"]
    if "com_y" in var:
        m.body_ipos[tb][1] += var["com_y"]
    if "torso_kg" in var:
        x = var["torso_kg"] / float(m.body_mass[tb])
        m.body_mass[tb] *= x
        m.body_inertia[tb] *= x
    if "pack" in var:
        GC.add_box_to_body(m, tb, var["pack"], GC.PACK_POS, GC.PACK_HALF)
    off = np.zeros(12)
    for i, n in enumerate(rec.POL):
        if n.endswith("hip_roll"):
            off[i] = -math.radians(var.get("hip_roll_off", 0.0))
        if n.endswith("ankle_roll"):
            off[i] = -math.radians(var.get("ankle_roll_off", 0.0))
    nm = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
    pads = {s: [g for g in range(m.ngeom) if nm(g).startswith(f"{s}_pad_")] for s in "LR"}
    if mu is not None:
        for g in pads["L"] + pads["R"] + [m.geom("floor").id]:
            m.geom_friction[g, 0] = mu
    d = mujoco.MjData(m)
    jq = [m.jnt_qposadr[m.joint(j).id] for j in rec.POL]
    eqs = [m.equality("fix_" + j).id for j in rec.POL]
    ks = max(0, i0 - 50)
    q0 = hw["q"][ks] + off
    for e, x in zip(eqs, q0):
        m.eq_data[e, 0] = x
    d.qpos[2] = 0.5
    d.qpos[jq] = q0
    mujoco.mj_forward(m, d)
    z = min(d.geom_xpos[g, 2] - m.geom_size[g, 0] for g in pads["L"] + pads["R"])
    d.qpos[2] -= z - 0.001
    for _ in range(int(1.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    t, qrec = hw["trel"], hw["q"] + off
    sole = (m.geom("L_sole").id, m.geom("R_sole").id)
    foot = {s_: m.body(f"{s_}_foot").id for s_ in "LR"}
    f6 = np.zeros(6)
    T, Q, GY, QT, SZ, CL, FZ = [], [], [], [], [], [], []
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
        CL.append(GC.foot_state(m, d, pads, sole))
        fz = {"L": 0.0, "R": 0.0}
        for ci in range(d.ncon):
            c = d.contact[ci]
            for s_ in "LR":
                if m.geom_bodyid[c.geom1] == foot[s_] or m.geom_bodyid[c.geom2] == foot[s_]:
                    mujoco.mj_contactForce(m, d, ci, f6)
                    fz[s_] += abs(f6[0])
        FZ.append((fz["L"], fz["R"]))
    out = metrics(T, Q, GY, QT, sole_z=SZ, clr=CL)
    T, FZ = np.array(T), np.array(FZ)
    w = (T >= 0.4) & (T <= 0.7)
    share = FZ[:, 1] / np.maximum(FZ.sum(1), 1e-9)
    upz = np.array([GC.up_of(x)[2] for x in QT])
    out.update(plant="bare plant", schedule=f"joints mu {mu if mu is not None else 'xml'}",
               fell=bool((np.degrees(np.arccos(np.clip(upz, -1, 1))) > 60.0).any()),
               rload_min=float(share[w].min()), rload_mean=float(share[w].mean()), variant=var)
    return out


STRIDE3_JOINTS = {"joints, plant": {},
                  "joints, mass (torso 0.4855 kg + 83 g pack)": dict(torso_kg=0.4855, pack=0.083),
                  "joints, mass + refit springs (real sprung joints)": dict(torso_kg=0.4855, pack=0.083,
                                                                           ser_ankle=17.75, ser_hip=11.0)}
STRIDE3_CLOSED = {"closed, grade as is": "", "closed, grade + MASS_SYM": GC.MASS_SYM,
                  "closed, grade + REFIT3": GC.REFIT3}
S3KEYS = [("torso_tilt_03", "tilt +0.3"), ("torso_tilt_05", "tilt +0.5"), ("torso_tilt_06", "tilt +0.6"),
          ("up_x_06", "up_x +0.6"), ("up_y_06", "up_y +0.6"),
          ("t_end_R", "R end up t"), ("tilt_end_R", "R tilt then"), ("t_off_R", "R off t"), ("tilt_off_R", "R tilt then"),
          ("t_off_L", "L off t"), ("lift_R", "R lift mm"), ("lift_L", "L lift mm"),
          ("yaw_07", "yaw +0.7"), ("yaw_end", "yaw +2 s"), ("gz_peak", "gz peak"), ("rate_peak", "guard rate peak")]
ROBOT_FEET_1110 = dict(t_end_R="~0.42 (video)", tilt_end_R="tilted (video)", t_off_R="~0.59 (video)",
                       tilt_off_R="~10-15 (video)", t_off_L="~0.59 (video)")


def stride3(seeds=8, workers=24):
    """round 3, item 3: the 2026-11-10 stride (GATE_REC=1110) on grade as is, + MASS_SYM, + REFIT3,
    closed loop under the recorded command, and the joints replay with the same mass / springs."""
    os.environ["GATE_REC"] = "1110"
    rows = [("robot 2026-11-10", robot_row())]
    with ProcessPoolExecutor(max_workers=workers) as ex:
        cl = {n: [ex.submit(closed_loop, ("grade", "record", k, GC.parse_terms(t))) for k in range(seeds)]
              for n, t in STRIDE3_CLOSED.items()}
        jt = {n: ex.submit(joints_replay, (0.5, v)) for n, v in STRIDE3_JOINTS.items()}
        for n, fs in cl.items():
            rs = [f.result() for f in fs]
            agg = {}
            for k, _ in S3KEYS:
                vals = [r[k] for r in rs if r.get(k) is not None]
                agg[k] = (float(np.mean(vals)), float(np.min(vals)), float(np.max(vals)), len(vals)) if vals else None
            agg["falls"] = sum(bool(r["fell"]) for r in rs)
            agg["n"] = len(rs)
            agg["trips"] = [r["trip"] for r in rs]
            rows.append((n, agg))
        for n, f in jt.items():
            rows.append((n, f.result()))
    L = ["Round 3: the 2026-11-10 stride (`flight_snapshot_0035Z`), torso tilt = acos(up_z) deg; up = the torso z "
         "axis in the world, yaw stripped (the record's own `up`); foot times s from the walk command: `end up` = "
         "the sole's highest pad 3 mm off the floor, `off` = every pad 1 mm off; tilt then = the sole's angle from "
         "level (deg). Closed loop: mean [min, max] over seeds (n = seeds where it happened).", "",
         "| row | " + " | ".join(h for _, h in S3KEYS) + " | falls |", "|---|" + "---|" * (len(S3KEYS) + 1)]
    for n, r in rows:
        cells = []
        for k, _ in S3KEYS:
            v = r.get(k)
            if v is None:
                cells.append(ROBOT_FEET_1110.get(k, "unknown") if n.startswith("robot") else "never")
            elif isinstance(v, tuple):
                cells.append(f"{v[0]:+.2f}" + (f" [{v[1]:+.2f}, {v[2]:+.2f}]" if v[2] - v[1] > 0.005 else "")
                             + (f" n={v[3]}" if v[3] < r["n"] else ""))
            else:
                cells.append(f"{v:+.2f}")
        fl = f"{r['falls']}/{r['n']}" if "n" in r else ("fell" if r.get("fell") else ("0" if "fell" in r else "-"))
        L.append(f"| {n} | " + " | ".join(cells) + f" | {fl} |")
    return rows, "\n".join(L)


def joint_variants(names=None, mu=0.5, workers=12):
    """item 2: the 11-09 joints replay under each JOINT_VARIANTS perturbation -> [(name, metrics)]"""
    names = names or list(JOINT_VARIANTS)
    with ProcessPoolExecutor(max_workers=workers) as ex:
        rs = list(ex.map(joints_replay, [(mu, JOINT_VARIANTS[n]) for n in names]))
    return list(zip(names, rs))


def variants_table(rows, robot):
    L = ["| joints replay (mu 0.5) | R load share +0.4..+0.7 s, min / mean | lift L / R mm | yaw +0.7 s deg | yaw +2 s deg | gz peak rad/s | guard-rate peak rad/s |",
         "|---|---|---|---|---|---|---|",
         f"| robot | unknown | unknown (FK {robot['fk_lift_L']:.1f} / {robot['fk_lift_R']:.1f}) | {robot['yaw_07']:+.1f} | "
         f"{robot['yaw_end']:+.1f} | {robot['gz_peak']:+.1f} | {robot['rate_peak']:.1f} |"]
    for n, r in rows:
        L.append(f"| {n} | {r['rload_min']:.2f} / {r['rload_mean']:.2f} | {r['lift_L']:.1f} / {r['lift_R']:.1f} | "
                 f"{r['yaw_07']:+.1f} | {r['yaw_end']:+.1f} | {r['gz_peak']:+.1f} | {r['rate_peak']:.1f} |")
    return "\n".join(L)


KEYS = [("lift_L", "lift L mm"), ("lift_R", "lift R mm"), ("clear_L", "pad clear L mm"), ("clear_R", "pad clear R mm"),
        ("fk_lift_L", "FK lift L mm"), ("fk_lift_R", "FK lift R mm"),
        ("roll_L", "torso roll to L deg"), ("roll_R", "torso roll to R deg"),
        ("hip_roll_L", "hip roll L deg"), ("hip_roll_R", "hip roll R deg"),
        ("knee_L", "knee L deg"), ("knee_R", "knee R deg"),
        ("yaw_04", "yaw +0.4 s deg"), ("yaw_07", "yaw +0.7 s deg"),
        ("yaw_end", "yaw +2 s deg"), ("yaw_min", "yaw min deg"), ("yaw_max", "yaw max deg"), ("gz_peak", "gz peak rad/s"),
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
        elif name == "robot" and record_name() == "1109":
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
    ap.add_argument("--stride3", action="store_true", help="round-3 item 3: the 2026-11-10 stride on grade / MASS_SYM / REFIT3")
    ap.add_argument("--joint-variants", action="store_true", help="round-2 item 2: the joints replay under JOINT_VARIANTS")
    ap.add_argument("--record", default=None, help="flight record: " + ", ".join(RECORDS) + " (default 1109)")
    ap.add_argument("--json", default=None)
    ap.add_argument("--md", default=None)
    ap.add_argument("--workers", type=int, default=int(os.environ.get("WORKERS", "12")))
    a = ap.parse_args(argv)
    if a.record:
        os.environ["GATE_REC"] = a.record
    terms = GC.parse_terms(a.terms)
    if a.stride3:
        rows, md = stride3(a.seeds, a.workers)
        print(md)
        if a.md:
            open(a.md, "w").write(md + "\n")
        if a.json:
            json.dump(rows, open(a.json, "w"), indent=1, default=str)
        return None
    if a.joint_variants:
        md = variants_table(joint_variants(workers=a.workers), robot_row())
        print(md)
        if a.md:
            open(a.md, "w").write(md + "\n")
        return None
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
    hdr = (f"Check 2, flight record {record_name()} ({os.path.basename(load_hw()['path'])}) on the walking plant ({GC.XML_NAME})"
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
