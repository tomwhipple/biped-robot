"""Shared pieces of the weight-transfer gate (issue #259): the walking plants, the opt-in probe
terms, and the metrics both checks use.

Plants (all on sim/bimo_biped_v6legs_meas_sole.xml, the plant every prec recipe trains on):

  train      the prec16 recipe's training actuator chain at the CENTRE of its domain
             randomisation, on the CPU walker_env (EP.make_env from the run's config.json, DR off).
             walker_env cannot express three of the recipe's MJX-only knobs, so the harness
             adds them in front of the env (TrainChain): per-type dead time (--act-delay-types
             sts3215=2:4 sts3250=0:1 -> 3 / 0.5 ticks), the ONE-stage action lag
             (act_lag_order 1, --act-lag 2.65..8 -> 5.3 Hz, x4 on the hip rolls by sts2026), and
             the servo kp/kd DR multipliers (0.6..1.2 -> 0.9, 0.12..1.0 -> 0.56). The rest is
             the env's own: roll lash 0.5..2.0 -> 1.25 deg, backlash 0..0.25 -> 0.125 deg,
             joint friction 0.1..0.4 -> 0.25 N m, latency 0..8 -> 4 ms, sole/floor mu
             0.3..0.7 -> 0.5, torso mass x1 and CoM 0 (the centres of +-40 % and +-15 mm).
             `train:lash0.5` / `train:lash2.0` put the roll lash at its DR ends.
  grade      what experiments/policy/shaper-train/grade.py's `robot` probes run (the
             stride1 0.3 robot probe that lifts prec16's R foot 2.1 cm): simh.build("fit") --
             which APPLIES simfit_plant.json's torso x0.702, torso CoM -6.8 mm, 81 mm sole,
             carpet floor_tc and the L ankle-roll zero error -- then simh.JF_PLANT, which sets
             the roll lash and the backlash to ZERO, then ROBOT_SPEED_PLANT (12 V, hip pitch /
             knee envelope), with grade.ROBOT_SPEED_SERVO (TypedServo: STS3215 dead 60 ms /
             tau 33 ms, STS3250 2nd order) in front of the env.

Probe terms (opt-in, `terms=`; none of them is a default anywhere):
  lash_fit    the one-leg fit's roll play: ankle rolls 0.963 deg, hip rolls 1.552 deg
              (simfit_plant.json), replacing the plant's roll lash
  lash=<deg>  one extra roll deadzone width on all four rolls (replaces the plant's)
  torso=<x>   torso mass and inertia x (from the XML's 0.4545 kg)
  legs=<x>    every other body's mass and inertia x (the legs and feet)
  com_y=<m>   torso CoM lateral offset from the XML's (m, + = robot's left)
  roll_k=<x>  x the roll servos' stiffness AND damping (kp/kd profile): a load-proportional
              roll compliance, the spring the walking plant does not have (it has play only)
  dead=<ms>   extra dead time on every STS3215 (loaded-servo tracking lag), ms
  ser_hip=<k>, ser_ankle=<k>   N m/rad: a spring in series with each hip roll / ankle roll on
              the servo's OUTPUT side, so the encoder (policy obs, Replay, clamp encoders) reads the
              servo angle, not the link (set_series_view: the quasi-static form; walker_env cannot
              take an extra joint inside the leg chain). The bare-plant joints replay uses a real
              sprung joint (add_series_joint)
  play_hip=<deg>, play_ankle=<deg>   free play (peak to peak) inside each hip roll / ankle roll
              servo BEYOND its encoder: walker_env roll_play_deg (the shaft is position-controlled
              and is what the encoder reads; the link floats inside +/-play/2 of it), fixed values,
              live at once and at every reset after; a run's own series spring is dropped
  play_L_hip=, play_R_hip=, play_L_ankle=, play_R_ankle=<deg>   the same for one side's joint,
              overriding play_hip / play_ankle there
  fric_aroll=<N m>, fric_hroll=<N m>   dry friction (MuJoCo frictionloss) on the ankle-roll /
              hip-roll joints, replacing the plant's: it acts on the link, so under the play it
              holds the link inside its band
"""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np

os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
MAIN = os.environ.get("GATE_MAIN", "/home/claw/code/robot")      # hw_sessions + sim/runs live here
HW = os.path.join(MAIN, "hw_sessions")
RUN = "robot_legs_prec16_f075_js20_s128r24"
XML_NAME = "bimo_biped_v6legs_meas_sole.xml"
XML = os.path.join(ROOT, "sim", XML_NAME)
SHAPER_HZ = 8.0                     # grade's SHAPE_HZ for the prec students (the deploy's C2 shaper)

for p in (os.path.join(ROOT, "sim"), os.path.join(ROOT, "sim", "mjx"),
          os.path.join(ROOT, "experiments", "policy", "shaper-train"),
          os.path.join(ROOT, "experiments", "policy", "stand-still"),
          os.path.join(ROOT, "experiments", "policy", "rca"),
          os.path.join(ROOT, "experiments", "one-leg-stand")):
    if p not in sys.path:
        sys.path.insert(0, p)

# the prec16 recipe's DR centres (config.json of the run; see the module doc)
TRAIN_MID = dict(lag_hz=0.5 * (2.65 + 8.0), lag_scale_hip_roll=4.0, dead15_ticks=3.0, dead50_ticks=0.5,
                 kp=0.9, kd=0.56, roll_lash=1.25, backlash=0.125, friction=0.25, latency_ms=4.0, mu=0.5)
TRAIN_VARIANTS = {"mid": {}, "lash0.5": dict(roll_lash=0.5), "lash2.0": dict(roll_lash=2.0)}
FIT_LASH = dict(ankle_roll=0.963, hip_roll=1.552)   # simfit_plant.json
FIT_TORSO, FIT_COM_Y = 0.702, -0.0068


def run_dir(run=RUN):
    for root in (os.path.join(ROOT, "sim", "runs"), os.path.join(MAIN, "sim", "runs")):
        if os.path.exists(os.path.join(root, run, "params.pkl")):
            return os.path.join(root, run)
    return os.path.join(MAIN, "sim", "runs", run)


def run_cfg(run=RUN):
    return json.load(open(os.path.join(run_dir(run), "config.json")))


# --------------------------------------------------------------------------------------------
# actuator chains in front of walker_env (target in, target out, one call per control tick)
# --------------------------------------------------------------------------------------------
class TrainChain:
    """The recipe's MJX-side servo chain the CPU env lacks: a per-servo dead time (fractional
    ticks, linear interpolation of the goal history) then ONE first-order lag at lag_hz
    (x lag_scale on the hip rolls). Seeded at the first target it sees."""

    def __init__(self, names, dt, lag_hz, lag_scale_hip_roll=1.0, dead15_ticks=0.0, dead50_ticks=0.0):
        self.names = list(names)
        hr = np.array(["hip_roll" in n for n in self.names])
        self.dead = np.where(hr, dead50_ticks, dead15_ticks).astype(float)
        hz = lag_hz * np.where(hr, lag_scale_hip_roll, 1.0)
        self.k = 1.0 - np.exp(-2 * np.pi * hz * dt) if lag_hz > 0 else np.ones(len(self.names))
        self.n = int(np.ceil(self.dead.max())) + 2
        self.buf = None

    def reset(self, q):
        q = np.asarray(q, float)
        self.buf = [q.copy()] * self.n
        self.y = q.copy()

    def __call__(self, g):
        g = np.asarray(g, float)
        if self.buf is None:
            self.reset(g)
        self.buf.append(g.copy())
        self.buf = self.buf[-self.n:]
        H = np.array(self.buf)
        i0 = np.floor(self.dead).astype(int)
        fr = self.dead - i0
        idx = len(H) - 1 - i0
        cols = np.arange(H.shape[1])
        x = (1 - fr) * H[idx, cols] + fr * H[np.clip(idx - 1, 0, None), cols]
        self.y = self.y + self.k * (x - self.y)
        return self.y.copy()


class TypedChain:
    """grade.py's TypedServo (simh), seeded at the first target it sees."""

    def __init__(self, spec):
        import simh
        self.sv = simh.TypedServo(**spec)
        self.started = False

    def reset(self, q):
        self.sv.reset(np.asarray(q, float))
        self.started = True

    def __call__(self, g):
        if not self.started:
            self.reset(g)
        return self.sv(g)


def install_chain(env, chain):
    """Put `chain` between env._action_to_ctrl and the env's own actuator path, whatever
    _action_to_ctrl a caller sets later (the one-leg Replay sets its own each run)."""
    step0 = env.step

    def step(a):
        f = env._action_to_ctrl
        env._action_to_ctrl = lambda a_: chain(f(a_))
        try:
            return step0(a)
        finally:
            env._action_to_ctrl = f
    env.step = step
    env._gate_chain = chain
    return env


# --------------------------------------------------------------------------------------------
# plants
# --------------------------------------------------------------------------------------------
def _role_vec(env, fn):
    return np.array([fn(n) for n in env._servo_names], float)


def apply_terms(env, terms):
    """The opt-in probe terms (module doc) on an already-built env."""
    terms = dict(terms or {})
    m = env.model
    b = env._torso_bid
    if not hasattr(env, "_gate_nom"):
        env._gate_nom = dict(mass=float(m.body_mass[b]), inertia=m.body_inertia[b].copy(),
                             ipos=m.body_ipos[b].copy(), kp=np.array(env._kp_prof, float).copy(),
                             kd=np.array(env._kd_prof, float).copy())
    nom = env._gate_nom
    if any(k in terms for k in ("lash_fit", "lash", "lash_hip", "lash_ankle")):
        cur = {n: math.degrees(env._roll_lash_rad * env._roll_mask[i]) for i, n in enumerate(env._servo_names)}
        if "lash_fit" in terms:
            w = {n: FIT_LASH.get(n[2:], 0.0) for n in env._servo_names}
        elif "lash" in terms:
            w = {n: (float(terms["lash"]) if "roll" in n else 0.0) for n in env._servo_names}
        else:
            w = dict(cur)
        for n in env._servo_names:
            if n.endswith("hip_roll") and "lash_hip" in terms:
                w[n] = float(terms["lash_hip"])
            if n.endswith("ankle_roll") and "lash_ankle" in terms:
                w[n] = float(terms["lash_ankle"])
        ref = max(w.values())
        env.roll_lash_deg_max = None
        env._roll_lash_rad = math.radians(ref)
        env._roll_mask = np.array([w[n] / ref if ref > 0 else 0.0 for n in env._servo_names])
    if "torso" in terms:
        x = float(terms["torso"])
        m.body_mass[b] = nom["mass"] * x
        m.body_inertia[b] = nom["inertia"] * x
    if "legs" in terms:         # every body but the torso (and the world): mass and inertia x
        if "legs_mass" not in nom:
            nom["legs_mass"] = m.body_mass.copy()
            nom["legs_inertia"] = m.body_inertia.copy()
        x = float(terms["legs"])
        for i in range(1, m.nbody):
            if i != b:
                m.body_mass[i] = nom["legs_mass"][i] * x
                m.body_inertia[i] = nom["legs_inertia"][i] * x
    if "com_y" in terms:
        m.body_ipos[b][1] = nom["ipos"][1] + float(terms["com_y"])
    if "torso_kg" in terms:     # absolute torso mass, inertia scaled with it
        x = float(terms["torso_kg"]) / float(m.body_mass[b])
        m.body_mass[b] *= x
        m.body_inertia[b] *= x
    if "com_y_abs" in terms:    # absolute torso CoM y in the torso frame (the XML's is -0.001)
        m.body_ipos[b][1] = float(terms["com_y_abs"])
    if "pack" in terms:
        add_box_to_body(m, b, float(terms["pack"]), PACK_POS, PACK_HALF)
    if "sole_sym" in terms:
        sole_symmetric(env)
    if "zoff_La" in terms:      # the L ankle-roll calibration zero error (deg); grade's fit carries -2.12
        env._gate_zoff_La = float(terms["zoff_La"])
        if env._zero_off is not None:
            env._zero_off[env._servo_names.index("L_ankle_roll")] = math.radians(float(terms["zoff_La"]))
    if "roll_k" in terms or "ser_hip" in terms or "ser_ankle" in terms:
        s = _role_vec(env, lambda n: float(terms.get("roll_k", 1.0)) if "roll" in n else 1.0)
        kp_abs = float(env._servo[0]) * nom["kp"] * s          # the servo's own stiffness, N m/rad
        ks = _role_vec(env, lambda n: float(terms.get("ser_hip", np.inf)) if n.endswith("hip_roll") else
                       float(terms.get("ser_ankle", np.inf)) if n.endswith("ankle_roll") else np.inf)
        x = np.where(np.isfinite(ks), ks / (kp_abs + np.where(np.isfinite(ks), ks, 0.0)), 1.0)
        env._kp_prof = nom["kp"] * s * x
        env._kd_prof = nom["kd"] * s * x
        set_series_view(env, 1.0 - x)
    if any(k.startswith("play_") for k in terms):
        import walker_env as W
        # the play replaces a run's own series spring (the env takes one or the other)
        env.roll_series_k = None
        env._ser_c = env._ser_defl = env._ser_dq = None
        spec = {r: float(terms.get("play_" + r, 0.0)) for r in W.ROLL_SERIES_ROLES}
        spec.update({k: float(terms["play_" + k]) for k in W.ROLL_PLAY_KEYS
                     if k not in W.ROLL_SERIES_ROLES and "play_" + k in terms})
        env.roll_play_deg = W.parse_roll_play_deg(spec)
        # live now too (a Replay places the robot without a reset): fixed widths, shaft on the link
        env._rp_h = (None if env.roll_play_deg is None else
                     W.roll_play_bounds(env.roll_play_deg, env._servo_names)[0])
        env._rp_off = np.zeros(env._n_servo)
        env._rp_dq = np.zeros(env._n_servo)
    for key, suf in (("fric_aroll", "ankle_roll"), ("fric_hroll", "hip_roll")):
        if key in terms:        # dry friction (N m) on the roll joints themselves: the link side
            for i, n in enumerate(env._servo_names):
                if n.endswith(suf):
                    m.dof_frictionloss[env._jv0 + i] = float(terms[key])
    if "dead" in terms:
        ch = getattr(env, "_gate_chain", None)
        add = float(terms["dead"])
        if isinstance(ch, TrainChain):
            ch.dead = ch.dead + np.array([0.0 if "hip_roll" in n else add / (1e3 * env.control_dt)
                                          for n in env._servo_names])
            ch.n = int(np.ceil(ch.dead.max())) + 2
        elif isinstance(ch, TypedChain):
            ch.sv.dead = ch.sv.dead + np.array([0.0 if "hip_roll" in n else add / 1e3 for n in env._servo_names])
            ch.sv.n = int(np.ceil(ch.sv.dead.max() / 0.02)) + 2
    return env


def add_series_joint(spec, joint, k, damping=0.05, armature=1e-4):
    """A passive sprung hinge in series with `joint`, on its output side: a second joint in the
    same body, same axis and anchor, stiffness k (N m/rad, spring reference 0). For bare-MuJoCo
    replays only: walker_env needs its actuated joints consecutive in qpos, which a joint inside
    the leg chain breaks (see the `ser_*` probe terms for the walker_env form)."""
    j0 = spec.joint(joint)
    b = j0.parent if hasattr(j0, "parent") else None
    if b is None:
        b = next(bd for bd in spec.bodies if any(j.name == joint for j in bd.joints))
    j = b.add_joint()
    j.name = joint + "_ser"
    j.type = j0.type
    j.axis = list(j0.axis)
    j.pos = list(j0.pos)
    j.stiffness = [float(k), 0.0, 0.0]        # MuJoCo 3.15: (linear, and two nonlinear terms)
    j.damping = [float(damping), 0.0, 0.0]
    j.armature = float(armature)
    j.range = [-0.5, 0.5]
    j.limited = 1
    return j


# The battery pack where the CAD puts it: gen_plant_v6's legs-plant torso with m_battery > 0 places a
# 3S pack box, transverse, at (-0.012, 0, 0.0585) in the torso frame, half-sizes 18 x 52.5 x 13 mm
# (build_v6_inertia.plant_xml(measured_sole(measured_stops(asbuilt_legs())) with m_battery set; the
# whole regenerated plant is NOT used: its CAD sums move other torso parts too). The pack's mass,
# 83 g, is 1621 g (2026-11-10, battery in) minus 1538 g (2026-11-01, no battery), both kitchen scale.
PACK_POS = np.array([-0.012, 0.0, 0.0585])
PACK_HALF = np.array([0.018, 0.0525, 0.013])
PACK_KG = 0.083
# the meas-sole plant weighs 1507 g against the robot's 1538 g without its pack: the 31 g are not
# located; MASS_SYM puts them in the torso at its own CoM (torso 0.4545 -> 0.4855 kg). On grade's
# plant MASS_SYM also zeroes the L ankle-roll zero error (-2.12 deg, the 10-30 fit), which is what puts
# 0.60 of the stand on the R foot (ladder.py, README Round 3); grade's torso CoM offset is kept: the
# ladder fits better with it (rms 0.017 against 0.027 without)
MASS_SYM = "torso_kg=0.4855,pack=0.083,zoff_La=0"
# round 3's refit: MASS_SYM plus the series roll springs fitted to the 2026-11-10 clamp signature
# (fit_clamp3.py: ankle rolls 17.75, hip rolls 11.0 N m/rad; the shank -> thigh slop lumped at the hip)
REFIT3 = MASS_SYM + ",ser_ankle=17.75,ser_hip=11.0"


def add_box_to_body(m, b, mass, pos, half):
    """Add a solid box (mass, centre `pos` and half-sizes in the body frame) to body b's
    inertial: new mass, CoM, and principal inertia / frame."""
    import mujoco
    m0, c0 = float(m.body_mass[b]), np.array(m.body_ipos[b], float)
    R0 = np.zeros(9)
    mujoco.mju_quat2Mat(R0, m.body_iquat[b])
    R0 = R0.reshape(3, 3)
    I0 = R0 @ np.diag(m.body_inertia[b]) @ R0.T
    a, bb, c = 2 * np.asarray(half, float)
    Ib = mass / 12.0 * np.diag([bb * bb + c * c, a * a + c * c, a * a + bb * bb])
    pos = np.asarray(pos, float)
    M = m0 + mass
    com = (m0 * c0 + mass * pos) / M
    shift = lambda I, mm, r: I + mm * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
    I = shift(I0, m0, c0 - com) + shift(Ib, mass, pos - com)
    w, V = np.linalg.eigh(I)
    if np.linalg.det(V) < 0:
        V[:, 0] *= -1
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, V.reshape(-1))
    m.body_mass[b] = M
    m.body_ipos[b] = com
    m.body_inertia[b] = w
    m.body_iquat[b] = q


def sole_symmetric(env, sole_w=0.0814):
    """Both feet's sole pads at the XML's positions scaled to one effective width (the one-leg
    fit's 81.4 mm, as simfit applies it): drops grade's L-only outer-edge pull-in."""
    import mujoco
    m = env.model
    m0 = mujoco.MjModel.from_xml_path(env.xml_path if os.path.exists(getattr(env, "xml_path", "")) else XML)
    nm = lambda mm, g: mujoco.mj_id2name(mm, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
    for side in "LR":
        ps = [g for g in range(m.ngeom) if nm(m, g).startswith(f"{side}_pad_")]
        p0 = {nm(m, g): m0.geom_pos[m0.geom(nm(m, g)).id].copy() for g in ps}
        ys = np.array([p0[nm(m, g)][1] for g in ps])
        yc, half = 0.5 * (ys.max() + ys.min()), 0.5 * (ys.max() - ys.min())
        for g in ps:
            y = p0[nm(m, g)][1]
            m.geom_pos[g][1] = yc + (y - yc) * (0.5 * sole_w) / half


def set_series_view(env, alpha):
    """The `ser_hip` / `ser_ankle` terms: a spring k_s in series with the servo, on its output
    side, in its quasi-static form. The link sees k_eff = kp k_s / (kp + k_s) (the env's PD on
    the roll joints is scaled by k_s / (kp + k_s)), and the ENCODER reads the servo shaft
    s = q_link + alpha (target - q_link), alpha = kp / (kp + k_s): the servo's own deflection only.
    Applied to the policy's joint-angle observation (every frame of the history) and read by the
    one-leg Replay and the clamp test through env._gate_alpha. The shaft is massless here (no
    extra mode); joint velocities stay the link's."""
    alpha = np.asarray(alpha, float)
    env._gate_alpha = alpha
    if getattr(env, "_gate_obs0", None) is None:
        env._gate_obs0 = env._obs
        nj = env._nq_act

        def obs():
            o = env._gate_obs0().copy()
            a = env._gate_alpha
            if np.any(a > 0):
                d = env.data
                ql = np.asarray(d.qpos[env._sqpos], float)
                tgt = np.asarray(env._last_target if env._last_target is not None else ql, float)
                o[0:nj] = o[0:nj] + (a * (tgt - ql))[:nj]
            return o
        env._obs = obs


def encoder_err(env, i):
    """what servo i's encoder reads as its error (target - shaft), rad, past the gear backlash:
    the link error minus the deadzones, times (1 - alpha) under a series spring."""
    a = float(getattr(env, "_gate_alpha", np.zeros(env._n_servo))[i])
    e = abs(float(env.data.ctrl[i]) - float(env.data.qpos[env._jq0 + i]))
    if getattr(env, "_rp_h", None) is not None:      # roll play: the encoder reads the shaft
        e = abs(float(env.data.ctrl[i]) - float(env.data.qpos[env._jq0 + i]) - float(env._rp_off[i]))
    half = 0.5 * (env._lash_rad + env._roll_lash_rad * env._roll_mask[i])
    return max(0.0, (1.0 - a) * max(e - half, 0.0) if a > 0 else e - half)


def clamp_run(env, side, ramp=1.0, hold=3.0, settle=1.0, dz=0.030, detail=None):
    """sim/roll_compliance.run (the 10-24 clamped-foot lift) with the encoder read through
    encoder_err (so a series spring's deflection stays invisible to it). -> (tilt toward the
    lifted side deg, {hip_roll, ankle_roll: stance encoder deg}, foot rise mm). `detail` (a dict)
    also gets the lifted foot's lowest-pad clearance (mm) and the stance rolls' deflection beyond
    their encoders (deg: link error minus what the encoder reads)."""
    import roll_compliance as RC
    stance = "R" if side == "L" else "L"
    env.reset(seed=0)
    qlift, q0 = RC.lift_targets(env, side, dz)
    pads = pad_gids(env)
    clr, beyond = [], []
    names = env._act_names
    base = env._default.copy()
    tgt = base.copy()
    n_ctrl = int((settle + ramp + hold) / env.control_dt)
    tilts, enc, lifts = [], [], []
    bl, bs = env.model.body(f"{side}_foot").id, env.model.body(f"{stance}_foot").id
    sgn = 1.0 if side == "L" else -1.0
    for k in range(n_ctrl):
        t = k * env.control_dt
        sc = np.clip((t - settle) / ramp, 0.0, 1.0)
        for n, v in qlift.items():
            tgt[names.index(n)] = base[names.index(n)] + sc * (v - q0[n])
        env._action_to_ctrl = lambda a, tgt=tgt.copy(): tgt
        env.step(np.zeros(len(names), np.float32))
        if t > settle + ramp + hold - 1.0:
            up = env.data.sensordata[env._up_id:env._up_id + 3]
            tilts.append(np.degrees(np.arctan2(sgn * up[1], up[2])))
            lifts.append(1e3 * (env.data.xpos[bl][2] - env.data.xpos[bs][2]))
            enc.append({j: math.degrees(encoder_err(env, env._sname2i[f"{stance}_{j}"]))
                        for j in ("hip_roll", "ankle_roll")})
            clr.append(clearance(env, pads)[side])
            bd = {}
            for j in ("hip_roll", "ankle_roll"):
                i = env._sname2i[f"{stance}_{j}"]
                e = abs(float(env.data.ctrl[i]) - float(env.data.qpos[env._jq0 + i]))
                bd[j] = math.degrees(e - encoder_err(env, i))
            beyond.append(bd)
    if detail is not None:
        detail.update(clear_mm=1e3 * float(np.mean(clr)), clear_min_mm=1e3 * float(np.min(clr)),
                      beyond={j: float(np.mean([x[j] for x in beyond])) for j in beyond[0]})
    return float(np.mean(tilts)), {j: float(np.mean([e[j] for e in enc])) for j in enc[0]}, float(np.mean(lifts))


def parse_terms(spec):
    """'lash_fit,torso=0.702,com_y=-0.0068' -> dict."""
    out = {}
    for tok in (spec or "").split(","):
        tok = tok.strip()
        if not tok:
            continue
        k, _, v = tok.partition("=")
        out[k] = float(v) if v else True
    return out


def train_env(variant="mid", xml=None, ep_s=120.0, terms=None, shaper_hz=0.0, run=RUN):
    """The training chain at the DR centre (module doc). shaper_hz: the firmware C2 shaper in
    the env (the policy path; the bench `pose` stream has none)."""
    import eval_precision as EP
    import walker_env as W
    c = dict(TRAIN_MID, **TRAIN_VARIANTS.get(variant, {}))
    cfg = run_cfg(run)
    extra = dict(roll_lash_deg=c["roll_lash"], roll_lash_deg_max=None, backlash_deg=c["backlash"],
                 backlash_deg_max=None, latency_ms=c["latency_ms"], joint_frictionloss=c["friction"],
                 fw_shaper_hz=shaper_hz, push_prob=0.0, push_force=0.0)
    extra = {k: v for k, v in extra.items() if k in EP._ENV_PARAMS}
    env = EP.make_env(cfg, ep_s, nominal=True, xml=xml or XML, act_lag_hz=0.0, extra=extra)
    EP.set_contact_friction(env, c["mu"])
    env._kp_prof = np.array(env._kp_prof, float) * c["kp"]
    env._kd_prof = np.array(env._kd_prof, float) * c["kd"]
    env.reset(seed=0)
    env._zero_off = None
    chain = TrainChain(env._servo_names, env.control_dt, c["lag_hz"], c["lag_scale_hip_roll"],
                       c["dead15_ticks"], c["dead50_ticks"])
    install_chain(env, chain)
    env._gate_plant = f"train:{variant}"
    return apply_terms(env, terms)


def grade_env(xml=None, ep_s=120.0, terms=None, run=RUN):
    """grade.py's `robot` probe plant (module doc). Returns (env, act, zoff)."""
    import grade as G
    import simh
    xml0 = simh.ESA._xml
    if xml:     # this plant XML instead of the run's (grade.py's GRADE_XML, applied here)
        simh.ESA._xml = lambda cfg: xml
    try:
        env, act, zoff = simh.build("fit", run=run, ep_s=ep_s)
    finally:
        simh.ESA._xml = xml0
    simh.apply_model(env, dict(simh.JF_PLANT, **G.ROBOT_SPEED_PLANT))
    env.domain_rand = False
    env.push_prob = 0.0
    env.reset(seed=0)
    env._zero_off = zoff.copy()
    chain = TypedChain(G.ROBOT_SPEED_SERVO)
    install_chain(env, chain)
    env._gate_plant = "grade"
    return apply_terms(env, terms), act, zoff


def make_plant(name, terms=None, xml=None, ep_s=120.0, shaper_hz=0.0):
    """'train[:variant]' | 'grade' -> env (the policy and zoff are dropped for 'grade')."""
    if name.startswith("train"):
        return train_env(name.split(":", 1)[1] if ":" in name else "mid", xml=xml, ep_s=ep_s,
                         terms=terms, shaper_hz=shaper_hz)
    if name == "grade":
        return grade_env(xml=xml, ep_s=ep_s, terms=terms)[0]
    raise ValueError(name)


# --------------------------------------------------------------------------------------------
# metrics shared by the checks
# --------------------------------------------------------------------------------------------
def pad_gids(env):
    import mujoco
    m = env.model
    nm = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
    return {s: [g for g in range(m.ngeom) if nm(g).startswith(f"{s}_pad_")] for s in "LR"}


def clearance(env, pads):
    """per foot: the lowest sole pad's bottom above the floor (m)."""
    d, m = env.data, env.model
    return {s: float(min(d.geom_xpos[g][2] - m.geom_size[g][0] for g in gs)) for s, gs in pads.items()}


def foot_state(m, d, pads, soles):
    """(lowest pad L, R, highest pad L, R above the floor in m, sole tilt from level L, R in deg):
    the lowest pad > 0 = wholly off the floor; only the highest > 0 = one end raised."""
    lo, hi, tl = [], [], []
    for s, sg in zip("LR", soles):
        z = [d.geom_xpos[g][2] - m.geom_size[g][0] for g in pads[s]]
        lo.append(float(min(z)))
        hi.append(float(max(z)))
        tl.append(math.degrees(math.acos(min(1.0, abs(float(d.geom_xmat[sg].reshape(3, 3)[2, 2]))))))
    return tuple(lo + hi + tl)


def up_of(q):
    """the torso's z axis in the world, yaw stripped (the flight record's `up`): (x fwd, y left, z)."""
    w, x, y, z = (float(c) for c in q)
    zx, zy, zz = 2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)
    psi = yaw_of(q)
    c, s = math.cos(psi), math.sin(psi)
    return np.array([c * zx + s * zy, -s * zx + c * zy, zz])


def flail_rate(gyro, dt=0.02, tau=0.2):
    """the firmware flail guard's body-rate EMA (|gyro| norm, tau 0.2 s) over a (T, 3) series."""
    a = 1 - math.exp(-dt / tau)
    r, out = 0.0, []
    for g in np.asarray(gyro, float):
        r += a * (float(np.linalg.norm(g)) - r)
        out.append(r)
    return np.array(out)


def yaw_of(q):
    w, x, y, z = (float(c) for c in q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def roll_of(q):
    """torso roll from (w, x, y, z), deg-free (rad); + = leaning to the robot's RIGHT (rotation
    about +x, the forward axis, lifts the left side). The 11-09 record's own `roll` column agrees."""
    w, x, y, z = (float(c) for c in q)
    return math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))


def fmt(x, nd=1):
    if x is None or (isinstance(x, float) and not np.isfinite(x)):
        return "unknown"
    return f"{x:+.{nd}f}" if isinstance(x, (int, float)) else str(x)
