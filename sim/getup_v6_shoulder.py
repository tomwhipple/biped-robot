"""Get-up study, round 4 (2026-09-16), Tom's Option 2: "add arms with some
kind of crude shoulder joint. (we should put the shoulders at the TOP of the
torso)". Round 2 (getup_v6_appendage.py) found that arms at HIP height (6 cm
below the yaw axis) stand the body up 10-12/12 and 6/6 robust, but arms at
the shoulder -- deck top, arm_z=None == deck_bot-0.015 == 0.059 m above the
yaw axis -- fail every time: every push pivots the body about the head into
a headstand (docs/design-v6/getup_search_arms*.txt, 0/N).

This script asks the sharper question Tom's phrasing raises: can an arm
DESIGN -- longer, and/or with a second (ab/adduction) shoulder DOF, and/or
an elbow -- recreate the working hip-level push from a mount that is
genuinely at the TOP of the torso (measured from the plant: deck_bot +
deck_t = 0.079 m above the yaw axis, level with the neck servo -- higher
even than the round-2 negative's 0.059 m), by reaching down and behind/
beside the seated hips.

New DesignParams (gen_plant_v6.py): arm_abd (+ arm_abd_add/arm_abd_abd), a
proximal ab/adduction joint (axis X, same sign convention as hip_roll) so
the arm can swing OUT to the side before pitching fore-aft; arm_shoulder_y_
extra for a lateral offset sweep. arm_z, arm_len, arm_elbow, arm_fore_len,
arm_shoulder_x already existed (round 2). Every run in this study uses
self_collide=True (2026-09-16 cherry-pick 7445c58/759fe96): the top-mounted
arms fold along the torso and past the head, and reach past the thighs when
planting beside the hips, so without real self-collision a "stand" here
would be geometry passing through geometry, not a result.

    .venv/bin/python sim/getup_v6_shoulder.py seat    > docs/design-v6/getup_search_shoulder_seat.txt
    .venv/bin/python sim/getup_v6_shoulder.py sideseat > docs/design-v6/getup_search_shoulder_sideseat.txt
    .venv/bin/python sim/getup_v6_shoulder.py pushup  > docs/design-v6/getup_search_shoulder_pushup.txt
    .venv/bin/python sim/getup_v6_shoulder.py robust  > docs/design-v6/getup_search_shoulder_robust.txt
    .venv/bin/python sim/getup_v6_shoulder.py render <config> <variant> out.mp4 [side|rear] [seat|pushup]
"""
import sys, os, math, itertools, dataclasses
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); os.environ.setdefault("MUJOCO_GL", "egl")
from gen_plant_v6 import DesignParams, build_xml
import getup_v6 as G

SP = os.environ.get("TMPDIR", "/tmp")
K, H = -130.0, -125.0
BASE = DesignParams(self_collide=True)   # knee 130 / hip 125 defaults, self-collision ON
TOP_Z = DesignParams().deck_bot + DesignParams().deck_t   # 0.079 m: deck top, beside the neck
OLD_NEG_Z = DesignParams().deck_bot - 0.015               # 0.059 m: round-2's failed "shoulder" mount

# ---------------------------------------------------------------- CONFIGS
CONFIGS = {
    # sanity check: reproduce the round-2 negative under self_collide=True at its own height
    "sanity_old_deck_12": dict(arms=True, arm_z=OLD_NEG_Z, arm_len=0.12),
    # Group A: 1-DOF shoulder PITCH only, at the true torso top, length sweep
    "top1_len12": dict(arms=True, arm_z=TOP_Z, arm_len=0.12),
    "top1_len20": dict(arms=True, arm_z=TOP_Z, arm_len=0.20),
    "top1_len28": dict(arms=True, arm_z=TOP_Z, arm_len=0.28),
    "top1_len35": dict(arms=True, arm_z=TOP_Z, arm_len=0.35),   # the "single long 1-DOF arm" crude baseline
    # Group B: 2-DOF pitch + elbow, at the torso top
    "top_elbow_15_15": dict(arms=True, arm_z=TOP_Z, arm_len=0.15, arm_elbow=True, arm_fore_len=0.15),
    "top_elbow_18_18": dict(arms=True, arm_z=TOP_Z, arm_len=0.18, arm_elbow=True, arm_fore_len=0.18),
    # Group C: 2-DOF ab/adduction + pitch, at the torso top
    "top_abd_20": dict(arms=True, arm_z=TOP_Z, arm_len=0.20, arm_abd=True),
    "top_abd_28": dict(arms=True, arm_z=TOP_Z, arm_len=0.28, arm_abd=True),
    # Group D: 3-DOF abd + pitch + elbow, at the torso top
    "top_abd_elbow_15_15": dict(arms=True, arm_z=TOP_Z, arm_len=0.15, arm_abd=True, arm_elbow=True, arm_fore_len=0.15),
    "top_abd_elbow_18_18": dict(arms=True, arm_z=TOP_Z, arm_len=0.18, arm_abd=True, arm_elbow=True, arm_fore_len=0.18),
    # Group E: shoulder x / lateral offset sweep on the round-C 2-DOF winner shape
    "top_abd_20_fore": dict(arms=True, arm_z=TOP_Z, arm_len=0.20, arm_abd=True, arm_shoulder_x=0.03),
    "top_abd_20_aft": dict(arms=True, arm_z=TOP_Z, arm_len=0.20, arm_abd=True, arm_shoulder_x=-0.05),
    "top_abd_20_wide": dict(arms=True, arm_z=TOP_Z, arm_len=0.20, arm_abd=True, arm_shoulder_y_extra=0.02),
    # round-4b (Tom 2026-09-16, elbows + hanging idle pose): the shortest robust
    # elbow arm (0.16+0.16 m), shoulder moved AFT to clear the hanging arm from
    # the swinging leg during the walk (getup_search_shoulder_elbow.txt,
    # gateD_shoulder.txt: default x fell on turn+0/mu0.3/play5, aft x=-0.05
    # passes all 4 gate cases with 31 arm-leg contacts over 44.6 s vs 7327).
    "top_elbow_16_16_aft": dict(arms=True, arm_z=TOP_Z, arm_len=0.16, arm_elbow=True, arm_fore_len=0.16, arm_shoulder_x=-0.05),
    "top_elbow_18_18_aft": dict(arms=True, arm_z=TOP_Z, arm_len=0.18, arm_elbow=True, arm_fore_len=0.18, arm_shoulder_x=-0.05),
    # round-5 (Tom 2026-09-19, the shoulder-girdle restyle): "put the shoulder
    # joint in the same plane as the hips", i.e. arm_shoulder_x = 0 -- directly
    # above the yaw axis, not 50 mm aft. That undoes the round-4b fix, so it has
    # to be re-measured, not assumed: the aft mount existed ONLY to keep the
    # hanging arm out of the swinging leg. The other lever on that same contact
    # is lateral: the study's own widen sweep (arm_shoulder_y_extra) cut contacts
    # too, and the CAD arm already hangs 7.7 mm wider than the plant's default
    # (dimensions_v6 ARM_Y 88.0 vs ARM_Y_SIM 80.35). So sweep x=0 against width.
    # _cad = the AS-DRAWN geometry (CAD arm plane and CAD shoulder height).
    "top_elbow_16_16_hip": dict(arms=True, arm_z=TOP_Z, arm_len=0.16, arm_elbow=True, arm_fore_len=0.16, arm_shoulder_x=0.0),
    "top_elbow_16_16_hip_cad": dict(arms=True, arm_z=0.08791, arm_len=0.16, arm_elbow=True, arm_fore_len=0.16, arm_shoulder_x=0.0, arm_shoulder_y_extra=0.0077),
    "top_elbow_16_16_hip_w20": dict(arms=True, arm_z=TOP_Z, arm_len=0.16, arm_elbow=True, arm_fore_len=0.16, arm_shoulder_x=0.0, arm_shoulder_y_extra=0.020),
    "top_elbow_16_16_hip_w30": dict(arms=True, arm_z=TOP_Z, arm_len=0.16, arm_elbow=True, arm_fore_len=0.16, arm_shoulder_x=0.0, arm_shoulder_y_extra=0.030),
    # ...and the same sweep again with arm_cad_servos=True (gen_plant_v6), i.e.
    # the elbow servo box where cad/v6/arm_v6.py actually draws it. The sweep
    # above says x=0 needs +30 mm of shoulder width; but the contact pairs are
    # ALWAYS *_forearm (never *_arm), and the widest inboard thing on the
    # forearm body is that servo box -- which the plant had ~20 mm too far
    # inboard and ~12 mm too high. Re-measure before paying for a 220 mm
    # shoulder span. "_r5cad" = as-drawn servo boxes; CAD shoulder height and
    # CAD arm plane where the name says so.
    "r5_aft_cad": dict(arms=True, arm_z=0.08791, arm_len=0.16, arm_elbow=True, arm_fore_len=0.16, arm_shoulder_x=-0.05, arm_shoulder_y_extra=0.0077, arm_cad_servos=True),
    "r5_hip_cad": dict(arms=True, arm_z=0.08791, arm_len=0.16, arm_elbow=True, arm_fore_len=0.16, arm_shoulder_x=0.0, arm_shoulder_y_extra=0.0077, arm_cad_servos=True),
    "r5_hip_cad_w10": dict(arms=True, arm_z=0.08791, arm_len=0.16, arm_elbow=True, arm_fore_len=0.16, arm_shoulder_x=0.0, arm_shoulder_y_extra=0.0177, arm_cad_servos=True),
    "r5_hip_cad_w20": dict(arms=True, arm_z=0.08791, arm_len=0.16, arm_elbow=True, arm_fore_len=0.16, arm_shoulder_x=0.0, arm_shoulder_y_extra=0.0277, arm_cad_servos=True),
    # THE PROPOSED ROUND-5 GEOMETRY (shoulder girdle, cad/v6/shoulder_girdle_v6.py).
    # arm_shoulder_x 0      -- "the same plane as the hips" (Tom, 2026-09-19).
    # arm_z 0.079           -- the study's OWN TOP_Z, exactly. Rotating the servo
    #                          90 deg about its output axis drops the case from
    #                          "standing on end on the deck" to "lying fore-aft
    #                          beside the deck", so the axis no longer has to sit
    #                          a half-case above the lid. Round-4's +8.9 mm
    #                          shoulder-height deviation is GONE.
    # y_extra 0.023         -- ARM_Y 103.35 mm. Not a free choice: the rotated
    #                          case must hang outboard of the deck skin (61.26)
    #                          with a 3 mm screw wall and 1 mm of air, which
    #                          lands the arm plane there. The walk gate wants
    #                          >= +10 mm at x=0 anyway (w10 above), so the
    #                          structural number and the measured one agree.
    "r5_girdle": dict(arms=True, arm_z=0.079, arm_len=0.16, arm_elbow=True, arm_fore_len=0.16, arm_shoulder_x=0.0, arm_shoulder_y_extra=0.023, arm_cad_servos=True),
    # the alternative that keeps the pod sitting ON the deck lid instead of
    # hanging beside it: axis a half-case + floor above the lid (deviation +11.2)
    "r5_girdle_ondeck": dict(arms=True, arm_z=0.09016, arm_len=0.16, arm_elbow=True, arm_fore_len=0.16, arm_shoulder_x=0.0, arm_shoulder_y_extra=0.023, arm_cad_servos=True),
    # controls for the one gate case (mu 0.3 play 5) the hip-plane geometry
    # keeps failing: is the fall CONTACT (arm hits leg) or MASS (arm inertia at
    # x=0)? _nocol removes every self-collision; _bare removes the arms.
    "r5_girdle_ondeck_nocol": dict(arms=True, arm_z=0.09016, arm_len=0.16, arm_elbow=True, arm_fore_len=0.16, arm_shoulder_x=0.0, arm_shoulder_y_extra=0.023, arm_cad_servos=True, self_collide=False),
    "r5_aft_cad_nocol": dict(arms=True, arm_z=0.08791, arm_len=0.16, arm_elbow=True, arm_fore_len=0.16, arm_shoulder_x=-0.05, arm_shoulder_y_extra=0.0077, arm_cad_servos=True, self_collide=False),
    "r5_bare": dict(arms=False),
    # ROUND 5b (2026-09-24, Tom: "We need to re-run the getup with these arms
    # anyway."): the robot AS DRAWN after the girdle + closed-box arm head. The
    # r5_girdle_ondeck geometry, plus: the shoulder servos and the 84 g girdle
    # on the TORSO (arm_girdle), and the printed links at their CAD masses
    # (arm_upper_v6 33.1 g, arm_fore_v6 33.7 g, vs the 20 / 12 g placeholder).
    "r5_asdrawn": dict(arms=True, arm_z=0.09016, arm_len=0.16, arm_elbow=True, arm_fore_len=0.16,
                       arm_shoulder_x=0.0, arm_shoulder_y_extra=0.023, arm_cad_servos=True,
                       arm_girdle=True, arm_mass=0.0331, arm_fore_mass=0.0337, m_girdle=0.084),
    # ...and with the hip's REAL flexion limit enforced by the plant. The CAD
    # hip ROM comes back to -120 (thigh front wall / jog block vs the roll
    # flange, 0.70 mm at -120 after the leg-link relief; -125 is out of reach
    # in either yoke variant). The walk plant keeps its -125 range for now:
    # the walk policy's action scaling is tied to it.
    "r5_asdrawn_rom120": dict(arms=True, arm_z=0.09016, arm_len=0.16, arm_elbow=True, arm_fore_len=0.16,
                              arm_shoulder_x=0.0, arm_shoulder_y_extra=0.023, arm_cad_servos=True,
                              arm_girdle=True, arm_mass=0.0331, arm_fore_mass=0.0337, m_girdle=0.084,
                              hip_pitch_range=(-120.0, 90.0)),
}
hits = []


def plant(name):
    p = dataclasses.replace(BASE, **CONFIGS[name])
    xml = os.path.join(SP, f"gu_sh_{name}_{os.getpid()}.xml")
    open(xml, "w").write(build_xml(p))
    return p, xml


def run(label, p, xml, seq, start="supine", render=None):
    r = G.run_sequence(p, xml, seq, start=start, play_deg=3.0, per_joint=G.PJ_DEFAULT, verbose=False, render=render)
    trail = " | ".join(f"{L[0][:11]}:{L[1]:+.2f}/{L[2]:.2f}/{'+'.join(c.replace('_','')[:5] for c in L[3])}" for L in r['log'][1:])
    print(f"{label:72s} {'STANDING' if r['ok'] else 'no      '} up {r['up']:+.2f} front {r['front']:+.2f} z {r['pelvis_z']:.3f}\n      {trail}", flush=True)
    if r['ok']:
        hits.append(label)
    return r


# ------------------------------------------------------------------ sequences
def seat_push(has_abd, s0, s1, t_push, a_push, abd0=0.0, abd1=0.0, elbow=None):
    """supine -> sit up (arms folded UP along the torso, shoulder 180, abd 0,
    so a top-mounted arm cannot jam the sit-up or drag on the floor) -> arm
    swung down+out behind/beside the hip (a brace) -> tuck the feet -> push
    the pelvis up while the shank leans forward (ankle dorsiflexes) -> rise.
    abd0/abd1 let the arm swing OUT to the side while bracing/pushing.
    has_abd must be False for a plant with no arm_abd DOF: q_from_offsets
    has no generic role fallback for an unmodelled 'abd' and would instead
    (wrongly) try to address it as a leg joint."""
    fold = dict(shoulder=180, **(dict(abd=0.0) if has_abd else {}))
    behind = dict(shoulder=s0, **(dict(abd=abd0) if has_abd else {}))
    push = dict(shoulder=s1, **(dict(abd=abd1) if has_abd else {}))
    rest = dict(shoulder=60, **(dict(abd=0.0) if has_abd else {}))
    if elbow:
        fold["elbow"] = 0; behind["elbow"] = elbow[0]; push["elbow"] = elbow[1]; rest["elbow"] = 0
    return [("lie", dict(**fold), 0.5, 1.5),
            ("sit up", dict(hip_pitch=-90, **fold), 1.5, 0.8),
            ("fold", dict(hip_pitch=-110, **fold), 1.0, 0.6),
            ("brace", dict(hip_pitch=-110, **behind), 1.0, 0.8),
            ("tuck", dict(hip_pitch=H, knee=K, ankle=0, **behind), 1.5, 0.8),
            ("push", dict(hip_pitch=H, knee=K, ankle=a_push, **push), t_push, 1.0),
            ("crouch hold", dict(hip_pitch=H, knee=K, ankle=a_push, **rest), 1.0, 2.0),
            ("rise 2", dict(hip_pitch=-80, knee=-70, ankle=-30, **rest), 1.5, 1.0),
            ("rise 3", dict(hip_pitch=-45, knee=-50, ankle=-25, **rest), 1.5, 1.0),
            ("stand", dict(hip_pitch=-20, knee=-40, ankle=-20, **rest), 1.5, 1.0),
            ("straight", dict(**rest), 1.0, 0.8)]


def side_seat_push(side, s0, s1, abd0, abd1, t_push, a_push):
    """supine -> sit up -> ONE arm swings out to the side (abd) and down (s1)
    to plant beside the hip on that side -> tuck -> push while the torso
    twists slightly toward the planted arm -> rise. The other arm stays
    folded along the torso the whole time."""
    o = "R" if side == "L" else "L"
    fold = {f"{side}_shoulder": 180, f"{side}_abd": 0.0, f"{o}_shoulder": 180, f"{o}_abd": 0.0}
    behind = {f"{side}_shoulder": s0, f"{side}_abd": abd0, f"{o}_shoulder": 180, f"{o}_abd": 0.0}
    push = {f"{side}_shoulder": s1, f"{side}_abd": abd1, f"{o}_shoulder": 180, f"{o}_abd": 0.0}
    rest = {f"{side}_shoulder": 60, f"{side}_abd": 0.0, f"{o}_shoulder": 60, f"{o}_abd": 0.0}
    return [("lie", dict(**fold), 0.5, 1.5),
            ("sit up", dict(hip_pitch=-90, **fold), 1.5, 0.8),
            ("fold", dict(hip_pitch=-110, **fold), 1.0, 0.6),
            ("brace", dict(hip_pitch=-110, **behind), 1.0, 0.8),
            ("tuck", dict(hip_pitch=H, knee=K, ankle=0, **behind), 1.5, 0.8),
            ("push", dict(hip_pitch=H, knee=K, ankle=a_push, **push), t_push, 1.0),
            ("crouch hold", dict(hip_pitch=H, knee=K, ankle=a_push, **rest), 1.0, 2.0),
            ("rise 2", dict(hip_pitch=-80, knee=-70, ankle=-30, **rest), 1.5, 1.0),
            ("rise 3", dict(hip_pitch=-45, knee=-50, ankle=-25, **rest), 1.5, 1.0),
            ("stand", dict(hip_pitch=-20, knee=-40, ankle=-20, **rest), 1.5, 1.0),
            ("straight", dict(**rest), 1.0, 0.8)]


def pushup_seq(has_abd, s_floor, s_push1, s_push2, abd_out=0.0, elbow=None):
    """prone -> both arms swing forward-down to the floor (shoulder toward
    the -90..-110 range = arms reaching in front of the head) -> push
    (elbow extends if modelled) -> knees under -> try to sit back onto the
    heels. Mirrors getup_v6_appendage.py's 'prone_pike' render mode, now
    with the abd/elbow DOFs a top-of-torso arm needs."""
    abdk = (lambda v: dict(abd=v)) if has_abd else (lambda v: {})
    fold = dict(shoulder=180, **abdk(0.0))
    floor = dict(shoulder=s_floor, **abdk(abd_out))
    push1 = dict(shoulder=s_push1, **abdk(abd_out))
    push2 = dict(shoulder=s_push2, **abdk(abd_out))
    if elbow:
        e0, e1, e2 = elbow
        fold["elbow"] = e0; floor["elbow"] = e0; push1["elbow"] = e1; push2["elbow"] = e2
    return [("lie prone", dict(**fold), 0.5, 0.8),
            ("arms to floor", dict(**floor), 0.8, 0.4),
            ("push (both arms)", dict(**push1), 1.5, 1.0),
            ("push more", dict(**push2), 1.5, 1.0),
            ("knees under", dict(hip_pitch=H, knee=K, ankle=40, **push2), 2.0, 1.0),
            ("try to sit back", dict(hip_pitch=-60, knee=K, ankle=40, shoulder=-45, **abdk(abd_out),
                                      **({"elbow": elbow[2]} if elbow else {})), 2.0, 1.5)]


def run_traced(p, xml_path, seq, start="supine", track=("L_shoulder", "R_shoulder", "L_elbow", "R_elbow"), **kw):
    """Like G.run_sequence, but tracks the PEAK |torque| on the named joints
    over every control substep (run_sequence's own log only samples torque
    at each labelled step's END, which can miss a mid-move peak). Copies
    run_sequence's loop (sim/getup_v6.py) rather than editing the shared
    helper, since other studies depend on its existing per-label sampling."""
    import numpy as np, mujoco as mj
    env = G.make_env(p, xml_path, mu=kw.get("mu", 0.7), play_deg=kw.get("play_deg", 3.0))
    obs, _ = env.reset(seed=0)
    G.EXTRA[:] = [env.model.actuator(i).name for i in range(12, env.model.nu)]
    kp, kd, stall, w0 = env._servo
    na = env._nq_act
    servo_scale = kw.get("servo_scale", 1.0)
    stall_a = np.full(na, stall) * servo_scale
    w0_a = np.full(na, w0) * servo_scale
    kpa = np.full(na, float(kp)); kda = np.full(na, float(kd))
    for i, n in enumerate(G.JN):
        if G.PJ_DEFAULT and n in G.PJ_DEFAULT:
            s = G.SERVOS[G.PJ_DEFAULT[n]]
            stall_a[i] = s["stall"] * servo_scale; w0_a[i] = s["w0"] * servo_scale
            kpa[i] *= s["kp_scale"]; kda[i] *= s["kp_scale"]
    env._servo = np.array([kpa, kda, stall_a, w0_a], dtype=object)
    env.fall_up_z = -2.0; env.fall_height = -1.0
    names = list(G.JN) + G.EXTRA
    idx = {n: names.index(n) for n in track if n in names}
    d0, hi, lo = env._default, env._hi, env._lo

    def inv(q):
        q = q[:na] if len(q) >= na else np.concatenate([q, np.zeros(na - len(q))])
        return np.clip(np.where(q >= d0, (q - d0) / np.maximum(hi - d0, 1e-6), (q - d0) / np.maximum(d0 - lo, 1e-6)), -1, 1)
    q_prev = G.q_from_offsets(seq[0][1])
    G.settle_fallen(env, p, start, q_prev)
    dt = env.control_dt
    peak = {n: 0.0 for n in idx}
    for label, off, move_s, hold_s in seq:
        q_tgt = G.q_from_offsets(off)
        n_steps = int(move_s / dt)
        for i in range(n_steps + int(hold_s / dt)):
            s = min(1.0, (i + 1) / max(n_steps, 1))
            s = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
            q = q_prev + (q_tgt - q_prev) * s
            env.step(inv(q))
            for n, j in idx.items():
                peak[n] = max(peak[n], abs(float(env._servo_tau[j])))
        q_prev = q_tgt
    d = env.data
    up = d.xmat[env._torso_bid].reshape(3, 3)[2, 2]
    ok = up > 0.9 and d.qpos[2] > 0.8 * p.z_yaw_above_sole
    return dict(ok=ok, pelvis_z=float(d.qpos[2]), peak=peak)


def walk_with_arm_contacts(p, xml_path, n_steps=8, turn_deg=0.0, mu=0.7, play_deg=3.0, per_joint=None, arm_pose=(0.0, 0.0)):
    """Gate D's own walk (static_gait.walk_timeline / run_walk's stepping
    loop, copied here to add arm-vs-leg contact accounting -- run_walk
    itself does not expose per-step contacts). The arm/elbow actuators are
    not in the timeline, so they default to shoulder 0 / elbow 0 = hanging
    straight down at the sides, exactly the idle pose Tom specified.

    arm_pose=(shoulder_deg, elbow_deg) HOLDS the arms somewhere else for the
    whole walk instead. Round 5 (2026-09-19) needs this: with the shoulder
    joint moved into the hip plane (arm_shoulder_x=0) the hanging arm is
    directly beside the swinging thigh, and the held pose is the one lever
    that clears it without moving the JOINT or widening the shoulders."""
    import mujoco as mj
    from static_gait import walk_timeline, make_env
    from design_gates import q_of, JN as _JN
    env = make_env(p, xml_path, mu=mu, play_deg=play_deg)
    m, d = env.model, env.data
    kp, kd, stall, w0 = env._servo
    na = env._nq_act
    if per_joint:
        stall = np.full(na, stall); w0 = np.full(na, w0); kpa = np.full(na, float(kp)); kda = np.full(na, float(kd))
        for i, n in enumerate(_JN):
            if n in per_joint:
                s = G.SERVOS[per_joint[n]]
                stall[i] = s["stall"]; w0[i] = s["w0"]; kpa[i] *= s["kp_scale"]; kda[i] *= s["kp_scale"]
        env._servo = np.array([kpa, kda, stall, w0], dtype=object)
    obs, _ = env.reset(seed=0)
    d0, hi, lo = env._default, env._hi, env._lo

    # q_of returns the 12 LEG joints; everything after them (neck, then
    # shoulder/elbow per side -- see the model's joint order) is padded. Pad
    # the arm entries with the held pose rather than zero.
    _pad = np.zeros(max(0, na - 12))
    if len(_pad) >= 5:
        _sh, _el = math.radians(arm_pose[0]), math.radians(arm_pose[1])
        _pad[1], _pad[2], _pad[3], _pad[4] = _sh, _el, _sh, _el

    def inv(q):
        if len(q) < na:
            q = np.concatenate([q, _pad[:na - len(q)]])
        return np.clip(np.where(q >= d0, (q - d0) / np.maximum(hi - d0, 1e-6), (q - d0) / np.maximum(d0 - lo, 1e-6)), -1, 1)
    tl, windows = walk_timeline(p, n_steps=n_steps, step=0.06, lift_h=0.04, turn_deg=turn_deg)
    dt = env.control_dt
    n_ticks = int(tl.T / dt) + 1
    arm_bodies = {b for b in range(m.nbody) if any(k in (m.body(b).name or "") for k in ("_arm", "_forearm", "_shoulder_abd"))}
    leg_bodies = {b for b in range(m.nbody) if any(k in (m.body(b).name or "") for k in ("_thigh", "_shin", "_hip", "_ankle", "_foot"))}
    arm_leg_contacts = 0
    pairs_seen = set()
    tilt_max = 0.0
    fell = False
    for k in range(n_ticks):
        t = k * dt
        key = tl.at(t)
        qc = q_of(p, key)
        obs, r, term, trunc, _ = env.step(inv(qc))
        up = d.xmat[env._torso_bid].reshape(3, 3)[2, 2]
        tilt_max = max(tilt_max, math.degrees(math.acos(max(-1.0, min(1.0, up)))) if up <= 1.0 else 0.0)
        if up < 0.5:
            fell = True
        for i in range(d.ncon):
            c = d.contact[i]
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if (b1 in arm_bodies and b2 in leg_bodies) or (b2 in arm_bodies and b1 in leg_bodies):
                arm_leg_contacts += 1
                pairs_seen.add(tuple(sorted((m.body(b1).name, m.body(b2).name))))
    return dict(fell=fell, tilt_max=tilt_max, arm_leg_contacts=arm_leg_contacts, pairs=sorted(pairs_seen), t_final=tl.T)


def main():
    mode = sys.argv[1]
    if mode == "gate4":
        # Gate D's four walk cases (the set round-4b judged the aft mount on:
        # docs/design-v6/gateD_shoulder.txt), re-run with arm-vs-leg contact
        # accounting. This is the gate a round-5 hip-plane shoulder has to pass.
        print("== GATE D, 4 cases x arm contacts (8 steps, STS3250 rolls+knees, hanging idle pose, self_collide=True)")
        pj = {"L_knee": "sts3250", "R_knee": "sts3250", "L_hip_roll": "sts3250", "R_hip_roll": "sts3250", "L_ankle_roll": "sts3250", "R_ankle_roll": "sts3250"}
        cases = (("turn  +0 mu 0.7 play 3", dict()),
                 ("turn +15 mu 0.7 play 3", dict(turn_deg=15.0)),
                 ("turn  +0 mu 0.3 play 5", dict(mu=0.3, play_deg=5.0)),
                 ("turn -15 mu 0.9 play 3", dict(turn_deg=-15.0, mu=0.9)))
        g4argv = sys.argv[2:]
        g4pose = (0.0, 0.0)
        if g4argv and g4argv[0].startswith("--pose="):
            g4pose = tuple(float(v) for v in g4argv[0].split("=", 1)[1].split(","))
            g4argv = g4argv[1:]
            print(f"   held idle arm pose: shoulder {g4pose[0]:+.0f} deg, elbow {g4pose[1]:+.0f} deg")
        for name in g4argv:
            p = dataclasses.replace(BASE, **CONFIGS[name])
            xml = os.path.join(SP, f"gu_g4_{name}_{os.getpid()}.xml"); open(xml, "w").write(build_xml(p))
            n_ok = 0
            for label, kw in cases:
                r = walk_with_arm_contacts(p, xml, per_joint=pj, arm_pose=g4pose, **kw)
                n_ok += not r["fell"]
                print(f"{name:18s} {label}  {'FELL' if r['fell'] else 'up  '} tilt_max {r['tilt_max']:5.1f} deg  "
                      f"arm-vs-leg contacts {r['arm_leg_contacts']:6d}  pairs {r['pairs']}", flush=True)
            print(f"-- {name}: {n_ok}/4 cases stayed up", flush=True)
        return
    if mode == "getupnamed":
        # Round 5b (2026-09-24): the round-4 seat-push search + robustness pass,
        # run on NAMED configs so the as-drawn robot and the round-4 baseline go
        # through the identical grid. Seat push from supine, the elbow grid the
        # round-4 winner was found on; every standing variant then gets the six
        # robustness conditions (play 3/5, mu 0.3/0.7/1.0, servo 100/80/65 %).
        conds = [dict(play_deg=3.0, mu=0.7, servo_scale=1.0), dict(play_deg=5.0, mu=0.7, servo_scale=1.0),
                 dict(play_deg=3.0, mu=0.3, servo_scale=1.0), dict(play_deg=3.0, mu=1.0, servo_scale=1.0),
                 dict(play_deg=3.0, mu=0.7, servo_scale=0.8), dict(play_deg=3.0, mu=0.7, servo_scale=0.65)]
        stall = __import__("design_gates").SERVOS["sts3215"]["stall"]
        # flags: --hip=DEG (tuck hip flexion, default H = -125; -117 is where
        # the CAD's thigh front wall meets the roll flange -- hip-yoke-single-
        # print.md section 6 item 1, in BOTH yoke variants), --knee=DEG, and
        # --broad (also sweep ankle push -10 and push time 3 s).
        global H, K
        names, broad = [], False
        for a in sys.argv[2:]:
            if a.startswith("--hip="):
                H = float(a.split("=", 1)[1])
            elif a.startswith("--knee="):
                K = float(a.split("=", 1)[1])
            elif a == "--broad":
                broad = True
            else:
                names.append(a)
        ankles = (-40, -25, -10) if broad else (-40, -25)
        t_pushes = (2.0, 3.0) if broad else (2.0,)
        print(f"== GET-UP (seat push from supine), named configs, self_collide=True; tuck hip {H:+.0f} knee {K:+.0f}; STS3215 sim stall {stall:.2f} N*m")
        for name in names:
            p, xml = plant(name)
            print(f"-- {name}: {CONFIGS[name]}", flush=True)
            winners = []
            grid = list(itertools.product(((60, -60), (90, -90)), ((20, -20), (0, 0), (30, 0)), ankles, t_pushes))
            for (s0, e0), (s1, e1), a_push, tp in grid:
                args = (False, s0, s1, tp, a_push, 0.0, 0.0)
                r = run(f"{name:18s} sh {s0:+4d}->{s1:+4d} el {e0:+4d}->{e1:+4d} ankle {a_push} t {tp:.0f}",
                        p, xml, seat_push(*args, elbow=(e0, e1)))
                if r["ok"]:
                    winners.append((s0, s1, a_push, (e0, e1), tp))
            print(f"-- {name}: {len(winners)}/{len(grid)} seat-push variants stand", flush=True)
            best = None
            for s0, s1, a_push, el, tp in winners:
                seq = seat_push(False, s0, s1, tp, a_push, 0.0, 0.0, elbow=el)
                n_ok, peak_max = 0, {}
                for c in conds:
                    rt = run_traced(p, xml, seq, start="supine", **c)
                    n_ok += rt["ok"]
                    for k, v in rt["peak"].items():
                        peak_max[k] = max(peak_max.get(k, 0.0), v)
                pk = " ".join(f"{k} {v:.2f}" for k, v in sorted(peak_max.items()))
                print(f"   robust {name:18s} sh {s0:+4d}->{s1:+4d} el {el[0]:+4d}->{el[1]:+4d} ankle {a_push} t {tp:.0f}: "
                      f"{n_ok}/6   peak {pk}", flush=True)
                if best is None or n_ok > best[0]:
                    best = (n_ok, (s0, s1, a_push, el, tp), pk)
            if best:
                print(f"== {name}: BEST {best[0]}/6 robust  {best[1]}  peak {best[2]}", flush=True)
            else:
                print(f"== {name}: NOTHING STANDS", flush=True)
        return
    if mode == "walkarmcontacts":
        print("== walk gate (8 steps, turn 0, nominal mu 0.7 play 3, STS3250 rolls+knees), hanging idle pose (shoulder 0 / elbow 0), self_collide=True -- arm-vs-leg contact count over the walk")
        argv = sys.argv[2:]
        pose = (0.0, 0.0)
        if argv and argv[0].startswith("--pose="):
            pose = tuple(float(v) for v in argv[0].split("=", 1)[1].split(","))
            argv = argv[1:]
            print(f"   held idle arm pose: shoulder {pose[0]:+.0f} deg, elbow {pose[1]:+.0f} deg")
        names = argv or ["top1_len35", "top_elbow_18_18"]
        pj = {"L_knee": "sts3250", "R_knee": "sts3250", "L_hip_roll": "sts3250", "R_hip_roll": "sts3250", "L_ankle_roll": "sts3250", "R_ankle_roll": "sts3250"}
        for name in names:
            if name in CONFIGS:
                p = dataclasses.replace(BASE, **CONFIGS[name])
            else:
                up_len, fore_len = [float(x) / 100 for x in name.replace("top_elbow_", "").split("_")]
                p = dataclasses.replace(BASE, arms=True, arm_z=TOP_Z, arm_len=up_len, arm_elbow=True, arm_fore_len=fore_len)
            xml = os.path.join(SP, f"gu_wc_{name}_{os.getpid()}.xml"); open(xml, "w").write(build_xml(p))
            r = walk_with_arm_contacts(p, xml, per_joint=pj, arm_pose=pose)
            print(f"{name:20s} {'FELL' if r['fell'] else 'up  '} tilt_max {r['tilt_max']:.1f} deg  "
                  f"arm-vs-leg contacts {r['arm_leg_contacts']} over {r['t_final']:.1f}s  pairs {r['pairs']}", flush=True)
        return
    if mode == "elbowsweep":
        print("== ELBOW-LENGTH SWEEP (Tom 2026-09-16: idle pose hangs at the sides, elbows wanted): shortest 2-DOF elbow arm at TOP_Z {:.3f} m that stands robustly. seat push, supine, self_collide=True.".format(TOP_Z))
        lens = eval(sys.argv[2]) if len(sys.argv) > 2 else [(0.16, 0.16), (0.18, 0.18), (0.20, 0.20), (0.18, 0.22), (0.22, 0.18)]
        winners = []
        for up_len, fore_len in lens:
            name = f"top_elbow_{up_len*100:.0f}_{fore_len*100:.0f}"
            p = dataclasses.replace(BASE, arms=True, arm_z=TOP_Z, arm_len=up_len, arm_elbow=True, arm_fore_len=fore_len)
            xml = os.path.join(SP, f"gu_sh_{name}_{os.getpid()}.xml"); open(xml, "w").write(build_xml(p))
            print(f"-- {name}: arm_len {up_len} arm_fore_len {fore_len}  reach {up_len+fore_len:.2f} m  4 servos (2/arm)", flush=True)
            any_ok = False
            for (s0, e0), (s1, e1), a_push in itertools.product(((60, -60), (90, -90)), ((20, -20), (0, 0)), (-40, -25)):
                r = run(f"{name:20s} sh {s0:+4d}->{s1:+4d} el {e0:+4d}->{e1:+4d} ankle {a_push}",
                        p, xml, seat_push(False, s0, s1, 2.0, a_push, elbow=(e0, e1)))
                if r["ok"]:
                    any_ok = True
                    winners.append((name, p, xml, (s0, s1, 2.0, a_push, 0.0, 0.0, (e0, e1))))
            print(f"-- {name}: {'at least one variant STANDING' if any_ok else 'no variant stood'}", flush=True)
        print("candidate winners for the robustness pass:", [(w[0]) for w in winners])
        return
    elif mode == "elbowrobust":
        print("== ROBUSTNESS + peak torque, elbow-length winners (play 3/5, mu 0.3/0.7/1.0, servo 100/80/65%; STS3215 sim stall {:.2f} N*m)".format(
            __import__("design_gates").SERVOS["sts3215"]["stall"]))
        winners = eval(sys.argv[2])   # [(name, args), ...] args = (s0,s1,t,ankle,abd0,abd1,(e0,e1))
        conds = [dict(play_deg=3.0, mu=0.7, servo_scale=1.0), dict(play_deg=5.0, mu=0.7, servo_scale=1.0), dict(play_deg=3.0, mu=0.3, servo_scale=1.0),
                 dict(play_deg=3.0, mu=1.0, servo_scale=1.0), dict(play_deg=3.0, mu=0.7, servo_scale=0.8), dict(play_deg=3.0, mu=0.7, servo_scale=0.65)]
        for name, args in winners:
            if name in CONFIGS:
                p, xml = plant(name)
            else:
                up_len, fore_len = [float(x) / 100 for x in name.replace("top_elbow_", "").split("_")]
                p = dataclasses.replace(BASE, arms=True, arm_z=TOP_Z, arm_len=up_len, arm_elbow=True, arm_fore_len=fore_len)
                xml = os.path.join(SP, f"gu_sh_{name}_{os.getpid()}.xml"); open(xml, "w").write(build_xml(p))
            seq = seat_push(False, *args)
            n_ok = 0
            for c in conds:
                rt = run_traced(p, xml, seq, start="supine", **c)
                n_ok += rt["ok"]
                pk = " ".join(f"{k} {v:.2f}" for k, v in rt["peak"].items())
                print(f"{name:20s} play {c['play_deg']:.0f} mu {c['mu']:.1f} servo {c['servo_scale']:.2f}  "
                      f"{'STANDING' if rt['ok'] else 'no      '} z {rt['pelvis_z']:.3f}  peak: {pk}", flush=True)
            print(f"-- {name}: {n_ok}/{len(conds)} robust", flush=True)
        return
    elif mode == "hangpose":
        print("== HANGING IDLE POSE (shoulder 0 / elbow 0 = straight down at the sides): hand position + leg clearance, standing plant, mj_forward only (no dynamics).")
        import mujoco as mj
        for name in (sys.argv[2:] or ["top1_len35", "top_elbow_18_18"]):
            if name in CONFIGS:
                p = dataclasses.replace(BASE, **CONFIGS[name])
            else:
                up_len, fore_len = [float(x) / 100 for x in name.replace("top_elbow_", "").split("_")]
                p = dataclasses.replace(BASE, arms=True, arm_z=TOP_Z, arm_len=up_len, arm_elbow=True, arm_fore_len=fore_len)
            xml = os.path.join(SP, f"gu_hang_{name}_{os.getpid()}.xml"); open(xml, "w").write(build_xml(p))
            m = mj.MjModel.from_xml_string(open(xml).read()); d = mj.MjData(m)
            mj.mj_forward(m, d)
            for side in ("L", "R"):
                hand_geom = f"{side}_forearm" if p.arm_elbow else f"{side}_arm"
                hb = m.body(f"{side}_forearm" if p.arm_elbow else f"{side}_arm").id
                # hand site: use the body's own frame + the known local hand offset along -z
                hand_len = p.arm_fore_len if p.arm_elbow else p.arm_len
                hand_world = d.xpos[hb] + d.xmat[hb].reshape(3, 3) @ np.array([0, 0, -hand_len])
                thigh_bid = m.body(f"{side}_thigh").id
                thigh_world = d.xpos[thigh_bid]
                print(f"{name:20s} {side} hand world z {hand_world[2]*1e3:.0f} mm (floor=0)  "
                      f"lateral gap to {side}_thigh axis {abs(hand_world[1]-thigh_world[1])*1e3:.0f} mm  "
                      f"hand xyz {hand_world[0]*1e3:.0f},{hand_world[1]*1e3:.0f},{hand_world[2]*1e3:.0f} mm", flush=True)
        return
    if mode == "seat":
        print("== SEAT PUSH, top-of-torso shoulders (TOP_Z {:.3f} m above the yaw axis = deck_bot+deck_t; sanity config at the round-2 negative height {:.3f} m): "
              "supine -> sit up (arms folded UP along the torso) -> arm(s) swung down/out behind or beside the hips (brace) -> tuck (knee -130, hip -125) -> "
              "push while the ankle dorsiflexes -> rise. self_collide=True.".format(TOP_Z, OLD_NEG_Z))
        for name in CONFIGS:
            p, xml = plant(name)
            extra = 0.055 * (2 if p.arm_abd else 0) + 0.055 * 2 + 0.055 * (2 if p.arm_elbow else 0)
            print(f"-- {name}: {CONFIGS[name]}  extra servo mass ~{extra:.3f} kg", flush=True)
            abd_opts = ((0.0, 0.0),) if not p.arm_abd else ((0.0, 0.0), (30.0, 60.0), (0.0, 60.0))
            if p.arm_elbow:
                for (s0, e0), (s1, e1), a_push, (abd0, abd1) in itertools.product(
                        ((60, -60), (90, -90)), ((20, -20), (0, 0), (30, 0)), (-40, -25), abd_opts):
                    run(f"{name:22s} sh {s0:+4d}->{s1:+4d} el {e0:+4d}->{e1:+4d} abd {abd0:.0f}->{abd1:.0f} ankle {a_push}",
                        p, xml, seat_push(p.arm_abd, s0, s1, 2.0, a_push, abd0, abd1, elbow=(e0, e1)))
                continue
            for s0, s1, a_push, (abd0, abd1) in itertools.product((70, 50, 30), (20, 0, -20), (-40, -25), abd_opts):
                run(f"{name:22s} sh {s0:+4d}->{s1:+4d} abd {abd0:.0f}->{abd1:.0f} ankle {a_push}",
                    p, xml, seat_push(p.arm_abd, s0, s1, 2.0, a_push, abd0, abd1))
    elif mode == "sideseat":
        print("== SIDE-SIT PUSH: one arm swings OUT (abd) and down to plant beside the hip on that side; the other stays folded. Only the arm_abd configs can try this.")
        for name in ("top_abd_20", "top_abd_28", "top_abd_elbow_15_15", "top_abd_elbow_18_18", "top_abd_20_wide"):
            p, xml = plant(name)
            for s0, s1, abd0, abd1, a_push in itertools.product((60, 40), (0, -20), (40, 70), (70, 100), (-40, -25)):
                run(f"{name:18s} L sh {s0:+4d}->{s1:+4d} abd {abd0:.0f}->{abd1:.0f} ankle {a_push}",
                    p, xml, side_seat_push("L", s0, s1, abd0, abd1, 2.0, a_push))
    elif mode == "pushup":
        print("== PRONE PUSH-UP with top-of-torso arms: both arms reach forward to the floor, push the torso up, knees tuck under, try to sit back onto the heels. Round-2 hip-level arms were 0/76 here; this asks whether a TOP mount (closer to a real push-up) changes it.")
        for name in ("top1_len28", "top1_len35", "top_elbow_15_15", "top_elbow_18_18", "top_abd_20", "top_abd_28", "top_abd_elbow_15_15", "top_abd_elbow_18_18"):
            p, xml = plant(name)
            abds = (0.0,) if not p.arm_abd else (0.0, 30.0)
            if p.arm_elbow:
                for s_floor, (e1, e2), abd in itertools.product((-60, -80), ((-40, -80), (0, -40)), abds):
                    run(f"{name:18s} floor sh {s_floor} elbow ->{e1}->{e2} abd {abd:.0f}",
                        p, xml, pushup_seq(p.arm_abd, s_floor, s_floor - 20, s_floor - 40, abd, elbow=(0, e1, e2)), start="prone")
                continue
            for s_floor, s1, s2, abd in itertools.product((-60, -80), (-90, -110), (-110, -130), abds):
                if s2 >= s1:
                    continue
                run(f"{name:18s} floor sh {s_floor} -> {s1} -> {s2} abd {abd:.0f}",
                    p, xml, pushup_seq(p.arm_abd, s_floor, s1, s2, abd), start="prone")
    elif mode == "robust":
        print("== ROBUSTNESS of the seat-push winners: play, friction, servo strength (nominal = play 3, mu 0.7, servo 1.0)")
        winners = eval(sys.argv[2]) if len(sys.argv) > 2 else []
        conds = [dict(play_deg=3.0, mu=0.7, servo_scale=1.0), dict(play_deg=5.0, mu=0.7, servo_scale=1.0), dict(play_deg=3.0, mu=0.3, servo_scale=1.0),
                 dict(play_deg=3.0, mu=1.0, servo_scale=1.0), dict(play_deg=3.0, mu=0.7, servo_scale=0.8), dict(play_deg=3.0, mu=0.7, servo_scale=0.65)]
        for name, args in winners:
            p, xml = plant(name)
            seq = seat_push(p.arm_abd, *args)
            for c in conds:
                r = G.run_sequence(p, xml, seq, start="supine", per_joint=G.PJ_DEFAULT, verbose=False, **c)
                worst = max(r["log"], key=lambda L: L[4])
                print(f"{name:20s} play {c['play_deg']:.0f} mu {c['mu']:.1f} servo {c['servo_scale']:.2f}  "
                      f"{'STANDING' if r['ok'] else 'no      '} up {r['up']:+.2f} z {r['pelvis_z']:.3f}  |tau|max {worst[4]:.2f} Nm ({worst[5]} @ {worst[0]})", flush=True)
                if r["ok"]: hits.append(name)
    elif mode == "render":
        # render <config> <variant s0,s1,t,ankle[,abd0,abd1][,e0,e1]> <out.mp4> [side|rear] [seat|pushup]
        name, variant, out = sys.argv[2], sys.argv[3], sys.argv[4]
        view = sys.argv[5] if len(sys.argv) > 5 else "side"
        what = sys.argv[6] if len(sys.argv) > 6 else "seat"
        p, xml = plant(name)
        v = [float(x) for x in variant.split(",")]
        cam = (1.25, -12, 90) if view == "side" else (1.3, -15, 135)
        cap = lambda lab, t: f"{name}  {what}   {lab}   t={t:4.1f}s"
        if what == "seat":
            s0, s1, t_push, a_push = v[:4]
            abd0, abd1 = (v[4], v[5]) if len(v) > 5 else (0.0, 0.0)
            elbow = (v[6], v[7]) if len(v) > 7 else None
            seq, start = seat_push(p.arm_abd, s0, s1, t_push, a_push, abd0, abd1, elbow=elbow), "supine"
        else:
            s_floor, s1, s2 = v[:3]
            abd = v[3] if len(v) > 3 else 0.0
            seq, start = pushup_seq(p.arm_abd, s_floor, s1, s2, abd), "prone"
        r = G.run_sequence(p, xml, seq, start=start, play_deg=3.0, per_joint=G.PJ_DEFAULT, verbose=False,
                           render=out, cam=cam, size=(540, 720), label_fn=cap)
        print(f"{name} {what} {variant}: {'STANDING' if r['ok'] else 'not standing'} -> {out}")
        return
    print("standing:", hits)


if __name__ == "__main__":
    main()
