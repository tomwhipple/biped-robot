"""Get-up study, Option 1 (2026-09-16, Tom: "redesign the chassis to be more
stable, perhaps with legs mounted on the sides, frog/bird style"): does moving
the hip axis OUTBOARD and UP the torso's own side wall (so the torso hangs
BETWEEN the legs instead of entirely above them) admit an open-loop get-up
from supine, prone AND the side, where the current body (hips at the torso's
bottom edge, tall torso above) cannot?

Plant: gen_plant_v6.hip_z (new, default 0.0 = today's body) is the height of
the hip YAW axis above the torso's own local origin; the torso's own internal
geometry (deck/housing/head/battery) is unchanged in ITS frame, so hip_z > 0
pulls that same block DOWN relative to the hip line -- part of the torso now
hangs below the hips (a bird "belly") and part stays above. Combine with
hip_sep (existing param) to put the hip axis outboard of the torso's own side
wall (deck_w/2 = 59 mm), and with hip_roll_abd (existing param) for a wide
frog splay. ALL runs here use self_collide=True (2026-09-16, Tom via the
coordinator: a splayed/raised leg can pass through the torso or the other leg
if it is off) -- see docs/design-v6-ankle-roll.md and the self_collide commit
(759fe96) for why.

Approximation, stated once: Option 2 of the brief ("horizontal bird torso,
head forward, legs under the middle") needs the torso's long axis horizontal
in the REST pose, which this generator cannot express -- _torso()/_head()
assume the hip yaw axis is vertical and the torso stacks vertically above/
below it (see gen_plant_v6._torso). The closest proxy this file can build is
hip_z pushed to the TOP of the torso stack (most structure hangs below the
hip line, "bird_topmount" below) -- tested and reported as an approximation,
not a horizontal reorientation.

    .venv/bin/python sim/getup_v6_side.py probe                    # quasi-static CoM/corridor numbers
    .venv/bin/python sim/getup_v6_side.py search supine             # keyframe search, supine -> stand
    .venv/bin/python sim/getup_v6_side.py search prone
    .venv/bin/python sim/getup_v6_side.py search side
    .venv/bin/python sim/getup_v6_side.py robust                    # robustness pass on any winners
    .venv/bin/python sim/getup_v6_side.py gate                      # walk-gate check on any winners
    .venv/bin/python sim/getup_v6_side.py render <config> <seq> out.mp4 [supine|prone|side_l|side_r]
"""
import sys, os, itertools, dataclasses, math
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402

from gen_plant_v6 import DesignParams, build_xml  # noqa: E402
import v6_kin as VK  # noqa: E402
import getup_v6 as G  # noqa: E402
import static_gait as SG  # noqa: E402

SP = os.environ.get("TMPDIR", "/tmp")
K, H = -130.0, -125.0   # deep-flexion knee/hip (docs sec 11.3), same targets as the other get-up studies
BASE = DesignParams(self_collide=True)   # EVERY run in this file: self-collision on

# hip_z landmarks (measured, gen_plant_v6 defaults): housing bottom->top
# 0.043 m (housing_h), deck top 0.079 m, neck/head stack to ~0.164 m above
# origin. deck_w/2 = 0.059 m is the torso's own half-width (today's hip_sep/2
# = 0.042 m sits INSIDE that; hip_sep 0.16 m puts the hip axis 0.021 m
# outboard of the side wall).
CONFIGS = {
    "stock": {},   # the default v7 plant (self_collide=True via BASE only) -- fall-census baseline
    # bird: hips raised + moved outboard, splay unchanged (45 deg, plant default)
    "bird_z06_sep160": dict(hip_z=0.06, hip_sep=0.160),     # "hip roll height" from the appendage study, for comparison
    "bird_z10_sep160": dict(hip_z=0.10, hip_sep=0.160),
    "bird_z15_sep160": dict(hip_z=0.15, hip_sep=0.160),     # roughly mid-stack: torso straddles the hip line
    "bird_z20_sep160": dict(hip_z=0.20, hip_sep=0.160),     # above the deck top: most of the torso hangs below the hips
    "bird_z15_sep200": dict(hip_z=0.15, hip_sep=0.200),     # wider stance, same height
    # frog splay only (today's hip mount, wide abduction range)
    "frog_splay_abd75": dict(hip_roll_abd=75.0),
    "frog_splay_abd90": dict(hip_roll_abd=90.0),
    # bird + frog splay combined
    "bird_z15_splay75": dict(hip_z=0.15, hip_sep=0.160, hip_roll_abd=75.0),
    "bird_z15_splay90": dict(hip_z=0.15, hip_sep=0.160, hip_roll_abd=90.0),
    # approximation of Option 2 ("horizontal torso"): hips at the TOP of the
    # stack, so nearly the whole torso (housing+deck+head) hangs BELOW the
    # hip line, like a torso slung low between long-legged sides
    "bird_topmount": dict(hip_z=0.20, hip_sep=0.180, hip_roll_abd=75.0),
    # ---- round 2 (2026-09-16): a GENUINELY horizontal torso, gen_plant_v6
    # torso_pitch/torso_block_x/torso_block_z. Root torso frame (hip yaw
    # axes, IMU site) is untouched -- only the deck/housing/battery/Pi/power/
    # neck+head block rotates 90 deg and is repositioned so the whole-robot
    # standing CoM sits over the hip line and the block's own bottom is just
    # below hip-yaw level (both measured with mj_forward, see the study doc):
    # tx=-0.06, tz=+0.03 -> CoM x +0.002 (box) / -0.0003 (round), CoM z
    # 0.280 m (vs 0.292 m stock), block bottom 16-22 mm below the hip line.
    # hip_sep 0.20 m clears the legs past the block sides (ncon 0 at stand).
    "horiz_box": dict(torso_pitch=90.0, torso_block_x=-0.06, torso_block_z=0.03, hip_sep=0.20),
    "horiz_round": dict(torso_pitch=90.0, torso_block_x=-0.06, torso_block_z=0.03, hip_sep=0.20, torso_round=True),
    # ---- round 3 (2026-09-17), Tom: "a flattened body, parallel to the
    # ground ... closer to the original bimo inspiration" -- ONE flat slab
    # (bird_body=True), hips at its sides at mid-height (hip_z stays 0.0:
    # the slab is centred ON the hip line by construction, see
    # gen_plant_v6._bird_body_torso). hip_sep 0.18 m = bird_W (0.14) +
    # SV_WID (0.02472) + 2x6 mm clearance, ROUNDED UP from 0.177 (the yaw
    # servo and the slab share the SAME rigid body, so self_collide's
    # contact count cannot check this pair -- measured analytically, then
    # confirmed 0 self-collision contacts for the LEG-vs-slab pair, which
    # self_collide CAN see, at standing). knee="both" (+-130/+95, already in
    # the plant), hip yaw +-180, hip_roll_abd swept 90/120.
    "bird3_box": dict(bird_body=True, hip_sep=0.18, knee="both", yaw_range=180.0, hip_roll_abd=90.0),
    "bird3_round": dict(bird_body=True, hip_sep=0.18, knee="both", yaw_range=180.0, hip_roll_abd=90.0, torso_round=True),
    "bird3_box_abd120": dict(bird_body=True, hip_sep=0.18, knee="both", yaw_range=180.0, hip_roll_abd=120.0),
    "bird3_round_abd120": dict(bird_body=True, hip_sep=0.18, knee="both", yaw_range=180.0, hip_roll_abd=120.0, torso_round=True),
}
hits = []


def plant(name, **extra):
    p = dataclasses.replace(BASE, **CONFIGS[name], **extra)
    xml = os.path.join(SP, f"gu_side_{name}_{os.getpid()}.xml")
    open(xml, "w").write(build_xml(p))
    return p, xml


def q_splay(v):
    """symmetric OUTWARD hip abduction (v > 0 = legs splayed apart): L abducts
    at +roll, R abducts at -roll (gen_plant_v6._leg roll_rng convention)."""
    return dict(L_hip_roll=v, R_hip_roll=-v)


def run(label, p, xml, seq, start="supine", render=None):
    r = G.run_sequence(p, xml, seq, start=start, play_deg=3.0, per_joint=G.PJ_DEFAULT, verbose=False, render=render)
    trail = " | ".join(f"{L[0][:11]}:{L[1]:+.2f}/{L[2]:.2f}/{'+'.join(c.replace('_','')[:5] for c in L[3])}" for L in r['log'][1:])
    print(f"{label:70s} {'STANDING' if r['ok'] else 'no      '} up {r['up']:+.2f} front {r['front']:+.2f} z {r['pelvis_z']:.3f}\n      {trail}", flush=True)
    if r['ok']:
        hits.append(label)
    return r


# --------------------------------------------------------------------------- sequences
def seq_situp_wide(splay, a_push):
    """supine -> wide base (frog splay) -> sit up -> tuck -> push (ankle
    dorsiflex) -> narrow the stance while rising -> stand."""
    sp = q_splay(splay)
    z = q_splay(0.0)
    return [("lie", dict(), 0.5, 1.0),
            ("splay", dict(**sp), 1.0, 0.6),
            ("sit up", dict(hip_pitch=-90, **sp), 1.5, 0.8),
            ("tuck", dict(hip_pitch=H, knee=K, ankle=0, **sp), 1.5, 0.8),
            ("push", dict(hip_pitch=H, knee=K, ankle=a_push, **sp), 1.5, 1.0),
            ("crouch hold", dict(hip_pitch=H, knee=K, ankle=a_push, **sp), 1.0, 1.5),
            ("narrow + rise 2", dict(hip_pitch=-80, knee=-70, ankle=-30, **z), 1.5, 1.0),
            ("rise 3", dict(hip_pitch=-45, knee=-50, ankle=-25, **z), 1.5, 1.0),
            ("stand", dict(hip_pitch=-20, knee=-40, ankle=-20, **z), 1.5, 1.0),
            ("straight", dict(**z), 1.0, 0.8)]


def seq_pushup_wide(splay, a_push):
    """prone -> splay wide (frog base) -> tuck knees under -> push the pelvis
    up on the wide splayed legs (knee/ankle extend) -> hinge up -> narrow the
    stance -> stand."""
    sp = q_splay(splay)
    z = q_splay(0.0)
    return [("lie prone", dict(), 0.5, 0.8),
            ("splay", dict(**sp), 1.0, 0.6),
            ("tuck knees under", dict(hip_pitch=H, knee=K, ankle=40, **sp), 2.0, 1.0),
            ("push pelvis up (knee/ankle extend)", dict(hip_pitch=H, knee=-40, ankle=a_push, **sp), 2.0, 1.5),
            ("hinge up", dict(hip_pitch=-60, knee=-40, ankle=-25, **sp), 2.0, 1.0),
            ("narrow", dict(hip_pitch=-60, knee=-40, ankle=-25, **z), 1.0, 0.8),
            ("stand", dict(hip_pitch=-20, knee=-40, ankle=-20, **z), 1.5, 1.0),
            ("straight", dict(**z), 1.0, 0.8)]


def seq_side_roll(side, splay, kick):
    """on the SIDE: the lower leg presses the floor (abduct), the upper leg
    swings up and over (adduct/extend) to roll the body prone or supine,
    then a push (reuses seq_pushup_wide's push shape). side = 'l' | 'r'
    names which side is DOWN (settle_fallen 'side_l'/'side_r')."""
    lower, upper = ("R", "L") if side == "l" else ("L", "R")
    s_lo = -1.0 if lower == "L" else 1.0     # lower leg presses OUTWARD (abduct away from the down side)
    s_up = +1.0 if upper == "L" else -1.0    # upper leg swings the other way, up and over
    return [("lie on side", dict(), 0.5, 0.8),
            (f"{lower} presses, {upper} up+over", {f"{lower}_hip_roll": s_lo * splay, f"{lower}_hip_pitch": -20,
             f"{upper}_hip_roll": s_up * kick, f"{upper}_hip_pitch": -30, f"{upper}_knee": -30}, 1.5, 1.2),
            ("legs neutral", dict(), 1.5, 1.5)]


def full_from_side(side, splay, kick, push_splay, a_push):
    roll = seq_side_roll(side, splay, kick)
    push = seq_pushup_wide(push_splay, a_push)
    return roll + push[1:]   # drop the push sequence's own 'lie prone' (the roll already ends lying down)


def seq_pushup_tuned(splay, knee_push, a_push, t_push, hold_push_s):
    """round 2, task 2: seq_pushup_wide with the push travel (knee_push, was
    fixed at -40), the ankle angle and the push/hold TIMING all open, to
    tune the bird_z15/horizontal-torso over-rotation (pelvis 0.193 m, up-
    vector -0.99) found in round 1 -- a smaller knee_push and a shorter push
    with a hold before continuing should stop it overshooting past vertical."""
    sp = q_splay(splay)
    z = q_splay(0.0)
    return [("lie prone", dict(), 0.5, 0.8),
            ("splay", dict(**sp), 1.0, 0.6),
            ("tuck knees under", dict(hip_pitch=H, knee=K, ankle=40, **sp), 2.0, 1.0),
            ("push pelvis up (knee/ankle extend)", dict(hip_pitch=H, knee=knee_push, ankle=a_push, **sp), t_push, hold_push_s),
            ("hinge up", dict(hip_pitch=-60, knee=-40, ankle=-25, **sp), 2.0, 1.0),
            ("narrow", dict(hip_pitch=-60, knee=-40, ankle=-25, **z), 1.0, 0.8),
            ("stand", dict(hip_pitch=-20, knee=-40, ankle=-20, **z), 1.5, 1.0),
            ("straight", dict(**z), 1.0, 0.8)]


def seq_roll_over(side, swing, yaw):
    """round 2, task 1 (supine roll): swing ONE leg via hip ROLL + YAW (the
    other leg counter-adducts a little for reaction) to try to log-roll the
    body about its fore-aft axis. Reports (not just standing) via the
    'rollcheck' mode below -- max tilt from the settled supine attitude and
    peak servo torque, since a partial roll is real information even when
    it does not reach standing."""
    o = "R" if side == "L" else "L"
    return [("lie", dict(), 0.5, 1.0),
            (f"{side} swing (roll+yaw)", {f"{side}_hip_roll": swing, f"{side}_hip_yaw": yaw, f"{o}_hip_roll": -0.3 * swing}, 1.5, 2.5)]


def rotation_angle_deg(q0, q1):
    """angle (deg) between two orientation quaternions [w,x,y,z]."""
    dp = min(1.0, abs(float(np.dot(q0, q1))))
    return math.degrees(2 * math.acos(dp))


def roll_check(name, side, swing, yaw):
    """run seq_roll_over under the full deploy servo model and report the
    PEAK tilt reached (deg, from the settled supine attitude) and the peak
    servo torque -- the number the brief asks for when a body 'cannot roll'."""
    p, xml = plant(name)
    env = SG.make_env(p, xml, mu=0.7, play_deg=3.0)
    obs, _ = env.reset(seed=0)
    G.EXTRA[:] = [env.model.actuator(i).name for i in range(12, env.model.nu)]
    m, d = env.model, env.data
    q0 = np.zeros(env._nq_act)
    G.settle_fallen(env, p, "supine", q0)
    q_init = d.qpos[3:7].copy()
    seq = seq_roll_over(side, swing, yaw)
    d0, hi, lo = env._default, env._hi, env._lo

    def inv(q):
        q = q[:env._nq_act] if len(q) >= env._nq_act else np.concatenate([q, np.zeros(env._nq_act - len(q))])
        return np.clip(np.where(q >= d0, (q - d0) / np.maximum(hi - d0, 1e-6), (q - d0) / np.maximum(d0 - lo, 1e-6)), -1, 1)
    q_prev = q0.copy()
    max_tilt, max_tau = 0.0, 0.0
    dt = env.control_dt
    for label, off, move_s, hold_s in seq[1:]:
        q_tgt = G.q_from_offsets(off)[:env._nq_act]
        n = int(move_s / dt)
        for i in range(n + int(hold_s / dt)):
            s = min(1.0, (i + 1) / max(n, 1))
            s = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
            q = q_prev + (q_tgt - q_prev) * s
            env.step(inv(q))
            max_tilt = max(max_tilt, rotation_angle_deg(q_init, d.qpos[3:7]))
            max_tau = max(max_tau, float(np.abs(env._servo_tau).max()))
        q_prev = q_tgt
    up = d.xmat[env._torso_bid].reshape(3, 3)[2, 2]
    print(f"{name:16s} roll {side} swing {swing:+4.0f} yaw {yaw:+4.0f}: peak tilt {max_tilt:5.1f} deg  peak tau {max_tau:5.2f} Nm  final up {up:+.2f}  z {float(d.qpos[2]):.3f}", flush=True)
    return max_tilt, max_tau, up


# --------------------------------------------------------------------------- round 3: flat bird_body
# folded start poses (Tom, coordinator review 09-17): after a real fall the
# legs are not straight -- q values measured in getup_v6_side.py (mj_forward
# settle, splay=90/knee=+-90 or +-45 both land flat, up +-1.00):
BIRD_SUPINE_FOLD = dict(L_hip_roll=90, R_hip_roll=-90, hip_pitch=0, knee=45, ankle=0)     # CoM z 0.049 m (flat on the back)
BIRD_SUPINE_STRAIGHT = dict()                                                             # CoM z 0.167 m (legs neutral, straight up)
BIRD_PRONE_FOLD = dict(L_hip_roll=90, R_hip_roll=-90, hip_pitch=0, knee=-90, ankle=0)      # up +1.00 flat prone (vs tipping onto an edge from straight legs)


def _bird_legs(L_roll=None, R_roll=None, **rest):
    """explicit-key dict builder for the round-3 sequences -- avoids the
    generic/specific key-order pitfall (q_from_offsets processes dict keys
    in order, so a generic 'knee' AFTER a specific 'L_knee' would clobber it)
    by always emitting fully explicit L_/R_ keys."""
    out = {}
    for k, v in rest.items():
        if k.startswith("L_") or k.startswith("R_"):
            out[k] = v
        else:
            out[f"L_{k}"] = v
            out[f"R_{k}"] = v
    if L_roll is not None:
        out["L_hip_roll"] = L_roll
    if R_roll is not None:
        out["R_hip_roll"] = R_roll
    return out


def seq_frog_roll(fold, knee_lift, ankle_lift, push_side, push_extra, yaw_first=0.0):
    """(a)/(b): from the folded start (legs already splayed ~90 deg in the
    floor plane, per BIRD_SUPINE_FOLD), optionally yaw the hips first (b),
    then press both knees to lift the slab off the floor, then push the
    PUSH_SIDE knee harder (more extension) than the other to roll the slab
    over its long edge. fold = BIRD_SUPINE_FOLD or BIRD_SUPINE_STRAIGHT."""
    o = "R" if push_side == "L" else "L"
    yawed = dict(fold, L_hip_yaw=yaw_first, R_hip_yaw=yaw_first) if yaw_first else dict(fold)
    lift = _bird_legs(L_roll=90, R_roll=-90, knee=knee_lift, ankle=ankle_lift)
    push = dict(lift)
    push[f"{push_side}_knee"] = knee_lift + push_extra
    return [("lie", fold, 0.5, 1.0),
            ("yaw" if yaw_first else "hold", yawed, 1.0, 0.6),
            ("lift both", lift, 1.5, 1.0),
            (f"push {push_side}", push, 1.5, 1.5),
            ("hold", push, 0.5, 1.5)]


def seq_stand_inverted(fold, knee_lift, ankle_lift, hip_pitch_over, t_over):
    """(c) "stand up inverted": from the folded start (upside down), press
    both legs to lift the slab clear of the floor, then walk the feet under
    (both knees flex the OTHER way, "either direction, the double joint is
    the point") and pitch the slab over the hip axis (hip_pitch through the
    range) to end right way up."""
    lift = _bird_legs(L_roll=45, R_roll=-45, knee=knee_lift, ankle=ankle_lift)
    tuck = _bird_legs(L_roll=20, R_roll=-20, knee=-90, ankle=ankle_lift, hip_pitch=hip_pitch_over)
    over = _bird_legs(L_roll=0, R_roll=0, knee=-40, ankle=-10, hip_pitch=-hip_pitch_over)
    stand = _bird_legs(L_roll=0, R_roll=0, knee=-30, ankle=-15, hip_pitch=-20)
    return [("lie (inverted)", fold, 0.5, 1.0),
            ("lift clear", lift, 1.5, 1.0),
            ("tuck + pitch start", tuck, t_over, 1.0),
            ("pitch over hip", over, t_over, 1.5),
            ("stand", stand, 1.5, 1.0)]


def seq_bird_situp(fold, hip_pitch_amt):
    """(d) reference: a sit-up-style hip_pitch flex from the folded supine
    start -- there is no torso to fold OVER, so this mostly tests whether
    hip_pitch alone does anything useful for a flat pod."""
    return [("lie", fold, 0.5, 1.0),
            ("pitch", dict(fold, hip_pitch=hip_pitch_amt), 1.5, 1.5),
            ("hold", dict(fold, hip_pitch=hip_pitch_amt), 0.5, 1.5)]


def seq_bird_pushup(splay, knee_push, ankle_push):
    """prone (BIRD_PRONE_FOLD, slab flat, right way up) -> push-up: extend
    the already-folded knees to lift the slab onto standing legs, THEN
    narrow the wide push-up stance in two steps (a single big step from 90
    deg splay to 0 loses the base of support before the legs get under the
    slab and it topples sideways -- measured, `narrow` alone at splay 90
    drops up 0.99 -> 0.33)."""
    fold = BIRD_PRONE_FOLD
    lift = _bird_legs(L_roll=splay, R_roll=-splay, knee=knee_push, ankle=ankle_push)
    half = _bird_legs(L_roll=splay * 0.5, R_roll=-splay * 0.5, knee=knee_push * 0.7, ankle=ankle_push)
    narrow = _bird_legs(L_roll=0, R_roll=0, knee=-40, ankle=-20)
    stand = _bird_legs(L_roll=0, R_roll=0, knee=-30, ankle=-15)
    return [("lie (folded prone)", fold, 0.5, 1.0),
            ("push up", lift, 2.0, 1.5),
            ("half narrow", half, 1.5, 1.0),
            ("narrow", narrow, 1.5, 1.0),
            ("stand", stand, 1.5, 1.0)]


def bird_run(name, label, seq, start, render=None):
    p, xml = plant(name)
    r = G.run_sequence(p, xml, seq, start=start, play_deg=3.0, per_joint=G.PJ_DEFAULT, verbose=False, render=render)
    worst = max(r["log"], key=lambda L: L[4])
    print(f"{name:18s} {label:52s} {'STANDING' if r['ok'] else 'no      '} up {r['up']:+.2f} front {r['front']:+.2f} z {r['pelvis_z']:.3f}  |tau|max {worst[4]:.2f} ({worst[5]})", flush=True)
    if r["ok"]:
        hits.append(f"{name} {label}")
    return r


# --------------------------------------------------------------------------- round 3: fall census
def classify_fall(up, front, side, flat=False):
    """standing / supine / prone / side_l / side_r / edge, from the torso's
    world-Z components of its own local Z/X/Y axes (up/front/side -- these
    are one orthonormal row, up^2+front^2+side^2 == 1).

    The tall (stock) body's LONG axis is local Z (spine) and its SHORT axis
    is local X (chest-normal) -- lying down, front dominant (>0.7) is the
    historical "flat on the ground" resting state (supine/prone,
    front=+-1). The flat bird_body's LONG axis is local X (slab length,
    0.20 m) and its SHORT axis is local Z (0.055 m) -- for THIS shape,
    front (or side) dominant means the LENGTH (or width) edge is vertical,
    a genuine, less-stable EDGE-balancing state, not a flat rest; only up
    dominant is a flat rest (standing right-way-up, or supine upside-down).
    `flat=True` selects that second scheme."""
    if up > 0.7:
        return "standing"
    if flat:
        return "supine" if up < -0.7 else "edge"
    if front > 0.7:
        return "supine"
    if front < -0.7:
        return "prone"
    if side > 0.7:
        return "side_l"
    if side < -0.7:
        return "side_r"
    if up < -0.7:
        return "supine"     # rare/degenerate for the tall body (would need to flip through vertical)
    return "edge"


def find_tip_speed(m, d, env, heading_rad, lo=0.15, hi=1.5, tries=6):
    """binary search (from a stand) for the smallest CoM velocity kick along
    `heading_rad` that knocks the body out of 'standing' after a 3 s settle
    -- "just past tipping"."""
    def falls(speed):
        mujoco.mj_resetData(m, d)
        d.qpos[2] = env.p_stand_z
        mujoco.mj_forward(m, d)
        d.qvel[0] = speed * math.cos(heading_rad)
        d.qvel[1] = speed * math.sin(heading_rad)
        for _ in range(int(3.0 / env.control_dt)):
            env.step(np.zeros(env._nq_act))
        up = d.xmat[env._torso_bid].reshape(3, 3)[2, 2]
        return up < 0.9
    if not falls(hi):
        return hi   # never tips in range -- use hi as "just past tipping"
    for _ in range(tries):
        mid = (lo + hi) / 2
        if falls(mid):
            hi = mid
        else:
            lo = mid
    return hi


def fall_census(name, headings=12, render_one=None):
    p, xml = plant(name)
    env = SG.make_env(p, xml, mu=0.7, play_deg=3.0)
    obs, _ = env.reset(seed=0)
    m, d = env.model, env.data
    env.p_stand_z = p.z_yaw_above_sole - p.hip_z
    tip = find_tip_speed(m, d, env, 0.0)
    print(f"-- {name}: tipping speed (heading 0) {tip:.2f} m/s -> testing {tip:.2f}/{1.5*tip:.2f}/{2*tip:.2f} m/s", flush=True)
    dist = {}
    n = 0
    for hi in range(headings):
        heading = 2 * math.pi * hi / headings
        for mult in (1.0, 1.5, 2.0):
            speed = tip * mult
            mujoco.mj_resetData(m, d)
            d.qpos[2] = env.p_stand_z
            mujoco.mj_forward(m, d)
            d.qvel[0] = speed * math.cos(heading)
            d.qvel[1] = speed * math.sin(heading)
            frames = [] if render_one == (hi, mult) else None
            rnd = cam = None
            if frames is not None:
                rnd = mujoco.Renderer(m, 480, 640)
                cam = mujoco.MjvCamera(); cam.distance, cam.elevation, cam.azimuth = (1.3, -15, 135)
            for k in range(int(3.0 / env.control_dt)):
                env.step(np.zeros(env._nq_act))
                if frames is not None and k % 2 == 0:
                    cam.lookat[:] = [d.qpos[0], d.qpos[1], 0.2]
                    rnd.update_scene(d, cam)
                    frames.append(rnd.render().copy())
            R = d.xmat[m.body("torso").id].reshape(3, 3)
            cls = classify_fall(R[2, 2], R[2, 0], R[2, 1], flat=p.bird_body)
            dist[cls] = dist.get(cls, 0) + 1
            n += 1
            print(f"{name:12s} heading {math.degrees(heading):5.0f} deg  speed {speed:.2f} m/s ({mult:.1f}x)  -> {cls:10s} up {R[2,2]:+.2f} front {R[2,0]:+.2f} side {R[2,1]:+.2f}", flush=True)
            if frames is not None:
                SG._write_video(frames, render_one[2] if len(render_one) > 2 else "sim/renders/getup_side/bird3_fall.mp4")
    print(f"{name:12s} DISTRIBUTION ({n} trials): " + ", ".join(f"{k}={v}" for k, v in sorted(dist.items())), flush=True)
    return dist


def fall_census_midstride(name):
    """same census, but the legs held in a walking mid-stride pose (one hip
    forward+lifted, one back) instead of neutral standing -- cheap: reuses
    the same tip-speed/impulse loop with a fixed non-zero q_hold."""
    p, xml = plant(name)
    env = SG.make_env(p, xml, mu=0.7, play_deg=3.0)
    obs, _ = env.reset(seed=0)
    m, d = env.model, env.data
    na = env._nq_act
    q_mid = G.q_from_offsets(dict(L_hip_pitch=-25, L_knee=-35, R_hip_pitch=20, R_knee=-15))[:na]
    d0, hi_, lo_ = env._default, env._hi, env._lo
    inv = lambda q: np.clip(np.where(q >= d0, (q - d0) / np.maximum(hi_ - d0, 1e-6), (q - d0) / np.maximum(d0 - lo_, 1e-6)), -1, 1)
    mujoco.mj_resetData(m, d)
    d.qpos[2] = p.z_yaw_above_sole - p.hip_z
    mujoco.mj_forward(m, d)
    for _ in range(int(1.0 / env.control_dt)):
        env.step(inv(q_mid))
    tip = 0.5
    dist = {}
    for hi in range(12):
        heading = 2 * math.pi * hi / 12
        for mult in (1.0, 1.5, 2.0):
            speed = tip * mult
            qpos0 = d.qpos.copy()
            d.qvel[:] = 0
            d.qvel[0] = speed * math.cos(heading)
            d.qvel[1] = speed * math.sin(heading)
            for _ in range(int(3.0 / env.control_dt)):
                env.step(inv(q_mid))
            R = d.xmat[m.body("torso").id].reshape(3, 3)
            cls = classify_fall(R[2, 2], R[2, 0], R[2, 1], flat=p.bird_body)
            dist[cls] = dist.get(cls, 0) + 1
            d.qpos[:] = qpos0
            mujoco.mj_forward(m, d)
    print(f"{name:12s} MIDSTRIDE DISTRIBUTION (36 trials, tip speed {tip} m/s fixed): " + ", ".join(f"{k}={v}" for k, v in sorted(dist.items())), flush=True)
    return dist


# --------------------------------------------------------------------------- quasi-static probe
def probe_config(name):
    """quasi-static (no servo dynamics, the plant's own position actuators):
    settle SEATED (from supine, sit up + tuck) and FALLEN PRONE poses, report
    pelvis z, whole-robot CoM vs each sole's outline, and whether a CoM-over-
    sole corridor exists anywhere below pelvis 0.33 m (the measured wall on
    the current body, docs sec 12.2) by sweeping the knee/hip rise arc."""
    p, xml = plant(name)
    m = mujoco.MjModel.from_xml_path(xml)
    d = mujoco.MjData(m)
    print(f"--- {name}  hip_z {p.hip_z*1e3:.0f} mm  hip_sep {p.hip_sep*1e3:.0f} mm  "
          f"abd {p.hip_roll_abd:.0f} deg  standing yaw-axis height {p.z_yaw_above_sole*1e3:.0f} mm")

    def settle(quat, q_hold, seconds=1.2):
        mujoco.mj_resetData(m, d)
        d.qpos[0:3] = [0, 0, 0.15]
        d.qpos[3:7] = quat
        d.qpos[7:7 + 12] = q_hold
        for _ in range(int(seconds / 0.002)):
            for i in range(12):
                lo, hi = m.actuator_ctrlrange[i]
                d.ctrl[i] = np.clip(q_hold[i], lo, hi)
            mujoco.mj_step(m, d)

    def report(label):
        com = d.subtree_com[0].copy()
        up = d.xmat[m.body("torso").id].reshape(3, 3)[2, 2]
        margs = {}
        for side in "LR":
            pos_f, _ = VK.foot_frame(m, d, side)
            grounded = (pos_f[2] - p.roll_h) < 0.03
            c = VK.in_foot(m, d, side, com)
            margs[side] = VK.sole_margin(p, c[:2], side) if grounded else None
        ms = " ".join(f"{s}:{(f'{margs[s]*1e3:+.0f}mm' if margs[s] is not None else '  air ')}" for s in "LR")
        print(f"  {label:26s} pelvis z {d.qpos[2]:.3f}  up {up:+.2f}  CoM ({com[0]:+.3f},{com[1]:+.3f},{com[2]:.3f})  sole margin[{ms}]  contacts {G.contacts_summary(m, d)}")

    # seated (supine -> sit up -> tuck), same targets as the legs-only study
    q0 = np.zeros(12)
    settle([math.cos(math.pi / 4), 0.0, -math.sin(math.pi / 4), 0.0], q0)
    report("supine (lie)")
    q1 = G.q_from_offsets(dict(hip_pitch=-90))[:12]
    settle([math.cos(math.pi / 4), 0.0, -math.sin(math.pi / 4), 0.0], q1)
    report("sit up (hip -90)")
    q2 = G.q_from_offsets(dict(hip_pitch=H, knee=K, ankle=0))[:12]
    settle([math.cos(math.pi / 4), 0.0, -math.sin(math.pi / 4), 0.0], q2)
    report("seated tuck (hip -125 knee -130)")
    # prone tuck (push-up start)
    q3 = G.q_from_offsets(dict(hip_pitch=H, knee=K, ankle=40))[:12]
    settle([math.cos(math.pi / 4), 0.0, math.sin(math.pi / 4), 0.0], q3)
    report("prone tuck (knees under)")
    # rise-arc corridor sweep from the seated tuck: does a (knee, ankle) pair
    # put a sole FLAT under the CoM at any pelvis height below 0.33 m?
    print("  rise-arc sweep (seated tuck, splay 0, hip -125): knee/ankle -> pelvis z / CoM margin")
    found = False
    for knee in (-130, -110, -90, -73, -50, -30, -10):
        for ankle in (-40, -25, -10, 0):
            q = G.q_from_offsets(dict(hip_pitch=H, knee=knee, ankle=ankle))[:12]
            settle([math.cos(math.pi / 4), 0.0, -math.sin(math.pi / 4), 0.0], q, seconds=0.8)
            com = d.subtree_com[0].copy()
            ok_any = False
            for side in "LR":
                pos_f, _ = VK.foot_frame(m, d, side)
                grounded = (pos_f[2] - p.roll_h) < 0.03
                if not grounded:
                    continue
                c = VK.in_foot(m, d, side, com)
                mgn = VK.sole_margin(p, c[:2], side)
                if mgn > 0 and d.qpos[2] < 0.33:
                    ok_any = True
            if ok_any:
                found = True
                print(f"    knee {knee:+4d} ankle {ankle:+3d}  pelvis z {d.qpos[2]:.3f}  CoM-over-sole: YES")
    if not found:
        print("    no (knee, ankle) pair in the sweep puts the CoM over a grounded sole below pelvis 0.33 m")


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "probe"
    if mode == "probe":
        for name in CONFIGS:
            probe_config(name)
    elif mode == "search":
        start_arg = sys.argv[2] if len(sys.argv) > 2 else "supine"
        names = sys.argv[3:] or list(CONFIGS)
        print(f"== SIDE-MOUNTED-LEG search, start={start_arg}, self_collide=True on every run")
        for name in names:
            p, xml = plant(name)
            if start_arg == "supine":
                for splay, a_push in itertools.product((0, 30, 60), (-40, -25)):
                    run(f"{name:20s} situp_wide splay {splay:+3d} ankle {a_push}", p, xml, seq_situp_wide(splay, a_push), start="supine")
            elif start_arg == "prone":
                for splay, a_push in itertools.product((30, 60, 90), (-40, -25)):
                    run(f"{name:20s} pushup_wide splay {splay:+3d} ankle {a_push}", p, xml, seq_pushup_wide(splay, a_push), start="prone")
            elif start_arg == "side":
                for side, splay, kick in itertools.product(("l", "r"), (45, 70), (45, 70)):
                    seq = full_from_side(side, splay, kick, 60.0, -40.0)
                    run(f"{name:20s} side_{side} press {splay:+3d} kick {kick:+3d}", p, xml, seq, start=f"side_{side}")
    elif mode == "robust":
        print("== ROBUSTNESS of any winners: play, friction, servo strength (nominal = play 3, mu 0.7, servo 1.0)")
        # winners filled in by hand after 'search' (kept here so this mode is
        # re-runnable without re-discovering them): (config, seq_fn, start)
        winners = eval(sys.argv[2]) if len(sys.argv) > 2 else []
        conds = [dict(play_deg=3.0, mu=0.7, servo_scale=1.0), dict(play_deg=5.0, mu=0.7, servo_scale=1.0),
                 dict(play_deg=3.0, mu=0.3, servo_scale=1.0), dict(play_deg=3.0, mu=1.0, servo_scale=1.0),
                 dict(play_deg=3.0, mu=0.7, servo_scale=0.8), dict(play_deg=3.0, mu=0.7, servo_scale=0.65)]
        for name, seq, start in winners:
            p, xml = plant(name)
            for c in conds:
                r = G.run_sequence(p, xml, seq, start=start, per_joint=G.PJ_DEFAULT, verbose=False, **c)
                worst = max(r["log"], key=lambda L: L[4])
                print(f"{name:16s} {start:8s} play {c['play_deg']:.0f} mu {c['mu']:.1f} servo {c['servo_scale']:.2f}  "
                      f"{'STANDING' if r['ok'] else 'no      '} up {r['up']:+.2f} z {r['pelvis_z']:.3f}  |tau|max {worst[4]:.2f} Nm ({worst[5]} @ {worst[0]})", flush=True)
    elif mode == "gate":
        # walk-gate check, same 4 cases as docs/design-v6/gateD_appendages.txt
        # (straight, +15 turn, mu 0.3/play 5, -15 turn mu 0.9) for direct
        # comparability, on the plant HANGING AT REST (no get-up sequence).
        names = sys.argv[2:] or list(CONFIGS)
        print("== Gate D (lumped-mass plant) on the side-mounted-leg body at rest, STS3250 rolls+knees, lift 4, 8 steps")
        print("   round 2: v6_kin.hip_roll_point/pose_from_feet now carry a hip_z term (2026-09-16), so hip_z != 0")
        print("   configs run here too -- tests/test_v6_design_gates.py still passes 8/8 at hip_z=0 (unchanged).")
        per_joint = {j: "sts3250" for j in SG.ROLLS + ("L_knee", "R_knee")}
        for name in names:
            p, xml = plant(name)
            # round 3, task 5: if the IK's foot-placement reach fails at this
            # hip_sep (wide-stance bodies need the mechanical hip_sep for leg/
            # slab clearance, but the gait planner's weight-shift was tuned
            # for hip_sep 0.084 -- see study doc round 2), SCALE hip_sep for
            # the IK/timeline solve only (p_ik) -- the real, wide-hip PLANT
            # (xml, built from `p`) still does the physics; p_ik just makes
            # the gait plant NARROWER feet than the real hips (an adducted,
            # narrower-footprint gait), one bird_sep_scale step down at a
            # time. p.knee == "both" also isn't handled by v6_kin.leg_ik
            # (only "fwd"/"bwd" pick a solution branch) -- p_ik forces "fwd"
            # for the walk (normal gait never uses the backward ROM anyway).
            p_ik = p
            if p.bird_body:
                for sep in (p.hip_sep, 0.16, 0.14, 0.12, 0.10):
                    cand = dataclasses.replace(p, hip_sep=sep, knee="fwd")
                    try:
                        SG.walk_timeline(cand, n_steps=8, step=0.06, lift_h=0.04, turn_deg=15.0)
                        p_ik = cand
                        break
                    except ValueError:
                        continue
                if p_ik.hip_sep != p.hip_sep:
                    print(f"{name:24s} gait IK uses hip_sep {p_ik.hip_sep:.2f} m (real hip_sep {p.hip_sep:.2f} m) -- "
                          f"an adducted, narrower-footprint gait; the mechanical hip mount is unchanged")
            for label, kw in (("turn  +0 mu 0.7 play 3", dict()), ("turn +15 mu 0.7 play 3", dict(turn_deg=15.0)),
                              ("turn  +0 mu 0.3 play 5", dict(mu=0.3, play_deg=5.0)), ("turn -15 mu 0.9 play 3", dict(turn_deg=-15.0, mu=0.9))):
                turn = kw.pop("turn_deg", 0.0)
                try:
                    tl2, windows2 = SG.walk_timeline(p_ik, n_steps=8, step=0.06, lift_h=0.04, turn_deg=turn)
                    r = SG.run_walk(p_ik, xml, tl2, windows2, per_joint=per_joint, **kw)
                    print(SG.fmt_row(f"{name:24s} {label}", r))
                except ValueError as e:
                    # a wide hip_sep can ask the gait's foot-placement offsets
                    # (bias_y/swing_out/turn) for more leg reach than thigh+
                    # shank give (0.220 m) -- a real kinematic limit of the
                    # EXISTING gait planner (tuned for hip_sep 0.084), not a
                    # bug in the hip_z fix: measured and reported, not guessed.
                    print(f"{name:24s} {label}  IK REACH EXCEEDED even at hip_sep {p_ik.hip_sep:.2f}: {e}")
    elif mode == "roll":
        # round 2, task 1: supine -> try to roll over by splaying/swinging the
        # legs (hip roll + yaw). Reports peak tilt + peak torque, not just a
        # standing bool, per the brief ("if the body cannot roll, say so with
        # the number").
        names = sys.argv[2:] or [n for n in CONFIGS if n.startswith("horiz") or n.startswith("bird")]
        print("== SUPINE ROLL-OVER attempt: one leg swings via hip roll+yaw, peak tilt from the settled supine attitude + peak torque")
        for name in names:
            for side in ("L", "R"):
                for swing, yaw in itertools.product((60.0, 90.0), (30.0, 45.0)):
                    roll_check(name, side, swing, yaw)
    elif mode == "tune":
        # round 2, task 2: finer keyframe search on the bird_z15 prone push
        # that over-rotated in round 1 (pelvis 0.193 m, up -0.99): sweep the
        # push travel (knee target), ankle angle and push/hold timing.
        names = sys.argv[2:] or ["bird_z15_sep160", "horiz_box", "horiz_round"]
        print("== TUNE the prone push: knee_push/ankle/timing sweep on the over-rotating path")
        best = {}
        for name in names:
            p, xml = plant(name)
            b = (-1e9, None)
            for knee_push, a_push, t_push, hold in itertools.product(
                    (-40, -60, -80, -100), (-40, -25, -10), (1.0, 2.0, 3.0), (0.5, 1.5)):
                seq = seq_pushup_tuned(60.0, knee_push, a_push, t_push, hold)
                r = G.run_sequence(p, xml, seq, start="prone", play_deg=3.0, per_joint=G.PJ_DEFAULT, verbose=False)
                score = r["up"] - 0.2 * abs(r["front"])   # reward upright, mildly penalize residual pitch
                if score > b[0]:
                    b = (score, (knee_push, a_push, t_push, hold, r))
            _, (kp, ap, tp, ho, r) = b
            print(f"{name:16s} BEST knee_push {kp:+4d} ankle {ap:+4d} t_push {tp:.1f}s hold {ho:.1f}s  "
                  f"{'STANDING' if r['ok'] else 'no      '} up {r['up']:+.2f} front {r['front']:+.2f} z {r['pelvis_z']:.3f}", flush=True)
            best[name] = (kp, ap, tp, ho, r["ok"], r["up"], r["pelvis_z"])
        print("best:", best)
    elif mode == "bird3":
        # round 3: flat bird_body get-up, supine (both start folds) / prone / side
        which = sys.argv[2] if len(sys.argv) > 2 else "supine"
        names = sys.argv[3:] or ["bird3_box", "bird3_round"]
        print(f"== BIRD3 flat-body get-up, {which}, self_collide=True")
        for name in names:
            if which == "supine":
                for fold_name, fold in (("straight", BIRD_SUPINE_STRAIGHT), ("folded", BIRD_SUPINE_FOLD)):
                    for knee_lift, ankle_lift in itertools.product((-90, -45, 45, 90), (-30, 0, 30)):
                        for push_side, push_extra in itertools.product(("L", "R"), (40, 80)):
                            seq = seq_frog_roll(fold, knee_lift, ankle_lift, push_side, push_extra)
                            bird_run(name, f"(a) {fold_name} lift{knee_lift:+d}/{ankle_lift:+d} push{push_side}{push_extra:+d}", seq, "bird_back")
                    for yaw in (45, 90):
                        seq = seq_frog_roll(fold, -60, 0, "L", 60, yaw_first=yaw)
                        bird_run(name, f"(b) {fold_name} yaw{yaw:+d} push L+60", seq, "bird_back")
                    for hip_pitch_amt in (-90, 90):
                        seq = seq_bird_situp(fold, hip_pitch_amt)
                        bird_run(name, f"(d) {fold_name} situp pitch{hip_pitch_amt:+d}", seq, "bird_back")
                for knee_lift, hip_pitch_over, t_over in itertools.product((-90, -45), (90, 120), (1.5, 2.5)):
                    seq = seq_stand_inverted(BIRD_SUPINE_STRAIGHT, knee_lift, -20, hip_pitch_over, t_over)
                    bird_run(name, f"(c) inverted lift{knee_lift:+d} pitch{hip_pitch_over:+d} t{t_over}", seq, "bird_back")
            elif which == "prone":
                for splay, knee_push, ankle_push in itertools.product((45, 90, 120), (-40, -60, -90), (-40, -25, -10)):
                    seq = seq_bird_pushup(splay, knee_push, ankle_push)
                    bird_run(name, f"pushup splay{splay:+d} knee{knee_push:+d} ankle{ankle_push:+d}", seq, "bird_flat")
            elif which == "side":
                for side in ("l", "r"):
                    for splay, knee_push in itertools.product((45, 90), (-40, -90)):
                        seq = seq_bird_pushup(splay, knee_push, -25)
                        bird_run(name, f"side_{side} pushup splay{splay:+d} knee{knee_push:+d}", seq, f"side_{side}")
    elif mode == "bird3search":
        # continuous keyframe hill-climb (getup_v6_legs.py style) on the
        # supine flat-body get-up, if the hand-built sequences above fail --
        # symmetric hip_roll/knee/ankle + a shared hip_yaw, 6 nodes.
        import random
        name = sys.argv[2] if len(sys.argv) > 2 else "bird3_round"
        fold_name = sys.argv[3] if len(sys.argv) > 3 else "folded"
        fold = BIRD_SUPINE_FOLD if fold_name == "folded" else BIRD_SUPINE_STRAIGHT
        random.seed(0); np.random.seed(0)
        p, xml = plant(name)

        def q_sym(yaw, roll, knee, ankle):
            return _bird_legs(L_roll=roll, R_roll=-roll, L_hip_yaw=yaw, R_hip_yaw=yaw, knee=knee, ankle=ankle)

        def eval_path(V):
            seq = [("lie", fold, 0.5, 1.0)] + [(f"k{j}", q_sym(*row), 1.4, 0.6) for j, row in enumerate(V)]
            res = G.run_sequence(p, xml, seq, start="bird_back", play_deg=3.0, per_joint=G.PJ_DEFAULT, verbose=False)
            return 3.0 * max(0, res["up"]) + 8.0 * res["pelvis_z"], res

        NK, bounds = 6, [(-180, 180), (0, 120), (-130, 95), (-45, 45)]
        best = (-1e9, None, None)
        for restart in range(6):
            x = np.array([[random.uniform(*b) for b in bounds] for _ in range(NK)])
            s_best, step = -1e9, [60.0, 30.0, 30.0, 15.0]
            for it in range(30):
                for _ in range(6):
                    y = x + np.array([[random.gauss(0, st) for st in step] for _ in range(NK)])
                    for j, b in enumerate(bounds):
                        y[:, j] = np.clip(y[:, j], *b)
                    s, res = eval_path(y)
                    if s > s_best:
                        s_best, x = s, y
                step = [st * 0.93 for st in step]
            s, res = eval_path(x)
            print(f"restart {restart}: score {s:.2f} up {res['up']:+.2f} z {res['pelvis_z']:.3f}", flush=True)
            if s > best[0]:
                best = (s, x.copy(), res)
        print(f"BEST score {best[0]:.2f} up {best[2]['up']:+.2f} z {best[2]['pelvis_z']:.3f} "
              f"STANDING={best[2]['ok']} path={best[1].tolist()}")
    elif mode == "falls":
        # round 3, task 4: fall census. 12 headings x 3 magnitudes (tip,
        # 1.5x, 2x -- found per body by binary search), classify the settled
        # end state. Same for the flat bird body and the stock v7 body.
        names = sys.argv[2:] or ["bird3_round", "stock"]
        print("== FALL CENSUS: 12 headings x 3 magnitudes (tip/1.5x/2x), 3 s settle, deploy model")
        dists = {}
        for name in names:
            dists[name] = fall_census(name)
        print()
        print("SIDE BY SIDE:")
        classes = sorted(set().union(*[set(d) for d in dists.values()]))
        for c in classes:
            print(f"  {c:10s} " + "  ".join(f"{name}={dists[name].get(c,0)}" for name in names))
    elif mode == "fallsmid":
        names = sys.argv[2:] or ["bird3_round", "stock"]
        print("== FALL CENSUS, walking mid-stride pose (cheap variant)")
        for name in names:
            fall_census_midstride(name)
    elif mode == "render":
        name, seqname, out = sys.argv[2], sys.argv[3], sys.argv[4]
        start = sys.argv[5] if len(sys.argv) > 5 else "supine"
        p, xml = plant(name)
        if seqname.startswith("tuned:"):
            kp, ap, tp, ho = (float(x) for x in seqname.split(":")[1].split(","))
            seq = seq_pushup_tuned(60.0, kp, ap, tp, ho)
        elif seqname.startswith("roll:"):
            side, swing, yaw = seqname.split(":")[1].split(",")
            seq = seq_roll_over(side, float(swing), float(yaw))
        else:
            seq = {"situp_wide": seq_situp_wide(60, -40), "pushup_wide": seq_pushup_wide(60, -40),
                   "side_l": full_from_side("l", 60, 60, 60, -40), "side_r": full_from_side("r", 60, 60, 60, -40)}[seqname]
        cam = (1.4, -14, 90) if start.startswith("side") else (1.3, -15, 135)
        cap = lambda lab, t: f"{name}  {seqname}  {start}   {lab}   t={t:4.1f}s"
        r = G.run_sequence(p, xml, seq, start=start, play_deg=3.0, per_joint=G.PJ_DEFAULT, verbose=True,
                           render=out, cam=cam, size=(540, 720), label_fn=cap)
        print(f"{name} {seqname} {start}: {'STANDING' if r['ok'] else 'not standing'} -> {out}")
        return
    print("standing:", hits)


if __name__ == "__main__":
    main()
