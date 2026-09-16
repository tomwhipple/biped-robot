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
            for label, kw in (("turn  +0 mu 0.7 play 3", dict()), ("turn +15 mu 0.7 play 3", dict(turn_deg=15.0)),
                              ("turn  +0 mu 0.3 play 5", dict(mu=0.3, play_deg=5.0)), ("turn -15 mu 0.9 play 3", dict(turn_deg=-15.0, mu=0.9))):
                turn = kw.pop("turn_deg", 0.0)
                try:
                    tl2, windows2 = SG.walk_timeline(p, n_steps=8, step=0.06, lift_h=0.04, turn_deg=turn)
                    r = SG.run_walk(p, xml, tl2, windows2, per_joint=per_joint, **kw)
                    print(SG.fmt_row(f"{name:24s} {label}", r))
                except ValueError as e:
                    # a wide hip_sep can ask the gait's foot-placement offsets
                    # (bias_y/swing_out/turn) for more leg reach than thigh+
                    # shank give (0.220 m) -- a real kinematic limit of the
                    # EXISTING gait planner (tuned for hip_sep 0.084), not a
                    # bug in the hip_z fix: measured and reported, not guessed.
                    print(f"{name:24s} {label}  IK REACH EXCEEDED: {e}")
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
