"""getup_v6_noappendage — creative open-loop fall recovery on the BARE v6 body
(no tail, no arms), exploiting the beyond-human joint ROM: hip yaw +-45,
hip roll -30/+45, hip pitch -125/+90, knee -130, ankle +-45, ankle roll +-25.

The appended study (getup_v6_appendage) proved prone->side->back is solvable
with the LEGS alone; only supine->standing needs an appendage. Here we attack
that remaining step with three no-appendage ideas:

  A. 'kip'  — from supine, whip the legs up and OVER the head (fast hip
              extension), using angular momentum to roll the body over its
              shoulders onto the feet in a deep crouch (gymnast backward
              somersault-to-stand, arms-free).
  B. 'heel' — bring one foot up and back beside/behind the hip (deep knee+hip
              flex, hip abduct + yaw), plant it flat on the floor behind the
              pelvis as a SELF-MADE third contact, press down to lift the
              pelvis (what the tail did), slide the other foot under, rise.
  C. 'tripod' - splay both legs wide to the ROM caps so the CoM sits inside a
              broad support triangle while rising from the deep crouch.

Sequences are lists of (label, {per-joint deg offsets}, move_s, hold_s).
Per-joint dicts key by role ('hip_pitch' -> both legs) or exact joint
('L_hip_yaw', ...). This harness is built directly on gen_plant_v6 + the
getup_v6 runner so it inherits the deploy servo model and adversarials.

    .venv/bin/python sim/getup_v6_noappendage.py --seq kip --render out.mp4
"""
from __future__ import annotations
import argparse, itertools, math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np  # noqa: E402
from gen_plant_v6 import DesignParams, params_from_args, build_xml  # noqa: E402
import getup_v6 as G  # noqa: E402

BASE = DesignParams()   # 130/125 ROM, NO tail, NO arms
SP = os.environ.get("TMPDIR", "/tmp")
OUT = os.path.join(HERE, "runs", "getup_noapp")
os.makedirs(OUT, exist_ok=True)

H, K = -125.0, -130.0     # deep ROM caps

# ------------------------------------------------------------------ sequences


def kip(throw_to, throw_t, swing="both", splay=0.0, catch=+20.0, rise=True):
    """Backward somersault to stand. From supine, legs throw up-and-over the
    head (hip EXTENSION, knee straightening) fast enough to carry the body
    over the shoulders; land in a deep crouch; rise.
      throw_to: hip pitch the legs whip to (positive = extension -> over head)
      throw_t : seconds to do the whip (shorter = more angular momentum)
      swing   : 'both' both legs together, else 'stagger' L then R
      splay   : hip roll splay applied during the throw (deg)
      catch   : hip pitch the crouch lands at (deep flex, negative)
    """
    ext = dict(hip_pitch=throw_to, knee=0, ankle=0, hip_roll=splay)
    tuck = dict(hip_pitch=H, knee=K, ankle=0)
    seq = [("lie", dict(), 0.5, 1.0)]
    if swing == "both":
        seq.append(("whip over head", dict(hip_pitch=throw_to, knee=0, ankle=0, hip_roll=splay),
                    throw_t, 0.2))
    else:  # stagger: whip L then R so the foot lands one at a time
        seq.append(("L over", dict(L_hip_pitch=throw_to, L_knee=0, R_hip_pitch=30, R_knee=-20, hip_roll=splay),
                    throw_t, 0.1))
        seq.append(("R over", dict(R_hip_pitch=throw_to, R_knee=0, hip_roll=splay),
                    throw_t, 0.1))
    seq.append(("catch crouch", dict(hip_pitch=catch, knee=-60, ankle=0), 0.5, 0.8))
    if rise:
        seq += [("deep", dict(hip_pitch=H, knee=K, ankle=0), 0.8, 0.6),
                ("rise 2", dict(hip_pitch=-80, knee=-70, ankle=-30), 1.2, 0.8),
                ("rise 3", dict(hip_pitch=-45, knee=-50, ankle=-25), 1.2, 0.8),
                ("stand", dict(hip_pitch=-20, knee=-40, ankle=-20), 1.2, 1.0),
                ("straight", dict(), 1.0, 0.8)]
    return seq


def heel(plant_side="R", hip_plant=-100, knee_plant=-120, yaw_plant=30, roll_plant=30,
         push=-40, slide=False, ankle0=-40):
    """Self-push: from supine sit-up, bring ONE foot up-and-back beside the hip
    (the heel-to-buttock plant), press DOWN with it to lift the pelvis (the
    tail's job, self-made), then bring the other foot under and rise.
      plant_side: the leg that becomes the third contact
      hip/knee_plant: the planted leg's deep-flex pose
      yaw_plant/roll_plant: how far out/rotated the planted foot is
      push: hip pitch when pressing down (extends the planted leg)
      slide: also tuck the free foot before pressing (feet both under)
    """
    side, oth = plant_side, ("L" if plant_side == "R" else "R")
    p = lambda: {f"{side}_hip_pitch": hip_plant, f"{side}_knee": knee_plant,
                 f"{side}_hip_yaw": yaw_plant, f"{side}_hip_roll": roll_plant,
                 f"{side}_ankle": 0, f"{side}_ankle_roll": 0}
    seq = [("lie", dict(), 0.5, 1.0),
           ("sit up", dict(hip_pitch=-90), 1.5, 0.8)]
    if slide:
        seq += [("both feet under", dict(hip_pitch=H, knee=K, ankle=ankle0),
                 1.5, 0.8)]
    seq += [("plant heel beside hip", p(), 1.5, 1.0),
            ("press pelvis up", {**p(), f"{side}_hip_pitch": push,
                                 f"{oth}_hip_pitch": H, f"{oth}_knee": K,
                                 f"{oth}_ankle": ankle0}, 2.0, 1.2),
            ("both feet under", dict(hip_pitch=H, knee=K, ankle=ankle0), 1.0, 0.8),
            ("rise 2", dict(hip_pitch=-80, knee=-70, ankle=-30), 1.2, 0.8),
            ("rise 3", dict(hip_pitch=-45, knee=-50, ankle=-25), 1.2, 0.8),
            ("stand", dict(hip_pitch=-20, knee=-40, ankle=-20), 1.2, 1.0),
            ("straight", dict(), 1.0, 0.8)]
    return seq


def tripod(hip_flex=-80, knee_flex=-90, splay=45, rise_ext=0.0):
    """Wide-tripod rise: splay both legs to the abduction cap into a broad A,
    crouch deep, shift the CoM between the splayed feet, rise.
    NOTE: splay must be OPPOSITE sign per leg -- on the L leg +hip_roll =
    abduction, on the R leg +hip_roll = ADDUCTION (gen_plant_v6). To splay
    BOTH outward: L_hip_roll=+splay, R_hip_roll=-splay."""
    base = dict(hip_pitch=hip_flex, knee=knee_flex, L_hip_roll=splay, R_hip_roll=-splay, ankle=-20)
    seq = [("lie", dict(), 0.5, 1.0),
           ("sit up", dict(hip_pitch=-90, L_hip_roll=splay, R_hip_roll=-splay), 1.5, 0.8),
           ("splay crouch", base, 2.0, 1.2),
           ("rise", dict(hip_pitch=rise_ext, knee=-40, L_hip_roll=splay, R_hip_roll=-splay, ankle=-20), 2.0, 1.0),
           ("close legs", dict(hip_pitch=rise_ext, knee=-40, ankle=-20), 1.5, 0.8),
           ("stand", dict(hip_pitch=-20, knee=-40, ankle=-20), 1.5, 1.0),
           ("straight", dict(), 1.0, 0.8)]
    return seq


def kneel_rise(hip_sit=-90, hip_kneel=-110, knee_kneel=-130, ankle_toes=-40,
               hip_squat=-125, knee_squat=-130, hip_stand=-20, knee_stand=-40,
               plant_toe=True):
    """HIGH-SEAT kneel path the deep ROM enables (v5 couldn't): from the
    seated rise, push up onto the toes in a deep crouch, then drive the hips
    up over the feet into a kneel / squat, then stand. The idea: kneeling
    raises the hip 4 -> ~20 cm, so the torso CoM no longer has to cross from
    behind-heels to over-feet at the hard low seat -- it crosses while high.
        hip_sit   : hip fold that reaches the sit (seated, legs forward)
        hip_kneel / knee_kneel : deep crouch on the toes (plant), shin vertical
        hip_squat : the deep squat (feet flat) that bridges to stand
    """
    tuck = dict(hip_pitch=hip_sit, knee=0, ankle=0)               # sit
    deep = dict(hip_pitch=hip_kneel, knee=knee_kneel,
                ankle=(ankle_toes if plant_toe else 0))            # toes under
    seq = [("lie", dict(), 0.5, 1.0),
           ("sit up", tuck, 1.5, 0.8),
           ("toes under", deep, 2.0, 1.2),
           ("hip drive / squat", dict(hip_pitch=hip_squat, knee=knee_squat,
                                      ankle=(ankle_toes if plant_toe else 0)), 2.0, 1.0),
           ("rise", dict(hip_pitch=hip_stand, knee=knee_stand, ankle=-20), 1.5, 1.0),
           ("stands", dict(hip_pitch=-20, knee=-40, ankle=-20), 1.2, 1.0),
           ("straight", dict(), 1.0, 0.8)]
    return seq


# ------------------------------------------------------------------ run


def run(label, p, xml, seq, start="supine", play=3.0, mu=0.7, servo=1.0, render=None, cam=(1.3, -15, 135)):
    r = G.run_sequence(p, xml, seq, start=start, play_deg=play, per_joint=G.PJ_DEFAULT,
                       mu=mu, servo_scale=servo, render=render, cam=cam,
                       size=(540, 720), label_fn=lambda lab, t: f"{label}  {lab}  t={t:4.1f}s")
    worst = max(r["log"], key=lambda L: L[4])
    fr = " | ".join(f"{L[0][:8]}:{L[1]:+.2f}/{L[2]:.2f}/+{'+'.join(c.replace('_','')[:4] for c in L[3])}"
                    for L in r["log"][1:])
    print(f"{label:56s} {'STANDING' if r['ok'] else 'no'} up {r['up']:+.2f} front {r['front']:+.2f} "
          f"z {r['pelvis_z']:.3f} |tau|max {worst[4]:.2f}({worst[5]})", flush=True)
    print(f"   {fr}")
    return r["ok"], r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seq", default="all")
    ap.add_argument("--render", default=None)
    ap.add_argument("--mu", type=float, default=0.7)
    ap.add_argument("--play", type=float, default=3.0)
    a, rest = ap.parse_known_args()
    p, _ = params_from_args(rest + ["-o", "/dev/null"])
    xml = os.path.join(SP, f"gu_noapp_{os.getpid()}.xml")
    open(xml, "w").write(build_xml(p))
    hits = []
    n_ok = 0; n = 0

    def go(name, seq, **kw):
        nonlocal n_ok, n
        n += 1
        ok, r = run(name, p, xml, seq, render=(a.render if a.seq in ("all",) or name.split(" ")[0] == a.seq else None), **kw)
        if ok: n_ok += 1; hits.append(name)

    if a.seq in ("all", "kip"):
        print("== KIP (backward somersault to stand): throw the legs up+over the head from supine")
        for throw_to, throw_t, swing in itertools.product((90, 110, 130), (0.3, 0.5, 0.8), ("both", "stagger")):
            go(f"kip to{throw_to} t{throw_t}s {swing}", kip(throw_to, throw_t, swing))
    if a.seq in ("all", "heel"):
        print("== HEEL self-push: one foot planted beside/behind the hip as a third contact")
        for side, hip_p, knee_p, yawp, rollp, psh in itertools.product(
                ("R", "L"), (-90, -100, -110), (-110, -120, -130), (20, 30, 45), (20, 30, 45), (-60, -80, -100)):
            go(f"heel {side} hip{hip_p} k{knee_p} y{yawp} r{rollp} s{psh}",
               heel(side, hip_p, knee_p, yawp, rollp, psh))
    if a.seq in ("all", "tripod"):
        print("== TRIPOD wide-splay rise")
        for splay, hip_f, kf in itertools.product((30, 45, 45), (-60, -80, -100), (-70, -90, -110)):
            go(f"tripod s{splay} h{hip_f} k{kf}", tripod(hip_f, kf, splay))
    if a.seq in ("all", "kneel"):
        print("== KNEEl high-seat rise (deep ROM)")
        for hip_sit, hip_kneel, kk, toe in itertools.product((-90, -100), (-110, -125), (-120, -130), (-40, -25)):
            go(f"kneel sit{hip_sit} hk{hip_kneel} kk{kk} toe{toe}", kneel_rise(hip_sit, hip_kneel, kk, toe))
    print(f"\n{n_ok}/{n} standing; hits={hits}")


if __name__ == "__main__":
    main()
