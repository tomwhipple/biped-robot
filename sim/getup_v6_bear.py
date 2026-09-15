"""No-appendage get-up: the 'deep-flex bear-plant' family. Tom's constraint:
no tail, no arms, think beyond-human ROM. The appended study already proved
leg-roll prone->side and side->back work with the legs alone. What it never
tested is using the EXTENDED ROM to plant the FEET as hand-like third contacts
beside/behind the torso (a 'bear-plant' or 'crow-stand'), then rise.

ROM on v6: hip yaw +-45, hip roll -30/+45 (abd 45), hip pitch -125/+90, knee
-130, ankle +-45, ankle roll +-25. The knee + hip flex both to -130/-125 lets
the heels-to-buttock fold; the 45-deg hip roll + 45-deg hip yaw let a foot be
planted BESIDE the torso (laterally) so the shin is vertical -- a hand-like
third/fourth contact on the floor, from which the hips press up into a bear
stance and then a crouch and a stand.

This script sweeps two entry poses:
  'side'  : from lying on the side, plant the NEAR foot + FAR foot beside the
            hips, press into a bear/tripod, rise to stand.
  'prone' : from prone, deep-flex the feet beside the hips (heel-to-buttock,
            spread by hip roll), press the shoulders+feet into a bear, rise.

Sequences are joint-offset keyframes (deg), roles apply to both legs unless
L_/R_-prefixed. Success = standing (up_z > 0.9, pelvis >= 80% standing).

    .venv/bin/python sim/getup_v6_bear.py --seq side --render out.mp4
    .venv/bin/python sim/getup_v6_bear.py --seq prone
"""
import sys, os, itertools, argparse
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); os.environ.setdefault("MUJOCO_GL", "egl")
from gen_plant_v6 import DesignParams, params_from_args, build_xml
import getup_v6 as G

BASE = DesignParams()   # 130/125, no tail, no arms
OUT = os.path.join(HERE, "runs", "getup_noapp")
os.makedirs(OUT, exist_ok=True)


def side_bear(side="L", yaw_plant=45, roll_plant=45, hip_plant=-100, knee_plant=-120,
              ankle_plant=0, press=-50, tuck_hip=-120, tuck_knee=-125, toe=-20):
    """Lie on the side -> plant the (bottom) near foot flat beside the hips
    (heel-to-buttock with the hip rolled out) -> bring the other foot forward
    under the torso -> press with the near leg to lift the hips into a bear/
    tripod on feet + shoulder -> rise to a crouch then stand.
      side : the side lying on ('L' = lying on left, near leg = L... but in the
             sim supine/side we settle; we simply command 'R' as the presser)
    We'll drive BOTH feet to plant: the presser (near side) heel-planted beside
    the hip, the far leg tucked under, then press the hips up.
    """
    near, far = ("L", "R") if side == "L" else ("R", "L")
    def leg(s, hp, kp_, ya, ro, an, rl):
        return {f"{s}_hip_pitch": hp, f"{s}_knee": kp_, f"{s}_hip_yaw": ya,
                f"{s}_hip_roll": ro, f"{s}_ankle": an, f"{s}_ankle_roll": rl}
    p_near = leg(near, hip_plant, knee_plant, yaw_plant, roll_plant if near == "L" else -roll_plant, ankle_plant, 0)
    p_far  = leg(far, tuck_hip, tuck_knee, 0, roll_plant if far == "L" else -roll_plant, toe, 0)
    # first press the near foot flat beside the hip, far tucked under
    init = {**p_near, **p_far}
    bear = {**leg(near, press, -60, yaw_plant, roll_plant if near == "L" else -roll_plant, -20, 0),
            **leg(far, tuck_hip, -70, 0, roll_plant if far == "L" else -roll_plant, -20, 0)}
    c2 = dict(hip_pitch=-80, knee=-70, ankle=-30)
    s3 = dict(hip_pitch=-45, knee=-50, ankle=-25)
    stand = dict(hip_pitch=-20, knee=-40, ankle=-20)
    return [("lie side", dict(), 0.5, 0.8),
            ("plant feet", init, 1.8, 1.0),
            ("press hips up (bear)", bear, 2.0, 1.4),
            ("rise 2", c2, 1.5, 1.0),
            ("rise 3", s3, 1.5, 1.0),
            ("stand", stand, 1.5, 1.5),
            ("straight", dict(), 1.0, 0.8)]


def prone_bear(hip_flex=-120, knee_flex=-125, roll=45, yaw=45, ankle=0,
               toe_press=False, press_hip=-90, press_knee=-40):
    """From PRONE: deep-flex the feet beside the hips (heel-to-buttock, spread
    wide by hip roll + yaw so the shins plant vertically as hand-like struts),
    then press the SHOULDERS + feet to lift the pelvis into a bear/crow stand,
    then rise. This is the appendage-free analogue of the 'knees under' push-up
    but with the deep-ROM feet as the struts (the bird-knee/hip-arm studies
    piked on the head; here the feet are beside the hips, not behind them)."""
    plant = dict(hip_pitch=hip_flex, knee=knee_flex, L_hip_roll=roll, R_hip_roll=-roll,
                 L_hip_yaw=yaw, R_hip_yaw=-yaw, ankle=ankle)
    bear = dict(hip_pitch=press_hip, knee=press_knee, L_hip_roll=roll, R_hip_roll=-roll,
                L_hip_yaw=yaw, R_hip_yaw=-yaw, ankle=ankle)
    c2 = dict(hip_pitch=-80, knee=-70, ankle=-30)
    s3 = dict(hip_pitch=-45, knee=-50, ankle=-25)
    stand = dict(hip_pitch=-20, knee=-40, ankle=-20)
    return [("lie prone", dict(), 0.5, 0.8),
            ("feet beside hips (deep plant)", plant, 2.0, 1.2),
            ("press pelvis up (bear)", bear, 2.0, 1.4),
            ("rise 2", c2, 1.5, 1.0),
            ("rise 3", s3, 1.5, 1.0),
            ("stand", stand, 1.5, 1.5),
            ("straight", dict(), 1.0, 0.8)]


def run(label, p, xml, seq, start="supine", play=3.0, mu=0.7, servo=1.0, render=None,
        cam=(1.4, -14, 100)):
    r = G.run_sequence(p, xml, seq, start=start, play_deg=play, per_joint=G.PJ_DEFAULT,
                       mu=mu, servo_scale=servo, render=render, cam=cam,
                       size=(540, 720), label_fn=lambda lab, t: f"{label}  {lab}  t={t:4.1f}s")
    fr = " | ".join(f"{L[0][:10]}:{L[1]:+.2f}/{L[2]:.3f}/+{'+'.join(c.replace('_','')[:4] for c in L[3])}"
                    for L in r["log"][1:])
    print(f"{label:62s} {'STANDING' if r['ok'] else 'no'} up {r['up']:+.2f} front {r['front']:+.2f} "
          f"z {r['pelvis_z']:.3f} | {fr}", flush=True)
    return r["ok"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seq", default="side")
    ap.add_argument("--mu", type=float, default=0.7)
    ap.add_argument("--play", type=float, default=3.0)
    ap.add_argument("--render", default=None)
    a, rest = ap.parse_known_args()
    p, _ = params_from_args(rest + ["-o", "/dev/null"])
    xml = os.path.join(os.environ.get("TMPDIR", "/tmp"), f"gu_bear_{os.getpid()}.xml")
    open(xml, "w").write(build_xml(p))
    n_ok = 0; n = 0
    def go(name, seq, start, render=None):
        nonlocal n_ok, n
        n += 1
        ok = run(name, p, xml, seq, start=start, render=render)
        if ok: n_ok += 1
    if a.seq == "side":
        print("== SIDE bear-plant: lie on the side -> plant one/both feet beside the hips -> press up (bear) -> stand")
        for yawp, rollp, hp, kp, press in itertools.product((30, 45), (30, 45), (-90, -100, -110), (-110, -120, -130), (-40, -60, -80)):
            go(f"side y{yawp} r{rollp} h{hp} k{kp} pr{press}", side_bear("L", yawp, rollp, hp, kp, press=press), "supine")
    elif a.seq == "prone":
        print("== PRONE deep-flex bear-plant: feet beside hips -> press shoulders+feet -> rise")
        for hf, kf, roll, yaw, press_h, press_k in itertools.product((-110, -125), (-120, -130), (30, 45), (0, 30, 45), (-90, -110), (-60, -40)):
            go(f"prone h{hf} k{kf} r{roll} y{yaw} ph{press_h} pk{press_k}", prone_bear(hf, kf, roll, yaw, press_hip=press_h, press_knee=press_k), "prone")
    print(f"\n{n_ok}/{n} standing")


if __name__ == "__main__":
    main()
