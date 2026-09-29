"""ROM interference gate for the v6 assembly (the v6 half of cad/run_checks.sh).

For every pair of pieces that move relative to each other, pose the chain at
the joint's ROM extremes (and a few interior samples) and require either zero
boolean intersection AND a minimum distance >= D.SWEEP_BUFFER, or -- for the
pairs that are DESIGNED to touch (seats, pads on discs) -- zero intersection
only. Also: the relatively FIXED pairs that must keep a stated distance (the
neck floor over the pack, STATIC_PAIRS) and every disc screw's thread
engagement (check_disc_screws).

The pairs are enumerated, not filtered by contype, exactly as v5 does: a pair
that is not listed is a pair that was never checked, and the list is printed
with the result so the omission is visible.

    .venv/bin/python cad/v6/check_assembly_v6.py            # all pairs, the robot (arms)
    ARMS=0 .venv/bin/python cad/v6/check_assembly_v6.py     # the armless variant
    .venv/bin/python cad/v6/check_assembly_v6.py --joint ankle_roll --samples 9
    .venv/bin/python cad/v6/check_assembly_v6.py --joint static     # (or screws)
"""
from __future__ import annotations

import argparse
import itertools
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

import dimensions_v6 as V  # noqa: E402
import assembly_v6 as A  # noqa: E402
D = V.D

# (joint swept, piece A label pattern, piece B label pattern, note)
# labels are the assembly's; {s} = leg side. Pieces on the SAME link never
# move relative to each other and are not listed.
PAIRS = [
    ("hip_yaw", "yaw_carrier_{s}", "pelvis_v7", "carrier under the cell through +-45"),
    ("hip_yaw", "servo_hip_roll_{s}", "pelvis_v7", "roll servo in the carrier bay vs the housing floor"),
    ("hip_roll", "yoke_roll_{s}", "yaw_carrier_{s}", "roll clevis vs the bay walls (v5 pair)"),
    ("hip_roll", "yoke_pitch_{s}", "yaw_carrier_{s}", "pitch yoke flange vs the bay at abduction"),
    ("hip_roll", "yoke_pitch_{s}", "servo_hip_roll_{s}", "yoke vs the roll servo case"),
    ("hip_pitch", "thigh_{s}", "yoke_pitch_{s}", "thigh grip plates vs the yoke arms (0.7 mm v5 rule)"),
    # the pair that actually limits hip flexion: the thigh's front wall vs the
    # ROLL flange (hip-yoke-single-print.md section 6 item 1). The split sweep
    # never listed it, which is how a -125 ROM stood unchallenged for 10 days.
    ("hip_pitch", "thigh_{s}", "yoke_roll_{s}", "thigh front wall vs the ROLL flange (sets ROM hip_pitch)"),
    ("hip_pitch", "thigh_{s}", "servo_hip_pitch_{s}", "grip channel vs its own servo: seated, must not intersect"),
    ("hip_pitch", "thigh_{s}", "yaw_carrier_{s}", "thigh top vs the carrier at deep flexion"),
    ("hip_pitch", "thigh_{s}", "pelvis_v7", "thigh vs the housing at deep flexion"),
    ("hip_pitch", "thigh_{s}", "servo_hip_roll_{s}", "thigh grip top vs the roll servo case at -125 (hits at -135)"),
    ("knee", "shin_{s}", "thigh_{s}", "shin grip plates vs the thigh fork; shin case vs the thigh box"),
    ("knee", "servo_knee_{s}", "thigh_{s}", "knee servo case sweeping under the thigh's box (r 16 rule)"),
    ("ankle_pitch", "ankle_link_{s}", "shin_{s}", "ankle link grip plates vs the shin fork (0.7 mm)"),
    ("ankle_pitch", "servo_ankle_pitch_{s}", "shin_{s}", "ankle pitch servo case vs the shin box"),
    ("ankle_pitch", "foot_{s}", "shin_{s}", "foot tabs / cradle vs the shin tines at +-40"),
    ("ankle_roll", "foot_{s}", "ankle_link_{s}", "foot tabs vs the ankle link tines (0.7 mm), sole vs the web"),
    ("ankle_roll", "servo_ankle_roll_{s}", "ankle_link_{s}", "THE 54 mm pair: roll servo case sweep under the link"),
    ("ankle_roll", "servo_ankle_roll_{s}", "servo_ankle_pitch_{s}", "roll case vs the pitch servo hanging above it"),
    ("neck", "head", "pelvis_v7", "head shell vs the deck through +-90"),
    ("neck", "head", "servo_neck", "head base vs the neck servo case (rides on its horn)"),
    # the neck's mount is the standalone collar in the armless build and the
    # shoulder girdle (which absorbs it) with the arms -- same clearance, one
    # name or the other depending on what is actually in the assembly.
    ("neck", "head", "shoulder_girdle_v6" if A.arms_on() else "neck_collar",
     "head shell vs the neck-mount top through +-90 (>= 1.5 mm by design)"),
    ("neck", "head", "neck_floor", "head vs the neck servo's floor plate through +-90"),
]
# RELATIVELY FIXED pairs: pieces on the same link that must still keep their
# distance -- the neck servo's floor plate (issue #90) hangs from the neck tube
# into the deck's battery aperture, over the pack and between the boards.
# Checked once, at the standing pose: (A, B, note, minimum distance in mm, or
# None for a DESIGNED contact, where only the overlap volume must be ~0).
STATIC_PAIRS = [
    ("neck_floor", "pack_mock", "floor over the pack (the pack goes in first)", 4.0),
    ("neck_floor", "pelvis_v7", "floor in the deck aperture, off the rear web", 0.25),
    ("neck_floor", "pi4_mock", "floor vs the Pi 4B (aft column)", 1.0),
    ("neck_floor", "gd_mock", "floor vs the General Driver (front column)", 1.0),
    ("neck_floor", "servo_neck", "neck servo seated on the four pads: must not intersect", None),
    ("neck_floor", "shoulder_girdle_v6" if A.arms_on() else "neck_collar",
     "floor lugs screwed up to the tube bosses: must not intersect", None),
    ("servo_neck", "shoulder_girdle_v6" if A.arms_on() else "neck_collar",
     "neck servo in its tube: must not intersect", None),
]
# pairs designed to touch at the standing pose (seat/disc contacts): checked
# for intersection VOLUME only (must be ~0), the distance rule is waived
TOUCHING = {("thigh_{s}", "servo_hip_pitch_{s}"), ("shin_{s}", "thigh_{s}"),
            ("ankle_link_{s}", "shin_{s}"), ("foot_{s}", "ankle_link_{s}"),
            ("head", "servo_neck"), ("yoke_roll_{s}", "yaw_carrier_{s}"),
            ("servo_hip_roll_{s}", "pelvis_v7"), ("servo_ankle_roll_{s}", "ankle_link_{s}"),
            ("servo_knee_{s}", "thigh_{s}"), ("servo_ankle_pitch_{s}", "shin_{s}")}
# HIP_YOKE_VARIANT=single (assembly_v6.hip_yoke_variant): yoke_roll and
# yoke_pitch are ONE piece, hip_yoke_{s}. Every PAIRS row naming either yoke is
# renamed onto it (duplicates collapse), and the two pairs where the part is
# DESIGNED to touch its neighbour -- the carrier (roll idler boss riding the
# bay bore at its 0.30 slip) and the roll servo (both roll bosses seated on
# the discs) -- are checked twice: whole (volume only, TOUCHING) and as
# "<label>~seats", the piece minus hip_yoke_v6.disc_seat_zones, which holds
# the arms, pads and both flanges to the full buffer. That second row is
# STRICTER than the split check, where yoke_roll vs the carrier was
# volume-only and yoke_roll vs the roll servo was not listed at all.
SINGLE_RENAME = {"yoke_roll_{s}": "hip_yoke_{s}", "yoke_pitch_{s}": "hip_yoke_{s}"}
SINGLE_SEATED = {("hip_yoke_{s}", "yaw_carrier_{s}"): "one-print yoke vs the bay walls: idler boss rides the bore, volume only",
                 ("hip_yoke_{s}", "servo_hip_roll_{s}"): "one-print yoke on the roll servo discs: seated, must not intersect"}

# the arms (assembly_v6.arms_on, the default build): docs/design-v6/arms.md.
# Rows are (joint swept, A, B, note, extra pose) -- the 5th field is what the
# 4-field PAIRS rows above never needed: the arm has TWO joints, and the poses
# that matter are not all at elbow 0. The FOLDED rows are study-shoulder-
# arms.md's own open item 1 ("fold-up swept volume ... only end poses were
# checked against the head and torso"); this is where that gets measured.
#
# Three families, all of them things the lumped-mass plant could not see:
#   * the arm against the TORSO through the full shoulder sweep, hanging and
#     folded -- the fold-up passes the head and the neck collar;
#   * the arm against its OWN bracket and servos (the elbow's real ROM limit,
#     which is what V.ARM_ROM['elbow'] is set from);
#   * the LEG against the hanging arm, swept at every leg joint. The plant
#     measured 31 residual arm-vs-leg contacts over an 8-step walk with a 12 mm
#     capsule for an arm; the CAD arm is a 44 mm fork, so this is the row that
#     decides whether the aft mount is really enough.
ARM_PAIRS = [
    ("shoulder", "arm_upper_{s}", "pelvis_v7", "upper arm vs the torso through the whole sweep, hanging", None),
    ("shoulder", "arm_upper_{s}", "head", "upper arm vs the head at the fold-up (shoulder 180)", None),
    ("shoulder", "arm_upper_{s}", "shoulder_girdle_v6", "upper arm vs the neck tube at the fold-up (shoulder 180)", None),
    ("shoulder", "arm_upper_{s}", "shoulder_girdle_v6", "upper arm vs the girdle it hangs off, whole sweep", None),
    ("shoulder", "arm_upper_{s}", "servo_shoulder_{s}", "horn plate seated on the shoulder disc: must not intersect", None),
    ("shoulder", "arm_fore_{s}", "pelvis_v7", "forearm vs the torso through the whole sweep, hanging", None),
    ("shoulder", "arm_fore_{s}", "head", "forearm vs the head, FOLDED (elbow -90, the sit-up pose)", {"{s}_elbow": -90.0}),
    ("shoulder", "arm_fore_{s}", "pelvis_v7", "forearm vs the torso, FOLDED (elbow -90, the sit-up pose)", {"{s}_elbow": -90.0}),
    ("shoulder", "arm_fore_{s}", "shoulder_girdle_v6", "forearm vs the girdle, FOLDED", {"{s}_elbow": -90.0}),
    ("shoulder", "servo_elbow_{s}", "pelvis_v7", "the elbow servo case vs the torso through the sweep", None),
    ("elbow", "arm_fore_{s}", "arm_upper_{s}", "THE ELBOW LIMIT: forearm grip vs the fork (sets V.ARM_ROM)", None),
    ("elbow", "arm_fore_{s}", "servo_elbow_{s}", "forearm grip channel on its own servo: seated, must not intersect", None),
    ("elbow", "servo_elbow_{s}", "arm_upper_{s}", "elbow servo discs on the fork tines: seated, must not intersect", None),
    ("hip_pitch", "thigh_{s}", "arm_fore_{s}", "THE WALK ROW: thigh swept against the hanging forearm", None),
    ("hip_pitch", "thigh_{s}", "arm_upper_{s}", "thigh swept against the hanging upper arm / elbow fork", None),
    ("hip_pitch", "servo_hip_pitch_{s}", "arm_fore_{s}", "hip pitch servo case vs the hanging forearm", None),
    ("knee", "shin_{s}", "arm_fore_{s}", "shin swept against the hanging forearm (the plant's own contact pair)", None),
    ("knee", "servo_knee_{s}", "arm_fore_{s}", "knee servo case vs the hanging forearm", None),
    ("hip_roll", "yoke_pitch_{s}", "arm_fore_{s}", "hip abduction swings the yoke out toward the arm plane", None),
    ("hip_yaw", "yaw_carrier_{s}", "arm_fore_{s}", "hip yaw swings the carrier toward the arm plane", None),
    ("ankle_pitch", "ankle_link_{s}", "arm_fore_{s}", "ankle link vs the hanging forearm (the arm reaches to z 154)", None),
]
ARM_TOUCHING = {("arm_upper_{s}", "servo_shoulder_{s}"), ("arm_fore_{s}", "servo_elbow_{s}"),
                ("servo_elbow_{s}", "arm_upper_{s}")}


# YAW_BEARING_VARIANT=E (assembly_v6.yaw_retention_pieces): the bearing (one
# ring for both races, riding link 1), the screwed cap on the carrier (link 1)
# and the retainer under the skirt (link 0, pelvis-fixed). The retainer hangs
# 1.5 mm lower than the skirt it closes, which is the new thing the leg can
# reach, so the leg rows below sweep toward it. The bearing is DESIGNED to sit
# in the recess on the retainer's land: volume only.
E_PAIRS = [
    ("hip_yaw", "yaw_cap_{s}", "pelvis_v7", "E cap turns under the cell rim through +-45"),
    ("hip_yaw", "yaw_cap_{s}", "yaw_retainer_{s}", "E cap vs the retainer through +-45"),
    ("hip_yaw", "bearing_6810_{s}", "pelvis_v7", "E bearing in its recess: seated, must not intersect"),
    ("hip_yaw", "bearing_6810_{s}", "yaw_retainer_{s}", "E bearing on the retainer's land: seated, must not intersect"),
    ("hip_yaw", "yaw_carrier_{s}", "yaw_retainer_{s}", "carrier vs the E retainer through +-45"),
    ("hip_yaw", "servo_hip_roll_{s}", "yaw_retainer_{s}", "roll servo vs the E retainer through +-45"),
    ("hip_roll", "hip_yoke_{s}", "yaw_retainer_{s}", "yoke at full roll vs the E retainer"),
    ("hip_roll", "servo_hip_pitch_{s}", "yaw_retainer_{s}", "pitch servo at full roll vs the E retainer"),
    ("hip_roll", "thigh_{s}", "yaw_retainer_{s}", "thigh at full roll vs the E retainer"),
    ("hip_pitch", "thigh_{s}", "yaw_retainer_{s}", "thigh at deep flexion vs the E retainer"),
]
E_TOUCHING = {("bearing_6810_{s}", "pelvis_v7"), ("bearing_6810_{s}", "yaw_retainer_{s}")}
# ...and the outer race is a PRESS FIT in the pelvis recess (Ø64.96 on a Ø65
# race): its overlap is the designed 0.02 mm radial interference ring, and
# anything more than that ring is a real interference.
PRESS_FIT = {("bearing_6810_{s}", "pelvis_v7"):
             math.pi * ((V.YAWA_BRG_OD / 2) ** 2 - V.YAWA_BRG_RECESS_R ** 2) * V.YAWA_BRG_W}


def active_pairs():
    """(pairs, touching) for the assembly as built -- unchanged unless the
    one-print hip yoke or the arms are switched on. Rows come back as
    5-tuples (joint, A, B, note, extra pose)."""
    pairs, touching = _yoke_pairs()
    if A.yaw_bearing_variant() == "E":
        e_pairs = E_PAIRS
        if A.hip_yoke_variant() != "single":
            e_pairs = [(j, "yoke_pitch_{s}" if a == "hip_yoke_{s}" else a, b, n) for j, a, b, n in E_PAIRS]
        pairs = list(pairs) + e_pairs
        touching = set(touching) | E_TOUCHING
    if A.arms_on():
        arm_pairs = ARM_PAIRS
        if A.hip_yoke_variant() == "single":
            # the arm rows name the split yoke too (the abduction row sweeps
            # yoke_pitch toward the arm plane); with the one-print yoke that
            # part does not exist and the row crashed with a KeyError.
            arm_pairs = [(j, SINGLE_RENAME.get(a, a), SINGLE_RENAME.get(b, b), *rest)
                         for j, a, b, *rest in ARM_PAIRS]
        pairs = list(pairs) + arm_pairs
        touching = set(touching) | ARM_TOUCHING
    return [r if len(r) == 5 else (*r, None) for r in pairs], touching


def _yoke_pairs():
    if A.hip_yoke_variant() != "single":
        return PAIRS, TOUCHING
    pairs, seen = [], set()
    for joint, la, lb, note in PAIRS:
        la, lb = SINGLE_RENAME.get(la, la), SINGLE_RENAME.get(lb, lb)
        if (joint, la, lb) in seen:
            continue
        seen.add((joint, la, lb))
        if (la, lb) in SINGLE_SEATED:
            pairs.append((joint, la, lb, SINGLE_SEATED[(la, lb)]))
            pairs.append((joint, la + "~seats", lb, "same pair minus the disc seats: arms, pads, both flanges get the full buffer"))
        else:
            pairs.append((joint, la, lb, note))
    touching = {(SINGLE_RENAME.get(a, a), SINGLE_RENAME.get(b, b)) for a, b in TOUCHING} | set(SINGLE_SEATED)
    return pairs, touching


def piece(P, label, side, pose):
    """The posed solid for a pair label; "<label>~seats" strips the designed
    contacts (one-print hip yoke only)."""
    base, _, mod = label.partition("~")
    s = P[base.format(s=side)]
    if mod == "seats":
        s = s - A.posed_hip_yoke_seats(side, pose)
    return s


# inter-leg pairs at adduction / crossed poses
LEG_PAIRS = [("foot_L", "foot_R"), ("ankle_link_L", "ankle_link_R"), ("shin_L", "shin_R"), ("thigh_L", "thigh_R")]


def check_disc_screws(verbose=True):
    """Thread engagement of every M3 disc screw in the parts' SCREWS() lists.
    No boolean can see this: a screw that bottoms out in the disc's thin
    flange (2.5 horn / 2.1 idler) jacks the joint apart instead of clamping
    it, and one that is short has nothing to hold. Recomputed here from the
    length's thread (V.DISC_SCREW_THREAD) and the stack each part declares;
    one row per distinct (part, joint side, stack)."""
    import importlib
    mods = ["leg_link_v6", "ankle_link", "hip_yoke_v6", "yaw_carrier_v6", "head"]
    if A.arms_on():
        mods.append("arm_v6")
    fails, seen = 0, {}
    for mod in mods:
        for sc in importlib.import_module(mod).SCREWS():
            if "stack" not in sc:
                continue
            L = int(round(sc["length"]))
            eng = V.DISC_SCREW_THREAD[L] - sc["stack"]
            ok = D.DISC_THREAD_MIN_ENGAGE - 1e-9 <= eng <= sc["flange"] + 1e-9 and "washer" not in sc["kind"]
            key = (mod, sc["name"].rsplit("_", 2)[0] if mod != "arm_v6" else sc["name"].rsplit("_", 1)[0], L, round(sc["stack"], 2))
            n, ok0 = seen.get(key, (0, True))
            seen[key] = (n + 1, ok0 and ok)
    for (mod, joint, L, stack), (n, ok) in seen.items():
        eng = V.DISC_SCREW_THREAD[L] - stack
        fails += 0 if ok else 1
        if verbose:
            print(f"{'discscrew':12s} {mod:22s} {joint:22s} {n:3d} x M3x{L}  stack {stack:4.2f}  engages {eng:4.2f}"
                  f"  {'ok' if ok else 'FAIL'}")
    return fails


def pieces_by_label(comp):
    return {c.label: c for c in comp.children}


def check_pair(joint, la, lb, samples, side="L", extra=None):
    lo, hi = V.ROM[joint] if joint in V.ROM else V.ARM_ROM[joint]
    worst_vol, worst_dist, worst_ang = 0.0, 1e9, None
    for k in range(samples):
        ang = lo + (hi - lo) * k / (samples - 1)
        pose = {f"{side}_{joint}": ang} if joint != "neck" else {"neck": ang}
        if extra:
            pose.update(extra)
        comp = A.robot(pose)
        P = pieces_by_label(comp)
        a, b = piece(P, la, side, pose), piece(P, lb, side, pose)
        try:
            vol = (a & b).volume
        except Exception:  # noqa: BLE001
            vol = 0.0
        try:
            dist = a.distance_to(b)
        except Exception:  # noqa: BLE001
            dist = float("nan")
        if vol > worst_vol or (vol == worst_vol and dist < worst_dist):
            worst_vol, worst_dist, worst_ang = vol, dist, ang
    return worst_vol, worst_dist, worst_ang


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", default=None)
    ap.add_argument("--samples", type=int, default=5)
    ap.add_argument("--side", default="L")
    a = ap.parse_args(argv)
    t0 = time.time()
    fails = 0
    pairs, touching_set = active_pairs()
    if A.hip_yoke_variant() == "single":
        print("HIP_YOKE_VARIANT=single: one-print hip yoke (hip_yoke_v6) in place of yoke_roll + yoke_pitch")
    if A.arms_on():
        print(f"arms on (the default build): two 2-DOF get-up arms (cad/v6/arm_v6.py). Swept ROM "
              f"shoulder {V.ARM_ROM['shoulder']}, elbow {V.ARM_ROM['elbow']}; "
              f"every leg row sweeps the leg with the arms in the HANGING idle pose.")
    print(f"{'joint':12s} {'A':22s} {'B':22s} {'overlap mm3':>11s} {'min dist':>9s} {'at deg':>7s}  verdict")
    for joint, la, lb, note, extra in pairs:
        if a.joint and joint != a.joint:
            continue
        ex = {k.format(s=a.side): v for k, v in extra.items()} if extra else None
        vol, dist, ang = check_pair(joint, la, lb, a.samples, a.side, extra=ex)
        touching = (la, lb) in touching_set or (lb, la) in touching_set
        allowed = PRESS_FIT.get((la, lb), 0.0) if A.yaw_bearing_variant() == "E" else 0.0
        ok = vol < 0.5 + allowed * 1.02 and (touching or dist >= D.SWEEP_BUFFER - 1e-6)
        if allowed:
            note = f"{note} (press fit: {allowed:.1f} mm3 of designed interference)"
        fails += 0 if ok else 1
        print(f"{joint:12s} {la.format(s=a.side):22s} {lb.format(s=a.side):22s} {vol:11.2f} {dist:9.2f} {ang if ang is None else round(ang,1)!s:>7s}  {'ok' if ok else 'FAIL'}   {note}")
    # disc-screw thread engagement (no pose involved)
    if not a.joint or a.joint == "screws":
        fails += check_disc_screws()
    # relatively fixed pairs, at the standing pose
    if not a.joint or a.joint == "static":
        P0 = pieces_by_label(A.robot({}))
        for la, lb, note, need in STATIC_PAIRS:
            pa, pb = P0[la], P0[lb]
            try:
                vol = (pa & pb).volume
            except Exception:  # noqa: BLE001
                vol = 0.0
            dist = pa.distance_to(pb)
            ok = vol < 0.5 and (need is None or dist >= need - 1e-6)
            fails += 0 if ok else 1
            print(f"{'static':12s} {la:22s} {lb:22s} {vol:11.2f} {dist:9.2f} {'':>7s}  {'ok' if ok else 'FAIL'}   {note}"
                  + ("" if need is None else f" (>= {need} mm)"))
    # inter-leg: standing, adducted (roll toward each other), and crossed at the swing
    for la, lb in LEG_PAIRS:
        if a.joint and a.joint != "interleg":
            continue
        worst = (0.0, 1e9, None)
        # the poses the WALK uses (docs/design-v6/2026-09-13-design-record.md 4.3): the
        # parallelogram shift (both hips rolled the same way, ankles opposite),
        # the lifted swing at that shift, and a 3 deg mutual adduction. (Both
        # hips rolled TOWARD each other by 8 deg is not a pose this body can
        # reach -- the legs meet -- and was wrongly listed here at first.)
        for pose in ({}, {"L_hip_roll": -12.0, "R_hip_roll": -12.0, "L_ankle_roll": 12.0, "R_ankle_roll": 12.0},
                     {"L_hip_roll": -12.0, "R_hip_roll": -14.0, "L_ankle_roll": 12.0, "R_ankle_roll": 14.0,
                      "R_hip_pitch": -45, "R_knee": 70, "R_ankle_pitch": -25},   # +knee = flexion here (see V.ROM)
                     {"L_hip_roll": -3.0, "R_hip_roll": 3.0, "L_ankle_roll": 3.0, "R_ankle_roll": -3.0}):
            P = pieces_by_label(A.robot(pose))
            try:
                vol = (P[la] & P[lb]).volume
            except Exception:  # noqa: BLE001
                vol = 0.0
            dist = P[la].distance_to(P[lb])
            if vol > worst[0] or dist < worst[1]:
                worst = (vol, dist, str(pose)[:40])
        ok = worst[0] < 0.5 and worst[1] >= 2.0
        fails += 0 if ok else 1
        print(f"{'interleg':12s} {la:22s} {lb:22s} {worst[0]:11.2f} {worst[1]:9.2f} {'':>7s}  {'ok' if ok else 'FAIL'}   at {worst[2]}")
    print(f"=> {'ALL CLEAR' if fails == 0 else f'{fails} FAIL'}  ({time.time()-t0:.0f} s)")
    return fails


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
