"""ROM interference gate for the v6 assembly (the v6 half of cad/run_checks.sh).

For every pair of pieces that move relative to each other, pose the chain at
the joint's ROM extremes (and a few interior samples) and require either zero
boolean intersection AND a minimum distance >= D.SWEEP_BUFFER, or -- for the
pairs that are DESIGNED to touch (seats, pads on discs) -- zero intersection
only. Also: the relatively FIXED pairs that must keep a stated distance (the
neck floor over the pack, STATIC_PAIRS), every disc screw's thread
engagement (check_disc_screws), the hip's bench-order INSERTION paths
(check_insertion) and a driver's ACCESS to every carrier and bearing-housing
screw in the state it is driven in (check_driver_access).

The pairs are enumerated, not filtered by contype, exactly as v5 does: a pair
that is not listed is a pair that was never checked, and the list is printed
with the result so the omission is visible.

    .venv/bin/python cad/v6/check_assembly_v6.py            # all pairs, the robot (arms)
    ARMS=0 .venv/bin/python cad/v6/check_assembly_v6.py     # the armless variant
    .venv/bin/python cad/v6/check_assembly_v6.py --joint ankle_roll --samples 9
    .venv/bin/python cad/v6/check_assembly_v6.py --joint static     # (or screws, insert, access)
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
    ("knee", "shin_{s}", "thigh_{s}", "shin grip plates vs the thigh fork; shin vs the thigh's end walls and pocket"),
    ("knee", "servo_knee_{s}", "thigh_{s}", "knee servo case sweeping under the thigh's bottom end wall (r 16 rule)"),
    ("ankle_pitch", "ankle_link_{s}", "shin_{s}", "ankle link grip plates vs the shin fork (0.7 mm)"),
    ("ankle_pitch", "servo_ankle_pitch_{s}", "shin_{s}", "ankle pitch servo case vs the shin's fork and bottom end wall"),
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
    ("bearing_inner_L", "yaw_carrier_L", "inner race pressed on the hub", None),
    ("bearing_outer_L", "yaw_bearing_housing", "outer race pressed into the housing's recess, under the shoulder", None),
    ("yaw_bearing_housing", "pelvis_v7", "housing screwed up against the cell block: must not intersect", None),
    ("servo_hip_yaw_L", "pelvis_v7", "yaw servo seated, idler case face on the cell ceiling (no idler disc): must not intersect", None),
    ("servo_hip_yaw_L", "pack_mock", "the yaw servo's free-hub post under the pack (on its 1 mm foam pads)", 0.8),
    ("yaw_bearing_housing", "pi4_mock", "housing under the Pi 4B's bottom edge", 0.5),
    ("yaw_bearing_housing", "gd_mock", "the General Driver's bottom edge on the housing's top face: must not intersect", None),
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
REST = {"{s}_shoulder": V.ARM_REST_SHOULDER, "{s}_elbow": V.ARM_REST_ELBOW}
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
    ("elbow", "arm_fore_{s}", "arm_upper_{s}", "THE ELBOW LIMIT: forearm fork vs the upper arm's grip channel (sets V.ARM_ROM)", None),
    ("elbow", "arm_fore_{s}", "servo_elbow_{s}", "forearm fork tines on the elbow servo's discs: seated, must not intersect", None),
    ("elbow", "servo_elbow_{s}", "arm_upper_{s}", "upper arm's grip channel on the elbow servo: seated, must not intersect", None),
    ("hip_pitch", "thigh_{s}", "arm_fore_{s}", "THE WALK ROW: thigh swept against the hanging forearm", None),
    ("hip_pitch", "thigh_{s}", "arm_upper_{s}", "thigh swept against the hanging upper arm / elbow grip", None),
    ("hip_pitch", "thigh_{s}", "servo_elbow_{s}", "thigh swept against the hanging elbow servo case", None),
    ("hip_pitch", "servo_hip_pitch_{s}", "arm_fore_{s}", "hip pitch servo case vs the hanging forearm", None),
    ("knee", "shin_{s}", "arm_fore_{s}", "shin swept against the hanging forearm (the plant's own contact pair)", None),
    ("knee", "servo_knee_{s}", "arm_fore_{s}", "knee servo case vs the hanging forearm", None),
    # The arms' REST pose (V.ARM_REST, folded: shoulder 15 deg forward, elbow
    # folded 95) is what they hold whenever the legs move, so every leg joint
    # is swept through its full range against the folded arm (DESIGN.md 5.4).
    # Hanging straight (the rows above) is only the torque-off pose.
    ("hip_yaw", "yoke_pitch_{s}", "arm_upper_{s}", "hip yaw: yoke vs the upper arm, arms at rest (folded)", REST),
    ("hip_yaw", "yoke_pitch_{s}", "servo_elbow_{s}", "hip yaw: yoke vs the elbow servo, arms at rest (folded)", REST),
    ("hip_yaw", "yoke_pitch_{s}", "arm_fore_{s}", "hip yaw: yoke vs the forearm, arms at rest (folded)", REST),
    ("hip_roll", "yoke_pitch_{s}", "arm_upper_{s}", "hip roll: yoke vs the upper arm, arms at rest (folded)", REST),
    ("hip_roll", "yoke_pitch_{s}", "servo_elbow_{s}", "hip roll: yoke vs the elbow servo, arms at rest (folded)", REST),
    ("hip_roll", "yoke_pitch_{s}", "arm_fore_{s}", "hip roll: yoke vs the forearm, arms at rest (folded)", REST),
    ("hip_roll", "servo_hip_pitch_{s}", "arm_upper_{s}", "hip roll: pitch servo vs the upper arm, arms at rest (folded)", REST),
    ("hip_roll", "servo_hip_pitch_{s}", "servo_elbow_{s}", "hip roll: pitch servo vs the elbow servo, arms at rest (folded)", REST),
    ("hip_roll", "servo_hip_pitch_{s}", "arm_fore_{s}", "hip roll: pitch servo vs the forearm, arms at rest (folded)", REST),
    ("hip_roll", "thigh_{s}", "arm_upper_{s}", "hip roll: thigh vs the upper arm, arms at rest (folded)", REST),
    ("hip_roll", "thigh_{s}", "servo_elbow_{s}", "hip roll: thigh vs the elbow servo, arms at rest (folded)", REST),
    ("hip_roll", "thigh_{s}", "arm_fore_{s}", "hip roll: thigh vs the forearm, arms at rest (folded)", REST),
    ("hip_pitch", "thigh_{s}", "arm_upper_{s}", "hip pitch: thigh vs the upper arm, arms at rest (folded)", REST),
    ("hip_pitch", "thigh_{s}", "servo_elbow_{s}", "hip pitch: thigh vs the elbow servo, arms at rest (folded)", REST),
    ("hip_pitch", "thigh_{s}", "arm_fore_{s}", "hip pitch: thigh vs the forearm, arms at rest (folded)", REST),
    ("hip_pitch", "servo_hip_pitch_{s}", "arm_fore_{s}", "hip pitch: pitch servo vs the forearm, arms at rest (folded)", REST),
    ("knee", "shin_{s}", "servo_elbow_{s}", "knee: shin vs the elbow servo, arms at rest (folded)", REST),
    ("knee", "shin_{s}", "arm_fore_{s}", "knee: shin vs the forearm, arms at rest (folded)", REST),
    ("knee", "servo_knee_{s}", "arm_fore_{s}", "knee: knee servo vs the forearm, arms at rest (folded)", REST),
    ("ankle_pitch", "ankle_link_{s}", "arm_fore_{s}", "ankle pitch: ankle link vs the forearm, arms at rest (folded)", REST),
    ("hip_yaw", "yaw_carrier_{s}", "arm_fore_{s}", "hip yaw swings the carrier toward the arm plane", None),
    ("ankle_pitch", "ankle_link_{s}", "arm_fore_{s}", "ankle link vs the hanging forearm (the arm reaches to z 154)", None),
]
ARM_TOUCHING = {("arm_upper_{s}", "servo_shoulder_{s}"), ("arm_fore_{s}", "servo_elbow_{s}"),
                ("servo_elbow_{s}", "arm_upper_{s}")}


# The hip-yaw bearing (assembly_v6.yaw_bearing_pieces), two races split at
# SKF's ring shoulders: the inner race turns with the carrier (link 1), the
# outer race sits in the bearing housing's recess (link 0, the housing is
# screwed under the pelvis). Through the yaw sweep the turning side must clear
# the fixed race and the housing -- this keeps the housing's shoulder off the
# inner race -- and the leg must clear the races' exposed bottom faces and the
# housing at full roll and flexion.
BEARING_PAIRS = [
    ("hip_yaw", "yaw_carrier_{s}", "bearing_outer_{s}", "turning carrier vs the fixed outer race through +-45"),
    ("hip_yaw", "servo_hip_roll_{s}", "bearing_outer_{s}", "roll servo (inside the hub) vs the fixed outer race"),
    ("hip_yaw", "bearing_inner_{s}", "yaw_bearing_housing", "turning inner race vs the housing's shoulder (SKF d1)"),
    ("hip_yaw", "bearing_inner_{s}", "pelvis_v7", "turning inner race vs the cell block's rim"),
    ("hip_yaw", "yaw_carrier_{s}", "yaw_bearing_housing", "turning carrier vs the housing through +-45"),
    ("hip_yaw", "servo_hip_roll_{s}", "yaw_bearing_housing", "roll servo vs the housing through +-45"),
    ("hip_roll", "hip_yoke_{s}", "bearing_inner_{s}", "yoke at full roll vs the inner race"),
    ("hip_roll", "hip_yoke_{s}", "bearing_outer_{s}", "yoke at full roll vs the outer race"),
    ("hip_roll", "hip_yoke_{s}", "yaw_bearing_housing", "yoke at full roll vs the housing"),
    ("hip_roll", "servo_hip_pitch_{s}", "bearing_outer_{s}", "pitch servo at full roll vs the outer race"),
    ("hip_roll", "servo_hip_pitch_{s}", "yaw_bearing_housing", "pitch servo at full roll vs the housing"),
    ("hip_roll", "thigh_{s}", "bearing_inner_{s}", "thigh at full roll vs the inner race"),
    ("hip_roll", "thigh_{s}", "bearing_outer_{s}", "thigh at full roll vs the outer race"),
    ("hip_roll", "thigh_{s}", "yaw_bearing_housing", "thigh at full roll vs the housing"),
    ("hip_pitch", "thigh_{s}", "bearing_inner_{s}", "thigh at deep flexion vs the inner race"),
    ("hip_pitch", "thigh_{s}", "bearing_outer_{s}", "thigh at deep flexion vs the outer race"),
    ("hip_pitch", "thigh_{s}", "yaw_bearing_housing", "thigh at deep flexion vs the housing"),
]
# the arms against the housing (the forearm folded is the sit-up pose)
ARM_HOUSING_PAIRS = [
    ("shoulder", "arm_upper_{s}", "yaw_bearing_housing", "upper arm vs the bearing housing, whole sweep", None),
    ("shoulder", "arm_fore_{s}", "yaw_bearing_housing", "forearm vs the bearing housing, hanging", None),
    ("shoulder", "arm_fore_{s}", "yaw_bearing_housing", "forearm vs the bearing housing, FOLDED", {"{s}_elbow": -90.0}),
]


def _designed_press_fits():
    """{(race label, seat label): designed overlap mm3} for the two press fits,
    the inner race on the carrier's hub and the outer race in the housing's
    recess, built here from the dimensions (not read off the parts, which is
    what the static rows measure): the interference ring over the race's
    width, less what the hub's and the recess's lead-ins take out of it."""
    import yaw_bearing_housing as H
    import yaw_carrier_v6 as YC
    import parts as Pv5
    from build123d import Cone, Pos
    y, zh = V.HIP_SEP / 2, V.HIP_YAW_Z
    z0, z1 = zh - V.YAW_BRG_W, zh
    race_in = Pv5.cyl_z(V.YAW_BRG_INNER_RING_R, z0, z1, 0, y) - Pv5.cyl_z(V.YAW_BRG_ID / 2, z0 - 1, z1 + 1, 0, y)
    hub = Pv5.cyl_z(V.YAW_BRG_BOSS_R, z0, z1, 0, y)
    hub -= Pos(0, y, z0 + YC.HUB_LEAD_IN / 2 - 0.05) * Cone(V.YAW_BRG_BOSS_R + 0.05, V.YAW_BRG_BOSS_R - YC.HUB_LEAD_IN,
                                                         YC.HUB_LEAD_IN + 0.1)
    zd = V.DECK_TOP_Z
    r0, r1 = V.YAW_BRG_RECESS_Z
    race_out = Pv5.cyl_z(V.YAW_BRG_OD / 2, r0 + zd, r1 + zd, 0, y) - Pv5.cyl_z(V.YAW_BRG_OUTER_RING_R, r0 + zd - 1, r1 + zd + 1, 0, y)
    seat_wall = Pv5.cyl_z(V.YAW_BRG_OD / 2 + 1, r0 + zd, r1 + zd, 0, y) - Pv5.cyl_z(V.YAW_BRG_RECESS_R, r0 + zd - 1, r1 + zd + 1, 0, y)
    hz0 = V.YAW_HOUSING_Z[0] + zd
    seat_wall -= Pos(0, y, hz0 + H.LEAD_IN / 2 - 0.05) * Cone(V.YAW_BRG_RECESS_R + H.LEAD_IN + 0.05, V.YAW_BRG_RECESS_R,
                                                           H.LEAD_IN + 0.1)
    return {("bearing_inner_L", "yaw_carrier_L"): (race_in & hub).volume,
            ("bearing_outer_L", "yaw_bearing_housing"): (race_out & seat_wall).volume}


def active_pairs():
    """(pairs, touching) for the assembly as built -- unchanged unless the
    one-print hip yoke or the arms are switched on. Rows come back as
    5-tuples (joint, A, B, note, extra pose)."""
    pairs, touching = _yoke_pairs()
    b_pairs = BEARING_PAIRS
    if A.hip_yoke_variant() != "single":
        b_pairs = [(j, "yoke_pitch_{s}" if a == "hip_yoke_{s}" else a, b, n) for j, a, b, n in BEARING_PAIRS]
    pairs = list(pairs) + b_pairs
    if A.arms_on():
        arm_pairs = ARM_PAIRS + ARM_HOUSING_PAIRS
        if A.hip_yoke_variant() == "single":
            # the arm rows name the split yoke too (the abduction row sweeps
            # yoke_pitch toward the arm plane); with the one-print yoke that
            # part does not exist and the row crashed with a KeyError.
            arm_pairs = [(j, SINGLE_RENAME.get(a, a), SINGLE_RENAME.get(b, b), *rest)
                         for j, a, b, *rest in arm_pairs]
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


# ---------------------------------------------------------------- the hip's bench order
# docs/assembly.md section 7: each part goes UP from below into the state the
# steps before it left. (step, moving pieces, pieces already in place). Left
# hip for the first three; the housing spans both hips, so it goes up with
# both bearings already pressed into it, under both built hips. A housing
# whose shoulder sits ABOVE the outer race cannot come up past a race that is
# already on the hub, so the bearings go into the housing on the bench first.
_HIP_L = ["servo_hip_yaw_L", "yaw_carrier_L", "servo_hip_roll_L", "bearing_inner_L", "bearing_outer_L"]
_HIP_R = [n[:-1] + "R" for n in _HIP_L]
_BEARINGS = ["bearing_inner_L", "bearing_outer_L", "bearing_inner_R", "bearing_outer_R"]
INSERT_STEPS = [
    ("7a yaw servo up into its cell", ["servo_hip_yaw_L"], ["pelvis_v7"]),
    ("7b carrier up onto the yaw horn", ["yaw_carrier_L"], ["pelvis_v7", "servo_hip_yaw_L"]),
    ("7c roll servo up into the bay", ["servo_hip_roll_L"], ["pelvis_v7", "servo_hip_yaw_L", "yaw_carrier_L"]),
    ("7d bearing into the housing (bench)", ["bearing_inner_L", "bearing_outer_L"], ["yaw_bearing_housing"]),
    ("7e housing + bearings up onto the hubs", ["yaw_bearing_housing"] + _BEARINGS,
     ["pelvis_v7"] + _HIP_L[:3] + _HIP_R[:3]),
]
INSERT_D = (0.0, 0.5, 1.0, 2.0, 3.0, 5.0, 8.0, 12.0, 20.0, 30.0, 45.0, 60.0)   # mm below the seat
INSERT_CLEARANCE_D = (1.0, 5.0, 8.0, 20.0)     # where the approach clearance is measured
# The only overlaps allowed along a path are the two press fits, up to their
# designed volume (they engage as the part goes home).


def check_insertion(verbose=True):
    """Sweep each bench step's moving part(s) up its path into place and
    require no overlap with what is already there, except the press fits."""
    from build123d import Pos
    fits = _designed_press_fits()
    fit_in, fit_out = fits[("bearing_inner_L", "yaw_carrier_L")], fits[("bearing_outer_L", "yaw_bearing_housing")]
    allowed = {("bearing_inner_L", "yaw_carrier_L"): fit_in, ("bearing_inner_R", "yaw_carrier_R"): fit_in,
               ("bearing_outer_L", "yaw_bearing_housing"): fit_out}
    P0 = pieces_by_label(A.robot({}))
    fails = 0
    for step, moving, static in INSERT_STEPS:
        bad, bad_at, clr = 0.0, None, (1e9, None, None)
        for d in INSERT_D:
            for m in moving:
                ms = Pos(0, 0, -d) * P0[m]
                mbb = ms.bounding_box()
                for st in static:
                    sbb = P0[st].bounding_box()
                    if (mbb.max.X < sbb.min.X - 25 or mbb.min.X > sbb.max.X + 25 or mbb.max.Y < sbb.min.Y - 25
                            or mbb.min.Y > sbb.max.Y + 25 or mbb.max.Z < sbb.min.Z - 25 or mbb.min.Z > sbb.max.Z + 25):
                        continue
                    try:
                        vol = (ms & P0[st]).volume
                    except Exception:  # noqa: BLE001
                        vol = 0.0
                    if (m, st) in allowed:
                        excess = vol - (allowed[(m, st)] * 1.02 + 0.5)
                    else:
                        excess = vol - 0.5
                    if excess > 0 and vol > bad:
                        bad, bad_at = vol, (st, d)
                    if d in INSERT_CLEARANCE_D and (m, st) not in allowed:
                        dist = ms.distance_to(P0[st])
                        if dist < clr[0]:
                            clr = (dist, st, d)
        ok = bad_at is None
        fails += 0 if ok else 1
        if verbose:
            where = f"FAIL: {bad:.1f} mm3 vs {bad_at[0]} at {bad_at[1]:g} mm" if not ok else "no overlap"
            print(f"{'insert':12s} {step:36s} {where:34s} approach clearance {clr[0]:5.2f} mm "
                  f"(vs {clr[1]}, {clr[2]:g} mm out)  {'ok' if ok else 'FAIL'}")
    if verbose:
        print(f"{'insert':12s} press fits allowed along the paths: inner race on the hub {fit_in:.1f} mm3, "
              f"outer race in the housing {fit_out:.1f} mm3 (designed)")
    return fails


# Driver access: a D.ACCESS_D cylinder from each screw head straight back out
# along its axis, in the state the screw is driven in (the bench order above).
ACCESS_STATES = [
    ("carrier horn screws, from inside the empty bay", "yaw_carrier_v6", "yaw_horn", "carrier",
     ["pelvis_v7", "servo_hip_yaw_L", "yaw_carrier_L"]),
    ("roll-servo wall screws, before the bearing", "yaw_carrier_v6", "roll_wall", "carrier",
     ["pelvis_v7", "servo_hip_yaw_L", "yaw_carrier_L", "servo_hip_roll_L"]),
    # the same screws with the hip fully assembled (bearing, housing, pelvis):
    # the roll servo can come out for service without taking the hip apart
    ("roll-servo wall screws, hip fully assembled", "yaw_carrier_v6", "roll_wall", "carrier",
     ["pelvis_v7", "servo_hip_yaw_L", "yaw_carrier_L", "servo_hip_roll_L",
      "yaw_bearing_housing", "bearing_inner_L", "bearing_outer_L"]),
    ("bearing-housing screws, before the legs", "yaw_bearing_housing", "housing", "pelvis",
     ["pelvis_v7", "yaw_bearing_housing"] + _HIP_L + _HIP_R),
]
ACCESS_REACH = 50.0


def check_driver_access(verbose=True):
    import importlib
    import parts as Pv5
    P0 = pieces_by_label(A.robot({}))
    fails = 0
    for group, mod, prefix, frame, present in ACCESS_STATES:
        worst = (0.0, None, None)
        n = 0
        for sc in importlib.import_module(mod).SCREWS():
            if not sc["name"].startswith(prefix):
                continue
            n += 1
            x, y, z = sc["pos"]
            if frame == "carrier":
                y, z = y + V.HIP_SEP / 2, z + V.HIP_YAW_Z
            else:
                z = z + V.DECK_TOP_Z
            ax = sc["axis"]
            r = D.ACCESS_D / 2
            if abs(ax[2]) > 0.5:
                drv = Pv5.cyl_z(r, z - ax[2] * 0.05, z - ax[2] * ACCESS_REACH, x, y)
            else:
                drv = Pv5.cyl_x(r, x - ax[0] * 0.05, x - ax[0] * ACCESS_REACH, y, z)
            for lab in present:
                try:
                    vol = (drv & P0[lab]).volume
                except Exception:  # noqa: BLE001
                    vol = 0.0
                if vol > worst[0]:
                    worst = (vol, sc["name"], lab)
        ok = worst[0] < 0.5
        fails += 0 if ok else 1
        if verbose:
            hit = f"worst {worst[0]:.1f} mm3 ({worst[1]} vs {worst[2]})" if worst[1] else "no part in the way"
            print(f"{'access':12s} {group:52s} {n:2d} screws, {D.ACCESS_D:g} mm driver: {hit}  {'ok' if ok else 'FAIL'}")
    return fails


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
              f"leg rows sweep the leg against the arms HANGING (torque off) and at REST "
              f"(folded: shoulder {V.ARM_REST_SHOULDER:g}, elbow {V.ARM_REST_ELBOW:g}).")
    print(f"{'joint':12s} {'A':22s} {'B':22s} {'overlap mm3':>11s} {'min dist':>9s} {'at deg':>7s}  verdict")
    for joint, la, lb, note, extra in pairs:
        if a.joint and joint != a.joint:
            continue
        ex = {k.format(s=a.side): v for k, v in extra.items()} if extra else None
        vol, dist, ang = check_pair(joint, la, lb, a.samples, a.side, extra=ex)
        touching = (la, lb) in touching_set or (lb, la) in touching_set
        ok = vol < 0.5 and (touching or dist >= D.SWEEP_BUFFER - 1e-6)
        fails += 0 if ok else 1
        print(f"{joint:12s} {la.format(s=a.side):22s} {lb.format(s=a.side):22s} {vol:11.2f} {dist:9.2f} {ang if ang is None else round(ang,1)!s:>7s}  {'ok' if ok else 'FAIL'}   {note}")
    # disc-screw thread engagement (no pose involved)
    if not a.joint or a.joint == "screws":
        fails += check_disc_screws()
    # relatively fixed pairs, at the standing pose
    if not a.joint or a.joint == "static":
        press_fit = _designed_press_fits()
        P0 = pieces_by_label(A.robot({}))
        for la, lb, note, need in STATIC_PAIRS:
            pa, pb = P0[la], P0[lb]
            try:
                vol = (pa & pb).volume
            except Exception:  # noqa: BLE001
                vol = 0.0
            dist = pa.distance_to(pb)
            fit = press_fit.get((la, lb))
            if fit is not None:
                ok = 0.98 * fit <= vol <= 1.02 * fit + 0.5
                note = f"{note} (designed {fit:.1f} mm3)"
            else:
                ok = vol < 0.5 and (need is None or dist >= need - 1e-6)
            fails += 0 if ok else 1
            print(f"{'static':12s} {la:22s} {lb:22s} {vol:11.2f} {dist:9.2f} {'':>7s}  {'ok' if ok else 'FAIL'}   {note}"
                  + ("" if need is None else f" (>= {need} mm)"))
    if not a.joint or a.joint == "insert":
        fails += check_insertion()
    if not a.joint or a.joint == "access":
        fails += check_driver_access()
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
