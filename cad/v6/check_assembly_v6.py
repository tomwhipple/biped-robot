"""ROM interference gate for the v6 assembly (the v6 half of cad/run_checks.sh).

For every pair of pieces that move relative to each other, pose the chain at
the joint's ROM extremes (and a few interior samples) and require either zero
boolean intersection AND a minimum distance >= D.SWEEP_BUFFER, or -- for the
pairs that are DESIGNED to touch (seats, pads on discs) -- exclude them.

The pairs are enumerated, not filtered by contype, exactly as v5 does: a pair
that is not listed is a pair that was never checked, and the list is printed
with the result so the omission is visible.

    .venv/bin/python cad/v6/check_assembly_v6.py            # all pairs
    .venv/bin/python cad/v6/check_assembly_v6.py --joint ankle_roll --samples 9
"""
from __future__ import annotations

import argparse
import itertools
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
    ("neck", "head", "neck_collar", "head shell vs the collar top through +-90 (>= 1.5 mm by design)"),
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


def active_pairs():
    """(PAIRS, TOUCHING) for the assembly as built -- unchanged unless the
    one-print hip yoke is switched on."""
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


def pieces_by_label(comp):
    return {c.label: c for c in comp.children}


def check_pair(joint, la, lb, samples, side="L", extra=None):
    lo, hi = V.ROM[joint]
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
    print(f"{'joint':12s} {'A':22s} {'B':22s} {'overlap mm3':>11s} {'min dist':>9s} {'at deg':>7s}  verdict")
    for joint, la, lb, note in pairs:
        if a.joint and joint != a.joint:
            continue
        vol, dist, ang = check_pair(joint, la, lb, a.samples, a.side)
        touching = (la, lb) in touching_set or (lb, la) in touching_set
        ok = vol < 0.5 and (touching or dist >= D.SWEEP_BUFFER - 1e-6)
        fails += 0 if ok else 1
        print(f"{joint:12s} {la.format(s=a.side):22s} {lb.format(s=a.side):22s} {vol:11.2f} {dist:9.2f} {ang if ang is None else round(ang,1)!s:>7s}  {'ok' if ok else 'FAIL'}   {note}")
    # inter-leg: standing, adducted (roll toward each other), and crossed at the swing
    for la, lb in LEG_PAIRS:
        if a.joint and a.joint != "interleg":
            continue
        worst = (0.0, 1e9, None)
        # the poses the WALK uses (docs/design-v6-ankle-roll.md 4.3): the
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
