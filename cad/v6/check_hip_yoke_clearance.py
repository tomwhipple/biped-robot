"""Clearance A/B for the one-print hip yoke: fused() vs the styled part.

check_assembly_v6.py is the gate, and it stays the gate -- but it poses the
whole robot for every sample, so it takes ~a minute and reports one worst-case
row per pair. When the question is "did this change to hip_yoke_v6 move a
clearance, and by how much, at WHICH angle", that is the wrong instrument.

This does the same arithmetic in the yoke's OWN frame, in about four seconds,
and against the same mocks: the yoke is link 2, so its parent (link 1: the yaw
carrier and the roll servo) appears rotated by -hip_roll about X through the
origin, and its child (link 3: the pitch servo and the thigh) appears rotated
by +hip_pitch about Y through (0, 0, -ROLL_TO_PITCH). It reproduces
check_assembly_v6's numbers exactly -- 0.076 mm on the roll servo at +-55,
1.000 mm on the bay walls, 271.70 / 273.85 mm3 on the thigh at -125 -- which
is the point: it is the gate's measurement, not a second opinion.

    .venv/bin/python cad/v6/check_hip_yoke_clearance.py

Numbers reported in docs/design-v6/hip-yoke-single-print.md section 7.4.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

from build123d import Pos, Rot  # noqa: E402
import check_assembly as CA  # noqa: E402  (v5 servo mocks -- the gate's own)
import dimensions_v6 as V  # noqa: E402
import assembly_v6 as A  # noqa: E402
import hip_yoke_v6 as HY  # noqa: E402
D = V.D

ROLL_SAMPLES = (-55, -30, 0, 30, 55)
PITCH_SAMPLES = (-110, -115, -117, -119, -121, -125)
_C = {}


def obstacles():
    """The neighbours, in the yoke's frame at zero pose."""
    if not _C:
        _C["carrier"] = Pos(0, 0, V.HIP_YAW_Z - V.HIP_ROLL_Z) * A.part_yaw_carrier()
        _C["roll_servo"] = CA.servo_mock_x()
        _C["thigh"] = Pos(0, 0, -D.ROLL_TO_PITCH) * A.part_leg_link()
        _C["pitch_servo"] = Pos(0, 0, -D.ROLL_TO_PITCH) * CA.servo_mock_y()
    return _C


def at_roll(s, a):
    """A link-1 solid as the yoke sees it at hip_roll = a."""
    return Rot(-a, 0, 0) * s


def at_pitch(s, a):
    """A link-3 solid as the yoke sees it at hip_pitch = a."""
    return (Pos(0, 0, -D.ROLL_TO_PITCH) * Rot(0, a, 0)
            * Pos(0, 0, D.ROLL_TO_PITCH) * s)


def pair(a, b):
    try:
        vol = (a & b).volume
    except Exception:  # noqa: BLE001 -- a refused boolean is not a clearance
        vol = float("nan")
    try:
        dist = a.distance_to(b)
    except Exception:  # noqa: BLE001
        dist = float("nan")
    return vol, dist


def first_contact(p, thigh, lo=-100.0, hi=-130.0, steps=8):
    """The flexion angle where the thigh first touches the yoke, bisected."""
    for _ in range(steps):
        mid = (lo + hi) / 2
        vol, _d = pair(p, at_pitch(thigh, mid))
        lo, hi = (mid, hi) if vol < 0.05 else (lo, mid)
    return lo


def report(parts=None):
    parts = parts or (("fused", HY.fused()), ("styled", HY.hip_yoke_v6(verbose=False)))
    o = obstacles()
    seats = HY.disc_seat_zones()
    print(f"{'measurement':46s} " + "".join(f"{n:>14s}" for n, _ in parts))
    rows = []
    for a in ROLL_SAMPLES:
        for name in ("roll_servo", "carrier"):
            rows.append((f"hip_roll {a:+4d}  {name:11s} min dist mm",
                         [pair(p - seats, at_roll(o[name], a))[1] for _, p in parts]))
    for a in PITCH_SAMPLES:
        rows.append((f"hip_pitch {a:+5d}  thigh       overlap mm3",
                     [pair(p, at_pitch(o["thigh"], a))[0] for _, p in parts]))
    rows.append(("hip_pitch -117     thigh       min dist mm",
                 [pair(p, at_pitch(o["thigh"], -117.0))[1] for _, p in parts]))
    rows.append(("flexion first contact, deg",
                 [first_contact(p, o["thigh"]) for _, p in parts]))
    for label, vals in rows:
        print(f"{label:46s} " + "".join(f"{v:14.3f}" for v in vals))
    print("\n(the 0.700 mm at -117 is the thigh grip plate vs the yoke arm gap, "
          "which\n does not vary with angle -- deep flexion shows up as overlap "
          "volume, so\n both are reported. D.SWEEP_BUFFER is "
          f"{D.SWEEP_BUFFER} mm.)")


if __name__ == "__main__":
    report()
