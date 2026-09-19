"""Ring-based interference check for hip-yaw bearing OPTION A (round hub,
round 2, docs/design-v6/study-yaw-bearing.md) -- same method as
check_yaw_bearing_combo.py (option C), models the 6810-2RS bearing itself as
two solid rings (inner + outer race), not just the printed parts against
each other. Sizes come from dimensions_v6.py's YAWA_* constants.

    YAW_BEARING_VARIANT=A .venv/bin/python cad/v6/check_yaw_bearing_optA.py
"""
import itertools
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
os.environ["YAW_BEARING_VARIANT"] = "A"

from build123d import Pos  # noqa: E402
import dimensions_v6 as V  # noqa: E402
import assembly_v6 as A  # noqa: E402
import parts as P  # noqa: E402
D = V.D
cyl_z = P.cyl_z

FIT_ALLOW = D.SWEEP_BUFFER
INNER_RING = (V.YAWA_BRG_BOSS_R + FIT_ALLOW, V.YAWA_EST_INNER_RING_OD)
OUTER_RING = (V.YAWA_EST_OUTER_RING_ID, V.YAWA_BRG_RECESS_R - FIT_ALLOW)
# WORLD frame -- same identity as check_yaw_bearing_combo.py: the carrier's
# hub (built in carrier-local z = V.YAWA_BAND_Z, placed at world
# Pos(0, y, V.HIP_YAW_Z)) lands at world z = [V.HIP_YAW_Z - YAWA_BRG_W,
# V.HIP_YAW_Z] -- UNCHANGED from option C's own RING_Z (same HIP_YAW_Z, no
# axial growth), just a shallower band (7 mm, not 9).
RING_Z = (V.HIP_YAW_Z - V.YAWA_BRG_W, V.HIP_YAW_Z)


def ring_body(by, r0, r1, z0, z1):
    return Pos(0, by, 0) * (cyl_z(r1, z0, z1, 0, 0) - cyl_z(r0, z0 - 1, z1 + 1, 0, 0))


def main():
    side = "L"
    by = V.HIP_SEP / 2 * (1 if side == "L" else -1)
    yaws = (V.ROM["hip_yaw"][0], 0.0, V.ROM["hip_yaw"][1])
    rolls = V.ROM["hip_roll"]
    pitches = V.ROM["hip_pitch"]
    poses = [{"L_hip_yaw": y, "L_hip_roll": r, "L_hip_pitch": p}
              for y, r, p in itertools.product(yaws, rolls, pitches)]

    inner_ring = ring_body(by, INNER_RING[0], INNER_RING[1], *RING_Z)
    outer_ring = ring_body(by, OUTER_RING[0], OUTER_RING[1], *RING_Z)

    checks = [
        ("yaw_carrier", inner_ring, "INNER_RING", "rotating carrier must clear it except the boss"),
        ("yaw_carrier", outer_ring, "OUTER_RING", "rotating carrier must never touch the fixed race"),
        ("pelvis_v7", outer_ring, "OUTER_RING", "fixed pelvis must clear it except the recess wall"),
        ("pelvis_v7", inner_ring, "INNER_RING", "fixed pelvis must never touch the rotating race"),
    ]
    worst = {i: (0.0, 1e9, None) for i in range(len(checks))}
    worst_direct = (0.0, 1e9, None)

    for pose in poses:
        comp = A.robot(pose)
        Pc = {c.label: c for c in comp.children}
        carrier = Pc[f"yaw_carrier_{side}"]
        pelvis = Pc["pelvis_v7"]
        for i, (part_label, ring, _, _) in enumerate(checks):
            part = carrier if part_label == "yaw_carrier" else pelvis
            try:
                vol = (part & ring).volume
            except Exception:  # noqa: BLE001
                vol = 0.0
            try:
                dist = part.distance_to(ring)
            except Exception:  # noqa: BLE001
                dist = float("nan")
            if vol > worst[i][0] or (vol == worst[i][0] and dist < worst[i][1]):
                worst[i] = (vol, dist, pose)
        try:
            vol = (carrier & pelvis).volume
        except Exception:  # noqa: BLE001
            vol = 0.0
        dist = carrier.distance_to(pelvis)
        if vol > worst_direct[0] or (vol == worst_direct[0] and dist < worst_direct[1]):
            worst_direct = (vol, dist, pose)

    fails = 0
    print(f"{'part':12s} {'ring':10s} {'overlap mm3':>11s} {'min dist':>9s}  {'at (yaw,roll,pitch)':24s}  verdict  note")
    for i, (part_label, _, ring_name, note) in enumerate(checks):
        vol, dist, at = worst[i]
        ok = vol < 0.5 and dist >= -1e-6
        fails += 0 if ok else 1
        atstr = str(at.values()) if at else ""
        print(f"{part_label:12s} {ring_name:10s} {vol:11.2f} {dist:9.2f}  {atstr:24s}  {'ok' if ok else 'FAIL'}   {note}")

    vol, dist, at = worst_direct
    ok = vol < 0.5 and dist >= D.SWEEP_BUFFER - 1e-6
    fails += 0 if ok else 1
    atstr = str(at.values()) if at else ""
    print(f"{'yaw_carrier':12s} {'(direct)':10s} {vol:11.2f} {dist:9.2f}  {atstr:24s}  {'ok' if ok else 'FAIL'}   carrier vs pelvis, direct (cross-check)")

    print(f"=> {'ALL CLEAR' if fails == 0 else f'{fails} FAIL'}  "
          f"({len(poses)} poses x {len(checks)+1} pairs)")
    return fails


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
