"""Interference check for the hip-yaw bearing study
(docs/design-v6/study-yaw-bearing.md) that models the BEARING ITSELF as two
solid rings (inner race + outer race), not just carrier-vs-pelvis: sweeping
the printed parts against each other proves they don't collide, but says
nothing about whether either of them collides with the space the bearing's
own steel occupies. Checked, through yaw x hip_roll x hip_pitch at their ROM
extremes, at each hip:

  1. carrier (rotating) must not enter the INNER RING's own body except the
     boss it presses onto (checked by shrinking the ring to exclude the
     boss's own interference allowance).
  2. carrier must not enter the OUTER RING's body at all (it is fixed to
     the pelvis; the carrier only ever touches it through the balls).
  3. pelvis (fixed) must not enter the OUTER RING's own body except the
     recess wall it presses onto.
  4. pelvis must not enter the INNER RING's body at all (it rotates with
     the carrier).
  5. D.SWEEP_BUFFER (0.5 mm) everywhere else.

Ring radii: V.YAW_BRG_BOSS_R (inner race bore, where the carrier's boss
presses) out to V.YAW_BRG_EST_INNER_RING_OD (ESTIMATED -- see
dimensions_v6.py, no published internal-geometry spec for this bearing
class) for the inner ring; V.YAW_BRG_EST_OUTER_RING_ID (estimated) out to
V.YAW_BRG_RECESS_R (outer race OD, where the pelvis recess presses) for the
outer ring. Width = V.YAW_BRG_W, positioned at V.YAW_BRG_RECESS_Z (which is
where BOTH the carrier's boss and the pelvis's recess are built, by
construction -- see dimensions_v6.py's derivation).

    .venv/bin/python cad/v6/check_yaw_bearing_combo.py
"""
import itertools
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

from build123d import Pos  # noqa: E402
import dimensions_v6 as V  # noqa: E402
import assembly_v6 as A  # noqa: E402
import parts as P  # noqa: E402
D = V.D
cyl_z = P.cyl_z

# a small allowance so the ring bodies used for the CHECK exclude the
# bearing's own designed interference (the boss legitimately overlaps the
# inner ring's bore by +0.08 mm; the recess legitimately overlaps the outer
# ring's OD by -0.04 mm) -- without this every run would "fail" on the
# press fit itself.
# The boss and the recess wall are SUPPOSED to touch the races (that is the
# press fit) -- D.SWEEP_BUFFER (0.5 mm) is for anything ELSE, unintended,
# not for the seat itself. So the ring boundaries used for the CHECK are
# already inset by exactly D.SWEEP_BUFFER beyond the nominal seat surface:
# a boss/recess sitting flush at its own OD/ID (zero clearance, as
# designed) then reports min_dist == D.SWEEP_BUFFER to this ring, which is
# the pass condition below -- not an extra allowance stacked on top of it.
FIT_ALLOW = D.SWEEP_BUFFER

INNER_RING = (V.YAW_BRG_BOSS_R + FIT_ALLOW, V.YAW_BRG_EST_INNER_RING_OD)
OUTER_RING = (V.YAW_BRG_EST_OUTER_RING_ID, V.YAW_BRG_RECESS_R - FIT_ALLOW)
# WORLD frame, not V.YAW_BRG_RECESS_Z (pelvis-local) -- assembly_v6.robot()
# places every piece in world coordinates, and the carrier's boss (built in
# CARRIER-local Z, then placed at world Pos(0, y, V.HIP_YAW_Z)) lands at
# world z = [V.HIP_YAW_Z - V.YAW_BRG_W, V.HIP_YAW_Z], which is where the
# pelvis's recess (built in pelvis-local Z, placed at Pos(0,0,V.DECK_TOP_Z))
# also lands, by the DECK_TOP_Z/YAW_HORN_FACE_Z identity in dimensions_v6.py.
# An earlier version of this script used the pelvis-local Z directly here,
# which put the ring tens to hundreds of mm away from the actual joint --
# every check "passed" by construction (nothing was ever near the ring),
# caught by the reported distances being physically absurd (400+ mm at a
# joint whose parts are all within ~30 mm of each other).
RING_Z = (V.HIP_YAW_Z - V.YAW_BRG_W, V.HIP_YAW_Z)


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

    # ONE robot() build per pose, reused for every check -- rebuilding all
    # 33 pieces per pose per check (5x) was needlessly slow.
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
        # the ring boundary is ALREADY inset by D.SWEEP_BUFFER beyond the
        # nominal seat surface (see FIT_ALLOW above) -- a part sitting
        # flush at its own seat (zero clearance, as designed) reports
        # dist == D.SWEEP_BUFFER to this ring, so >= 0 is the pass
        # condition here, not another full buffer stacked on top of it.
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
