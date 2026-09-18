"""yaw_carrier_v6_optA: hip-yaw bearing OPTION A, round 2 (2026-09-18,
docs/design-v6/study-yaw-bearing.md; Tom: "the hip stack must NOT get
taller" -- NO axial growth -- and "modify the yaw joints to be round").

Unlike round 1's (rejected) round-hub idea, which sized the hub to the small
YAW-servo horn disc, this one is sized to what is ACTUALLY inside the
bearing band once the band is pinned to the carrier's EXISTING envelope: the
band runs from the horn face (carrier z=0) down the bearing's own width
(V.YAWA_BAND_Z = (-7.0, 0.0)) -- no new axial length anywhere, HIP_YAW_Z /
CARRIER_ROLL_AXIS / d_yaw_roll are all untouched. Within that band the
carrier already holds (v5, unchanged): the horn mount plate (z 0..-3) and,
once the band passes -3 mm, the TOP of the roll-servo bay -- occupied by the
ROLL SERVO's own case (cable end at V.YAWA_ROLL_CASE_TOP_Z = -5.00, see
dimensions_v6.py for the measurement). The hub therefore has to clear that
case's own corner reach (V.YAWA_BAND_CASE_CORNER_R = 22.23 mm, measured), not
the smaller yaw-horn-disc figure -- hence a 50 mm bore (6810-2RS), not the
round-1 attempt's 35 mm class.

Construction, entirely within z in [-7, 0] (carrier-local); UNCHANGED below
that (v5's rectangular bay walls, U-slot, retention screw rows, idler seat --
none of it moves, since the axial envelope is fixed):
  1. trim the carrier's existing material (plate corners AND the top of the
     bay walls) back to a cylinder of radius V.YAWA_BRG_BOSS_R -- this clips
     the FOUR CORNERS of the existing 25.12 mm rectangle by a hair (25.12 -
     25.04 = 0.08 mm, well under print tolerance, and not a load path: the
     carrier's load goes through the horn screws at the centre and the
     bearing at the OD, not through these corners) and, along the flat
     sides (19.95 and 15.26 mm half-widths, both < 25.04), ADDS a little
     material -- net effect: the top of the carrier becomes a true cylinder.
  2. re-open the SAME rectangular cavity the bay walls already had for
     z in [-7, -3] (where the roll servo's case passes through) so the
     round hub does not seal it off -- this is the SAME operation
     yaw_carrier_v6.py (option C) already does one z-band lower, just
     applied here because the band now reaches into that same territory.
  3. re-cut the 4 horn screw bores + the centre relief (the disk add in
     step 1 would otherwise refill them where they fall within the band).

The horn screws are UNCHANGED (same 4x M3, same 3 mm plate engagement,
bolt circle r 7, self-tap head ~1.7 mm -> 8.7 mm reach, 16.3 mm of hub wall
outside that -- no interaction with the bearing boss at r 25.04). Both roll-
servo retention screw rows stay OUTSIDE the band (V.YAWA_ROLL_SCREW_ROWS_Z,
asserted in dimensions_v6.py) -- only the very top ~2 mm of the idler-side
SEAT PAD (added 2026-08-01, cad/parts.py yaw_carrier docstring) is trimmed
where it happened to reach up to -5.00 inside the band; both retention rows
themselves (at -11.11 and -7.36) are untouched.

Print: SAME orientation as v5's yaw_carrier (RX180, horn-plate face on the
bed, bay walls rise) -- nothing about the print direction changes; the
round hub is a smaller-radius, shallower feature than option C's boss, at
the SAME end of the part.

    .venv/bin/python cad/v6/yaw_carrier_v6_optA.py       # STL + STEP
"""
from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

from build123d import export_step, export_stl  # noqa: E402
import dimensions_v6 as V  # noqa: E402
import parts as P  # noqa: E402

D = V.D
box, cyl_z = P.box, P.cyl_z


def yaw_carrier_v6_optA(print_fins=False):
    p = P.yaw_carrier(print_fins=print_fins)

    r_hub = V.YAWA_BRG_BOSS_R
    z0, z1 = V.YAWA_BAND_Z          # (-7.0, 0.0), carrier-local

    # 1. trim everything OUTSIDE the round hub, within the band only
    p -= box(-100, 100, -100, 100, z0, z1 + 1) - cyl_z(r_hub, z0 - 1, z1 + 1, 0, 0)
    # 2. add the full round hub for the whole band height
    p += cyl_z(r_hub, z0, z1, 0, 0)
    # 3. re-open the bay's own interior footprint through the part of the
    # band BELOW the existing plate (z0 .. -PLATE) -- the SAME box v5's own
    # front/rear/cheek walls already bound at this height, so the roll
    # servo's case (cable end at V.YAWA_ROLL_CASE_TOP_Z, INSIDE this z
    # range) still has its clearance; the plate's own z-band (-PLATE..0) is
    # left solid, as it always was.
    cy0 = D.SV_WID / 2 + D.BAY_CHEEK_GAP        # 12.66, cheek inner face
    p -= box(-D.SV_TOPFACE, D.SV_TOPFACE, -cy0, cy0, z0 - 0.5, -D.YAW_CARRIER_PLATE + 0.001)
    # 4. re-cut the 4 horn screw bores + centre relief (refilled by step 2
    # wherever they fall inside the band, z in [-4, 1] like v5's own cut)
    r = D.BCD / 2
    for dx, dy in ((r, 0), (-r, 0), (0, r), (0, -r)):
        p -= cyl_z(D.PAD_HOLE / 2, -4, 1, dx, dy)
    p -= cyl_z(D.HORN_CENTER_RELIEF_D / 2, -4, 1, 0, 0)
    return p


if __name__ == "__main__":
    os.makedirs(os.path.join(HERE, "stl"), exist_ok=True)
    os.makedirs(os.path.join(HERE, "step"), exist_ok=True)
    p = yaw_carrier_v6_optA()
    export_stl(p, os.path.join(HERE, "stl", "yaw_carrier_v6_optA.stl"))
    export_step(p, os.path.join(HERE, "step", "yaw_carrier_v6_optA.step"))
    print(f"yaw_carrier_v6_optA: volume {p.volume/1000:.2f} cm3, bbox {p.bounding_box().size}")
