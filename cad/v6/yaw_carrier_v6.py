"""yaw_carrier_v6: v5's yaw_carrier (cad/parts.py) with a round hub for the
hip-yaw bearing's inner race (6810-2RS, dimensions_v6 YAW_BRG_*;
docs/design-v6/study-yaw-bearing.md, option A).

The bearing band adds no height: it runs from the horn face (carrier z = 0)
down the bearing's width, V.YAW_BRG_BAND_Z = (-7, 0), so HIP_YAW_Z,
CARRIER_ROLL_AXIS and d_yaw_roll are untouched. Within the band the carrier
holds its horn plate (z 0..-3) and, below that, the top of the roll-servo bay,
where the ROLL servo's cable end sits (V.YAW_ROLL_CASE_TOP_Z = -5.00). The hub
is sized to clear that case's corner reach (V.YAW_BAND_CASE_CORNER_R = 22.23
mm, measured), which is why the bore is 50 mm.

Construction, entirely within z in [-7, 0]; v5's bay walls, U-slot, retention
screw rows and idler seat below the band are unchanged:
  1. trim the plate corners and the top of the bay walls back to a cylinder
     of radius V.YAW_BRG_BOSS_R (25.04). That clips the 25.12 mm corners of
     the rectangle by 0.08 mm, under print tolerance and not a load path (the
     load goes through the horn screws at the centre and the bearing at the
     OD), and adds material along the flat sides (19.95 and 15.26 mm
     half-widths): the top of the carrier becomes a true cylinder.
  2. re-open the bay's own rectangular cavity for z in [-7, -3], where the
     roll servo's case passes through, so the hub does not seal it off.
  3. re-cut the 4 horn screw bores and the centre relief, which step 1's disk
     refills where they fall within the band.

The horn screws are v5's (4 x M3 on the r 7 bolt circle through the 3 mm
plate), with 16 mm of hub wall outside their heads. Both roll-servo retention
screw rows stay below the band (V.YAW_ROLL_SCREW_ROWS_Z, asserted in
dimensions_v6.py); only the top ~2 mm of the idler-side seat pad, which
reached up to -5.00, is trimmed.

The bearing goes onto the hub from the horn-face side: below the band the
bay's rectangular corners (r 25.12) are bigger than the 25.0 mm bore.

Print: v5's orientation (RX180, horn-plate face on the bed, bay walls rise).

    .venv/bin/python cad/v6/yaw_carrier_v6.py       # STL + STEP
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


def yaw_carrier_v6(print_fins=False):
    p = P.yaw_carrier(print_fins=print_fins)

    r_hub = V.YAW_BRG_BOSS_R
    z0, z1 = V.YAW_BRG_BAND_Z          # (-7.0, 0.0), carrier-local

    # 1. trim everything OUTSIDE the round hub, within the band only
    p -= box(-100, 100, -100, 100, z0, z1 + 1) - cyl_z(r_hub, z0 - 1, z1 + 1, 0, 0)
    # 2. add the full round hub for the whole band height
    p += cyl_z(r_hub, z0, z1, 0, 0)
    # 3. re-open the bay's interior footprint below the plate (z0 .. -PLATE):
    # the box v5's front/rear/cheek walls bound, so the roll servo's case
    # (cable end at V.YAW_ROLL_CASE_TOP_Z, inside this range) keeps its
    # clearance; the plate's own band (-PLATE..0) stays solid.
    cy0 = D.SV_WID / 2 + D.BAY_CHEEK_GAP        # 12.66, cheek inner face
    p -= box(-D.SV_TOPFACE, D.SV_TOPFACE, -cy0, cy0, z0 - 0.5, -D.YAW_CARRIER_PLATE + 0.001)
    # 4. re-cut the 4 horn screw bores + centre relief (z in [-4, 1], like v5's)
    r = D.BCD / 2
    for dx, dy in ((r, 0), (-r, 0), (0, r), (0, -r)):
        p -= cyl_z(D.PAD_HOLE / 2, -4, 1, dx, dy)
    p -= cyl_z(D.HORN_CENTER_RELIEF_D / 2, -4, 1, 0, 0)
    return p


def SCREWS():
    """The carrier's screws, carrier-local (horn face z = 0): 4 disc screws up
    through the bay ceiling into the yaw horn, length by the stack
    (V.disc_screw), and 8 M2.5 x 8 flat-heads through the bay's front and rear
    walls into the roll servo (cad/fasteners.py yaw_wall_screws). `axis`
    points from the head toward the tip."""
    r = D.BCD / 2
    stack = D.YAW_CARRIER_PLATE
    length, eng, flange = V.disc_screw(stack, "horn")
    s = [dict(name=f"yaw_horn_{dx:+.0f}_{dy:+.0f}", kind=f"M3x{length} button head",
              pos=(dx, dy, -stack), axis=(0, 0, 1), length=float(length),
              stack=stack, engage=eng, flange=flange)
         for dx, dy in ((r, 0), (-r, 0), (0, r), (0, -r))]
    for face, rows, sgn in (("front", D.CASE_HOLES_TOP, -1), ("rear", D.CASE_HOLES_BOT, +1)):
        for zrow in rows:
            for lat in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
                s.append(dict(name=f"roll_wall_{face}_{zrow:.0f}_{lat:+.0f}",
                              kind="M2.5x8 self-tap, flat head (bay wall into the roll servo)",
                              pos=(-sgn * 19.95, lat, D.CARRIER_ROLL_AXIS + zrow), axis=(sgn, 0, 0),
                              length=8.0))
    return s


if __name__ == "__main__":
    os.makedirs(os.path.join(HERE, "stl"), exist_ok=True)
    os.makedirs(os.path.join(HERE, "step"), exist_ok=True)
    p = yaw_carrier_v6()
    export_stl(p, os.path.join(HERE, "stl", "yaw_carrier_v6.stl"))
    export_step(p, os.path.join(HERE, "step", "yaw_carrier_v6.step"))
    print(f"yaw_carrier_v6: volume {p.volume/1000:.2f} cm3, bbox {p.bounding_box().size}")
