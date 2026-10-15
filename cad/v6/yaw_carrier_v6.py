"""yaw_carrier_v6: v5's yaw_carrier (cad/parts.py) with a spacer under its
horn plate and a round hub for the hip-yaw bearing's inner race (6810-2RS,
dimensions_v6 YAW_BRG_*; docs/design-v6/study-yaw-bearing.md, option A).

The SPACER: v5's carrier is cut just under the horn plate, at SPLIT_Z, where
its cross-section is the bay walls alone, and that section is extruded
V.YAW_CARRIER_DROP (5 mm) down. Everything below -- the roll bay, its screw
rows, the idler seat, the U-slot -- is v5's, 5 mm lower: the roll axis is at
V.CARRIER_ROLL_AXIS (-45.11). That puts the roll servo (case top at
V.YAW_ROLL_CASE_TOP_Z, -10.00) and its two upper wall screws below the bearing
band, with a full driver path under the band and the housing once both are
on.

The HUB, entirely within the band V.YAW_BRG_BAND_Z = (-7, 0) (the horn plate,
z 0..-3, and the top of the spacer):
  1. trim the plate corners and the spacer back to a cylinder of radius
     V.YAW_BRG_BOSS_MODEL_R (25.29): the hub to print at V.YAW_BRG_BOSS_OD
     (50.08, +0.08 on the bore) plus the printer's measured shortfall
     (V.YAW_BRG_BOSS_PRINT_ALLOW). It wraps the walls' rectangle (corners
     25.12) and adds material along the flat sides (19.95 and 15.26 mm
     half-widths): the top of the carrier becomes a true cylinder.
  2. re-open the bay's interior below the plate (z -7..-3): the horn screws are
     driven up through it.
  3. re-cut the 4 horn screw bores and the centre relief, which step 1's disk
     refills.
Under the band, a LIP (V.YAW_CARRIER_LIP_*, r 26.14, 1.2 thick) stops the
inner race's bottom face: the carrier goes up through the bearing, already in
its housing under the pelvis, until the lip seats (docs/assembly.md section
7), and the leg's upward thrust reaches the race on that face. The hub's top
edge, which enters the bore first, gets a lead-in.

Print: v5's orientation (RX180, horn-plate face on the bed, bay walls rise).

    .venv/bin/python cad/v6/yaw_carrier_v6.py       # STL + STEP
"""
from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", ".."))

from build123d import Cone, Plane, Pos, export_step, export_stl, extrude, section  # noqa: E402
import dimensions_v6 as V  # noqa: E402
import parts as P  # noqa: E402

D = V.D
box, cyl_z = P.box, P.cyl_z
HUB_LEAD_IN = 0.3                        # 45 deg chamfer on the hub's top edge


SPLIT_Z = -4.0     # v5 carrier-local: under the 3 mm plate, above every bay feature (constant section -3.05..-4.5)


def carrier_spaced(print_fins=False):
    """v5's carrier with the V.YAW_CARRIER_DROP spacer inserted at SPLIT_Z."""
    c = P.yaw_carrier(print_fins=print_fins)
    drop = V.YAW_CARRIER_DROP
    big = 200.0
    top = c & box(-big, big, -big, big, SPLIT_Z, big)
    bottom = Pos(0, 0, -drop) * (c & box(-big, big, -big, big, -big, SPLIT_Z))
    sec = section(c, Plane.XY.offset(SPLIT_Z))
    spacer = extrude(sec, amount=drop, dir=(0, 0, -1))
    return (top + spacer + bottom).clean()


def yaw_carrier_v6(print_fins=False):
    base = carrier_spaced(print_fins=print_fins)
    p = base

    r_hub = V.YAW_BRG_BOSS_MODEL_R          # with the print allowance: prints at V.YAW_BRG_BOSS_OD
    z0, z1 = V.YAW_BRG_BAND_Z          # (-7.0, 0.0), carrier-local

    # 1. trim everything OUTSIDE the round hub, within the band only
    p -= box(-100, 100, -100, 100, z0, z1 + 1) - cyl_z(r_hub, z0 - 1, z1 + 1, 0, 0)
    # 2. add the full round hub for the whole band height
    p += cyl_z(r_hub, z0, z1, 0, 0)
    # 3. re-open the bay's own interior below the plate (z0 .. -PLATE): the
    # horn screws are driven up through it; the plate's band (-PLATE..0)
    # stays solid.
    hw = D.SV_WID / 2 + D.BAY_CHEEK_GAP + D.WALL     # 15.26, the cheeks' outer face
    p -= box(-19.95, 19.95, -hw, hw, z0 - 0.5, -D.YAW_CARRIER_PLATE + 0.001) - base
    # 4. re-cut the 4 horn screw bores + centre relief (z in [-4, 1], like v5's)
    r = D.BCD / 2
    for dx, dy in ((r, 0), (-r, 0), (0, r), (0, -r)):
        p -= cyl_z(D.PAD_HOLE / 2, -4, 1, dx, dy)
    p -= cyl_z(D.HORN_CENTER_RELIEF_D / 2, -4, 1, 0, 0)
    # 5. the lip under the band, out to V.YAW_CARRIER_LIP_R, with the bay's
    # interior open through it like the band's
    l0, l1 = V.YAW_CARRIER_LIP_Z
    p += cyl_z(V.YAW_CARRIER_LIP_R, l0, l1, 0, 0)
    p -= box(-19.95, 19.95, -hw, hw, l0 - 0.5, l1 + 0.001) - base
    # 6. a lead-in at the hub's top edge, which goes into the bore first
    p -= Pos(0, 0, z1 - HUB_LEAD_IN / 2 + 0.05) * Cone(r_hub - HUB_LEAD_IN, r_hub + 0.05, HUB_LEAD_IN + 0.1) \
        - cyl_z(r_hub - HUB_LEAD_IN - 0.01, z1 - 1, z1 + 1, 0, 0)
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
                              pos=(-sgn * 19.95, lat, V.CARRIER_ROLL_AXIS + zrow), axis=(sgn, 0, 0),
                              length=8.0))
    return s


if __name__ == "__main__":
    from paths import build_dir
    os.makedirs(os.path.join(HERE, "stl"), exist_ok=True)
    p = yaw_carrier_v6()
    export_stl(p, os.path.join(HERE, "stl", "yaw_carrier_v6.stl"))
    export_step(p, os.path.join(build_dir("v6", "step"), "yaw_carrier_v6.step"))
    print(f"yaw_carrier_v6: volume {p.volume/1000:.2f} cm3, bbox {p.bounding_box().size}")
