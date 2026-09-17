"""yaw_carrier_v6: v5's yaw_carrier (cad/parts.py) plus a hip-yaw bearing boss
(2026-09-17, docs/design-v6/study-yaw-bearing.md).

Today the yaw servo's horn spline + 4 horn screws are the ONLY connection
between the carrier (the whole leg, a 0.335 m lever) and the pelvis -- every
thrust and moment load at the hip goes through the servo's output shaft. This
adds a boss around the OUTSIDE of the carrier's horn-mount plate (NOT
concentric with the servo's O19.2 horn disc -- a boss that tight leaves only
a 0.4 mm wall) for a 6709-2RS deep-groove ball bearing's INNER race. The
matching OUTER-race recess is a new skirt on the pelvis
(cad/v6/pelvis_v7.py), hanging from the existing yaw-cell tube rim.

Boss radius V.YAW_BRG_BOSS_R (22.55 mm) clears the servo case's own
rectangular footprint at every height (the case's forward corner is
hypot(D.SV_AXIS_FROM_OUT_END, D.SV_WID/2) = 15.97 mm from the axis -- see
dimensions_v6.py's derivation), so the boss can be as tall as the bearing's
full 6 mm width without ever meeting the case. A first pass used a smaller
6707-2RS (35x44x5) but the carrier's own horn-mount PLATE (not just the
disc) is bigger than the case -- it doubles as the roll bay's ceiling, so
its own corner (25.12 mm) needs clearance too; 6709 gives that room (see
dimensions_v6.py) plus the corner trim below.

Construction: add a full disk (radius YAW_BRG_BOSS_R) below the horn plate's
z=0 face, out to YAW_BRG_BOSS_H, THEN re-cut the roll bay's own interior
footprint (the same x/y box the front/rear/cheek walls already bound) through
that same height, so the boss becomes an ANNULUS outside the existing walls
and the bay's cavity (where the roll servo slides up) is simply 2 mm deeper
at its mouth -- unchanged everywhere it already mattered. The 4 horn-screw
holes (bolt circle radius D.BCD/2 = 7 mm) sit well inside that re-cut area,
so they stay exactly as v5 built them: driven from below, through the SAME
3 mm plate (z=0..-3), reachable through the (now slightly deeper) bay mouth.

Print: SAME orientation as v5's yaw_carrier (RX180, horn-plate face on the
bed, bay walls rise) -- the boss sits at MORE NEGATIVE local z than the
plate's z=0 face, i.e. it rises in print with the bay walls, not against
them; no new orientation, no new overhang class.

    .venv/bin/python cad/v6/yaw_carrier_v6.py       # STL + STEP + section render
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

    r_boss = V.YAW_BRG_BOSS_OD / 2
    z0, z1 = V.YAW_BRG_BOSS_CARRIER_Z          # (-6.0, 0.0), carrier-local
    p += cyl_z(r_boss, z0, z1, 0, 0)

    # re-open the roll bay's own footprint through the new boss height --
    # the SAME x/y bound the front/rear walls and cheeks already use, so
    # nothing that was solid before (the walls, the screw-hole plate) changes;
    # only the annulus outside the existing bay walls (their outer face at
    # D.YAW_BOX_HW_OUT-style 15.26ish half-width) becomes new boss material.
    cy0 = D.SV_WID / 2 + D.BAY_CHEEK_GAP        # 12.66, cheek inner face
    p -= box(-D.SV_TOPFACE, D.SV_TOPFACE, -cy0, cy0, z0 - 0.5, -D.YAW_CARRIER_PLATE + 0.001)

    # ROUND THE TOP PLATE'S 4 CORNERS (z = -3..0 ONLY -- the horn-mount plate
    # itself, not the bay walls below it, which are untouched). v5's plate is
    # a v3.9 x 30.52 rectangle built as the bay's own ceiling, so its own
    # corner is hypot(19.95, 15.26) = 25.12 mm from the axis -- bigger than
    # the pelvis-side bearing shoulder can clear (checked by
    # check_assembly_v6.py --joint hip_yaw: the FIRST pass at this design
    # left only 0.38 mm at the shoulder, < D.SWEEP_BUFFER). Trim the top
    # slab back to V.YAW_BRG_BOSS_R + 0.5 (23.05 mm, just past the boss
    # itself) so the pelvis shoulder land has real room; the walls below
    # (and the wall-to-plate joints inboard of this radius) are unaffected.
    hw_plate = cy0 + D.WALL                     # 15.26, matches yaw_carrier()'s own hw
    trim_r = V.YAW_BRG_BOSS_R + 0.5              # 23.05
    corner_box = box(-19.95 - 0.5, 19.95 + 0.5, -hw_plate - 0.5, hw_plate + 0.5,
                      -D.YAW_CARRIER_PLATE - 0.1, 0.5)
    p -= corner_box - cyl_z(trim_r, -D.YAW_CARRIER_PLATE - 0.2, 0.7, 0, 0)
    return p


if __name__ == "__main__":
    os.makedirs(os.path.join(HERE, "stl"), exist_ok=True)
    os.makedirs(os.path.join(HERE, "step"), exist_ok=True)
    p = yaw_carrier_v6()
    export_stl(p, os.path.join(HERE, "stl", "yaw_carrier_v6.stl"))
    export_step(p, os.path.join(HERE, "step", "yaw_carrier_v6.step"))
    print(f"yaw_carrier_v6: volume {p.volume/1000:.2f} cm3, bbox {p.bounding_box().size}")
