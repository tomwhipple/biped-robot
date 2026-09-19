"""yaw_carrier_v6: v5's yaw_carrier (cad/parts.py) plus a hip-yaw bearing boss
(2026-09-17, docs/design-v6/study-yaw-bearing.md; REVISED after coordinator
review -- see the write-up's history section for the two rejected passes).

Today the yaw servo's horn spline + 4 horn screws are the ONLY connection
between the carrier (the whole leg, a 0.335 m lever) and the pelvis -- every
thrust and moment load at the hip goes through the servo's output shaft. This
adds a boss that WRAPS the OUTSIDE of the carrier's ENTIRE horn-mount plate
AND the top of the roll bay's walls below it (they share the same footprint)
for a 6811-2RS deep-groove ball bearing's inner race -- NOT a boss concentric
with just the servo's O19.2 horn disc (leaves an unprintable 0.4 mm wall) and
NOT a boss sized to the disc-or-case footprint alone (the carrier's own
plate/wall corner, 25.12 mm from the axis, is bigger than either and sweeps
through the bearing band regardless).

Boss radius V.YAW_BRG_BOSS_R (27.54 mm) encloses that 25.12 mm corner with a
2.42 mm wall -- nothing is trimmed off the carrier's existing plate or bay
walls; the boss is purely ADDED material outside them. It also clears the
servo case's own (smaller) corner, hypot(D.SV_AXIS_FROM_OUT_END, D.SV_WID/2)
= 15.97 mm, by 11.6 mm.

Construction: add a full disk (radius YAW_BRG_BOSS_R) below the horn plate's
z=0 face, out to YAW_BRG_BOSS_H (9 mm -- the plate's own 3 mm plus 6 mm of
the bay walls below it, which share the same X/Y footprint so wrapping them
costs no extra radius), THEN re-open the roll bay's own interior footprint
(the same x/y box the front/rear/cheek walls already bound) through the
part of that height BELOW the plate (z = -9..-3), so the bay's cavity (where
the roll servo slides up) is simply 6 mm deeper at its mouth -- unchanged
everywhere it already mattered. The plate itself (z = -3..0) is untouched
(still solid, still where the 4 horn screws seat). Nothing is subtracted
from the carrier anywhere by this module -- see check_yaw_bearing_combo.py
for the interference-ring proof that the boss's OD is the carrier's true
outer radius at every z in the band.

Print: SAME orientation as v5's yaw_carrier (RX180, horn-plate face on the
bed, bay walls rise) -- the boss sits at MORE NEGATIVE local z than the
plate's z=0 face, i.e. it rises in print with the bay walls, not against
them; no new orientation, no new overhang class.

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

    r_boss = V.YAW_BRG_BOSS_OD / 2
    z0, z1 = V.YAW_BRG_BOSS_CARRIER_Z          # (-9.0, 0.0), carrier-local
    p += cyl_z(r_boss, z0, z1, 0, 0)

    # re-open the roll bay's own footprint through the part of the new boss
    # BELOW the existing plate (z = z0 .. -PLATE) -- the SAME x/y bound the
    # front/rear/cheek walls already use, so nothing that was solid before
    # (the walls, the plate, the screw-hole seating) changes; the plate's
    # own z-band (-PLATE..0) is left alone -- it was already solid out to
    # its own +-19.95/+-hw corner, and the boss simply wraps further out.
    cy0 = D.SV_WID / 2 + D.BAY_CHEEK_GAP        # 12.66, cheek inner face
    p -= box(-D.SV_TOPFACE, D.SV_TOPFACE, -cy0, cy0, z0 - 0.5, -D.YAW_CARRIER_PLATE + 0.001)
    return p


if __name__ == "__main__":
    os.makedirs(os.path.join(HERE, "stl"), exist_ok=True)
    os.makedirs(os.path.join(HERE, "step"), exist_ok=True)
    p = yaw_carrier_v6()
    export_stl(p, os.path.join(HERE, "stl", "yaw_carrier_v6.stl"))
    export_step(p, os.path.join(HERE, "step", "yaw_carrier_v6.step"))
    print(f"yaw_carrier_v6: volume {p.volume/1000:.2f} cm3, bbox {p.bounding_box().size}")
