"""yaw_bearing_housing: the hip-yaw bearings' outer-race seats, both hips in
one print, screwed up under the pelvis's cell block (dimensions_v6
YAW_HOUSING_*, YAW_BRG_*). Pelvis-local frame: deck top at z = 0.

Per hip, a skirt (OD V.YAW_BRG_SKIRT_OD) hangs from the top face, which bears
on the cell block's underside (V.YAW_BOX_BOT_Z). A shoulder (ID
V.YAW_BRG_SHOULDER_ID) in the 1.8 mm gap between that rim and the horn face
stops the outer race's top face and passes the leg's upward thrust into the
pelvis; below it a recess (ID V.YAW_BRG_RECESS_ID, -0.04 mm on the race OD)
locates the outer race. A bridge joins the two skirts between the hips and a
bar crosses the rear; five M2.5 screws (V.YAW_HOUSING_SCREWS) go up into pilot
bosses on the pelvis: four through the rear bar and one through the bridge,
on the centre line, into the cell block's front web. Skirts, bridge and bar
make one frame, so a moment on either bearing is shared by all five screws
and the whole contact face. Nothing stands proud of the pelvis's front face
but the front of the two skirts.

It is a separate part, not part of the pelvis print, because of what has to
pass through its space before it goes on: the yaw servo, up into its cell, and
the driver for the roll servo's two upper idler-side screws (dimensions_v6,
YAW_HOUSING_*). It goes on last in the hip's bench order (docs/assembly.md
section 7): offered up from below over the carriers and roll servos, its
recesses taking the two outer races, and screwed from below.

Print: top face (the face against the pelvis) on the bed; the skirts and the
bridge rise from it, the recess (wider than the shoulder) widens upward, and
the countersinks open upward. No supports.

    .venv/bin/python cad/v6/yaw_bearing_housing.py      # STL + STEP + mass
"""
from __future__ import annotations

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

from build123d import Cone, Pos, export_step, export_stl  # noqa: E402
import dimensions_v6 as V  # noqa: E402
import parts as P  # noqa: E402

D = V.D
box, cyl_z, csk_z = P.box, P.cyl_z, P.csk_z

# print orientation for check_printability: top face down (RX180), as the pelvis
PRINT_ORIENT = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)

HIPS = (V.HIP_SEP / 2, -V.HIP_SEP / 2)
REAR_BAR = (-44.5, -32.5, 50.8)          # x0, x1, half length in y
BRIDGE = (-15.0, 17.0, 9.0)              # x0, x1 (short of the General Driver's PCB at 18.01), half width in y
LEAD_IN = 0.5                            # 45 deg chamfer at the recess mouth: starts the race's press


def yaw_bearing_housing():
    z0, z1 = V.YAW_HOUSING_Z
    zt = z1 - V.YAW_HOUSING_EAR_T
    p = cyl_z(V.YAW_BRG_SKIRT_OD / 2, z0, z1, 0, HIPS[0]) + cyl_z(V.YAW_BRG_SKIRT_OD / 2, z0, z1, 0, HIPS[1])
    p += box(BRIDGE[0], BRIDGE[1], -BRIDGE[2], BRIDGE[2], z0, z1)
    p += box(REAR_BAR[0], REAR_BAR[1], -REAR_BAR[2], REAR_BAR[2], zt, z1)
    sho_z0, _ = V.YAW_BRG_SHOULDER_Z
    rec_z0, rec_z1 = V.YAW_BRG_RECESS_Z
    for by in HIPS:
        p -= cyl_z(V.YAW_BRG_SHOULDER_ID / 2, sho_z0 - 0.01, z1 + 0.5, 0, by)
        p -= cyl_z(V.YAW_BRG_RECESS_R, z0 - 0.6, rec_z1 + 0.01, 0, by)
        p -= Pos(0, by, z0 + LEAD_IN / 2 - 0.05) * Cone(V.YAW_BRG_RECESS_R + LEAD_IN + 0.05,
                                                        V.YAW_BRG_RECESS_R, LEAD_IN + 0.1)
    for hx, hy in V.YAW_HOUSING_SCREWS:
        p -= cyl_z(D.CASE_SCREW_CLEAR / 2, zt - 1, z1 + 1, hx, hy)
        p -= csk_z(hx, hy, zt, -1)
        # under the head: driver room through the full-height bridge (a no-op
        # under the bar, which is only YAW_HOUSING_EAR_T thick)
        p -= cyl_z(D.ACCESS_D / 2 + 0.1, z0 - 1, zt - 0.01, hx, hy)
    return p


def SCREWS():
    """The five housing screws, pelvis-local: M2.5 x 8 flat heads driven UP
    from below through the bar or the bridge into the pelvis's pilot bosses. `pos` is the
    head's bearing face (the bar's underside), `axis` points head to tip,
    `bite` is the thread in the pelvis."""
    z1 = V.YAW_HOUSING_Z[1]
    zt = z1 - V.YAW_HOUSING_EAR_T
    L = V.YAW_HOUSING_SCREW_L
    return [dict(name=f"housing_{hx:+.0f}_{hy:+.0f}",
                 kind=f"M2.5x{L:.0f} self-tap, flat head (bearing housing up into the pelvis)",
                 pos=(hx, hy, zt), axis=(0, 0, 1), length=L, bite=L - V.YAW_HOUSING_EAR_T)
            for hx, hy in V.YAW_HOUSING_SCREWS]


if __name__ == "__main__":
    os.makedirs(os.path.join(HERE, "stl"), exist_ok=True)
    os.makedirs(os.path.join(HERE, "step"), exist_ok=True)
    p = yaw_bearing_housing()
    export_stl(p, os.path.join(HERE, "stl", "yaw_bearing_housing.stl"))
    export_step(p, os.path.join(HERE, "step", "yaw_bearing_housing.step"))
    bb = p.bounding_box()
    m = p.volume * D.FILAMENT_RHO * D.PRINT_MASS_FACTOR
    print(f"yaw_bearing_housing: {m:.1f} g, bbox {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, "
          f"{len(p.solids())} solid(s)")
