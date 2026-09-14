"""Hip-pitch clevis, v6 (qty 2): cad/parts.py::yoke_pitch (v5, geometry
unchanged) with the flange's front-bottom edge chamfered (V.YOKE_FLEX_CHAMFER,
45 deg, full width) so the thigh's grip plates clear it to ~127 deg of hip
flexion instead of 123 (docs/design-v6-ankle-roll.md section 11.3). The
alternative -- relieving the thigh plate's front edge -- is not available:
the lower grip screw's countersink sits exactly where the flange lands.
Print as v5 (on its back, brim + slicer supports); the chamfer is on the
flange's underside edge and prints as a 45 deg face.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

import dimensions_v6 as V  # noqa: E402
import parts as v5  # noqa: E402
D = V.D


def yoke_pitch_v6():
    p = v5.yoke_pitch()
    zf1 = D.PITCH_ARM_REACH                          # flange bottom (arms side)
    xf = D.YOKE_FLANGE_X / 2                         # flange front face
    c = V.YOKE_FLEX_CHAMFER
    hy1 = D.SV_HORN_FACE + D.PLATE
    iy0 = D.IDLER_ARM_INNER - D.PLATE
    # hypotenuse through (xf - c, zf1) and (xf, zf1 + c); the +-1 margins
    # keep the boolean faces off the flange's own faces
    p -= v5.wedge_y([(xf + 1, zf1 - 1), (xf + 1, zf1 + c + 1), (xf - c - 1, zf1 - 1)],
                    iy0 - 1, hy1 + 1)
    return p


if __name__ == "__main__":
    from build123d import export_stl
    s = yoke_pitch_v6()
    bb = s.bounding_box()
    print(f"yoke_pitch_v6 {s.volume * D.FILAMENT_RHO * D.PRINT_MASS_FACTOR:.1f} g  bbox "
          f"{bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f}")
    os.makedirs(os.path.join(HERE, "stl"), exist_ok=True)
    export_stl(s, os.path.join(HERE, "stl", "yoke_pitch_v6.stl"))
