"""Hip yoke, v6 ONE-PRINT variant (qty 2): the hip-roll clevis (v5
`yoke_roll`) and the hip-pitch clevis (`yoke_pitch_v6`) fused into a single
solid, replacing the two parts that were screwed flange to flange with 4x
M3x10 into 4x M3 heat-set inserts (cad/fasteners.py::flange_bolts).

Local frame == yoke_roll's: the ROLL axis is X through the origin, +X forward
(roll servo horn side), +Z up. The pitch clevis hangs D.ROLL_TO_PITCH below,
pitch axis along Y through (0, 0, -50) -- exactly where the assembly used to
place yoke_pitch with its own translation, so HIP_ROLL_Z, HIP_PITCH_Z and
every other kinematic constant are untouched, and both servo interfaces (roll
horn boss + sunk idler pad on X, pitch horn plate + idler hub on Y, all four
O14 bolt circles, centre reliefs) are the v5 parts' own geometry, not a copy.

WHAT CHANGED vs the two parts
  - the two 4 mm flanges (roll: z -20..-16, pitch: z -24..-20) are one 8 mm
    block; the mating face at z -20 is gone
  - the 4 bolt columns at (+-YOKE_BOLT_SQ/2)^2 -- O3.4 clearance through the
    roll flange, O4.1 heat-set pilots in the pitch flange -- are filled back
    to solid (one cylinder per column, trimmed by the flexion chamfer where
    the front pair grazes it, exactly as the heat-set bores did)
  - the hip-flexion chamfer (V.YOKE_FLEX_CHAMFER, docs/design-v6-ankle-roll.md
    section 11.3) is kept as-is: the thigh's grip plates still sweep the same
    flange corner at -125 deg, and this part's envelope is the union of the
    two it replaces, so the clearance cannot get worse. Re-verified by
    check_assembly_v6.py with HIP_YOKE_VARIANT=single.
  - nothing else. In particular the 8 mm block is NOT thinned: the flange
    thicknesses are what set PITCH_ARM_REACH (26 = 50 - 16 - 2*4), which is
    a kinematic-adjacent constant every hip check is written against.

PRINT ORIENTATION: WALL, on edge, model -Y on the bed (check_printability's
RY_ROLL_WALL -- yoke_roll's own orientation), WITH SLICER SUPPORTS + BRIM.

  Why this is the failure mode's orientation, not a compromise. yoke_roll's
  arms snapped when printed flange-down because the layers were stacked
  ALONG the arm (model Z), and a cantilevered arm carries bending as tension
  along its length -- straight across the interlayer bond. The fix was to
  put the arm length in the bed plane. In this frame BOTH clevises have
  their arm length along model Z (roll arms: flange z -16 up to the pads at
  z 0; pitch arms: flange z -24 down to the pads at z -50), so ANY
  orientation with model Z in the bed plane keeps every arm's bending
  tension along the filament. That leaves model +X up (yoke_pitch's RY_XUP)
  or model +Y up (yoke_roll's RY_ROLL_WALL); a 45 deg diagonal would let both
  arm sets lean at exactly the overhang limit but stands the part on one
  edge, so it is out.

  Either way ONE clevis prints as walls and the other as horizontal slabs,
  because the roll plates are normal to X and the pitch plates normal to Y.
  A slab arm is still sound against the recorded failure: its layers are
  planes containing the arm length, so bending tension (in-plane or
  out-of-plane) runs along filament; only a pure pull along the plate
  normal loads the interlayer bond, and that direction is the bolt preload
  (compression onto the disc, the servo body as the spacer). The choice
  between the two is therefore print quality, and +Y up wins on every count:

    - first layer: the pitch IDLER arm (r14 hub + riser) and the pitch
      flange's end land flat on the bed, ~830 mm2 (23 % of the footprint)
      vs ~360 mm2 (11 %) for +X up, where only the sunk roll idler pad and
      the roll flange's end touch
    - height: 43.85 mm vs 48.4
    - the ROLL arms -- the ones that broke -- print in exactly their proven
      wall orientation, and the 26 mm pitch idler arm (1.6x the root moment
      of a roll arm) prints as a bed-flat slab, the best case an FDM part
      gets
    - the bolt-circle bores that are horizontal in the print (the roll
      pads', axis X) are the O3.4 ones with a 4 mm-class roof, a NOTE not a
      fail in check_printability; the pitch pads' bores are vertical

  Supports (slicer): the pitch HORN arm is a horizontal slab ~41 mm up over
  the pitch servo void (its underside, y 20.45, is the horn seating face --
  a support-interface finish on a face clamped to a metal disc by 4 screws;
  the +X-up alternative puts the same finish on the ROLL horn boss face
  instead, so this is a wash), the roll pads' lower rims and the roll arm
  plates start in mid-air as in the yoke_roll print, and the roll flange's
  -Y face sits 3.4 mm above the bed. Brim: 44 mm tall on a T of 4 mm walls.
  No modelled fins -- see the note above yoke_roll in cad/parts.py.

    .venv/bin/python cad/v6/hip_yoke_v6.py   # stl/step/renders + audit + mass
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

from build123d import Pos, export_step, export_stl  # noqa: E402
import dimensions_v6 as V  # noqa: E402
import parts as v5  # noqa: E402
import yoke_pitch_v6  # noqa: E402
D = V.D

OUT_STL = os.path.join(HERE, "stl")
OUT_STEP = os.path.join(HERE, "step")
OUT_REN = os.path.join(HERE, "renders")

# check_printability's RY_ROLL_WALL, spelled out (rows = print axes in model
# coords): model +Y -> print +Z, i.e. the part lies on its -Y face.
PRINT_ORIENT = ((1.0, 0.0, 0.0),
                (0.0, 0.0, -1.0),
                (0.0, 1.0, 0.0))
SUPPORT_NOTE = ("supports on + brim: pitch horn arm slab over the servo void, "
                "roll pad rims / arm plates start in mid-air (as yoke_roll)")

# the 4 M3x10 + 4 heat-sets this part removes, per hip (fasteners.flange_bolts)
REMOVED_FASTENERS = {"M3x10 button head": 4, "M3 heat-set insert": 4}


def pitch_offset():
    """Location of the pitch yoke's frame in this part's (roll) frame."""
    return Pos(0, 0, -D.ROLL_TO_PITCH)


def hip_yoke_v6():
    roll = v5.yoke_roll()
    pitch = pitch_offset() * yoke_pitch_v6.yoke_pitch_v6()
    p = roll + pitch
    # Fill the flange bolt columns. Both bores live only in the two flanges
    # (the columns at +-10 miss every arm), so one cylinder from the pitch
    # flange's bottom to the roll flange's top restores solid material; the
    # radius covers the larger (heat-set) bore with a hair of margin so no
    # coincident cylindrical face is left for the fuse to chew on.
    zf_top = -D.ROLL_AXIS_TO_FLANGE                       # -16
    zf_bot = zf_top - 2 * D.YOKE_FLANGE_T                 # -24
    r_fill = max(D.M3_CLEAR, D.HEATSET_D) / 2 + 0.05
    b = D.YOKE_BOLT_SQ / 2
    fill = None
    for sx, sy in ((b, b), (b, -b), (-b, b), (-b, -b)):
        c = v5.cyl_z(r_fill, zf_bot, zf_top, sx, sy)
        fill = c if fill is None else fill + c
    # the front pair's heat-set bores just grazed the flexion chamfer (bore
    # edge x 12.05 vs the chamfer's foot at x 12.0); trim the fill the same way
    fill -= pitch_offset() * yoke_pitch_v6.flex_chamfer_wedge()
    return p + fill


def disc_seat_zones():
    """The volumes of this part that are DESIGNED to touch a neighbour, for
    check_assembly_v6's buffer rule (in this frame): the roll idler boss,
    which rides the yaw_carrier's bay bore at the 0.30 slip fit and seats on
    the idler disc, and the roll horn boss, which seats on the horn disc.
    Subtracting these from the posed part lets everything else -- arms,
    pads, both flanges, the whole pitch clevis -- be held to the full
    D.SWEEP_BUFFER against the carrier and the roll servo, which is
    STRICTER than the split-part check (yoke_roll vs carrier was
    volume-only there)."""
    idler = v5.cyl_x(D.IDLER_BOSS_D / 2 + 0.5, D.ROLL_ARM_INNER - 0.01,
                     D.SV_IDLER_FACE + 0.1, 0, 0)
    horn = v5.cyl_x(D.HORN_BOSS_D / 2 + 0.5, D.SV_HORN_FACE - 0.1,
                    D.SV_HORN_FACE + D.HORN_BOSS_H + 0.01, 0, 0)
    return idler + horn


def mass_g(solid):
    return solid.volume * D.FILAMENT_RHO * D.PRINT_MASS_FACTOR


def audit(stl_path):
    """check_printability.py on the exported STL, in PRINT_ORIENT, with the
    slicer-support waiver the yokes already have (runtime overrides only --
    v5's tables are not edited)."""
    import numpy as np
    import check_printability as CP
    CP.STL = os.path.dirname(stl_path)
    CP.ORIENT["hip_yoke_v6"] = (np.array(PRINT_ORIENT), "WALL: on edge like yoke_roll (-Y on the bed) + supports, brim")
    CP.PRINT_STL["hip_yoke_v6"] = os.path.basename(stl_path)
    CP.SUPPORTED["hip_yoke_v6"] = SUPPORT_NOTE
    return CP.audit("hip_yoke_v6")


def render(stl_path, png_path, px=640):
    """MuJoCo offscreen render (iso / bottom / back), like leg_link_v6.
    MUJOCO_GL must be set before mujoco is imported (cgl on a Mac)."""
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco  # noqa: E402
    import numpy as np
    import imageio.v2 as imageio
    xml = f"""
    <mujoco>
      <asset><mesh name="part" file="{stl_path}" scale="0.001 0.001 0.001"/></asset>
      <visual><headlight ambient="0.4 0.4 0.4" diffuse="0.7 0.7 0.7"/>
        <global offwidth="{px}" offheight="{px}"/></visual>
      <worldbody><geom type="mesh" mesh="part" rgba="0.45 0.62 0.90 1"/></worldbody>
    </mujoco>"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    m = model.mesh(0)
    v = model.mesh_vert[m.vertadr[0]:m.vertadr[0] + m.vertnum[0]]
    radius = np.linalg.norm(v - v.mean(0), axis=1).max()
    center = np.zeros(3)
    mujoco.mju_rotVecQuat(center, v.mean(0), model.mesh_quat[0])
    center += model.mesh_pos[0]
    ren = mujoco.Renderer(model, px, px)
    frames = []
    for az, el in ((135, -30), (90, 89), (180, -15)):
        cam = mujoco.MjvCamera()
        cam.lookat, cam.distance = center, 2.6 * radius
        cam.azimuth, cam.elevation = az, el
        ren.update_scene(data, cam)
        frames.append(ren.render())
    imageio.imwrite(png_path, np.concatenate(frames, axis=1))
    return png_path


if __name__ == "__main__":
    for d in (OUT_STL, OUT_STEP, OUT_REN):
        os.makedirs(d, exist_ok=True)
    p = hip_yoke_v6()
    stl_path = os.path.join(OUT_STL, "hip_yoke_v6.stl")
    export_stl(p, stl_path)
    export_step(p, os.path.join(OUT_STEP, "hip_yoke_v6.step"))
    bb = p.bounding_box()
    print(f"hip_yoke_v6  bbox x {bb.min.X:.2f}..{bb.max.X:.2f}  y {bb.min.Y:.2f}..{bb.max.Y:.2f}  "
          f"z {bb.min.Z:.2f}..{bb.max.Z:.2f} mm  (print height {bb.size.Y:.2f} on edge)")
    # mass: before (two parts) vs after (one), same density assumption
    r, q = v5.yoke_roll(), yoke_pitch_v6.yoke_pitch_v6()
    print(f"volume  yoke_roll {r.volume:8.1f} + yoke_pitch_v6 {q.volume:8.1f} = {r.volume + q.volume:8.1f} mm3"
          f"  ->  hip_yoke_v6 {p.volume:8.1f} mm3  (+{p.volume - r.volume - q.volume:.1f} = the 4 filled bolt columns)")
    print(f"mass    PETG {D.FILAMENT_RHO * 1e3:.2f} g/cm3 x {D.PRINT_MASS_FACTOR} print factor: "
          f"{mass_g(r) + mass_g(q):.2f} g -> {mass_g(p):.2f} g per hip, plus the removed hardware "
          f"{REMOVED_FASTENERS}")
    findings = audit(stl_path)
    print("printability:", "PASS" if not findings else f"{len(findings)} finding(s) above")
    print("wrote", render(stl_path, os.path.join(OUT_REN, "hip_yoke_v6.png")))
