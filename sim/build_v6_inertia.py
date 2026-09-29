"""CAD-true inertials for the v6 plant: per-body <inertial> from the printed
parts' STLs (cad/v6/stl, PETG density x print factor), the servos from their
case mocks at their CAD places, and the pack, boards and bearings at their real
places (cad/v6/dimensions_v6.py), in the body frames sim/gen_plant_v6.py uses.
Prints the per-body table and, with --write, writes sim/bimo_biped_v6ar.xml --
the committed plant, which is THE ROBOT AS DRAWN:

  * 17 joints and 17 position actuators: 6 per leg, the neck, and per arm the
    shoulder pitch and the elbow. The arms are the generator's as-drawn arm
    (the get-up plant r5_asdrawn: girdle-mounted shoulder servos, CAD servo
    placement), with every arm number taken from dimensions_v6 (ARM_*), and
    the shoulders' rest pose (qpos0) at the WALKING HOLD, ARM_WALK_HOLD = 15
    deg back (gen_plant_v6 DesignParams.arm_hold: the joint angle still reads
    0 = hanging).
  * Plan B: an STS3215 (55 g) at every joint. --servo-plan 3250 puts the
    STS3250's 74.5 g at the six roll + knee joints (the fallback).
  * the one-print hip yoke (hip_yoke_v6), the girdle, the neck floor, the
    arm links, and the hip-yaw bearing variant's pelvis, carriers and bearing
    (--bearing, default YAW_BEARING_VARIANT or C, a placeholder until #75).

    .venv/bin/python sim/build_v6_inertia.py                      # print the per-body table
    .venv/bin/python sim/build_v6_inertia.py --write              # regenerate sim/bimo_biped_v6ar.xml
    .venv/bin/python sim/build_v6_inertia.py --bearing E          # the table for option E
    .venv/bin/python sim/build_v6_inertia.py --armless -o /tmp/armless.xml --write

Body frames (torso origin = the hip yaw axis on the centreline, at the yaw horn
face; leg and arm bodies at their joint axes; see gen_plant_v6): each mass
lands in the body whose joint moves it.
    torso      pelvis, yaw servos, pack, boards, wiring, neck servo, neck floor,
               girdle + both shoulder servos (or neck_collar, armless), the
               bearings' outer halves (+ option E's retainers)
    head       head (shell + face fused) + camera
    _hip_yaw   yaw carrier + hip-roll servo + the bearing's inner half (+ E's cap)
    _hip       hip_yoke_v6
    _thigh / _shin / _ankle_blk   leg_link_v6 / leg_link_v6 / ankle_link + their servo
    _foot      foot + TPU sole + the ankle-roll servo
    _arm       arm_upper_v6
    _forearm   arm_fore_v6 + the elbow servo

For studies, plant_xml(p, m3250_g) returns the same plant for a DesignParams
p (arms added unless arms=False) with m3250_g grams at the six STS3250 places.
"""
from __future__ import annotations

import argparse
import dataclasses
import functools
import os
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "cad", "v6"))
sys.path.insert(0, os.path.join(HERE, "..", "cad"))

import trimesh  # noqa: E402
import dimensions_v6 as V  # noqa: E402
from gen_plant_v6 import DesignParams, build_xml, SV_T  # noqa: E402

STL = os.path.join(HERE, "..", "cad", "v6", "stl")
RHO = V.D.FILAMENT_RHO * V.D.PRINT_MASS_FACTOR * 1e-3   # kg/mm^3
RHO_TPU = 1.21e-3 * V.D.PRINT_MASS_FACTOR * 1e-3
BEARINGS = ("A", "C", "E")


def asdrawn(p: DesignParams | None = None, hold: float = V.ARM_WALK_HOLD) -> DesignParams:
    """p with the robot's arms as drawn (the r5_asdrawn get-up plant's arm
    options), every number from dimensions_v6, held `hold` deg back."""
    p = p or DesignParams()
    y_default = p.deck_w / 2 + SV_T / 2 + 0.004          # the generator's torso-hugging arm plane
    return dataclasses.replace(
        p, arms=True, arm_elbow=True, arm_cad_servos=True, arm_girdle=True,
        arm_len=V.ARM_UPPER / 1e3, arm_fore_len=V.ARM_FORE / 1e3,
        arm_shoulder_x=V.ARM_SHOULDER_X / 1e3, arm_z=V.ARM_SHOULDER_ABOVE_YAW / 1e3,
        arm_shoulder_y_extra=round(V.ARM_Y / 1e3 - y_default, 6), arm_hold=hold)


# ------------------------------------------------------------------ mass items
def mesh_props(path, offset_mm, rho=RHO, mirror_y=False, mass=None):
    """(mass kg, com m, I about the com kg m^2) of an STL placed at offset_mm;
    `mass` (kg) overrides the density (a servo's catalogue mass on its mock)."""
    m = trimesh.load(path, force="mesh")
    if mirror_y:
        m.apply_scale([1, -1, 1])
        m.fix_normals()
    m.apply_translation(offset_mm)
    if not m.is_watertight:
        m.fill_holes()
    vol = abs(m.volume)
    r = rho if mass is None else mass / vol
    com = m.center_mass * 1e-3
    I = m.moment_inertia * r * 1e-6      # mm^5 * kg/mm^3 -> kg mm^2 -> kg m^2
    return vol * r, com, I


def box_props(mass, center_mm, size_mm):
    c = np.array(center_mm) * 1e-3
    s = np.array(size_mm) * 1e-3   # full sizes
    I = mass / 12.0 * np.diag([s[1] ** 2 + s[2] ** 2, s[0] ** 2 + s[2] ** 2, s[0] ** 2 + s[1] ** 2])
    return mass, c, I


def ring_props(mass, r0_mm, r1_mm, z0_mm, z1_mm, x_mm=0.0, y_mm=0.0):
    """a hollow cylinder about a vertical axis (a bearing ring)."""
    r0, r1, h = r0_mm * 1e-3, r1_mm * 1e-3, (z1_mm - z0_mm) * 1e-3
    izz = mass / 2 * (r0 ** 2 + r1 ** 2)
    ixx = mass / 12 * (3 * (r0 ** 2 + r1 ** 2) + h ** 2)
    return mass, np.array([x_mm, y_mm, (z0_mm + z1_mm) / 2]) * 1e-3, np.diag([ixx, ixx, izz])


def combine(items):
    """items: [(mass, com, I_about_com)] -> (mass, com, I_about_com, diag, quat)"""
    M = sum(m for m, _, _ in items)
    com = sum(m * c for m, c, _ in items) / M
    I = np.zeros((3, 3))
    for m, c, Ic in items:
        d = c - com
        I += Ic + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    w, v = np.linalg.eigh(I)
    if np.linalg.det(v) < 0:
        v[:, 0] *= -1
    q = trimesh.transformations.quaternion_from_matrix(np.vstack([np.hstack([v, [[0], [0], [0]]]), [0, 0, 0, 1]]))
    return M, com, I, w, q


@functools.lru_cache(maxsize=None)
def _mock_stl(key):
    """STL (temp file) of a servo case mock placed in its body frame, built
    from the same mocks and placements cad/v6/assembly_v6.py uses."""
    from build123d import Pos, Rot, export_stl
    import check_assembly as CA
    import shoulder_girdle_v6 as SG
    import arm_v6
    zdeck = V.DECK_TOP_Z - V.HIP_YAW_Z
    solids = {
        "yaw_L": lambda: Pos(0, V.HIP_SEP / 2, V.D.SV_HORN_FACE) * CA.servo_mock_z(),
        "yaw_R": lambda: Pos(0, -V.HIP_SEP / 2, V.D.SV_HORN_FACE) * CA.servo_mock_z(),
        "neck": lambda: Pos(V.NECK_X, 0, zdeck + V.NECK_AXIS_Z) * Rot(180, 0, 0) * CA.servo_mock_z(),
        "shoulder_L": lambda: Pos(0, 0, zdeck) * SG.shoulder_servo_mock("L"),
        "shoulder_R": lambda: Pos(0, 0, zdeck) * SG.shoulder_servo_mock("R"),
        "roll": lambda: Pos(0, 0, V.HIP_ROLL_Z - V.HIP_YAW_Z) * CA.servo_mock_x(),
        "pitch": lambda: CA.servo_mock_y(),
        "foot_L": lambda: _foot_roll_mock("L"),
        "foot_R": lambda: _foot_roll_mock("R"),
        "elbow_L": lambda: arm_v6.elbow_servo_mock("L"),
        "elbow_R": lambda: arm_v6.elbow_servo_mock("R"),
    }
    path = os.path.join(tempfile.gettempdir(), f"v6inertia_{os.getpid()}_{key}.stl")
    export_stl(solids[key](), path)
    return path


def _foot_roll_mock(side):
    import assembly_v6 as A
    return A.mock_roll_servo_in_foot(side)


def servo(key, grams):
    return mesh_props(_mock_stl(key), [0, 0, 0], mass=grams * 1e-3)


def stl(name):
    return os.path.join(STL, name + ".stl")


def have(name):
    return os.path.exists(stl(name))


def bearing_files(bearing, arms=True):
    """(pelvis STL, carrier STL, bearing kg, ring radii mm (id/2, od/2), ring
    z band mm below the yaw horn face) for a bearing variant -- what
    cad/v6/parts_v6.py exports for it."""
    suffix = "" if arms else "_armless"
    if bearing == "C":
        return (f"pelvis_v7{suffix}", "yaw_carrier_v6", V.YAW_BRG_MASS_G * 1e-3,
                (V.YAW_BRG_ID / 2, V.YAW_BRG_OD / 2), (-V.YAW_BRG_W, 0.0))
    import yaw_retention_optE as E
    band = (V.YAWA_BAND_Z[0], V.YAWA_BAND_Z[1]) if bearing == "A" else E.E_RACE_Z
    return (f"pelvis_v7_opt{bearing}{suffix}", f"yaw_carrier_v6_opt{bearing}", V.YAWA_BRG_MASS_G * 1e-3,
            (V.YAWA_BRG_ID / 2, V.YAWA_BRG_OD / 2), band)


def bodies(p: DesignParams, m3250_g: float | None = None, bearing: str = "C"):
    """dict body -> list of (mass kg, com m, I) in the BODY frame.
    m3250_g: servo mass at the SERVO_3250_JOINTS places (hip roll, knee,
    ankle roll); None = Plan B, an STS3215 (55 g) there too."""
    g15 = V.SERVO_MASS_3215
    g50 = V.SERVO_MASS_3215 if m3250_g is None else m3250_g
    zdeck = V.DECK_TOP_Z - V.HIP_YAW_Z          # pelvis-local z = 0 is the deck top
    pelvis, carrier, brg_kg, (ri, ro), (bz0, bz1) = bearing_files(bearing, p.arms)
    for n in (pelvis, carrier):
        if not have(n):
            raise FileNotFoundError(f"{stl(n)} missing -- export it: "
                                    f"YAW_BEARING_VARIANT={bearing}{'' if p.arms else ' ARMS=0'} "
                                    f"python cad/v6/parts_v6.py --only {n}")
    out = {}
    # ---- torso
    t = [mesh_props(stl(pelvis), [0, 0, zdeck]),
         servo("yaw_L", g15), servo("yaw_R", g15), servo("neck", g15),
         mesh_props(stl("neck_floor"), [0, 0, zdeck])]
    t.append(box_props(V.BATT_MASS * 1e-3, [(V.BATT_X[0] + V.BATT_X[1]) / 2, 0, zdeck + (V.BATT_Z[0] + V.BATT_Z[1]) / 2], [V.BATT[1], V.BATT[0], V.BATT[2]]))
    t.append(box_props(V.PI4_MASS * 1e-3, [(V.PI_PCB_X0 + V.PI_COMP_X) / 2, 0, zdeck + V.PI_CZ], [V.PI_COMP_TOP + V.PI_PCB_T, V.PI_OUTLINE[0], V.PI_OUTLINE[1]]))
    t.append(box_props(V.GD_MASS * 1e-3, [(V.GD_PCB_X0 + V.GD_COMP_X) / 2, 0, zdeck + (V.GD_BOT_Z + V.GD_TOP_Z) / 2], [V.D.BOARD_GD_COMP + V.D.BR_PCB_T, V.D.BOARD_GD_OUTLINE[0], V.D.BOARD_GD_OUTLINE[1]]))
    t.append(box_props(V.PWR_MASS * 1e-3, [(V.PWR_X[0] + V.PWR_X[1]) / 2, 0, zdeck + (V.PWR_Z[0] + V.PWR_Z[1]) / 2], [V.PWR_BOARD[1], V.PWR_BOARD[0], V.PWR_BOARD[2]]))
    t.append(box_props(V.WIRING_MASS * 1e-3, [-10, 0, zdeck - 20], [60, 80, 20]))
    if p.arms:
        t += [mesh_props(stl("shoulder_girdle_v6"), [0, 0, zdeck]),
              servo("shoulder_L", g15), servo("shoulder_R", g15)]
    else:
        t.append(mesh_props(stl("neck_collar"), [0, 0, zdeck]))
    for sgn in (1, -1):   # the bearing's outer half rides the pelvis
        t.append(ring_props(brg_kg / 2, (ri + ro) / 2, ro, bz0, bz1, 0, sgn * V.HIP_SEP / 2))
        if bearing == "E":
            t.append(mesh_props(stl("yaw_retainer_optE"), [0, 0, zdeck], mirror_y=sgn < 0))
    out["torso"] = t
    # ---- head (frame: the neck horn face)
    out["head"] = [mesh_props(stl("head"), [0, 0, 0]),
                   box_props(V.CAM3_MASS * 1e-3, [V.HEAD_D / 2 - 5, 0, V.HEAD_BASE_T + V.CAM_Z_ABOVE_HORN], [2, 25, 24])]
    # ---- legs (the same parts both sides; the feet are a mirrored pair)
    for side in ("L", "R"):
        hy = [mesh_props(stl(carrier), [0, 0, 0]), servo("roll", g50),
              ring_props(brg_kg / 2, ri, (ri + ro) / 2, bz0, bz1)]      # the bearing's inner half turns with the leg
        if bearing == "E":
            hy.append(mesh_props(stl("yaw_cap_optE"), [0, 0, 0]))
        out[f"{side}_hip_yaw"] = hy
        out[f"{side}_hip"] = [mesh_props(stl("hip_yoke_v6"), [0, 0, 0])]
        out[f"{side}_thigh"] = [mesh_props(stl("leg_link_v6"), [0, 0, 0]), servo("pitch", g15)]
        out[f"{side}_shin"] = [mesh_props(stl("leg_link_v6"), [0, 0, 0]), servo("pitch", g50)]
        out[f"{side}_ankle_blk"] = [mesh_props(stl("ankle_link"), [0, 0, 0]), servo("pitch", g15)]
        z_foot = -V.ANKLE_ROLL_Z + V.TPU_SOLE_T
        out[f"{side}_foot"] = [mesh_props(stl(f"foot_{side}"), [0, 0, z_foot]),
                               mesh_props(stl(f"sole_tpu_{side}"), [0, 0, z_foot], rho=RHO_TPU),
                               servo(f"foot_{side}", g50)]
        if p.arms:
            out[f"{side}_arm"] = [mesh_props(stl(f"arm_upper_v6_{side}"), [0, 0, 0])]
            out[f"{side}_forearm"] = [mesh_props(stl(f"arm_fore_v6_{side}"), [0, 0, 0]), servo(f"elbow_{side}", g15)]
    return out


def inertial_xml(name, items):
    M, com, I, w, q = combine(items)
    return (f'<inertial pos="{com[0]:.4f} {com[1]:.4f} {com[2]:.4f}" quat="{q[0]:.4f} {q[1]:.4f} {q[2]:.4f} {q[3]:.4f}" '
            f'mass="{M:.4f}" diaginertia="{w[0]:.4e} {w[1]:.4e} {w[2]:.4e}"/>'), M, com


def plant_xml(p: DesignParams, m3250_g: float | None = None, verbose: bool = False,
              arms: bool = True, bearing: str | None = None):
    """the generator's MJCF with the CAD-true <inertial> blocks patched in.
    arms=True adds the as-drawn arms (asdrawn()) unless p already has them.
    Returns (xml text, total mass kg)."""
    import re
    bearing = bearing or os.environ.get("YAW_BEARING_VARIANT", "C")
    if arms and not p.arms:
        p = asdrawn(p)
    B = bodies(p, m3250_g, bearing)
    total = 0.0
    blocks = {}
    if verbose:
        print(f"{'body':14s} {'mass g':>7s}  com (mm)")
    for name, items in B.items():
        xml, M, com = inertial_xml(name, items)
        blocks[name] = xml
        total += M
        if verbose:
            print(f"{name:14s} {1e3*M:7.1f}  ({1e3*com[0]:+6.1f}, {1e3*com[1]:+6.1f}, {1e3*com[2]:+6.1f})")
    if verbose:
        print(f"{'TOTAL':14s} {1e3*total:7.1f}")
    src = build_xml(p)
    for name, xml in blocks.items():
        # insert the inertial right after the body's first joint/freejoint line
        pat = re.compile(rf'(<body name="{name}"[^>]*>\n\s*<(?:freejoint|joint)[^>]*/>[^\n]*\n)')
        src, n = pat.subn(lambda mo: mo.group(1) + "        " + xml + "\n", src, count=1)
        if n == 0:
            raise RuntimeError(f"could not place the inertial for body {name!r}")
    servos = "Plan B, STS3215 everywhere" if m3250_g is None else f"{m3250_g:g} g servos at the STS3250 places"
    tag = f"CAD-true inertials, {servos}, hip-yaw bearing {bearing}"
    return src.replace("GENERATED by sim/gen_plant_v6.py", f"GENERATED by sim/gen_plant_v6.py + build_v6_inertia.py ({tag})"), total


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--servo-plan", choices=("B", "3250"), default="B",
                    help="B: STS3215 (55 g) everywhere; 3250: 74.5 g at the hip roll / knee / ankle roll")
    ap.add_argument("--all-3215", action="store_true", help="the same as --servo-plan B (the default)")
    ap.add_argument("--bearing", choices=BEARINGS, default=os.environ.get("YAW_BEARING_VARIANT", "C"))
    ap.add_argument("--armless", action="store_true", help="the ARMS=0 variant (13 actuators)")
    ap.add_argument("-o", "--out", default=os.path.join(HERE, "bimo_biped_v6ar.xml"))
    a = ap.parse_args(argv)
    p = DesignParams()
    m3250 = V.SERVO_MASS_3250 if a.servo_plan == "3250" else None
    src, total = plant_xml(p, m3250, verbose=True, arms=not a.armless, bearing=a.bearing)
    if a.write:
        with open(a.out, "w") as f:
            f.write(src)
        import mujoco
        m = mujoco.MjModel.from_xml_path(a.out)
        print(f"wrote {a.out}: nu {m.nu}, nq {m.nq}, total mass {sum(m.body_mass):.3f} kg")


if __name__ == "__main__":
    main()
