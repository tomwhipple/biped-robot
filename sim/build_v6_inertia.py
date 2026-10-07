"""CAD-true inertials for the v6 plant: per-body <inertial> from the printed
parts' STLs (cad/v6/stl, PETG density x print factor), the servos from their
case mocks at their CAD places, and the pack, boards and bearings at their real
places (cad/v6/dimensions_v6.py), in the body frames sim/gen_plant_v6.py uses.
Prints the per-body table and, with --write, writes sim/bimo_biped_v6ar.xml --
the committed plant, which is THE ROBOT AS DRAWN:

  * 17 joints and 17 position actuators: 6 per leg, the neck, and per arm the
    shoulder pitch and the elbow. The arms are the generator's as-drawn arm
    (the get-up plant r6_asdrawn: girdle-mounted shoulder servos, CAD servo
    placement, the elbow servo on the upper arm), with every arm number taken
    from dimensions_v6 (ARM_*), and
    the arms' rest pose (qpos0) FOLDED, ARM_REST = shoulder -15 / elbow -95
    (gen_plant_v6 DesignParams.arm_hold / arm_elbow_hold: the joint angles still read
    0 = hanging).
  * the robot's servos (DESIGN.md section 4): an STS3250 (74.5 g) at each
    hip roll, an STS3215 (55 g) everywhere else. --servo-plan B is 17 x
    STS3215; 3250 puts STS3250s at the six roll + knee joints.
  * the one-print hip yoke (hip_yoke_v6), the girdle, the neck floor, the
    arm links, the bearing housing, and the two hip-yaw bearings (6810-2RS)
    split between the housing and the carriers.

    .venv/bin/python sim/build_v6_inertia.py                      # print the per-body table
    .venv/bin/python sim/build_v6_inertia.py --write              # regenerate sim/bimo_biped_v6ar.xml (+ CAD meshes, CAD-fitted collision)
    .venv/bin/python sim/build_v6_inertia.py --armless -o /tmp/armless.xml --write

Body frames (torso origin = the hip yaw axis on the centreline, at the yaw horn
face; leg and arm bodies at their joint axes; see gen_plant_v6): each mass
lands in the body whose joint moves it.
    torso      pelvis, bearing housing, yaw servos, pack, boards, wiring, neck
               servo, neck floor, girdle + both shoulder servos (or
               neck_collar, armless), the bearings' outer halves
    head       head (shell + face fused) + camera
    _hip_yaw   yaw carrier + hip-roll servo + the bearing's inner half
    _hip       hip_yoke_v6
    _thigh / _shin / _ankle_blk   leg_link_v6 / leg_link_v6 / ankle_link + their servo
    _foot      foot + silicone sole + the ankle-roll servo
    _arm       arm_upper_v6 + the elbow servo (its case runs up the upper arm)
    _forearm   arm_fore_v6

For studies, plant_xml(p, m3250_g) returns the same plant for a DesignParams
p (arms added unless arms=False) with m3250_g grams at all six roll and knee
places (V.SERVO_3250_JOINTS_SIX), whatever the plan.
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
# the sole is a 2 mm silicone sheet cut to the outline (not printed): solid,
# so no print factor; 1.21 g/cm^3 stands until a cut sole is weighed (silicone 1.1-1.2)
RHO_SOLE = 1.21e-3 * 1e-3


def asdrawn(p: DesignParams | None = None, rest: tuple = V.ARM_REST) -> DesignParams:
    """p with the robot's arms as drawn (the r6_asdrawn get-up plant's arm
    options), every number from dimensions_v6, at rest = (shoulder, elbow) deg."""
    p = p or DesignParams()
    y_default = p.deck_w / 2 + SV_T / 2 + 0.004          # the generator's torso-hugging arm plane
    return dataclasses.replace(
        p, arms=True, arm_elbow=True, arm_cad_servos=True, arm_elbow_servo_upper=True, arm_girdle=True,
        arm_len=V.ARM_UPPER / 1e3, arm_fore_len=V.ARM_FORE / 1e3,
        arm_shoulder_x=V.ARM_SHOULDER_X / 1e3, arm_z=V.ARM_SHOULDER_ABOVE_YAW / 1e3,
        arm_shoulder_y_extra=round(V.ARM_Y / 1e3 - y_default, 6),
        arm_hold=rest[0], arm_elbow_hold=rest[1])


# ------------------------------------------------------------------ mass items
def mesh_props(path, offset_mm, rho=RHO, mirror_y=False, mass=None):
    """(mass kg, com m, I about the com kg m^2) of an STL placed at offset_mm;
    `mass` (kg) overrides the density (a servo's catalogue mass on its mock)."""
    m = trimesh.load(path, force="mesh")
    if mirror_y:
        m.apply_scale([1, -1, 1])
        m.fix_normals()
    m.apply_translation(offset_mm)
    # Several exported STLs are not strictly watertight (tessellation slivers
    # at fillets); trimesh's fill_holes leaves every one's volume unchanged
    # (checked 2026-09-29) and needs networkx, so it is not called: the plant
    # comes out byte-identical on any box.
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
        "yaw_L": lambda: Pos(0, V.HIP_SEP / 2, V.D.SV_HORN_FACE) * CA.servo_mock_z(idler_disc=False),
        "yaw_R": lambda: Pos(0, -V.HIP_SEP / 2, V.D.SV_HORN_FACE) * CA.servo_mock_z(idler_disc=False),
        "neck": lambda: Pos(V.NECK_X, 0, zdeck + V.NECK_AXIS_Z) * Rot(180, 0, 0) * CA.servo_mock_z(),
        "shoulder_L": lambda: Pos(0, 0, zdeck) * SG.shoulder_servo_mock("L"),
        "shoulder_R": lambda: Pos(0, 0, zdeck) * SG.shoulder_servo_mock("R"),
        "roll": lambda: Pos(0, 0, V.HIP_ROLL_Z - V.HIP_YAW_Z) * CA.servo_mock_x(),
        "pitch": lambda: CA.servo_mock_y(),
        "foot_L": lambda: _foot_roll_mock("L"),
        "foot_R": lambda: _foot_roll_mock("R"),
        "elbow_L": lambda: Pos(0, 0, -V.ARM_UPPER) * arm_v6.elbow_servo_mock("L"),
        "elbow_R": lambda: Pos(0, 0, -V.ARM_UPPER) * arm_v6.elbow_servo_mock("R"),
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


SERVO_PLANS = {"hips": V.SERVO_3250_JOINTS, "B": (), "3250": V.SERVO_3250_JOINTS_SIX}


def servo_grams(joint, plan="hips", m3250_g=None):
    """the servo mass at a roll or knee joint: m3250_g at all six places when
    given (studies), else the plan's STS3250 or STS3215 mass."""
    if m3250_g is not None:
        return m3250_g if joint in V.SERVO_3250_JOINTS_SIX else V.SERVO_MASS_3215
    return V.SERVO_MASS_3250 if joint in SERVO_PLANS[plan] else V.SERVO_MASS_3215


def bodies(p: DesignParams, m3250_g: float | None = None, plan: str = "hips"):
    """dict body -> list of (mass kg, com m, I) in the BODY frame.
    plan: the servo set (SERVO_PLANS); m3250_g overrides it with that mass at
    all six roll and knee places."""
    g15 = V.SERVO_MASS_3215
    g_roll, g_knee, g_aroll = (servo_grams(j, plan, m3250_g) for j in ("hip_roll", "knee", "ankle_roll"))
    zdeck = V.DECK_TOP_Z - V.HIP_YAW_Z          # pelvis-local z = 0 is the deck top
    pelvis, carrier = "pelvis_v7" + ("" if p.arms else "_armless"), "yaw_carrier_v6"
    brg_kg = V.YAW_BRG_MASS_G * 1e-3
    ri, ro = V.YAW_BRG_ID / 2, V.YAW_BRG_OD / 2
    bz0, bz1 = V.YAW_BRG_BAND_Z
    for n in (pelvis, carrier, "yaw_bearing_housing"):
        if not have(n):
            raise FileNotFoundError(f"{stl(n)} missing -- export it: "
                                    f"{'' if p.arms else 'ARMS=0 '}python cad/v6/parts_v6.py --only {n}")
    out = {}
    # ---- torso
    t = [mesh_props(stl(pelvis), [0, 0, zdeck]), mesh_props(stl("yaw_bearing_housing"), [0, 0, zdeck]),
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
    for sgn in (1, -1):   # the bearing's outer half rides the housing, on the torso
        t.append(ring_props(brg_kg / 2, (ri + ro) / 2, ro, bz0, bz1, 0, sgn * V.HIP_SEP / 2))
    out["torso"] = t
    # ---- head (frame: the neck horn face)
    out["head"] = [mesh_props(stl("head"), [0, 0, 0]),
                   box_props(V.CAM3_MASS * 1e-3, [V.HEAD_D / 2 - 5, 0, V.HEAD_BASE_T + V.CAM_Z_ABOVE_HORN], [2, 25, 24])]
    # ---- legs (the same parts both sides; the feet are a mirrored pair)
    for side in ("L", "R"):
        hy = [mesh_props(stl(carrier), [0, 0, 0]), servo("roll", g_roll),
              ring_props(brg_kg / 2, ri, (ri + ro) / 2, bz0, bz1)]      # the bearing's inner half turns with the leg
        out[f"{side}_hip_yaw"] = hy
        out[f"{side}_hip"] = [mesh_props(stl("hip_yoke_v6"), [0, 0, 0])]
        out[f"{side}_thigh"] = [mesh_props(stl("leg_link_v6"), [0, 0, 0]), servo("pitch", g15)]
        out[f"{side}_shin"] = [mesh_props(stl("leg_link_v6"), [0, 0, 0]), servo("pitch", g_knee)]
        out[f"{side}_ankle_blk"] = [mesh_props(stl("ankle_link"), [0, 0, 0]), servo("pitch", g15)]
        z_foot = -V.ANKLE_ROLL_Z + V.TPU_SOLE_T
        out[f"{side}_foot"] = [mesh_props(stl(f"foot_{side}"), [0, 0, z_foot]),
                               mesh_props(stl(f"sole_tpu_{side}"), [0, 0, z_foot], rho=RHO_SOLE),
                               servo(f"foot_{side}", g_aroll)]
        if p.arms:
            out[f"{side}_arm"] = [mesh_props(stl(f"arm_upper_v6_{side}"), [0, 0, 0]), servo(f"elbow_{side}", g15)]
            out[f"{side}_forearm"] = [mesh_props(stl(f"arm_fore_v6_{side}"), [0, 0, 0])]
    return out


def inertial_xml(name, items):
    M, com, I, w, q = combine(items)
    return (f'<inertial pos="{com[0]:.4f} {com[1]:.4f} {com[2]:.4f}" quat="{q[0]:.4f} {q[1]:.4f} {q[2]:.4f} {q[3]:.4f}" '
            f'mass="{M:.4f}" diaginertia="{w[0]:.4e} {w[1]:.4e} {w[2]:.4e}"/>'), M, com


def plant_xml(p: DesignParams, m3250_g: float | None = None, verbose: bool = False,
              arms: bool = True, plan: str = "hips"):
    """the generator's MJCF with the CAD-true <inertial> blocks patched in.
    arms=True adds the as-drawn arms (asdrawn()) unless p already has them.
    Returns (xml text, total mass kg)."""
    import re
    if arms and not p.arms:
        p = asdrawn(p)
    B = bodies(p, m3250_g, plan)
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
    servos = (f"{m3250_g:g} g servos at the six roll and knee places" if m3250_g is not None else
              {"hips": "STS3250 at the hip rolls, STS3215 elsewhere", "B": "STS3215 everywhere",
               "3250": "STS3250 at the rolls and knees"}[plan])
    tag = f"CAD-true inertials, {servos}, hip-yaw bearings 6810-2RS"
    return src.replace("GENERATED by sim/gen_plant_v6.py", f"GENERATED by sim/gen_plant_v6.py + build_v6_inertia.py ({tag})"), total


def dress(src, out_path, arms=True, collision=True):
    """src with the CAD visual meshes (sim/add_cad_meshes.py), for a plant
    written to out_path (mesh paths are relative to it), and with collision=True
    its collision primitives fitted to the same CAD parts (sim/fit_cad_collision.py)."""
    import add_cad_meshes
    src = add_cad_meshes.dress(src, os.path.dirname(os.path.abspath(out_path)), arms=arms)
    if collision:
        import fit_cad_collision
        src = fit_cad_collision.fit(src, arms=arms)
    return src


def committed_xml(path=os.path.join(HERE, "bimo_biped_v6ar.xml")):
    """exactly what `build_v6_inertia.py --write` writes to sim/bimo_biped_v6ar.xml."""
    src, _ = plant_xml(DesignParams(), None)
    return dress(src, path)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--servo-plan", choices=tuple(SERVO_PLANS), default="hips",
                    help="hips: STS3250 (74.5 g) at the hip rolls; B: STS3215 (55 g) everywhere; "
                         "3250: STS3250 at the hip roll / knee / ankle roll")
    ap.add_argument("--all-3215", action="store_true", help="the same as --servo-plan B")
    ap.add_argument("--armless", action="store_true", help="the ARMS=0 variant (13 actuators)")
    ap.add_argument("--no-meshes", action="store_true",
                    help="leave out the CAD visual meshes (sim/add_cad_meshes.py; no physics either way)")
    ap.add_argument("--no-cad-collision", action="store_true",
                    help="keep the generator's collision primitives instead of fitting them to the CAD")
    ap.add_argument("-o", "--out", default=os.path.join(HERE, "bimo_biped_v6ar.xml"))
    a = ap.parse_args(argv)
    p = DesignParams()
    plan = "B" if a.all_3215 else a.servo_plan
    src, total = plant_xml(p, None, verbose=True, arms=not a.armless, plan=plan)
    if a.write and not a.no_meshes:
        src = dress(src, a.out, arms=not a.armless, collision=not a.no_cad_collision)
    if a.write:
        with open(a.out, "w") as f:
            f.write(src)
        import mujoco
        m = mujoco.MjModel.from_xml_path(a.out)
        print(f"wrote {a.out}: nu {m.nu}, nq {m.nq}, total mass {sum(m.body_mass):.3f} kg")


if __name__ == "__main__":
    main()
