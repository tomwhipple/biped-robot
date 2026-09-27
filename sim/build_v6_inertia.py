"""CAD-true inertials for the v6 plant: per-body <inertial> from the printed
parts' STLs (cad/v6/stl, PETG density x print factor) plus the servos, pack
and boards as boxes at their real places (cad/v6/dimensions_v6.py), in the
same body frames sim/gen_plant_v6.py uses. Prints the blocks and, with
--write, patches them into sim/bimo_biped_v6ar.xml (replacing the geom-derived
inertia the generator ships with). Same role as sim/build_v2_inertia.py for v5.

    .venv/bin/python sim/build_v6_inertia.py            # print the per-body table
    .venv/bin/python sim/build_v6_inertia.py --write    # regenerate the plant with <inertial> blocks
    .venv/bin/python sim/build_v6_inertia.py --all-3215 --write -o /tmp/v6_3215.xml
        # the same plant with an STS3215 (55 g) wherever the design puts an
        # STS3250 (74.5 g) -- the no-STS3250 study (design doc section 14);
        # never overwrites the committed plant unless -o points at it

Body frames (torso origin = hip yaw axis on the centreline; leg bodies at
their joint axes; see gen_plant_v6._leg): masses land in the body whose
joint moves them. Printed part -> body: pelvis_v7, head -> torso / head;
yaw_carrier -> {L,R}_hip_yaw (+ the hip roll servo); yoke_roll + yoke_pitch
-> _hip; leg_link_v6 -> _thigh and _shin (+ the pitch / knee servo);
ankle_link -> _ankle_blk (+ the ankle pitch servo); foot + sole -> _foot
(+ the roll servo).
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "cad", "v6"))
sys.path.insert(0, os.path.join(HERE, "..", "cad"))

import trimesh  # noqa: E402
import dimensions_v6 as V  # noqa: E402
from gen_plant_v6 import DesignParams, build_xml, SV_LEN, SV_WID, SV_T, SV_AX_OUT  # noqa: E402

STL = os.path.join(HERE, "..", "cad", "v6", "stl")
STL_V5 = os.path.join(HERE, "..", "cad", "stl")
RHO = V.D.FILAMENT_RHO * V.D.PRINT_MASS_FACTOR * 1e-3   # kg/mm^3
RHO_TPU = 1.21e-3 * V.D.PRINT_MASS_FACTOR * 1e-3


def mesh_props(path, offset_mm, rho=RHO, mirror_y=False):
    m = trimesh.load(path, force="mesh")
    if mirror_y:
        m.apply_scale([1, -1, 1])
        m.fix_normals()
    m.apply_translation(offset_mm)
    if not m.is_watertight:
        m.fill_holes()
    vol = abs(m.volume)
    mass = vol * rho
    com = m.center_mass * 1e-3
    # inertia about the COM, in kg m^2 (trimesh gives volume-based, unit density, mm^5)
    I = m.moment_inertia * rho * 1e-6   # mm^5 * kg/mm^3 = kg mm^2 -> *1e-6 m^2
    return mass, com, I


def box_props(mass, center_mm, size_mm):
    c = np.array(center_mm) * 1e-3
    s = np.array(size_mm) * 1e-3   # full sizes
    I = mass / 12.0 * np.diag([s[1] ** 2 + s[2] ** 2, s[0] ** 2 + s[2] ** 2, s[0] ** 2 + s[1] ** 2])
    return mass, c, I


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
    # rotation matrix v -> quaternion (w, x, y, z)
    q = trimesh.transformations.quaternion_from_matrix(np.vstack([np.hstack([v, [[0], [0], [0]]]), [0, 0, 0, 1]]))
    return M, com, I, w, q


def servo_box(mass, center_mm, axis):
    """servo case as a box: sizes by output-axis direction (gen_plant_v6 convention)"""
    if axis == "y":
        size = (SV_WID, SV_T, SV_LEN)
    elif axis == "x_v":
        size = (SV_T, SV_WID, SV_LEN)
    elif axis == "x_h":
        size = (SV_T, SV_LEN, SV_WID)
    else:
        size = (SV_LEN, SV_WID, SV_T)
    return box_props(mass, center_mm, [s * 1e3 for s in size])


def bodies(p: DesignParams, m3250_g: float | None = None):
    """dict body -> list of (mass kg, com m, I) in the BODY frame (mm inputs).
    m3250_g: servo mass at the SERVO_3250_JOINTS places (hip roll, knee,
    ankle roll); None = the STS3250's 74.5 g (the design), 55.0 = an STS3215
    there (the no-STS3250 study)."""
    m3215 = V.SERVO_MASS_3215 * 1e-3
    m3250 = (V.SERVO_MASS_3250 if m3250_g is None else m3250_g) * 1e-3
    zdn = -(SV_LEN / 2 - SV_AX_OUT) * 1e3          # hanging case centre, mm
    out = {}
    have = lambda n: os.path.exists(os.path.join(STL, n + ".stl"))
    # ---- torso: pelvis print at (0,0,DECK_TOP - torso origin) ... pelvis local z=0 is the deck top
    zdeck = (V.DECK_TOP_Z - V.HIP_YAW_Z)           # deck top above the yaw HORN face...
    # torso origin in gen_plant is the yaw AXIS level == the yaw horn face plane (HIP_YAW_Z) in CAD
    t = []
    if have("pelvis_v7"):
        t.append(mesh_props(os.path.join(STL, "pelvis_v7.stl"), [0, 0, zdeck]))
    else:
        t.append(box_props(V.SERVO_MASS_3215 * 0 + 0.20, [-16, 0, zdeck - 40], [99, 115, 80]))
    for sgn in (1, -1):   # yaw servos hang under the cells, horn down at the yaw axis
        t.append(servo_box(m3215, [-(SV_LEN / 2 - SV_AX_OUT) * 1e3, sgn * V.HIP_SEP / 2, V.D.SV_HORN_FACE + SV_T * 1e3 / 2], "z"))
    t.append(box_props(V.BATT_MASS * 1e-3, [(V.BATT_X[0] + V.BATT_X[1]) / 2, 0, zdeck + (V.BATT_Z[0] + V.BATT_Z[1]) / 2], [V.BATT[1], V.BATT[0], V.BATT[2]]))
    t.append(box_props(V.PI4_MASS * 1e-3, [(V.PI_PCB_X0 + V.PI_COMP_X) / 2, 0, zdeck + V.PI_CZ], [V.PI_COMP_TOP + V.PI_PCB_T, V.PI_OUTLINE[0], V.PI_OUTLINE[1]]))
    t.append(box_props(V.GD_MASS * 1e-3, [(V.GD_PCB_X0 + V.GD_COMP_X) / 2, 0, zdeck + (V.GD_BOT_Z + V.GD_TOP_Z) / 2], [V.D.BOARD_GD_COMP + V.D.BR_PCB_T, V.D.BOARD_GD_OUTLINE[0], V.D.BOARD_GD_OUTLINE[1]]))
    t.append(box_props(V.PWR_MASS * 1e-3, [(V.PWR_X[0] + V.PWR_X[1]) / 2, 0, zdeck + (V.PWR_Z[0] + V.PWR_Z[1]) / 2], [V.PWR_BOARD[1], V.PWR_BOARD[0], V.PWR_BOARD[2]]))
    t.append(box_props(V.WIRING_MASS * 1e-3, [-10, 0, zdeck - 20], [60, 80, 20]))
    t.append(servo_box(m3215, [-(SV_LEN / 2 - SV_AX_OUT) * 1e3, 0, zdeck + V.NECK_AXIS_Z], "z"))
    if have("neck_collar"):   # flange-down print, local z=0 = deck top, at the neck servo's x
        t.append(mesh_props(os.path.join(STL, "neck_collar.stl"), [V.NECK_X, 0, zdeck]))
    out["torso"] = t
    # ---- head (frame: neck horn face)
    h = []
    if have("head"):   # shell + lid + camera sled, fused, PETG
        h.append(mesh_props(os.path.join(STL, "head.stl"), [0, 0, 0]))
    else:
        h.append(box_props(V.HEAD_PETG_MASS * 1e-3, [(V.HEAD_X[0] + V.HEAD_X[1]) / 2, 0, V.HEAD_H / 2 + V.HEAD_BASE_T],
                           [V.HEAD_D, V.HEAD_W, V.HEAD_H]))
    # the stereo head's glass (masses from the squares' dimensions at 2.5
    # g/cm^3 -- the cheap front-surface squares have no datasheet) and the
    # camera, at their CAD centroids; the mirrors are 1.1 mm plates, so their
    # own inertia is taken as a plate across y-z (they stand at 30 deg to that
    # -- a small error next to the m*r^2 of sitting 51 mm off the neck axis)
    for c in (V.HEAD_VTILE_C_L, V.HEAD_VTILE_C_R):          # the V's two small squares
        h.append(box_props(V.HEAD_VTILE_MASS * 1e-3, c, [18.0, 18.0, 20.0]))
    for sy in (1, -1):
        c = (V.HEAD_MIRROR_C[0], sy * V.HEAD_MIRROR_C[1], V.HEAD_MIRROR_C[2])
        h.append(box_props(V.HEAD_MIRROR_MASS * 1e-3, c, [1.1, 50.0, 50.0]))
    h.append(box_props(V.CAM3_MASS * 1e-3, V.HEAD_CAM_C, [12.4, 25, 24]))
    out["head"] = h
    # ---- legs (same for L and R; foot mirrored)
    for side, sgn in (("L", 1), ("R", -1)):
        hy = []   # hip yaw body: carrier print + roll servo (axis X, vertical case in the bay)
        hy.append(mesh_props(os.path.join(STL_V5, "yaw_carrier.stl"), [0, 0, 0]))
        hy.append(servo_box(m3250, [0, 0, -V.D.CARRIER_ROLL_AXIS + (SV_LEN / 2 - SV_AX_OUT) * 1e3], "x_v"))
        out[f"{side}_hip_yaw"] = hy
        hp = []   # hip body: yokes
        hp.append(mesh_props(os.path.join(STL_V5, "yoke_roll.stl"), [0, 0, 0]))
        hp.append(mesh_props(os.path.join(STL, "yoke_pitch_v6.stl"), [0, 0, -V.ROLL_TO_PITCH]))   # v6 clevis (flange chamfer)
        out[f"{side}_hip"] = hp
        for body, servo_m in ((f"{side}_thigh", m3215), (f"{side}_shin", m3250)):
            b = []
            if have("leg_link_v6"):
                b.append(mesh_props(os.path.join(STL, "leg_link_v6.stl"), [0, 0, 0]))
            else:
                b.append(box_props(0.030, [0, 1, -V.LINK_DROP / 2], [30, 44, V.LINK_DROP + 20]))
            b.append(servo_box(servo_m, [0, 1.3, zdn], "y"))
            out[body] = b
        ab = []
        if have("ankle_link"):
            ab.append(mesh_props(os.path.join(STL, "ankle_link.stl"), [0, 0, 0]))
        else:
            ab.append(box_props(0.030, [0, 1, -V.AL_DROP / 2], [44, 44, V.AL_DROP + 20]))
        ab.append(servo_box(m3215, [0, 1.3, zdn], "y"))
        out[f"{side}_ankle_blk"] = ab
        ft = []
        if have(f"foot_{side}"):
            ft.append(mesh_props(os.path.join(STL, f"foot_{side}.stl"), [0, 0, -V.ANKLE_ROLL_Z + V.TPU_SOLE_T]))
            if have(f"sole_tpu_{side}"):
                ft.append(mesh_props(os.path.join(STL, f"sole_tpu_{side}.stl"), [0, 0, -V.ANKLE_ROLL_Z + V.TPU_SOLE_T], rho=RHO_TPU))
        else:
            ft.append(box_props(0.070, [V.FOOT_TOE - V.FOOT_L_LEN / 2, sgn * V.FOOT_Y_OFF, -V.ANKLE_ROLL_Z + 3], [V.FOOT_L_LEN, V.FOOT_W, 6]))
        ft.append(servo_box(m3250, [1.3, sgn * (SV_LEN / 2 - SV_AX_OUT) * 1e3, 0], "x_h"))
        out[f"{side}_foot"] = ft
    return out


def inertial_xml(name, items):
    M, com, I, w, q = combine(items)
    return (f'<inertial pos="{com[0]:.4f} {com[1]:.4f} {com[2]:.4f}" quat="{q[0]:.4f} {q[1]:.4f} {q[2]:.4f} {q[3]:.4f}" '
            f'mass="{M:.4f}" diaginertia="{w[0]:.4e} {w[1]:.4e} {w[2]:.4e}"/>'), M, com


def plant_xml(p: DesignParams, m3250_g: float | None = None, verbose: bool = False):
    """the generator's MJCF with the CAD-true <inertial> blocks patched in.
    Returns (xml text, total mass kg)."""
    import re
    B = bodies(p, m3250_g)
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
        pat = re.compile(rf'(<body name="{name}"[^>]*>\n\s*<(?:freejoint|joint)[^>]*/>\n)')
        src, n = pat.subn(lambda mo: mo.group(1) + "        " + xml + "\n", src, count=1)
        if n == 0:
            print("  ! could not place inertial for", name)
    tag = "CAD-true inertials" + ("" if m3250_g is None else f", {m3250_g:g} g servos at the STS3250 places")
    return src.replace("GENERATED by sim/gen_plant_v6.py", f"GENERATED by sim/gen_plant_v6.py + build_v6_inertia.py ({tag})"), total


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--all-3215", action="store_true",
                    help="STS3215 mass (55 g) at the hip roll / knee / ankle roll instead of the STS3250's 74.5 g")
    ap.add_argument("-o", "--out", default=os.path.join(HERE, "bimo_biped_v6ar.xml"))
    a = ap.parse_args(argv)
    p = DesignParams()
    src, _ = plant_xml(p, V.SERVO_MASS_3215 if a.all_3215 else None, verbose=True)
    if a.write:
        with open(a.out, "w") as f:
            f.write(src)
        print("wrote", a.out)


if __name__ == "__main__":
    main()
