"""Dress the v6 plant in its CAD parts: every printed part's STL (cad/v6/stl)
and every servo case mock, as visual-only mesh geoms in the body that carries
them, at the same body-frame placements sim/build_v6_inertia.py uses for the
inertials. The physics is untouched: the meshes have no mass and no contacts
(contype/conaffinity 0, group 2), and the primitive geoms that do the
colliding stay where they are, moved to group 3 so a viewer or a render shows
the robot as drawn (toggle group 3 to see the collision boxes).

    .venv/bin/python sim/build_v6_inertia.py --write             # the committed plant, dressed (calls dress())
    .venv/bin/python sim/add_cad_meshes.py -i X.xml -o Y.xml     # dress any undressed v6 plant
    .venv/bin/python sim/add_cad_meshes.py --export-servos      # (re)write sim/meshes/v6/servo_*.stl from the CAD mocks
"""
from __future__ import annotations

import argparse
import os
import re
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import build_v6_inertia as B  # noqa: E402  (also puts cad/ and cad/v6 on sys.path)

V = B.V
SERVO_DIR = os.path.join(HERE, "meshes", "v6")
PLA = "0.82 0.84 0.87 1"
SERVO = "0.12 0.12 0.14 1"
SOLE = "0.93 0.93 0.91 1"     # the white 2 mm silicone sheet

# servo mock keys of build_v6_inertia._mock_stl, per body
SERVOS = {"torso": ["yaw_L", "yaw_R", "neck", "shoulder_L", "shoulder_R"],
          "L_hip_yaw": ["roll"], "R_hip_yaw": ["roll"],
          "L_thigh": ["pitch"], "R_thigh": ["pitch"], "L_shin": ["pitch"], "R_shin": ["pitch"],
          "L_ankle_blk": ["pitch"], "R_ankle_blk": ["pitch"],
          "L_foot": ["foot_L"], "R_foot": ["foot_R"],
          "L_forearm": ["elbow_L"], "R_forearm": ["elbow_R"]}


def parts(arms=True):
    """body -> [(stl name, offset mm, rgba)]: the printed parts, as placed in
    build_v6_inertia.bodies()."""
    zdeck = V.DECK_TOP_Z - V.HIP_YAW_Z
    z_foot = -V.ANKLE_ROLL_Z + V.TPU_SOLE_T
    out = {"torso": [("pelvis_v7" + ("" if arms else "_armless"), (0, 0, zdeck), PLA),
                     ("yaw_bearing_housing", (0, 0, zdeck), PLA),
                     ("neck_floor", (0, 0, zdeck), PLA)]
           + ([("shoulder_girdle_v6", (0, 0, zdeck), PLA)] if arms else [("neck_collar", (0, 0, zdeck), PLA)]),
           "head": [("head", (0, 0, 0), PLA)]}
    for s in ("L", "R"):
        out[f"{s}_hip_yaw"] = [("yaw_carrier_v6", (0, 0, 0), PLA)]
        out[f"{s}_hip"] = [("hip_yoke_v6", (0, 0, 0), PLA)]
        out[f"{s}_thigh"] = [("leg_link_v6", (0, 0, 0), PLA)]
        out[f"{s}_shin"] = [("leg_link_v6", (0, 0, 0), PLA)]
        out[f"{s}_ankle_blk"] = [("ankle_link", (0, 0, 0), PLA)]
        out[f"{s}_foot"] = [(f"foot_{s}", (0, 0, z_foot), PLA), (f"sole_tpu_{s}", (0, 0, z_foot), SOLE)]
        if arms:
            out[f"{s}_arm"] = [(f"arm_upper_v6_{s}", (0, 0, 0), PLA)]
            out[f"{s}_forearm"] = [(f"arm_fore_v6_{s}", (0, 0, 0), PLA)]
    return out


def export_servos():
    os.makedirs(SERVO_DIR, exist_ok=True)
    keys = sorted({k for ks in SERVOS.values() for k in ks})
    for k in keys:
        shutil.copyfile(B._mock_stl(k), os.path.join(SERVO_DIR, f"servo_{k}.stl"))
        print(f"servo_{k}.stl")


def dress(src: str, xml_dir: str, arms=True) -> str:
    # one meshdir (the repo root): walker_env and env_mjx load the plant from a
    # string and absolutize meshdir against the XML's directory, not file=
    root = os.path.normpath(os.path.join(HERE, ".."))
    rel_cad = os.path.relpath(B.STL, root)
    rel_sv = os.path.relpath(SERVO_DIR, root)
    assets, used = [], set()

    def mesh(name, path):
        if name not in used:
            used.add(name)
            assets.append(f'    <mesh name="{name}" file="{path}" scale="0.001 0.001 0.001"/>')
        return name

    P = parts(arms)
    for body in sorted(set(P) | set(SERVOS)):
        geoms = []
        for stl, off, rgba in P.get(body, []):
            if not os.path.exists(B.stl(stl)):
                raise FileNotFoundError(B.stl(stl))
            n = mesh(f"cad_{stl}", f"{rel_cad}/{stl}.stl")
            pos = " ".join(f"{x * 1e-3:.5f}" for x in off)
            geoms.append(f'<geom name="{body}_cad_{stl}" class="cad" mesh="{n}" pos="{pos}" rgba="{rgba}"/>')
        for k in SERVOS.get(body, []):
            if k.startswith(("shoulder", "elbow")) and not arms:
                continue
            f = os.path.join(SERVO_DIR, f"servo_{k}.stl")
            if not os.path.exists(f):
                raise FileNotFoundError(f"{f} -- run with --export-servos first")
            n = mesh(f"servo_{k}", f"{rel_sv}/servo_{k}.stl")
            geoms.append(f'<geom name="{body}_servo_{k}" class="cad" mesh="{n}" rgba="{SERVO}"/>')
        if not geoms:
            continue
        # after the body's inertial line (build_v6_inertia puts one in every body we dress)
        pat = re.compile(rf'(<body name="{body}"[^>]*>\n(?:[^\n]*\n)*?\s*<inertial[^>]*/>\n)')
        src, nsub = pat.subn(lambda mo: mo.group(1) + "".join(f"        {g}\n" for g in geoms), src, count=1)
        if nsub == 0:
            raise RuntimeError(f"no inertial found in body {body!r}")
    # the primitives (collision boxes and the old visual boxes) go to group 3
    src = re.sub(r'(<geom\b[^>]*?)\bgroup="1"', r'\1group="3"', src)
    src = src.replace('<geom type="sphere" size="0.003" contype="1"', '<geom type="sphere" size="0.003" group="3" contype="1"', 1)
    src = src.replace("<asset>", "<asset>\n" + "\n".join(assets), 1) if "<asset>" in src else \
        src.replace("<worldbody>", "<asset>\n" + "\n".join(assets) + "\n  </asset>\n  <worldbody>", 1)
    # the mesh defaults (visual only: the v5 plants' cad class collided with
    # the floor, contype 2; env_mjx.mesh_floor keys off this class)
    cad_cls = ('    <default class="cad">\n      <geom type="mesh" contype="0" conaffinity="0" '
               'mass="0" group="2" rgba="' + PLA + '"/>\n    </default>\n')
    src, n = re.subn(r'(?m)^  </default>$', lambda mo: cad_cls + mo.group(0), src, count=1)
    if n != 1:
        raise RuntimeError("no top-level </default>")
    rel_root = os.path.relpath(root, xml_dir)
    src, n = re.subn(r'<compiler ([^>]*?)/>', lambda mo: f'<compiler {mo.group(1)} meshdir="{rel_root}"/>', src, count=1)
    if n != 1:
        raise RuntimeError("no <compiler> tag")
    return src


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--export-servos", action="store_true")
    ap.add_argument("-i", "--input")
    ap.add_argument("-o", "--out", required=False)
    ap.add_argument("--armless", action="store_true")
    a = ap.parse_args(argv)
    if a.export_servos:
        export_servos()
    if not a.out:
        return
    src = open(a.input).read()
    out = dress(src, os.path.dirname(os.path.abspath(a.out)), arms=not a.armless)
    with open(a.out, "w") as f:
        f.write(out)
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
