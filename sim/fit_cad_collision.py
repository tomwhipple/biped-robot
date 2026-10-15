"""Fit the v6 plant's collision primitives to the CAD parts they stand for.

gen_plant_v6 sizes its collision boxes from DesignParams (catalogue servo
cases, a 24 x 38 mm leg-link bar, 30 mm thigh/shank capsules inherited from
the v5 plants, 6 mm arm rods). This rewrites each one to the body-frame
bounding box of the CAD part(s) it represents -- the same STLs and placements
sim/add_cad_meshes.py draws and sim/build_v6_inertia.py weighs -- keeping
every geom's name, class and contact bits, so walker_env / env_mjx, the
foot-only contact mode and the referee's sole tracking see the same geoms:

  floor/fall (contype 2)  servo cases -> the servo mock; torso_housing -> the
                          pelvis; torso_deck -> the bearing housing (the wide
                          bottom plate); girdle; head_shell -> head; the link
                          bars -> leg_link_v6 / ankle_link; the arm rods ->
                          boxes around arm_upper / arm_fore
  inter-leg pairs         {L,R}_col_thigh / _col_shank -> capsules the length
                          of the leg link, radius = its half-width across the
                          body (the v5 capsules covered its top 30 mm);
                          _col_ankle -> ankle_link; _col_foot -> the foot
                          and its sole
  unchanged               the sole pads (the only floor contact in walking),
                          {L,R}_sole, the foot plates, the hand spheres

The v5 plants' legcol capsules were calibrated to bench onsets
(tests/test_rom_contacts.py, v4rom only); v6 has no bench yet, so its
reference is the CAD inter-leg check (cad/v6/check_assembly_v6.py), which
tests/test_v6_design_gates.py compares this geometry against.
"""
from __future__ import annotations

import functools
import os
import re

import numpy as np
import trimesh

import add_cad_meshes as M
import build_v6_inertia as B


@functools.lru_cache(maxsize=None)
def _aabb(path, off_mm):
    """(centre m, half extents m) of an STL placed at off_mm, in its body frame."""
    lo, hi = trimesh.load(path, force="mesh").bounds
    off = np.array(off_mm, float)
    lo, hi = lo + off, hi + off
    return tuple((lo + hi) / 2e3), tuple((hi - lo) / 2e3)


def _union(boxes):
    lo = np.min([np.subtract(c, h) for c, h in boxes], 0)
    hi = np.max([np.add(c, h) for c, h in boxes], 0)
    return tuple((lo + hi) / 2), tuple((hi - lo) / 2)


def part_boxes(arms=True):
    """body -> {label: (centre, half)}: every CAD part's box in its body frame."""
    out = {}
    for body, items in M.parts(arms).items():
        for stl, off, _ in items:
            out.setdefault(body, {})[stl] = _aabb(B.stl(stl), tuple(off))
    for body, keys in M.SERVOS.items():
        for k in keys:
            if k.startswith(("shoulder", "elbow")) and not arms:
                continue
            out.setdefault(body, {})[f"servo_{k}"] = _aabb(os.path.join(M.SERVO_DIR, f"servo_{k}.stl"), (0, 0, 0))
    return out


def _f(x):
    return f"{x:.5f}".rstrip("0").rstrip(".") if abs(x) >= 1e-9 else "0"


def _box_attrs(c, h):
    return f'type="box" pos="{_f(c[0])} {_f(c[1])} {_f(c[2])}" size="{_f(h[0])} {_f(h[1])} {_f(h[2])}"'


def _set_box(line, c, h):
    """the geom line as a box at (c, h): drop fromto/type/pos/size, keep the rest."""
    line = re.sub(r'\s(?:fromto|type|pos|size)="[^"]*"', "", line)
    return re.sub(r"<geom\b", "<geom " + _box_attrs(c, h), line, count=1)


def _set_capsule(line, c, h):
    """the inter-leg pair as a capsule along the link's length (z): radius = the
    link's half-width across the body (y, the axis the legs close on), axis
    through the box centre. A box pair is exact but box-box costs MJX ~17 % a
    step; capsule-capsule/box is cheap, and the capsule's only excess is
    fore-aft (radius vs the link's x half-depth)."""
    r = h[1]
    z0, z1 = c[2] + h[2] - r, c[2] - h[2] + r
    line = re.sub(r'\s(?:fromto|type|pos|size)="[^"]*"', "", line)
    return re.sub(r"<geom\b", f'<geom type="capsule" fromto="{_f(c[0])} {_f(c[1])} {_f(z0)} {_f(c[0])} {_f(c[1])} {_f(z1)}" size="{_f(r)}"', line, count=1)


def _centre(line):
    m = re.search(r'pos="([^"]+)"', line)
    return np.array([float(v) for v in m.group(1).split()]) if m else np.zeros(3)


def fit(src: str, arms=True) -> str:
    P = part_boxes(arms)
    out, stack = [], []
    for line in src.split("\n"):
        mb = re.search(r'<body name="([^"]+)"', line)
        if mb:
            stack.append(mb.group(1))
        body = stack[-1] if stack else None
        s = body[2:] if body and body[:2] in ("L_", "R_") else body
        boxes = P.get(body, {})
        if "<geom" in line and boxes and 'class="cad"' not in line:
            name = (re.search(r'name="([^"]+)"', line) or [None, ""])[1]
            if 'class="servo"' in line:
                sv = [(k, v) for k, v in boxes.items() if k.startswith("servo_")]
                k, v = min(sv, key=lambda kv: np.linalg.norm(np.array(kv[1][0]) - _centre(line)))
                line = _set_box(line, *v)
            elif name == "torso_housing":
                line = _set_box(line, *boxes["pelvis_v7" if arms else "pelvis_v7_armless"])
            elif name == "torso_deck":
                line = _set_box(line, *boxes["yaw_bearing_housing"])
            elif name == "girdle":
                line = _set_box(line, *boxes["shoulder_girdle_v6"])
            elif name == "head_shell":
                line = _set_box(line, *boxes["head"])
            elif s in ("thigh", "shin") and 'class="fallcol"' in line:
                line = _set_box(line, *boxes["leg_link_v6"])
            elif name.endswith(("_col_thigh", "_col_shank")):
                line = _set_capsule(line, *boxes["leg_link_v6"])
            elif s == "ankle_blk" and ('class="fallcol"' in line or name.endswith("_col_ankle")):
                line = _set_box(line, *boxes["ankle_link"])
            elif s == "foot" and name.endswith("_col_foot"):
                side = body[0]
                line = _set_box(line, *_union([boxes[f"foot_{side}"], boxes[f"sole_tpu_{side}"]]))
            elif s in ("arm", "forearm") and 'class="fallcol"' in line and 'type="capsule"' in line:
                side = body[0]
                line = _set_box(line, *boxes[f"arm_upper_v6_{side}" if s == "arm" else f"arm_fore_v6_{side}"])
        if "</body>" in line and stack:
            stack.pop()
        out.append(line)
    return "\n".join(out)
