"""Before/after joint stills + a section view for hip-yaw bearing OPTION A
(round hub, round 2, docs/design-v6/study-yaw-bearing.md). Same method as
render_yaw_bearing.py (option C): zoomed on the yaw joint only.

    .venv/bin/python cad/v6/render_yaw_bearing_optA.py
"""
import importlib.util
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
os.environ.setdefault("MUJOCO_GL", "egl")

from build123d import Pos, Compound, export_stl, Box  # noqa: E402
import dimensions_v6 as V  # noqa: E402
import parts as v5parts  # noqa: E402
import yaw_carrier_v6_optA  # noqa: E402
import pelvis_v7  # noqa: E402
from render_yaw_bearing import render_zoom  # noqa: E402

OUT = os.path.join(HERE, "renders")
os.makedirs(OUT, exist_ok=True)


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def joint_compound(carrier_solid, pelvis_solid, y):
    return Compound(label="joint", children=[
        Pos(0, y, V.HIP_YAW_Z) * carrier_solid,
        Pos(0, 0, V.DECK_TOP_Z) * pelvis_solid,
    ])


def main():
    y = V.HIP_SEP / 2
    # BEFORE is the SAME pre-study state as option C's (no yaw-bearing feature
    # at all) -- reuse cad/v6/renders/yaw_bearing_before.png rather than
    # re-rendering an identical image.

    # ---- AFTER: the round-hub design
    after_carrier = yaw_carrier_v6_optA.yaw_carrier_v6_optA()
    after_pelvis = pelvis_v7.pelvis_v7(bearing_variant="A")
    render_zoom(joint_compound(after_carrier, after_pelvis, y),
                os.path.join(OUT, "yaw_bearing_optA_after.png"))

    # ---- SECTION: same cut convention as render_yaw_bearing.py
    carrier_cut = after_carrier - Pos(0, 200, 0) * Box(500, 400, 500)
    pelvis_cut = after_pelvis - Pos(0, -200, 0) * Box(500, 400, 500)
    render_zoom(joint_compound(carrier_cut, pelvis_cut, y),
                os.path.join(OUT, "yaw_bearing_optA_section.png"), dist=0.16)


if __name__ == "__main__":
    main()
