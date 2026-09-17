"""Before/after joint stills + a section view for the hip-yaw bearing study
(docs/design-v6/study-yaw-bearing.md). Zoomed on the yaw joint only (the
carrier + the pelvis cell it sits under), not the whole robot.

    .venv/bin/python cad/v6/render_yaw_bearing.py
"""
import importlib.util
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
os.environ.setdefault("MUJOCO_GL", "egl")

from build123d import Pos, Compound, Color, export_stl, Box  # noqa: E402
import dimensions_v6 as V  # noqa: E402
import parts as v5parts  # noqa: E402
import yaw_carrier_v6  # noqa: E402
import pelvis_v7  # noqa: E402
import assembly_v6 as A  # noqa: E402

OUT = os.path.join(HERE, "renders")
os.makedirs(OUT, exist_ok=True)


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def joint_compound(carrier_solid, pelvis_solid, y):
    """Carrier at the standing pose, pelvis housing, both labelled for render()."""
    return Compound(label="joint", children=[
        Pos(0, y, V.HIP_YAW_Z) * carrier_solid,
        Pos(0, 0, V.DECK_TOP_Z) * pelvis_solid,
    ])


def render_zoom(comp, png_path, dist=0.22):
    """Same MuJoCo-from-STL render as assembly_v6.render(), but zoomed on the
    yaw axis at one hip instead of the whole standing robot."""
    import mujoco
    import numpy as np
    import imageio
    import tempfile
    tmp = tempfile.mkdtemp(prefix="v6joint_")
    geoms = []
    for i, ch in enumerate(comp.children):
        path = os.path.join(tmp, f"p{i}.stl")
        export_stl(ch, path)
        geoms.append((f"p{i}", path))
    colors = ["0.85 0.55 0.2 1", "0.75 0.78 0.82 1"]
    xml = ['<mujoco><compiler meshdir="."/><visual><global offwidth="1200" offheight="1200"/>'
           '<headlight diffuse="0.7 0.7 0.7" ambient="0.5 0.5 0.5"/></visual><asset>']
    xml += [f'<mesh name="{n}" file="{os.path.basename(p)}" scale="0.001 0.001 0.001"/>' for n, p in geoms]
    xml += ['</asset><worldbody><light pos="0.3 0.3 0.6" dir="-0.3 -0.3 -1" directional="true"/>']
    xml += [f'<geom type="mesh" mesh="{n}" rgba="{colors[i % 2]}"/>' for i, (n, _) in enumerate(geoms)]
    xml += ['</worldbody></mujoco>']
    with open(os.path.join(tmp, "j.xml"), "w") as f:
        f.write("\n".join(xml))
    m = mujoco.MjModel.from_xml_path(os.path.join(tmp, "j.xml"))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, 900, 900)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0, V.HIP_SEP / 2000.0, V.HIP_YAW_Z / 1000.0 - 0.02]
    cam.distance = dist
    imgs = []
    for az, el in ((35, -18), (100, -25)):
        cam.azimuth, cam.elevation = az, el
        r.update_scene(d, cam)
        imgs.append(r.render().copy())
    imageio.imwrite(png_path, np.concatenate(imgs, axis=1))
    print("wrote", png_path)


def main():
    y = V.HIP_SEP / 2

    # ---- BEFORE: v5 carrier (unchanged), the pre-study pelvis (git HEAD)
    before_pelvis_mod = load_module("/tmp/pelvis_v7_orig.py", "pelvis_v7_orig")
    before_carrier = v5parts.yaw_carrier()
    before_pelvis = before_pelvis_mod.pelvis_v7()
    render_zoom(joint_compound(before_carrier, before_pelvis, y),
                os.path.join(OUT, "yaw_bearing_before.png"))

    # ---- AFTER: the new boss+skirt design
    after_carrier = yaw_carrier_v6.yaw_carrier_v6()
    after_pelvis = pelvis_v7.pelvis_v7()
    render_zoom(joint_compound(after_carrier, after_pelvis, y),
                os.path.join(OUT, "yaw_bearing_after.png"))

    # ---- SECTION: cut the carrier at its OWN y (reveals its boss/plate),
    # and cut the PELVIS at the housing's mid-sagittal plane (y=0, not the
    # hip's own y) so the section shows the near skirt's full recess/
    # shoulder stack AND the bridge connecting it to the far skirt (the
    # bridge spans y=-8..+8 in pelvis-local, so this cut clips it at its
    # midpoint rather than hiding it entirely).
    carrier_cut = after_carrier - Pos(0, 200, 0) * Box(500, 400, 500)
    pelvis_cut = after_pelvis - Pos(0, -200, 0) * Box(500, 400, 500)
    render_zoom(joint_compound(carrier_cut, pelvis_cut, y),
                os.path.join(OUT, "yaw_bearing_section.png"), dist=0.16)


if __name__ == "__main__":
    main()
