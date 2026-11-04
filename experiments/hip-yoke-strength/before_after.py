"""Before/after render of the glued hip yoke: rows = an older commit's STLs (BEFORE_REF,
default ab92371, the failed design) and the working tree's, columns = three views.
    cd <repo root> && python experiments/hip-yoke-strength/before_after.py <out.png>
(docs/design-v6/figs/hip_yoke_before_after.png)"""
import io
import os
import subprocess
import sys

os.environ.setdefault("MUJOCO_GL", "cgl")
import imageio.v2 as imageio  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import trimesh  # noqa: E402

TMP = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                   "build", "experiments", "hip-yoke-strength")
os.makedirs(TMP, exist_ok=True)
ROLL, PITCH = (0.92, 0.55, 0.25, 1), (0.42, 0.60, 0.88, 1)
VIEWS = [(135, -25), (45, -25), (-60, -20)]
PX = 520


def stls(ref):
    out = []
    for name in ("hip_yoke_glued_roll", "hip_yoke_glued_pitch"):
        if ref:
            data = subprocess.run(["git", "show", f"{ref}:cad/v6/stl/{name}.stl"], capture_output=True, check=True).stdout
            m = trimesh.load(io.BytesIO(data), file_type="stl")
            path = os.path.join(TMP, f"ba_{ref.replace('/', '_')}_{name}.stl")
            m.export(path)
        else:
            path = os.path.abspath(f"cad/v6/stl/{name}.stl")
        out.append(path)
    return out


def row(paths, lo, hi):
    assets = "".join(f'<mesh name="m{i}" file="{p}" scale="0.001 0.001 0.001"/>' for i, p in enumerate(paths))
    geoms = "".join(f'<geom type="mesh" mesh="m{i}" rgba="{" ".join(map(str, c))}"/>'
                    for i, c in enumerate((ROLL, PITCH)))
    model = mujoco.MjModel.from_xml_string(f"""<mujoco><asset>{assets}</asset>
      <visual><headlight ambient="0.45 0.45 0.45" diffuse="0.65 0.65 0.65"/><global offwidth="{PX}" offheight="{PX}"/></visual>
      <worldbody>{geoms}</worldbody></mujoco>""")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    ren = mujoco.Renderer(model, PX, PX)
    frames = []
    for az, el in VIEWS:
        cam = mujoco.MjvCamera()
        cam.lookat = (lo + hi) / 2
        cam.distance = 1.55 * np.linalg.norm(hi - lo)
        cam.azimuth, cam.elevation = az, el
        ren.update_scene(data, cam)
        frames.append(ren.render())
    return np.concatenate(frames, axis=1)


if __name__ == "__main__":
    old, new = stls(os.environ.get("BEFORE_REF", "ab92371")), stls(None)
    pts = np.concatenate([trimesh.load(p).vertices for p in old + new]) * 0.001
    lo, hi = pts.min(0), pts.max(0)
    img = np.concatenate([row(old, lo, hi), row(new, lo, hi)], axis=0)
    imageio.imwrite(sys.argv[1], img)
    print("wrote", sys.argv[1], img.shape)
