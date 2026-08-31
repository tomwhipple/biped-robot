"""Re-render a sil_twin --hang obs capture from fixed cameras.

    .venv/bin/python tools/rerender_hang_views.py \
        sim/bimo_biped_v5body_hang.xml <parsed_obs.csv> <out_prefix>

The hang plant welds the torso, so the parsed obs CSV's q_* columns ARE the
full pose trajectory (torso = qpos0). Renders <out_prefix>_side.mp4 and
_rear.mp4 at 50 fps. Standing bench deliverable (Tom, 2026-08-31): every
hardware attempt gets a 2x2 composite -- real side | real rear over
sim side | sim rear, e.g.:

    ffmpeg -ss <arm_edge> -t <T> -i camN_side.mp4 \
           -ss <arm_edge> -t <T> -i camM_rear.mp4 \
           -i sim_side.mp4 -i sim_rear.mp4 -filter_complex \
      "[0:v]scale=640:480,fps=30[a];[1:v]scale=640:480,fps=30[b];\
       [2:v]scale=640:480,fps=30[c];[3:v]scale=640:480,fps=30[d];\
       [a][b][c][d]xstack=inputs=4:layout=0_0|w0_0|0_h0|w0_h0" \
      -c:v libx264 -crf 22 -pix_fmt yuv420p quad.mp4

The obs capture's row 0 is the ARM edge (rows only exist with torque on), so
sync the real clips by trimming them to their own arm edge (frame-diff motion
onset, or the arm_script timeline).

Rear check: the green board faces the rear camera, matching the physical
rear webcam (the board is on the robot's back).
"""
import csv
import os
import sys

os.environ.setdefault("MUJOCO_GL", "egl")
import imageio.v2 as imageio
import mujoco
import numpy as np

xml, csv_path, out_prefix = sys.argv[1], sys.argv[2], sys.argv[3]

model = mujoco.MjModel.from_xml_path(xml)
data = mujoco.MjData(model)
jnames = [model.joint(i).name for i in range(model.njnt)]
print("joints:", jnames)

with open(csv_path) as f:
    r = csv.reader(f)
    cols = next(r)
    rows = [[float(v) for v in row] for row in r]

qcols = [c for c in cols if c.startswith("q_")]
# map obs joint order onto qpos addresses by joint name
addr = []
for c in qcols:
    name = c[2:]
    j = model.joint(name)
    addr.append(int(j.qposadr[0]))
print("q columns -> qpos addr:", list(zip(qcols, addr)))
iq = [cols.index(c) for c in qcols]

ren = mujoco.Renderer(model, height=480, width=640)
views = {"side": 90, "rear": 0}
writers = {v: imageio.get_writer(f"{out_prefix}_{v}.mp4", fps=50)
           for v in views}
torso = np.array(model.qpos0[:7])
for k, row in enumerate(rows):
    data.qpos[:7] = torso
    for a, i in zip(addr, iq):
        data.qpos[a] = row[i]
    mujoco.mj_forward(model, data)
    for v, az in views.items():
        cam = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(cam)
        cam.lookat[:] = [float(torso[0]), float(torso[1]), 0.28]
        cam.distance, cam.azimuth, cam.elevation = 1.0, az, -8
        ren.update_scene(data, cam)
        writers[v].append_data(ren.render())
for w in writers.values():
    w.close()
print(f"wrote {out_prefix}_side.mp4 / _rear.mp4 ({len(rows)} frames @50fps)")
