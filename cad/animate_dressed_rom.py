"""Leg range-of-motion video of the DRESSED robot (cables, ties, battery,
board -- dress.py): every joint sweeps its working range while the cable
segments are RE-SWEPT each frame from the posed attachment points, so the
wiring visibly follows the legs instead of tearing off.

Run:  .venv/bin/python cad/animate_dressed_rom.py
      -> cad/renders/dressed_rom.mov        (orbiting 3/4 view, h264, no gif)

      .venv/bin/python cad/animate_dressed_rom.py --back
      -> cad/renders/dressed_rom_back.mov   (camera locked on the rear the
         whole time so the cable raceways stay in frame -- you watch exactly
         where each red servo lead moves as the joints sweep)

Kinematic pose demo like sim/render_rom.py (base fixed in the world, no
dynamics; both legs posed identically, same-sign roll so the legs never
pass through each other). Rigid parts export ONCE and are posed per frame
via body pos/quat; only the 6 joint-crossing cables rebuild per frame.
"""
import multiprocessing
import os
import shutil
import struct
import subprocess
import sys
import numpy as np
import mujoco
import imageio.v2 as imageio
import imageio_ffmpeg
from PIL import Image, ImageDraw, ImageFont
from build123d import export_stl

import dimensions as D
import dress

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "renders")
TMP = os.path.join(OUT, "_dressed")
os.makedirs(TMP, exist_ok=True)

FPS, W, H = 14, 640, 520

# body name -> (leaf labels, frame key) ; frame key None = torso (fixed)
BODIES = {
    # v5 torso (2026-08-04): battery_tray + board_frame, both at servo level; no
    # camera or IMU until the head bolt-on exists. The yaw risers (trench ->
    # roll servo) are torso-fixed too, since dress draws the carriers at neutral
    # yaw. Labels must match dress.dressed_robot's leaves exactly -- a stale
    # name here silently drops the piece from the video.
    "torso": (["pelvis", "battery_tray", "board_frame", "battery_3s_mock",
               "board_pcb_mock", "board_parts_mock", "pigtail_xt30",
               "cable_board_L", "cable_board_R", "cable_yaw_L", "cable_yaw_R",
               "L_servo_hip_roll", "R_servo_hip_roll"], None),
}
for tag in ("L", "R"):
    BODIES[f"{tag}_yoke"] = ([f"{tag}_yoke_roll", f"{tag}_yoke_pitch"], "yoke")
    BODIES[f"{tag}_thigh"] = ([f"{tag}_servo_hip_pitch", f"{tag}_link_thigh",
                               f"{tag}_ties_thigh"], "thigh")
    BODIES[f"{tag}_shin"] = ([f"{tag}_servo_knee", f"{tag}_link_shin",
                              f"{tag}_ties_shin"], "shin")
    BODIES[f"{tag}_foot"] = ([f"{tag}_servo_ankle", f"{tag}_foot"], "foot")

CABLE_LABELS = [f"{t}_cable_{s}" for t in ("L", "R")
                for s in ("hip", "thigh", "shin")]


def collect_leaves():
    """Export every leaf of the NEUTRAL dressed robot once; return
    label -> (stl path, rgb)."""
    robot = dress.dressed_robot()
    out = {}
    def walk(c, tag=""):
        for ch in c.children:
            if list(ch.children):
                walk(ch, ch.label.replace("leg_", "") + "_")
            else:
                label = tag + ch.label
                stl = os.path.join(TMP, f"{label}.stl")
                export_stl(ch, stl)
                out[label] = (stl, tuple(ch.color)[:3])
    walk(robot)
    return out


def stl_ok(path):
    """True if the file is a binary STL mujoco will load: 84-byte header,
    1..200000 faces, and a size that matches the face count."""
    try:
        size = os.path.getsize(path)
        with open(path, "rb") as f:
            hdr = f.read(84)
        if len(hdr) < 84:
            return False
        nf = struct.unpack("<I", hdr[80:84])[0]
        return 1 <= nf <= 200000 and size == 84 + 50 * nf
    except OSError:
        return False


def mat2quat(R):
    q = np.empty(4)
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        q[:] = [0.25 * s, (R[2, 1] - R[1, 2]) / s,
                (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(R))); j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
        q[0] = (R[k, j] - R[j, k]) / s
        q[1 + i] = 0.25 * s
        q[1 + j] = (R[j, i] + R[i, j]) / s
        q[1 + k] = (R[k, i] + R[i, k]) / s
    return q / np.linalg.norm(q)


# --------------------------------------------------------------- timeline
# (label, target (roll, hip, knee, ankle), travel frames, hold frames)
KEYS = [
    ("",               (0, 0, 0, 0),        0,  6),
    ("hip flexion",    (0, -110, 0, 0),     16, 4),
    ("hip extension",  (0, 60, 0, 0),       20, 4),
    ("",               (0, 0, 0, 0),        10, 0),
    ("knee",           (0, -20, -95, 0),    14, 4),
    ("",               (0, 0, 0, 0),        12, 0),
    ("ankle",          (0, 0, 0, -40),      8,  2),
    ("ankle",          (0, 0, 0, 40),       10, 2),
    ("",               (0, 0, 0, 0),        6,  0),
    ("hip roll",       (25, 0, 0, 0),       8,  2),
    ("hip roll",       (-25, 0, 0, 0),      10, 2),
    ("",               (0, 0, 0, 0),        6,  0),
    ("get-up pike",    (0, -110, -95, 40),  18, 10),
    ("",               (0, 0, 0, 0),        14, 6),
]


def timeline():
    frames = []               # (angles, label)
    prev = np.zeros(4)
    for label, tgt, travel, hold in KEYS:
        tgt = np.asarray(tgt, float)
        for f in range(travel):
            t = (f + 1) / travel
            e = t * t * (3 - 2 * t)
            frames.append((prev + (tgt - prev) * e, label))
        frames.extend([(tgt, label)] * hold)
        prev = tgt
    return frames


def label_frame(img, text):
    if not text:
        return img
    im = Image.fromarray(img)
    d = ImageDraw.Draw(im)
    try:
        font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 26)
    except OSError:
        font = ImageFont.load_default()
    d.text((18, H - 44), text, font=font, fill=(255, 255, 255),
           stroke_width=2, stroke_fill=(0, 0, 0))
    return np.asarray(im)


def build_xml(rigid_assets, rigid_bodies, cable_assets, cable_geoms):
    return f"""<mujoco>
      <compiler meshdir="."/>
      <visual><headlight ambient="0.5 0.5 0.52" diffuse="0.45 0.45 0.45"/>
        <global offwidth="{W}" offheight="{H}"/>
        <quality shadowsize="2048"/></visual>
      <asset>
        <texture name="sky" type="skybox" builtin="gradient"
                 rgb1="0.93 0.94 0.96" rgb2="0.80 0.83 0.88"
                 width="256" height="256"/>
        <texture name="grid" type="2d" builtin="checker" rgb1="0.90 0.91 0.93"
                 rgb2="0.84 0.86 0.89" width="512" height="512"/>
        <material name="floor" texture="grid" texrepeat="12 12"/>
        {"".join(rigid_assets)}{"".join(cable_assets)}
      </asset>
      <worldbody>
        <light pos="0.5 -0.7 1.2" dir="-0.4 0.55 -0.8" castshadow="true"/>
        <geom type="plane" size="2 2 0.1" material="floor"/>
        {"".join(rigid_bodies)}
        {"".join(cable_geoms)}
      </worldbody>
    </mujoco>"""


# ------------------------------------------------------------ parallel render
# The per-frame cost is dominated by OCC cable sweeps + a full MuJoCo recompile
# (the cable meshes change shape every frame, so the model can't be reused).
# Frames are independent, so we fan them out across processes. Rigid part STLs
# are exported ONCE by the parent and read-only shared; each worker sweeps its
# own cables into a private dir so filenames never collide.
_WK = {}   # per-process render context, populated by _init_worker


def _init_worker(leaves, rigid_assets, rigid_bodies, back_view, total):
    cdir = os.path.join(TMP, f"w{os.getpid()}")
    os.makedirs(cdir, exist_ok=True)
    _WK.update(leaves=leaves, rigid_assets=rigid_assets,
               rigid_bodies=rigid_bodies, back_view=back_view,
               total=total, cdir=cdir)


def _render_frame(job):
    """Render one frame; returns (index, rgb array). Runs in a worker or,
    for jobs==1, in the main process after _init_worker was called once."""
    i, ang, text = job
    roll, hip, knee, ankle = ang
    leaves = _WK["leaves"]
    cdir = _WK["cdir"]

    cable_assets, cable_geoms = [], []
    for tag, ly in (("L", D.HIP_SEP / 2), ("R", -D.HIP_SEP / 2)):
        segs = dress.leg_cables(ly, roll, hip, knee, ankle)
        chained = None
        for k, (s, seg) in enumerate(zip(("hip", "thigh", "shin"), segs)):
            lb = f"{tag}_cable_{s}"
            path = os.path.join(cdir, f"{lb}.stl")
            export_stl(seg, path)
            if not stl_ok(path):
                # OCC sweep produced an unloadable tessellation for this pose
                # -- rebuild this leg's cables as segment chains.
                if chained is None:
                    chained = dress.leg_cables(ly, roll, hip, knee, ankle,
                                               chain=True)
                export_stl(chained[k], path)
                assert stl_ok(path), f"frame {i} {lb} unexportable"
            c = leaves[lb][1]
            cable_assets.append(
                f'<mesh name="{lb}" file="{path}" scale="0.001 0.001 0.001"/>')
            cable_geoms.append(f'<geom type="mesh" mesh="{lb}" contype="0" '
                               f'conaffinity="0" rgba="{c[0]} {c[1]} {c[2]} 1"/>')

    xml = build_xml(_WK["rigid_assets"], _WK["rigid_bodies"],
                    cable_assets, cable_geoms)
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    for tag, ly in (("L", D.HIP_SEP / 2), ("R", -D.HIP_SEP / 2)):
        fr = dress.leg_frames(ly, roll, hip, knee, ankle)
        for bname, (_, key) in BODIES.items():
            if key is None or not bname.startswith(tag):
                continue
            M = fr[key]
            mid = m.body(bname).mocapid[0]
            d.mocap_pos[mid] = M[:3, 3] / 1000.0
            d.mocap_quat[mid] = mat2quat(M[:3, :3])
    mujoco.mj_forward(m, d)

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.lookat[:] = [0.0, 0.0, 0.18]
    cam.distance, cam.elevation = 0.92, -12
    # Rear cable raceways face -X; azimuth 0 puts the camera dead-on behind the
    # robot so every red servo lead stays in frame. Orbit otherwise.
    cam.azimuth = 0 if _WK["back_view"] else 150 + 110 * i / _WK["total"]

    renderer = mujoco.Renderer(m, height=H, width=W)
    renderer.update_scene(d, cam)
    out = label_frame(renderer.render().copy(), text)
    renderer.close()
    return i, out


def main(back_view=False, jobs=None):
    if jobs is None:
        jobs = max(1, (os.cpu_count() or 4) - 2)
    leaves = collect_leaves()
    frames_spec = timeline()
    total = len(frames_spec)

    rigid_assets, rigid_bodies = [], []
    for bname, (labels, _) in BODIES.items():
        geoms = []
        for lb in labels:
            stl, c = leaves[lb]
            rigid_assets.append(
                f'<mesh name="{lb}" file="{stl}" scale="0.001 0.001 0.001"/>')
            geoms.append(f'<geom type="mesh" mesh="{lb}" contype="0" '
                         f'conaffinity="0" rgba="{c[0]} {c[1]} {c[2]} 1"/>')
        rigid_bodies.append(f'<body name="{bname}" mocap="true">'
                            + "".join(geoms) + "</body>")

    jobs = min(jobs, total)
    print(f"{total} frames, {len(leaves)} rigid+cable leaves, {jobs} worker(s)")
    init_args = (leaves, rigid_assets, rigid_bodies, back_view, total)
    jobspec = [(i, ang, text) for i, (ang, text) in enumerate(frames_spec)]

    frames = [None] * total
    done = 0
    if jobs == 1:
        _init_worker(*init_args)
        for job in jobspec:
            i, arr = _render_frame(job)
            frames[i] = arr
            done += 1
            if done % 20 == 0 or done == total:
                print(f"  frame {done}/{total}")
    else:
        # 'spawn' gives each worker a clean GL/OCC state (fork + GL is unsafe
        # on macOS). Workers stream results back as they finish.
        ctx = multiprocessing.get_context("spawn")
        with ctx.Pool(jobs, initializer=_init_worker,
                      initargs=init_args) as pool:
            for i, arr in pool.imap_unordered(_render_frame, jobspec):
                frames[i] = arr
                done += 1
                if done % 20 == 0 or done == total:
                    print(f"  frame {done}/{total}")

    tmp_mp4 = os.path.join(TMP, "raw.mp4")
    imageio.mimwrite(tmp_mp4, frames, fps=FPS, codec="libx264", quality=8)
    mov = os.path.join(OUT, "dressed_rom_back.mov" if back_view
                       else "dressed_rom.mov")
    subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-i", tmp_mp4,
                    "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2",
                    "-c:v", "libx264", "-crf", "20", "-pix_fmt", "yuv420p",
                    "-movflags", "+faststart", mov],
                   check=True, capture_output=True)
    shutil.rmtree(TMP)
    print(f"wrote {mov}  ({total} frames @ {FPS} fps, "
          f"{os.path.getsize(mov) // 1024} KB)")


if __name__ == "__main__":
    jobs = None
    for a in sys.argv:
        if a.startswith("--jobs="):
            jobs = int(a.split("=", 1)[1])
    main(back_view="--back" in sys.argv, jobs=jobs)
