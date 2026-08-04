"""Per-joint range-of-motion video: ONE LEG AT A TIME, ONE JOINT AT A TIME.

Run:  .venv/bin/python cad/animate_joint_rom.py [--jobs=N]
      -> cad/renders/joint_rom.mov

Why this exists alongside animate_dressed_rom.py
------------------------------------------------
`animate_dressed_rom.py` poses BOTH legs identically and drives only the four
v2 joints (roll/hip/knee/ankle) -- it is a "does the wiring survive the sweep"
video. This one answers a different question, asked 2026-07-30: *show each
joint's actual travel, isolated*. So:

  * one leg moves at a time; the other stays at neutral as a visual reference,
  * one joint moves at a time, driven neutral -> max -> min -> neutral,
  * all FIVE v3yaw joints, hip-yaw included.

Hip-yaw is why this is a separate file rather than a flag on the old one.
`dress.leg_frames()` has no yaw term, and its cable sweeps are routed against a
yaw-less chain -- teaching them to yaw is a real change to the cable model. So
this script drops the cables and poses the same SIX rigid bodies per leg that
`freecad_articulate.py` groups (torso + carrier + yoke + thigh + shin + foot),
including the servo mocks and modeled fasteners, so it shows the machine.

Joint limits are the sim's, read straight off dimensions/the plant:
    hip yaw  +/-45   hip roll +/-25   hip pitch -110..+60
    knee     -5..+95   ankle  +/-40
The knee entry is the PHYSICAL rotation. The MJCF says range="-95 5" on a
"0 -1 0" axis (2026-08-02 sign fix), which is the same envelope written the
other way round: 95 deg of flexion, 5 deg of hyperextension.
"""
import multiprocessing
import os
import shutil
import subprocess
import sys

import numpy as np
import mujoco
import imageio.v2 as imageio
import imageio_ffmpeg
from build123d import Pos, Rot, Compound, export_stl

import check_assembly as CA
import dimensions as D
import fasteners as F
import parts
import export_assembly as A
from animate_dressed_rom import build_xml, label_frame, mat2quat, stl_ok

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "renders")
TMP = os.path.join(OUT, "_jointrom")
os.makedirs(TMP, exist_ok=True)

FPS, W, H = 14, 640, 520

# ------------------------------------------------------------- kinematics
# _rx/_ry live in dress.py; yaw needs a _rz, which dress.py has never had.
def _rz(a, c):
    t = np.eye(4)
    r = np.radians(a)
    t[0, 0] = t[1, 1] = np.cos(r)
    t[0, 1] = -np.sin(r)
    t[1, 0] = np.sin(r)
    out = np.eye(4); out[:3, 3] = c
    back = np.eye(4); back[:3, 3] = -np.asarray(c, float)
    return out @ t @ back


def _rx(a, c):
    t = np.eye(4); r = np.radians(a)
    t[1:3, 1:3] = [[np.cos(r), -np.sin(r)], [np.sin(r), np.cos(r)]]
    out = np.eye(4); out[:3, 3] = c
    back = np.eye(4); back[:3, 3] = -np.asarray(c, float)
    return out @ t @ back


def _ry(a, c):
    t = np.eye(4); r = np.radians(a)
    t[0, 0] = t[2, 2] = np.cos(r); t[0, 2] = np.sin(r); t[2, 0] = -np.sin(r)
    out = np.eye(4); out[:3, 3] = c
    back = np.eye(4); back[:3, 3] = -np.asarray(c, float)
    return out @ t @ back


def leg_frames5(ly, yaw, roll, hip, knee, ankle):
    """World 4x4 per moving body of one leg, 5-DOF v3yaw chain (degrees).

    Yaw sits ABOVE roll (yaw-first stack, Open Duck arrangement), so the whole
    roll->pitch->knee->ankle chain rides the carrier -- matching the plant and
    freecad_articulate.py."""
    m_car = _rz(yaw, (0, ly, D.HIP_YAW_Z))
    m_yoke = m_car @ _rx(roll, (0, ly, D.HIP_ROLL_Z))
    m_thigh = m_yoke @ _ry(hip, (0, ly, D.HIP_PITCH_Z))
    m_shin = m_thigh @ _ry(knee, (0, ly, D.KNEE_Z))
    m_foot = m_shin @ _ry(ankle, (0, ly, D.ANKLE_Z))
    return {"carrier": m_car, "yoke": m_yoke, "thigh": m_thigh,
            "shin": m_shin, "foot": m_foot}


# --------------------------------------------------------- rigid body groups
# leaf label -> (neutral-pose solid, colour). Grouped exactly as the six rigid
# bodies of freecad_articulate.py. The ROLL SERVO rides the CARRIER (v3yaw
# relocated it off the torso), which is why it is not in the torso group.
def _leaves(ly, tag):
    at = lambda z: Pos(0, ly, z)
    return {
        f"{tag}_carrier": [
            ("servo_hip_yaw", A.COL_SERVO,
             Pos(0, ly, D.HIP_YAW_Z + D.SV_HORN_FACE) * CA.servo_mock_z()),
            ("yaw_carrier", A.COL_PRINT, at(D.HIP_YAW_Z) * parts.yaw_carrier()),
            ("screws_yaw_stack", A.COL_STEEL,
             at(D.HIP_YAW_Z) * F.yaw_carrier_screws()),
            ("servo_hip_roll", A.COL_SERVO, at(D.HIP_ROLL_Z) * CA.servo_mock_x()),
        ],
        f"{tag}_yoke": [
            ("yoke_roll", A.COL_PRINT, at(D.HIP_ROLL_Z) * parts.yoke_roll()),
            ("yoke_pitch", A.COL_PRINT, at(D.HIP_PITCH_Z) * parts.yoke_pitch()),
            ("screws_hip_roll", A.COL_STEEL, at(D.HIP_ROLL_Z) * F.disc_screws_x()),
            ("screws_flange", A.COL_STEEL, at(D.HIP_ROLL_Z) * F.flange_bolts()),
            ("screws_hip_pitch", A.COL_STEEL, at(D.HIP_PITCH_Z) * F.disc_screws_y()),
        ],
        f"{tag}_thigh": [
            ("servo_hip_pitch", A.COL_SERVO, at(D.HIP_PITCH_Z) * CA.servo_mock_y()),
            ("link_thigh", A.COL_PRINT, at(D.HIP_PITCH_Z) * parts.leg_link()),
            ("screws_thigh_grip", A.COL_STEEL,
             at(D.HIP_PITCH_Z) * F.leg_link_screws()),
            ("screws_knee", A.COL_STEEL, at(D.KNEE_Z) * F.disc_screws_y()),
        ],
        f"{tag}_shin": [
            ("servo_knee", A.COL_SERVO, at(D.KNEE_Z) * CA.servo_mock_y()),
            ("link_shin", A.COL_PRINT, at(D.KNEE_Z) * parts.leg_link()),
            ("screws_shin_grip", A.COL_STEEL, at(D.KNEE_Z) * F.leg_link_screws()),
            ("screws_ankle", A.COL_STEEL, at(D.ANKLE_Z) * F.disc_screws_y()),
        ],
        f"{tag}_foot": [
            ("servo_ankle", A.COL_SERVO,
             at(D.ANKLE_Z) * Rot(0, 90, 0) * CA.servo_mock_y()),
            ("foot", A.COL_FOOT, at(D.TPU_PROUD) * parts.foot()),
            ("screws_foot", A.COL_STEEL, at(D.TPU_PROUD) * F.foot_screws()),
        ],
    }


BODY_KEY = {"carrier": "carrier", "yoke": "yoke", "thigh": "thigh",
            "shin": "shin", "foot": "foot"}

# --------------------------------------------------------------- timeline
# (label, joint index, target) -- index into (yaw, roll, hip, knee, ankle).
# Limits are the sim's; see the module docstring.
JOINTS = [
    ("hip yaw",   0, (-45, 45)),
    ("hip roll",  1, (-25, 25)),
    ("hip pitch", 2, (-110, 60)),
    ("knee",      3, (-5, 95)),   # PHYSICAL +y rotation: +95 = flexion (the
                                  # plant's knee axis is "0 -1 0" since
                                  # 2026-08-02, so its -95..+5 is this
                                  # mirrored -- see dimensions.ROM["knee"])
    ("ankle",     4, (-40, 40)),
]
TRAVEL, HOLD, SETTLE = 12, 3, 5


def timeline():
    """neutral -> max -> min -> neutral, per joint, per leg. One leg moves at a
    time; the other holds neutral as a reference."""
    frames = []          # (L_angles, R_angles, caption)
    for tag in ("L", "R"):
        for name, idx, (lo, hi) in JOINTS:
            cur = np.zeros(5)
            for tgt in (hi, lo, 0.0):
                nxt = cur.copy(); nxt[idx] = tgt
                for f in range(TRAVEL):
                    t = (f + 1) / TRAVEL
                    e = t * t * (3 - 2 * t)          # smoothstep
                    a = cur + (nxt - cur) * e
                    frames.append((a, f"{tag} {name}  {a[idx]:+.0f} deg"))
                frames.extend([(nxt, f"{tag} {name}  {tgt:+.0f} deg")] * HOLD)
                cur = nxt
            frames.extend([(np.zeros(5), "")] * SETTLE)
    # split into (L, R): the non-moving leg stays at neutral
    out = []
    n_per_leg = len(frames) // 2
    for i, (a, cap) in enumerate(frames):
        if i < n_per_leg:
            out.append((a, np.zeros(5), cap))
        else:
            out.append((np.zeros(5), a, cap))
    return out


_WK = {}


def _init_worker(rigid_assets, rigid_bodies, total):
    _WK.update(rigid_assets=rigid_assets, rigid_bodies=rigid_bodies, total=total)


def _render_frame(job):
    i, la, ra, text = job
    xml = build_xml(_WK["rigid_assets"], _WK["rigid_bodies"], [], [])
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    for tag, ly, ang in (("L", D.HIP_SEP / 2, la), ("R", -D.HIP_SEP / 2, ra)):
        fr = leg_frames5(ly, *ang)
        for key in BODY_KEY:
            bname = f"{tag}_{key}"
            M = fr[key]
            mid = m.body(bname).mocapid[0]
            d.mocap_pos[mid] = M[:3, 3] / 1000.0
            d.mocap_quat[mid] = mat2quat(M[:3, :3])
    mujoco.mj_forward(m, d)

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.lookat[:] = [0.0, 0.0, 0.18]
    cam.distance, cam.elevation = 0.95, -12
    cam.azimuth = 140 + 80 * i / _WK["total"]

    r = mujoco.Renderer(m, height=H, width=W)
    r.update_scene(d, cam)
    out = label_frame(r.render().copy(), text)
    r.close()
    return i, out


def main(jobs=None):
    if jobs is None:
        jobs = max(1, (os.cpu_count() or 4) - 2)

    # rigid parts are pose-invariant: export their STLs ONCE, then move them
    # per frame via mocap body pos/quat.
    rigid_assets, rigid_bodies = [], []
    # v6 torso (2026-08-04): one printed part + the camera mount on the deck.
    torso = [("pelvis", A.COL_PRINT, Pos(0, 0, A.DECK_TOP_Z) * parts.pelvis()),
             ("gopro_base", A.COL_PRINT,
              Pos(D.GP_MOUNT_X, 0, A.DECK_TOP_Z) * parts.gopro_base())]
    groups = {"torso": torso}
    for tag, ly in (("L", D.HIP_SEP / 2), ("R", -D.HIP_SEP / 2)):
        groups.update(_leaves(ly, tag))

    for bname, leaves in groups.items():
        geoms = []
        for label, colour, solid in leaves:
            path = os.path.join(TMP, f"{label if bname == 'torso' else bname + '_' + label}.stl")
            export_stl(solid, path)
            assert stl_ok(path), f"{label} unexportable"
            nm = os.path.splitext(os.path.basename(path))[0]
            rigid_assets.append(
                f'<mesh name="{nm}" file="{path}" scale="0.001 0.001 0.001"/>')
            # build123d Color is not subscriptable; tuple() is the idiom
            # collect_leaves() in animate_dressed_rom.py uses.
            c = tuple(colour)[:3]
            geoms.append(f'<geom type="mesh" mesh="{nm}" contype="0" '
                         f'conaffinity="0" rgba="{c[0]} {c[1]} {c[2]} 1"/>')
        if bname == "torso":
            rigid_bodies.append(f'<body name="torso">{"".join(geoms)}</body>')
        else:
            # mocap bodies carry their leaves at the NEUTRAL pose, so the frame
            # matrix is applied about the world origin -- same convention as
            # animate_dressed_rom.
            rigid_bodies.append(
                f'<body name="{bname}" mocap="true">{"".join(geoms)}</body>')

    spec = timeline()
    total = len(spec)
    jobs = min(jobs, total)
    print(f"{total} frames, {len(rigid_assets)} rigid leaves, {jobs} worker(s)")

    jobspec = [(i, la, ra, cap) for i, (la, ra, cap) in enumerate(spec)]
    frames = [None] * total
    init_args = (rigid_assets, rigid_bodies, total)
    if jobs == 1:
        _init_worker(*init_args)
        for job in jobspec:
            i, arr = _render_frame(job)
            frames[i] = arr
    else:
        # SPAWN, not fork. This module drags in OCC/build123d, whose
        # Objective-C runtime on macOS aborts the child if it was mid-
        # +initialize when fork() hit ("may have been in progress in another
        # thread when fork() was called. Crashing instead"). spawn re-imports
        # cleanly; the initargs below are plain strings, so they pickle.
        ctx = multiprocessing.get_context("spawn")
        done = 0
        with ctx.Pool(jobs, initializer=_init_worker, initargs=init_args) as pool:
            for i, arr in pool.imap_unordered(_render_frame, jobspec):
                frames[i] = arr
                done += 1
                if done % 25 == 0 or done == total:
                    print(f"  frame {done}/{total}")

    tmp_mp4 = os.path.join(TMP, "raw.mp4")
    imageio.mimwrite(tmp_mp4, frames, fps=FPS, codec="libx264", quality=8)
    mov = os.path.join(OUT, "joint_rom.mov")
    subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-i", tmp_mp4,
                    "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2",
                    "-c:v", "libx264", "-crf", "20", "-pix_fmt", "yuv420p",
                    "-movflags", "+faststart", mov],
                   check=True, capture_output=True)
    print(f"wrote {mov}  ({total} frames @ {FPS} fps, "
          f"{os.path.getsize(mov) // 1024} KB)")


if __name__ == "__main__":
    j = None
    for a in sys.argv:
        if a.startswith("--jobs="):
            j = int(a.split("=", 1)[1])
    main(jobs=j)
