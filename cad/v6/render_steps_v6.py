"""Step-by-step assembly figures for docs/assembly.md, rendered from cad/v6.

    MUJOCO_GL=cgl .venv/bin/python cad/v6/render_steps_v6.py      # Mac
    MUJOCO_GL=egl .venv/bin/python cad/v6/render_steps_v6.py      # Linux
        -> docs/assembly/v6_NN_<step>.png

One still per bench step, in docs/assembly.md's order. Parts already installed
render in their natural colours (assembly_v6's); the part going in renders
ORANGE, backed off along its insertion path, with a translucent ghost at its
seat and a dashed leader between the two. The paths are animate_v6.INSERT's
(the fly-in's), except where the bench goes in a different direction than the
bottom-up fly-in, each named in BENCH below with the part module that says so.

Everything is the robot to print: the default build (ARMS=1, bearing C -- a
placeholder until #75, so the bearing itself is not drawn), both legs built
the same way (the left one is shown).
"""
from __future__ import annotations

import math
import os
import sys
import tempfile

os.environ.setdefault("MUJOCO_GL", "cgl" if sys.platform == "darwin" else "egl")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

import numpy as np  # noqa: E402
from build123d import export_stl  # noqa: E402
import assembly_v6 as A  # noqa: E402
import animate_v6 as AN  # noqa: E402

OUT = os.path.join(HERE, "..", "..", "docs", "assembly")
ORANGE = (0.93, 0.46, 0.10, 1)
GHOST = (0.93, 0.46, 0.10, 0.20)
DASH = (0.80, 0.36, 0.06, 1)
HIDE = np.array([0.0, 0.0, 8.0])
N_DASH = 80
DASH_LEN, DASH_GAP, DASH_R = 0.004, 0.006, 0.0010
W, H = 960, 720

# Where the bench differs from the fly-in's bottom-up path ("comes from" unit
# vectors, world frame, as animate_v6.INSERT):
BENCH = {
    "servo_ankle_pitch_L": (0, 0, -1),   # grip channels take the case from below (leg_link_v6.SERVO_INSERT)
    "servo_knee_L": (0, 0, -1),
    "servo_hip_pitch_L": (0, 0, -1),
    "yaw_carrier_L": (0, 0, -1),         # offered UP onto the yaw horn, pelvis upright on a stand
    "servo_hip_roll_L": (0, 0, -1),      # up into the downward-open bay (cad/check_assembly SERVO_INSERT yaw_carrier)
    "neck_floor": (0, 0, -1),            # up into the tube from below, on the bench (neck_floor.INSERT)
}

LEG = ["hip_yoke_L", "servo_hip_pitch_L", "thigh_L", "servo_knee_L", "shin_L",
       "servo_ankle_pitch_L", "ankle_link_L", "servo_ankle_roll_L", "foot_L"]
LEG_R = [n[:-1] + "R" for n in LEG]
HIP = ["servo_hip_yaw_L", "yaw_carrier_L", "servo_hip_roll_L"]
HIP_R = [n[:-1] + "R" for n in HIP]
TORSO = ["pelvis_v7"]
BOARDS = ["pi4_mock", "gd_mock"]
NECK = ["shoulder_girdle_v6", "neck_floor", "servo_neck"]
ARM = ["servo_shoulder_L", "arm_upper_L", "servo_elbow_L", "arm_fore_L"]
ARM_R = [n[:-1] + "R" for n in ARM]

# (file stem, caption, installed, incoming, offset mm, (azimuth, elevation), frame-on or None)
STEPS = [
    ("01_foot_servo", "3. Ankle-roll servo down into the foot cradle",
     ["foot_L"], ["servo_ankle_roll_L"], 45, (135, -25), None),
    ("02_ankle_link_grip", "4. Ankle-pitch servo up into the ankle link's grip channel",
     ["ankle_link_L"], ["servo_ankle_pitch_L"], 45, (140, -5), None),
    ("03_ankle_roll_fork", "5. Ankle link lowered onto the roll servo's two discs",
     ["foot_L", "servo_ankle_roll_L"], ["ankle_link_L", "servo_ankle_pitch_L"], 45, (135, -15), None),
    ("04_shin_on_ankle", "5. Shin (knee servo in its grip) lowered onto the ankle-pitch servo",
     ["foot_L", "servo_ankle_roll_L", "ankle_link_L", "servo_ankle_pitch_L"],
     ["shin_L", "servo_knee_L"], 50, (135, -10), ["shin_L", "servo_knee_L", "ankle_link_L"]),
    ("05_thigh_on_knee", "5. Thigh (hip-pitch servo in its grip) lowered onto the knee servo",
     ["foot_L", "servo_ankle_roll_L", "ankle_link_L", "servo_ankle_pitch_L", "shin_L", "servo_knee_L"],
     ["thigh_L", "servo_hip_pitch_L"], 50, (135, -10), ["thigh_L", "servo_hip_pitch_L", "shin_L"]),
    ("06_hip_yoke", "6. Hip yoke's pitch clevis lowered onto the hip-pitch servo",
     LEG[1:], ["hip_yoke_L"], 40, (135, -10), ["hip_yoke_L", "servo_hip_pitch_L", "thigh_L"]),
    ("07_yaw_servo", "7a. Yaw servo up into its cell, horn down",
     TORSO, ["servo_hip_yaw_L"], 45, (150, 20), ["servo_hip_yaw_L", "yaw_carrier_L"]),
    ("08_yaw_carrier", "7b. Yaw carrier up onto the yaw horn (bearing not drawn: option open, #75)",
     TORSO + ["servo_hip_yaw_L"], ["yaw_carrier_L"], 40, (150, 15), ["servo_hip_yaw_L", "yaw_carrier_L"]),
    ("09_hip_roll_servo", "7c. Hip-roll servo up into the carrier's bay",
     TORSO + ["servo_hip_yaw_L", "yaw_carrier_L"], ["servo_hip_roll_L"], 50, (150, 10),
     ["yaw_carrier_L", "servo_hip_roll_L"]),
    ("10_leg_to_hip", "8. The leg offered up: the yoke's roll clevis onto the roll servo",
     TORSO + HIP + HIP_R + LEG_R, LEG, 45, (145, -8), ["servo_hip_roll_L", "yaw_carrier_L", "hip_yoke_L"]),
    ("11_boards", "9. Pi 4B (aft) and General Driver (front) down their channels",
     TORSO + HIP + HIP_R + LEG + LEG_R, BOARDS, 45, (150, -20), ["pelvis_v7"]),
    ("12_pack", "10. The pack down through the deck aperture, before the girdle",
     TORSO + HIP + HIP_R + LEG + LEG_R + BOARDS, ["pack_mock"], 45, (150, -30), ["pelvis_v7"]),
    ("13_neck_floor", "11 (bench). Neck floor up into the girdle's tube, four screws into the bosses",
     ["shoulder_girdle_v6"], ["neck_floor"], 30, (150, 25), ["neck_floor"]),
    ("14_neck_servo", "11 (bench). Neck servo down into the tube onto the floor's pads",
     ["shoulder_girdle_v6", "neck_floor"], ["servo_neck"], 45, (150, -25), ["shoulder_girdle_v6"]),
    ("15_girdle", "11a. The girdle, neck built in, lowered onto the deck",
     TORSO + HIP + HIP_R + LEG + LEG_R + BOARDS + ["pack_mock"], NECK, 45, (150, -25), ["pelvis_v7", "shoulder_girdle_v6"]),
    ("16_shoulder_servos", "11b. Shoulder servos down into their pods, screwed from outboard",
     TORSO + BOARDS + ["pack_mock"] + NECK, ["servo_shoulder_L", "servo_shoulder_R"], 45, (150, -30),
     ["shoulder_girdle_v6"]),
    ("17_upper_arm", "11c. Upper arm straight in onto the shoulder horn",
     TORSO + NECK + ["servo_shoulder_L", "servo_shoulder_R"], ["arm_upper_L"], 45, (110, -10),
     ["servo_shoulder_L", "arm_upper_L"]),
    ("18_elbow_servo", "11c. Elbow servo into the forearm's grip channel, from the front",
     ["arm_fore_L"], ["servo_elbow_L"], 45, (150, -10), None),
    ("19_forearm", "11c. Forearm lifted up between the upper arm's fork tines",
     ["arm_upper_L", "servo_shoulder_L"], ["arm_fore_L", "servo_elbow_L"], 45, (120, -5),
     ["arm_fore_L", "servo_elbow_L", "arm_upper_L"]),
    ("20_head", "11e. Head down onto the neck horn",
     TORSO + NECK + ["servo_shoulder_L", "servo_shoulder_R"], ["head"], 45, (150, -15),
     ["head", "shoulder_girdle_v6"]),
    ("21_complete", "The robot as drawn",
     TORSO + HIP + HIP_R + LEG + LEG_R + BOARDS + ["pack_mock", "head"] + NECK
     + ["servo_shoulder_L", "servo_shoulder_R"] + ARM[1:] + ARM_R[1:], [], 0, (150, -10), None),
]


# per-step camera distance, in units of "the framed box's diagonal just fits"
ZOOM = {"07_yaw_servo": 2.2, "08_yaw_carrier": 2.0, "09_hip_roll_servo": 2.0, "10_leg_to_hip": 1.6,
        "13_neck_floor": 2.4, "14_neck_servo": 1.4, "17_upper_arm": 1.5}


def comes_from(label):
    if label in BENCH:
        return np.array(BENCH[label], float)
    for pref, d, _ in AN.INSERT:
        if label.startswith(pref):
            v = np.array(d, float)
            if v[1] and label.endswith("_R"):
                v = v * np.array([1.0, -1.0, 1.0])
            return v
    return np.array([0.0, 0.0, 1.0])


def quat_to(v):
    z = np.array([0.0, 0.0, 1.0])
    v = v / np.linalg.norm(v)
    if np.allclose(v, z):
        return np.array([1.0, 0, 0, 0])
    if np.allclose(v, -z):
        return np.array([0.0, 1.0, 0, 0])
    ax = np.cross(z, v)
    q = np.concatenate([[1.0 + z @ v], ax])
    return q / np.linalg.norm(q)


def caption(img, text):
    from PIL import Image, ImageDraw, ImageFont
    im = Image.fromarray(img)
    dr = ImageDraw.Draw(im)
    font = None
    for path in ("/System/Library/Fonts/Helvetica.ttc", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if os.path.exists(path):
            font = ImageFont.truetype(path, 22)
            break
    dr.rectangle([0, 0, W, 36], fill=(245, 246, 248))
    dr.text((14, 7), text, fill=(30, 30, 34), font=font)
    # a palette PNG keeps each figure well under 200 KB
    return im.convert("RGB").quantize(colors=128, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE)


def main():
    import mujoco
    os.makedirs(OUT, exist_ok=True)
    comp = A.robot({})
    tmp = tempfile.mkdtemp(prefix="v6steps_")
    parts = {}
    for i, ch in enumerate(comp.children):
        path = os.path.join(tmp, f"p{i}.stl")
        export_stl(ch, path)
        bb = ch.bounding_box()
        c = tuple(ch.color) if ch.color else (0.8, 0.8, 0.8, 1)
        parts[ch.label] = (f"p{i}", (c[0], c[1], c[2], 1),
                           np.array([bb.min.X, bb.min.Y, bb.min.Z]) / 1e3, np.array([bb.max.X, bb.max.Y, bb.max.Z]) / 1e3)
    xml = [f'<mujoco><compiler meshdir="{tmp}"/>',
           f'<visual><global offwidth="{W}" offheight="{H}" fovy="35"/><headlight ambient="0.30 0.30 0.32" diffuse="0.38 0.38 0.38"/></visual>',
           '<asset><texture name="sky" type="skybox" builtin="gradient" rgb1="0.95 0.96 0.97" rgb2="0.84 0.86 0.90" width="256" height="256"/>']
    xml += [f'<mesh name="{n}" file="{n}.stl" scale="0.001 0.001 0.001"/>' for n, *_ in parts.values()]
    xml += ['</asset><worldbody><light pos="0.5 -0.7 1.4" dir="-0.4 0.55 -0.8" directional="true" diffuse="0.45 0.45 0.45"/>']
    for lab, (n, rgba, *_ ) in parts.items():
        for sfx in ("", "__ghost"):
            xml.append(f'<body name="{n}{sfx}" mocap="true" pos="0 0 8"><geom name="{n}{sfx}" type="mesh" mesh="{n}" '
                       f'contype="0" conaffinity="0" rgba="{rgba[0]:.3f} {rgba[1]:.3f} {rgba[2]:.3f} 1"/></body>')
    for i in range(N_DASH):
        xml.append(f'<body name="dash{i}" mocap="true" pos="0 0 8"><geom type="capsule" size="{DASH_R} {DASH_LEN / 2}" '
                   f'contype="0" conaffinity="0" rgba="{DASH[0]} {DASH[1]} {DASH[2]} 1"/></body>')
    xml.append('</worldbody></mujoco>')
    m = mujoco.MjModel.from_xml_string("\n".join(xml))
    d = mujoco.MjData(m)
    r = mujoco.Renderer(m, H, W)
    cam = mujoco.MjvCamera()
    for stem, text, installed, incoming, mag, (az, el), frame_on in STEPS:
        used = 0
        lo, hi = np.full(3, 1e9), np.full(3, -1e9)
        for lab, (n, rgba, bmin, bmax) in parts.items():
            mid, gmid = m.body(n).mocapid[0], m.body(n + "__ghost").mocapid[0]
            gid, ggid = m.geom(n).id, m.geom(n + "__ghost").id
            d.mocap_pos[gmid] = HIDE
            m.geom_rgba[ggid] = GHOST
            if lab in incoming:
                off = comes_from(lab) * mag / 1e3
                d.mocap_pos[mid] = off
                m.geom_rgba[gid] = ORANGE
                d.mocap_pos[gmid] = (0, 0, 0)
                ctr = (bmin + bmax) / 2
                seg = -off
                L = float(np.linalg.norm(seg))
                nd = max(1, int(L / (DASH_LEN + DASH_GAP)))
                for k in range(nd):
                    if used >= N_DASH:
                        break
                    dm = m.body(f"dash{used}").mocapid[0]
                    d.mocap_pos[dm] = ctr + off + seg * (k + 0.5) / nd
                    d.mocap_quat[dm] = quat_to(seg)
                    used += 1
                if frame_on is None or lab in frame_on:
                    lo = np.minimum(lo, np.minimum(bmin, bmin + off))
                    hi = np.maximum(hi, np.maximum(bmax, bmax + off))
            elif lab in installed:
                d.mocap_pos[mid] = (0, 0, 0)
                m.geom_rgba[gid] = rgba
                if frame_on is None or lab in frame_on:
                    lo, hi = np.minimum(lo, bmin), np.maximum(hi, bmax)
            else:
                d.mocap_pos[mid] = HIDE
        for i in range(used, N_DASH):
            d.mocap_pos[m.body(f"dash{i}").mocapid[0]] = HIDE
        mujoco.mj_forward(m, d)
        cam.lookat[:] = (lo + hi) / 2
        diag = float(np.linalg.norm(hi - lo))
        cam.distance = max(0.12, ZOOM.get(stem, 1.15) * diag / (2 * math.tan(math.radians(35) / 2)))
        cam.azimuth, cam.elevation = az, el
        r.update_scene(d, cam)
        img = caption(r.render(), text)
        path = os.path.join(OUT, f"v6_{stem}.png")
        img.save(path, optimize=True)
        print(f"wrote {os.path.relpath(path)}  {os.path.getsize(path) / 1024:.0f} KB")


if __name__ == "__main__":
    main()
