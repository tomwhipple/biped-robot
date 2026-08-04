"""Fly-in assembly animation: every part travels its real insertion path.

Run:  .venv/bin/python cad/animate_assembly.py
      -> cad/renders/assembly_flyin.gif

Purpose is assembly *feasibility*, not eye candy: parts approach their seats
along the direction they must actually be inserted (roll servos slide UP into
the carrier bays, links slide onto cases from the front, the driver board
slides FORWARD into its frame from behind and the pack drops through the
guard's top aperture), so a blocked insertion path shows up as one part passing
through another. Complements check_assembly.py, which only tests the assembled
state.

Parts are enumerated from export_assembly.robot, so this survives part-set
changes as long as labels keep their prefixes.
"""
import os
import shutil
import numpy as np
import mujoco
import imageio.v2 as imageio
from build123d import export_stl

import export_assembly as A

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "renders")
TMP = os.path.join(OUT, "_anim")
os.makedirs(TMP, exist_ok=True)

# label prefix -> (insertion vector in mm world frame, extra delay in frames
# before this stage starts -- the battery waits for the guard to finish so the
# drop into the bay is actually demonstrated, not raced)
# Each fastener group flies in RIGHT AFTER the part it holds, along the axis the
# driver actually works on -- so a screw that cannot be reached shows up here as
# a screw passing through something on its way in. Matching is by bare label
# (stage_of strips no prefix, and the L_/R_ tag is added afterwards), so these
# must stay full labels: "screws_hip_roll" and "screws_hip_pitch" are distinct.
PLAN = [
    ("pelvis",           (0, 0, 120), 0),  # chassis lowered in from above
    ("servo_hip_yaw",    (0, 0, -70), 0),  # presses UP against the deck underside
    ("screws_deck",      (0, 0, -90), 0),  # stator screws up through the deck
    ("yaw_carrier",      (0, 0, -70), 4),  # bolts UP onto the yaw horn
    ("screws_yaw_stack", (0, 0, -90), 0),  # carrier onto the horn, from below
    ("servo_hip_roll",   (0, 0, -80), 0),  # slides UP into the carrier U-slot
    ("yoke_roll",        (0, 0, -60), 0),  # clevis up over the roll servo
    ("screws_hip_roll",  (90, 0, 0), 0),   # roll disc screws along the roll axis
    ("screws_flange",    (0, 0, -90), 0),  # flange bolts up into the carrier
    ("yoke_pitch",       (0, 0, -60), 0),  # bolts to the roll flange from below
    ("servo_hip_pitch",  (0, 0, -60), 0),  # thigh servo up into the pitch clevis
    ("screws_hip_pitch", (0, 90, 0), 0),   # pitch disc screws along the pitch axis
    ("link_thigh",       (80, 0, 0), 0),   # grip channel slides on from the front
    ("screws_thigh_grip", (0, 90, 0), 0),  # grip screws into the case, sideways
    ("servo_knee",       (0, 0, -60), 0),  # up into the thigh fork
    ("screws_knee",      (0, 90, 0), 0),   # knee disc screws
    ("link_shin",        (80, 0, 0), 0),   # grip channel from the front
    ("screws_shin_grip", (0, 90, 0), 0),
    ("servo_ankle",      (0, 0, -60), 0),  # up into the shin fork
    ("screws_ankle",     (0, 90, 0), 0),   # ankle disc screws
    ("foot",             (0, 0, -60), 0),  # sole rises to pocket the ankle servo
    ("screws_foot",      (0, 90, 0), 0),   # retention screws through the tabs
    # --- v5 torso (2026-08-04). Every one of these directions changed with the
    # revision, because the torso stopped being a stack on top of the deck and
    # became two assemblies hung off the housing's two faces. Order is the real
    # assembly order: tray up from underneath, frame on the back, board into the
    # frame, pack last -- it is the only item that is ever removed again, and it
    # goes in through the deck aperture with everything else already bolted on.
    ("battery_tray",     (0, 0, -90), 0),  # UP from BELOW: the tray is
                                           # underslung and bolts to the deck
                                           # underside, so it is offered up into
                                           # place, not dropped on
    ("screws_tray",      (0, 0, 90), 0),   # ...but its four M3 come from ABOVE,
                                           # down through the deck into the
                                           # tray's heat-sets. Opposite
                                           # directions in consecutive stages,
                                           # which is exactly the point of
                                           # giving each group its own axis
    ("board_frame",      (-60, 0, 0), 0),  # slides FORWARD from aft onto the
                                           # housing's rear wall: the frame is
                                           # a U opening aft, entirely behind
                                           # DECK_AFT_X, so nothing is threaded
    ("screws_board_frame", (-40, 0, 0), 0),  # 4x M3 follow it in along +x into
                                           # the rear-wall rib heat-sets, board
                                           # still OFF (the PCB covers them)
    ("board_pcb",        (-70, 0, 0), 0),  # board slides forward from aft onto
                                           # the standoffs, down the chases
    ("board_parts",      (-70, 0, 0), -9),  # ...its fitted components are the
                                           # SAME physical board, so cancel one
                                           # STAGGER and fly them together --
                                           # staggered, they read as a separate
                                           # part being installed
    ("screws_driver_board", (-60, 0, 0), 0),  # M2.5 follow it in, +x from aft
    ("battery_3s",       (0, 0, 90), 30),  # pack drops from ABOVE through the
                                           # deck APERTURE into the tray -- the
                                           # swap path, and the reason there is
                                           # a hole in the deck at all. The 30
                                           # frames of delay buy the camera time
                                           # to come back round to the front
                                           # before the pack falls (see the
                                           # azimuth block below)
]
# The belt is NOT modelled: it is a 20 mm hook-loop strap, not a part, and a
# rigid mock of it flying in on a straight line would be the one lie in an
# animation whose whole purpose is that the motions are real.
COLOR = {"servo": (0.22, 0.23, 0.27, 1), "camera": (0.10, 0.10, 0.12, 1),
         "battery_3s": (0.16, 0.30, 0.55, 1),   # NOT "battery": that prefix
         "board_pcb": (0.05, 0.32, 0.18, 1),    # also catches battery_tray,
         "board_parts": (0.15, 0.15, 0.17, 1),  # which is a printed part
         "foot": (0.70, 0.72, 0.78, 1), "": (0.80, 0.82, 0.86, 1)}


def stage_of(label):
    for i, (prefix, vec, _) in enumerate(PLAN):
        if label.startswith(prefix):
            return i, np.array(vec) / 1000.0
    raise KeyError(f"no insertion plan for part '{label}' — extend PLAN")


def color_of(label):
    # order matters for the same reason PLAN's does: "battery_3s" must be
    # tested before any shorter "battery*" key would be, or battery_tray --
    # a printed part -- comes out battery-blue.
    for prefix in ("servo", "camera", "battery_3s", "board_pcb", "board_parts",
                   "foot"):
        if label.startswith(prefix):
            return COLOR[prefix]
    return COLOR[""]


parts = []                                # (name, stage, vec, stl, rgba)
def collect(c, tag=""):
    for ch in c.children:
        if list(ch.children):
            collect(ch, ch.label.replace("leg_", "") + "_")
        else:
            name = tag + ch.label
            stage, vec = stage_of(ch.label)
            stl = os.path.join(TMP, f"{name}.stl")
            export_stl(ch, stl)
            parts.append((name, stage, vec, stl, color_of(ch.label)))
collect(A.robot)
print(f"{len(parts)} parts across {len(PLAN)} assembly stages")

assets = "\n".join(
    f'<mesh name="{n}" file="{s}" scale="0.001 0.001 0.001"/>'
    for n, _, _, s, _ in parts)
bodies = "\n".join(
    f'<body name="{n}" mocap="true">'
    f'<geom type="mesh" mesh="{n}" contype="0" conaffinity="0" '
    f'rgba="{c[0]} {c[1]} {c[2]} {c[3]}"/></body>'
    for n, _, _, _, c in parts)

XML = f"""
<mujoco>
  <visual><headlight ambient="0.45 0.45 0.47" diffuse="0.5 0.5 0.5"/>
    <global offwidth="560" offheight="460"/>
    <quality shadowsize="2048"/></visual>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient"
             rgb1="0.93 0.94 0.96" rgb2="0.80 0.83 0.88" width="256" height="256"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.90 0.91 0.93"
             rgb2="0.84 0.86 0.89" width="512" height="512"/>
    <material name="floor" texture="grid" texrepeat="12 12"/>
    {assets}
  </asset>
  <worldbody>
    <light pos="0.5 -0.7 1.2" dir="-0.4 0.55 -0.8" castshadow="true"/>
    <geom type="plane" size="2 2 0.1" material="floor"/>
    {bodies}
  </worldbody>
</mujoco>
"""
m = mujoco.MjModel.from_xml_string(XML)
d = mujoco.MjData(m)
mocap = {n: m.body(n).mocapid[0] for n, *_ in parts}

FPS, INTRO, STAGGER, TRAVEL, HOLD = 16, 8, 9, 24, 20
START = []                                           # per-stage start frame
acc = INTRO
for i, (_, _, extra) in enumerate(PLAN):
    acc += (STAGGER if i else 0) + extra
    START.append(acc)
total = START[-1] + TRAVEL + HOLD
_stages = [p for p, _, _ in PLAN]
BATT_START = START[_stages.index("battery_3s")]
# v5: the first thing inserted from AFT is the FRAME, not the board -- the swing
# has to be round before it arrives, or the frame lands off-screen and only its
# contents are seen going in.
AFT_START = START[_stages.index("board_frame")]
r = mujoco.Renderer(m, height=460, width=560)
cam = mujoco.MjvCamera()
mujoco.mjv_defaultCamera(cam)
cam.lookat[:] = [0.01, 0.0, 0.185]
cam.distance, cam.elevation = 0.80, -10


def smoothstep(t):
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3 - 2 * t)


frames = []
for f in range(total):
    for n, stage, vec, _, _ in parts:
        ease = smoothstep((f - START[stage]) / TRAVEL)
        d.mocap_pos[mocap[n]] = (1 - ease) * vec
    mujoco.mj_forward(m, d)
    # Slow orbit for depth cues, plus TWO keyed 180 deg swings. v3 had one, to
    # catch the pack going in through the tower's -x wall window. Since v4 there
    # are two insertions worth watching and they are on opposite faces, so the
    # camera goes round for them and comes back for the pack:
    #   +180 before the AFT block -> frame, then board, then both screw groups
    #   -180 before the battery   -> back to the +x FRONT, where the aperture is
    # It nets to the front, which is also the right note to end the hold on.
    # Each swing is keyed to COMPLETE before its stage starts and the return is
    # keyed off the pack's own delay, so neither insertion is watched through a
    # moving camera -- at the first cut the return began while the board was
    # still traveling and the arrival happened off-screen.
    cam.azimuth = (140 + 30 * f / total
                   + 180 * smoothstep((f - (AFT_START - 20)) / 20)
                   - 180 * smoothstep((f - (BATT_START - 26)) / 20))
    r.update_scene(d, cam)
    frames.append(r.render().copy())

gif = os.path.join(OUT, "assembly_flyin.gif")
imageio.mimwrite(gif, frames, fps=FPS, loop=0)     # for embedding in the report
mov = os.path.join(OUT, "assembly_flyin.mov")      # for local viewing (Preview
import subprocess, imageio_ffmpeg                  # doesn't animate GIFs)
subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-i", gif,
                "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2",
                "-c:v", "libx264", "-crf", "20", "-pix_fmt", "yuv420p",
                "-movflags", "+faststart", mov],
               check=True, capture_output=True)
shutil.rmtree(TMP)
for path in (gif, mov):
    print(f"wrote {path}  ({total} frames @ {FPS} fps, "
          f"{os.path.getsize(path) // 1024} KB)")
