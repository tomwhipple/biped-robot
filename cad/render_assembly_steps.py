"""Step-by-step assembly figures for docs/assembly.md.

Run:  .venv/bin/python cad/render_assembly_steps.py
      -> docs/assembly/step_*.png

One still per assembly step, in the bench order of docs/assembly.md: parts
already installed render in their natural colors, the part(s) being installed
render ORANGE, offset along their real insertion direction. Since 2026-07-30
each figure also SHOWS that direction rather than implying it:

  * a translucent GHOST of the incoming part at its seated pose, so you can see
    where it is going;
  * a DASHED LEADER from the ghost to the orange part, along the insertion
    axis -- which for every servo is check_assembly.SERVO_INSERT's axis, the
    one the interference sweep actually proved clear;
  * the step's screws at their seats plus their translucent INSERTION SWEEPS
    (fasteners.py, via export_assembly.insertion_paths) -- the head/driver
    envelope swept back off each seat, i.e. the air the screw and the bit need.
    Same solids check_assembly intersects with the parts, so a figure cannot
    show an approach nobody checked.

Parts from later steps are hidden. Both legs are shown in the leg steps
because you build two of everything.

Parts are enumerated from export_assembly.robot, so this survives part-set
changes as long as labels keep their prefixes (extend FIGS if parts appear).
"""
import os
import numpy as np
import mujoco
import imageio.v2 as imageio
from build123d import export_stl

import check_assembly as CA
import export_assembly as A

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "docs", "assembly")
TMP = os.path.join(HERE, "renders", "_steps")
os.makedirs(OUT, exist_ok=True)
os.makedirs(TMP, exist_ok=True)

ORANGE = (0.93, 0.46, 0.10, 1)
GHOST = (0.93, 0.46, 0.10, 0.18)          # incoming part at its SEATED pose
SWEEP = (0.20, 0.55, 0.95, 0.22)          # screw/driver insertion sweeps
DASH = (0.80, 0.36, 0.06, 1)              # leader dashes along the axis
COLOR = {"servo": (0.22, 0.23, 0.27, 1), "camera": (0.10, 0.10, 0.12, 1),
         "battery": (0.16, 0.30, 0.55, 1), "screws": (0.62, 0.64, 0.70, 1),
         "foot": (0.70, 0.72, 0.78, 1), "": (0.80, 0.82, 0.86, 1)}
HIDE = np.array([0.0, 0.0, 8.0])          # parked far above every camera frame
N_DASH = 60                               # pool of leader-dash bodies
DASH_LEN, DASH_GAP, DASH_R = 0.005, 0.010, 0.0012      # metres

# whole leg subassembly, proximal to distal (base labels, both legs match)
LEG = ["yoke_roll", "yoke_pitch", "servo_hip_pitch", "link_thigh",
       "servo_knee", "link_shin", "servo_ankle", "foot"]
# v3yaw hip-yaw stack (x2): yaw servo under the deck + carrier on its horn
YAW = ["servo_hip_yaw", "yaw_carrier"]
ALL = ["pelvis", "tower", "battery_3s_mock", "imu_carrier", "imu_bno055_mock",
       "gopro_base", "camera_gopro_max_mock"] + YAW + LEG


def along(key, mag, back=False):
    """`mag` mm along check_assembly's insertion axis for a component. `back`
    flips it, which is what a part that the SERVO enters needs: the servo
    leaves the leg_link channel along +z, so the link leaves the servo along
    -z. Offsets are exaggerated for legibility, never re-aimed."""
    ax = CA.SERVO_INSERT[key][0]
    s = -mag if back else mag
    return tuple(c * s for c in ax)


# exploded-view offsets (mm) = the fly-in insertion vectors, exaggerated. The
# four servo insertions are generated from check_assembly.SERVO_INSERT so the
# drawing and the check can never disagree about which way a servo goes in;
# 2026-07-30 that caught three of these entries pointing the wrong way (the
# yaw servo drawn rising THROUGH its deck seat, the ankle servo drawn sinking
# INTO the sole, and both leg_links drawn sliding on "from the front" -- which
# drives the horn disc into the grip plate, 185 mm3 in).
EXPLODE = {"pelvis": (0, 0, 120), "servo_hip_yaw": along("pelvis", 55),
           "yaw_carrier": (0, 0, -30),
           "servo_hip_roll": along("yaw_carrier", 95),
           "yoke_roll": (0, 0, -70), "yoke_pitch": (0, 0, -125),
           "servo_hip_pitch": (0, 0, -95),
           "link_thigh": along("leg_link", 55, back=True),
           "servo_knee": (0, 0, -78),
           "link_shin": along("leg_link", 55, back=True),
           "servo_ankle": along("foot", 60), "foot": (0, 0, -80),
           "tower": (0, 0, 175), "battery_3s_mock": (-95, 0, 175),
           "imu_carrier": (0, 0, 195), "imu_bno055_mock": (0, 0, 220),
           "gopro_base": (0, 0, 245), "camera_gopro_max_mock": (0, 0, 290)}

# (file, placed base-labels, {incoming base-label: offset mm},
#  (lookat y, lookat z, distance, azimuth, elevation), name-prefix filter)
# The single-leg close-ups (steps 2-5) render the LEFT leg only -- both legs
# overlap too much in perspective; the doc says "x2" instead.
L = ("L_",)
FIGS = [
    ("step00_exploded", [], EXPLODE, (0.0, 0.30, 0.95, 140, -8), None),
    ("step01_foot_servo", ["foot"], {"servo_ankle": along("foot", 55)},
     (0.0, 0.035, 0.30, 125, -18), None),
    ("step02_link_on_case", ["servo_knee"],
     {"link_shin": along("leg_link", 60, back=True)},
     (0.028, 0.055, 0.30, 140, -8), L),
    ("step03_fork_to_horn", ["foot", "servo_ankle"],
     {"link_shin": (0, 0, 60), "servo_knee": (0, 0, 60)},
     (0.028, 0.065, 0.34, 125, -12), L),
    ("step04_yoke_pitch", ["servo_hip_pitch", "link_thigh"],
     {"yoke_pitch": (0, 0, 50)}, (0.028, 0.215, 0.28, 130, -8), L),
    ("step05_yoke_roll", ["yoke_pitch", "servo_hip_pitch", "link_thigh"],
     {"yoke_roll": (0, 0, 50)}, (0.028, 0.235, 0.28, 130, -8), L),
    # v3yaw pelvis sequence: yaw servos seat UP under the deck, carriers bolt UP
    # onto the yaw horns, roll servos slide UP into the carrier U-slots.
    ("step06_yaw_servos", ["pelvis"], {"servo_hip_yaw": along("pelvis", 55)},
     (0.0, 0.30, 0.40, 140, 12), None),
    ("step06a_carriers", ["pelvis", "servo_hip_yaw"],
     {"yaw_carrier": (0, 0, -70)}, (0.0, 0.275, 0.44, 140, 4), None),
    ("step06b_roll_servos", ["pelvis"] + YAW,
     {"servo_hip_roll": along("yaw_carrier", 70)},
     (0.0, 0.255, 0.46, 140, -12), None),
    ("step07_legs_to_pelvis", ["pelvis"] + YAW + ["servo_hip_roll"],
     {b: (0, 0, -55) for b in LEG}, (0.0, 0.19, 0.66, 140, -10), None),
    ("step08_tower", ["pelvis"] + YAW + ["servo_hip_roll"] + LEG,
     {"tower": (0, 0, 85)}, (0.0, 0.32, 0.54, 140, -8), None),
    ("step09_battery", ["pelvis"] + YAW + ["servo_hip_roll", "tower"] + LEG,
     {"battery_3s_mock": (-85, 0, 0)}, (0.0, 0.34, 0.48, 320, -8), None),
    ("step10_gopro_base",
     ["pelvis"] + YAW + ["servo_hip_roll", "tower", "battery_3s_mock"] + LEG,
     {"imu_carrier": (0, 0, 40), "imu_bno055_mock": (0, 0, 55),
      "gopro_base": (0, 0, 75)}, (0.0, 0.365, 0.36, 140, -6), None),
    ("step11_camera",
     ["pelvis"] + YAW + ["servo_hip_roll", "tower", "battery_3s_mock",
                         "imu_carrier", "imu_bno055_mock", "gopro_base"] + LEG,
     {"camera_gopro_max_mock": (0, 0, 75)}, (0.0, 0.385, 0.60, 140, -6), None),
    ("step12_complete", ALL + ["servo_hip_roll"], {},
     (0.0, 0.22, 0.82, 155, -8), None),
]

# Fasteners consumed by each step: their seated screws AND their insertion
# sweeps ride along, so every figure that installs hardware also shows where
# the driver has to come from. Names are export_assembly's fastener-frame
# names (screws_<name> / paths_<name>); the yaw stack's horn bolts and wall
# screws share one frame, so 7b and 7c both point at it.
STEP_SCREWS = {"step01_foot_servo": ["foot"],
               "step02_link_on_case": ["shin_grip"],
               "step03_fork_to_horn": ["ankle"],
               "step04_yoke_pitch": ["hip_pitch"],
               "step05_yoke_roll": ["flange"],
               "step06_yaw_servos": ["deck"],
               "step06a_carriers": ["yaw_stack"],
               "step06b_roll_servos": ["yaw_stack"],
               "step07_legs_to_pelvis": ["hip_roll"],
               "step08_tower": ["tower"],
               "step10_gopro_base": ["head_stack"]}


def color_of(label):
    for prefix in ("servo", "camera", "battery", "foot", "screws"):
        if label.startswith(prefix):
            return COLOR[prefix]
    return COLOR[""]


parts = []                                # (name, base label, stl, rgba, center)
def collect(c, tag="", rgba=None):
    for ch in c.children:
        if list(ch.children):
            collect(ch, ch.label.replace("leg_", "") + "_", rgba)
        else:
            name = tag + ch.label
            stl = os.path.join(TMP, f"{name}.stl")
            export_stl(ch, stl)
            ctr = ch.bounding_box().center()
            parts.append((name, ch.label, stl, rgba or color_of(ch.label),
                          np.array([ctr.X, ctr.Y, ctr.Z]) / 1000.0))
collect(A.robot)
collect(A.insertion_paths(), rgba=SWEEP)      # paths_* bodies, translucent
print(f"{len(parts)} bodies, {len(FIGS)} figures")

assets = "\n".join(
    f'<mesh name="{n}" file="{s}" scale="0.001 0.001 0.001"/>'
    for n, _, s, _, _ in parts)
# Every part gets a GHOST twin (same mesh, parked out of frame unless the
# figure is installing it) so an incoming part can be drawn at BOTH poses.
bodies = "\n".join(
    f'<body name="{n}{sfx}" mocap="true">'
    f'<geom name="{n}{sfx}" type="mesh" mesh="{n}" contype="0" conaffinity="0"/>'
    f'</body>' for n, *_ in parts for sfx in ("", "__ghost"))
dashes = "\n".join(
    f'<body name="dash{i}" mocap="true">'
    f'<geom name="dash{i}" type="capsule" size="{DASH_R} {DASH_LEN / 2}"'
    f' contype="0" conaffinity="0" rgba="{DASH[0]} {DASH[1]} {DASH[2]} 1"/>'
    f'</body>' for i in range(N_DASH))

XML = f"""
<mujoco>
  <visual><headlight ambient="0.5 0.5 0.52" diffuse="0.45 0.45 0.45"/>
    <global offwidth="880" offheight="660"/></visual>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient"
             rgb1="0.93 0.94 0.96" rgb2="0.80 0.83 0.88" width="256" height="256"/>
    {assets}
  </asset>
  <worldbody>
    <light pos="0.5 -0.7 1.2" dir="-0.4 0.55 -0.8"/>
    {bodies}
    {dashes}
  </worldbody>
</mujoco>
"""
m = mujoco.MjModel.from_xml_string(XML)
d = mujoco.MjData(m)
r = mujoco.Renderer(m, height=660, width=880)
cam = mujoco.MjvCamera()
mujoco.mjv_defaultCamera(cam)


def quat_to(v):
    """Quaternion (w,x,y,z) taking the capsule's +z onto unit vector v."""
    z = np.array([0.0, 0.0, 1.0])
    v = v / np.linalg.norm(v)
    if np.allclose(v, z):
        return np.array([1.0, 0, 0, 0])
    if np.allclose(v, -z):
        return np.array([0.0, 1.0, 0, 0])
    ax = np.cross(z, v)
    q = np.concatenate([[1.0 + z @ v], ax])
    return q / np.linalg.norm(q)


def lay_dashes(used, start, end):
    """Dashed leader from `start` to `end` (metres, world). Returns the new
    count of dash bodies used."""
    seg = end - start
    L = float(np.linalg.norm(seg))
    if L < 1e-6:
        return used
    q = quat_to(seg)
    step = DASH_LEN + DASH_GAP
    n = max(1, int(L / step))
    for i in range(n):
        if used >= N_DASH:
            break
        t = (i + 0.5) * step / L
        mid = m.body(f"dash{used}").mocapid[0]
        d.mocap_pos[mid] = start + seg * t
        d.mocap_quat[mid] = q
        used += 1
    return used


for fname, placed, incoming, (ly, lz, dist, azim, elev), only in FIGS:
    exploded = fname == "step00_exploded"
    show_screws = STEP_SCREWS.get(fname, [])
    used = 0
    for n, base, _, rgba, ctr in parts:
        gid, gg = m.geom(n).id, m.geom(n + "__ghost").id
        mid = m.body(n).mocapid[0]
        gmid = m.body(n + "__ghost").mocapid[0]
        d.mocap_pos[gmid] = HIDE
        m.geom_rgba[gg] = GHOST
        shown = only is None or n.startswith(only) or n == base
        fastener = base.split("_", 1)[0] in ("screws", "paths")
        if fastener:
            # hardware rides with the step that consumes it (or, in the
            # exploded overview, with everything else)
            if shown and (exploded or base.split("_", 1)[1] in show_screws):
                d.mocap_pos[mid] = (0, 0, 0)
                m.geom_rgba[gid] = rgba
            else:
                d.mocap_pos[mid] = HIDE
            continue
        if base in incoming and shown:
            off = np.array(incoming[base]) / 1000.0
            d.mocap_pos[mid] = off
            m.geom_rgba[gid] = rgba if exploded else ORANGE
            if not exploded:              # ghost at the seat + leader dashes
                d.mocap_pos[gmid] = (0, 0, 0)
                used = lay_dashes(used, ctr + off, ctr)
        elif base in placed and shown:
            d.mocap_pos[mid] = (0, 0, 0)
            m.geom_rgba[gid] = rgba
        else:
            d.mocap_pos[mid] = HIDE
    for i in range(used, N_DASH):         # park the unused leader dashes
        d.mocap_pos[m.body(f"dash{i}").mocapid[0]] = HIDE
    mujoco.mj_forward(m, d)
    cam.lookat[:] = [0.01, ly, lz]
    cam.distance, cam.azimuth, cam.elevation = dist, azim, elev
    r.update_scene(d, cam)
    path = os.path.join(OUT, f"{fname}.png")
    imageio.imwrite(path, r.render())
    print("wrote", path)

for _, _, stl, _, _ in parts:
    os.remove(stl)
os.rmdir(TMP)
