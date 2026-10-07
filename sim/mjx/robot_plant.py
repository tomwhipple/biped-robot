"""The robot's training plant: the env settings that make BimoMJXEnv /
BimoWalkerEnv the 17-joint robot (DESIGN.md) instead of the 10-joint
prototype. One place, so the trainer (train_mjx.py --robot), the referee and
the tests configure the same robot.

Nothing here is read by the envs themselves: a run's config.json pins every
value explicitly (the kp map as {actuator: factor}, the holds as
{actuator: deg}), so a later edit here never re-means an old run.

    ROBOT_XML        sim/bimo_biped_v6ar.xml (CAD inertials; weighed masses
                     replace them once the parts exist, issue #82)
    policy joints    the 12 leg joints -- 6 per leg, ankle roll included
    held joints      neck yaw at 0 (the camera looks ahead), and -- where
                     the plant has them -- both arms at their folded rest
                     pose, shoulder 15 deg forward and elbow folded 95 deg
                     (cad/v6/dimensions_v6.ARM_REST, DESIGN.md 5.4: there the
                     arms clear every leg joint's full range)
    stiffness        the robot's servo set (DESIGN.md section 4): the hip
                     rolls are STS3250s (the model's 4x), the ankle-roll and
                     knee STS3215s run the bench's P 96 / D 0 (~2.8x). The
                     STS3250's larger stall and speed are not modelled (the
                     env has one STS3215 envelope), which under-credits the
                     hip rolls
    hip pitch        policy range -120 deg flexion (the leg link's relief
                     gives 120; DESIGN.md section 3)
    payload          none: the camera is in the head, which the plant models
    imitation        the sole-level ankle reference (mimic_sole_level)

Pure python + mujoco (no jax), so the CPU side imports it without the MJX
stack.
"""
import os
import sys

import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT_XML = "bimo_biped_v6ar.xml"

# rest pose per held joint role (deg); a joint the plant does not have is
# simply not held. The arms' values are cad/v6/dimensions_v6.ARM_REST
# (tests/test_v6_design_gates.py pins them to it and to the plant's qpos0).
HOLD_DEG = {"neck_yaw": 0.0, "shoulder": -15.0, "elbow": -95.0}
KP_SCALE = "robot"              # env_mjx.SERVO_KP_PRESETS
HIP_FLEX_DEG = 120.0


# A 17-servo stand-in for tests, until the committed plant carries the arms:
# sim/gen_plant_v6.py with the as-drawn arm geometry (upper arm and forearm
# 160 mm with an elbow servo, the shoulder girdle and its servos on the
# torso, CAD link masses -- the get-up study's r5_asdrawn_rom120 arms).
# Generator defaults otherwise (geometry-derived inertials, not the CAD ones
# the committed plant carries): it exercises the 17-servo layout, it is not
# the robot's mass model.
ARMS_TEST_PARAMS = dict(
    arms=True, arm_z=0.09016, arm_len=0.16, arm_elbow=True, arm_fore_len=0.16,
    arm_shoulder_x=0.0, arm_shoulder_y_extra=0.023, arm_cad_servos=True,
    arm_girdle=True, arm_mass=0.0331, arm_fore_mass=0.0337, m_girdle=0.084,
    arm_hold=HOLD_DEG["shoulder"], arm_elbow_hold=HOLD_DEG["elbow"],
    hip_pitch_range=(-120.0, 90.0))


def write_arms_test_plant(path):
    """Write the 17-servo stand-in plant (ARMS_TEST_PARAMS) to path."""
    sim_dir = os.path.normpath(os.path.join(HERE, ".."))
    if sim_dir not in sys.path:
        sys.path.insert(0, sim_dir)
    from gen_plant_v6 import DesignParams, build_xml
    with open(path, "w") as f:
        f.write(build_xml(DesignParams(**ARMS_TEST_PARAMS)))
    return path


def robot_xml_path(xml=None):
    """Absolute path of the robot plant (a bare basename resolves in sim/)."""
    xml = xml or ROBOT_XML
    if os.path.exists(xml):
        return os.path.abspath(xml)
    return os.path.normpath(os.path.join(HERE, "..", os.path.basename(xml)))


def _role(name):
    for side in ("L_", "R_"):
        if name.startswith(side):
            return name[2:]
    return name


def is_robot_model(m):
    """The robot's plant, told apart from every prototype plant by what it
    has: the ankle-roll joints (the 6-DOF leg) and the head camera site. The
    referee keys its per-plant conditions on this, not on a file name."""
    names = {m.joint(j).name for j in range(m.njnt)}
    has_cam = any(m.site(i).name == "camera" for i in range(m.nsite))
    return {"L_ankle_roll", "R_ankle_roll"} <= names and has_cam


def robot_env_kwargs(xml=None):
    """Env kwargs for the robot plant, resolved against the plant's own
    actuators: held_joints lists only the neck/arm servos it has, and the
    stiffness map is explicit per actuator (what config.json records)."""
    sim_dir = os.path.normpath(os.path.join(HERE, ".."))
    if sim_dir not in sys.path:
        sys.path.insert(0, sim_dir)
    from walker_env import servo_kp_scale_vector   # jax-free copy
    path = robot_xml_path(xml)
    m = mujoco.MjModel.from_xml_path(path)
    names = [m.joint(int(j)).name for j in m.actuator_trnid[:, 0]]
    held = {n: HOLD_DEG[_role(n)] for n in names if _role(n) in HOLD_DEG}
    scale = servo_kp_scale_vector(names, KP_SCALE)
    kp_map = {n: float(s) for n, s in zip(names, scale) if s != 1.0}
    return dict(xml_path=path, held_joints=held, servo_kp_scale=kp_map,
                hip_flex_deg=HIP_FLEX_DEG, payload_mass=0.0,
                mimic_sole_level=True)
