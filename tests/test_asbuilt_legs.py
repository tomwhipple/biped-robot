"""The legs-only as-built body (2026-10-24 bench) and what was calibrated on it.

  * the committed plant sim/bimo_biped_v6legs.xml is build_v6_inertia's output
    (generated files are generated), has the 12 leg actuators and no neck, head,
    pack, Pi or arms, and the masses the record states;
  * the committed 17-servo plant is untouched by the generator's new switches;
  * the robot preset and the referee recognise it as the robot's plant;
  * the roll-compliance bench lift (sim/roll_compliance.py) reproduces the
    measured ~5 deg torso tilt at ~1 deg encoder error with the calibrated
    roll deadzone, and is much stiffer without it;
  * the new DR (roll deadzone per roll joint, torso mass and lateral CoM, IMU
    mount sigma) lands where it should, in the CPU env and the MJX env.
"""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "sim")
for p in (SIM, os.path.join(SIM, "mjx")):
    sys.path.insert(0, p)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import mujoco  # noqa: E402

import robot_plant  # noqa: E402

LEGS = os.path.join(SIM, "bimo_biped_v6legs.xml")


@pytest.fixture(scope="module")
def m():
    return mujoco.MjModel.from_xml_path(LEGS)


def test_legs_plant_is_generated():
    import build_v6_inertia as BI
    src = BI.committed_legs_xml(LEGS)
    with open(LEGS) as f:
        assert f.read() == src, "sim/bimo_biped_v6legs.xml is stale: run sim/build_v6_inertia.py --asbuilt-legs --write"


def test_legs_plant_layout(m):
    names = [m.actuator(i).name for i in range(m.nu)]
    assert m.nu == 12 and m.nq == 19
    assert names == [f"{s}_{j}" for s in "LR" for j in
                     ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle", "ankle_roll")]
    bodies = {m.body(i).name for i in range(m.nbody)}
    assert not ({"head", "L_arm", "R_arm", "L_forearm", "R_forearm"} & bodies)
    joints = {m.joint(j).name for j in range(m.njnt)}
    assert not any(k in n for n in joints for k in ("neck", "shoulder", "elbow"))
    assert not any(m.site(i).name == "camera" for i in range(m.nsite))
    assert robot_plant.is_robot_model(m)


def test_legs_plant_masses(m):
    """CAD / catalogue masses: the control board only on the torso (no pack, Pi,
    power module, neck servo or head)."""
    total = float(m.body_mass.sum())
    torso = float(m.body_mass[m.body("torso").id])
    assert 1.45 < total < 1.50, total
    assert 0.40 < torso < 0.45, torso
    ar = mujoco.MjModel.from_xml_path(os.path.join(SIM, "bimo_biped_v6ar.xml"))
    assert total < 0.7 * float(ar.body_mass.sum())


def test_generator_switches_leave_the_default_plant_alone():
    """head=True and nonzero pack/Pi/power masses (the defaults) emit the plant they always did."""
    import gen_plant_v6 as G
    default = G.build_xml(G.DesignParams())
    assert "neck_yaw" in default and 'name="head"' in default and "3S 2200 mAh" in default
    import build_v6_inertia as BI
    legs = G.build_xml(BI.asbuilt_legs())
    assert "neck_yaw" not in legs and 'name="head"' not in legs and "3S 2200 mAh" not in legs
    assert "Pi 4B on the aft wall" not in legs and "General Driver on the front wall" in legs


def test_asbuilt_params_differ_only_where_stated():
    import dataclasses
    import build_v6_inertia as BI
    import gen_plant_v6 as G
    a, b = dataclasses.asdict(BI.asbuilt_legs()), dataclasses.asdict(G.DesignParams())
    diff = {k for k in a if a[k] != b[k]}
    assert diff == {"head", "m_battery", "m_pi4", "m_power", "m_neck_servo", "m_head"}


def test_robot_preset_on_the_legs_plant():
    kw = robot_plant.robot_env_kwargs(robot_plant.ROBOT_LEGS_XML)
    assert kw["held_joints"] == {}
    assert set(kw["servo_types"]) == {"L_hip_roll", "R_hip_roll"}


# ------------------------------------------------------------ roll compliance
def _lift(lash_total_extra, side="L"):
    import roll_compliance as RC
    env = RC.make_env(robot_plant.ROBOT_LEGS_XML, "R" if side == "L" else "L", roll_lash_deg=lash_total_extra)
    return RC.run(env, side)


def test_roll_compliance_matches_the_bench():
    """Measured 2026-10-24: 5.0-5.2 deg torso tilt toward the lifted foot, stance
    ankle-roll encoder 1.0-1.5 deg, hip-roll 0.7-0.9 deg. With the calibrated
    2.6 deg total roll deadzone the sim lands in those bands; the rigid-roll
    plant (no deadzone) tilts less than half as much."""
    tilt, enc, lift = _lift(2.6)
    assert 4.6 <= tilt <= 5.6, tilt
    assert 1.0 <= enc["ankle_roll"] <= 1.6, enc
    assert 0.6 <= enc["hip_roll"] <= 1.0, enc
    assert lift > 15.0          # the foot actually came off the desk (30 mm commanded)
    tilt0, enc0, _ = _lift(0.0)
    assert tilt0 < 0.55 * tilt


def test_roll_compliance_is_symmetric():
    tl, _, _ = _lift(2.6, "L")
    tr, _, _ = _lift(2.6, "R")
    assert abs(tl - tr) < 0.3


# ------------------------------------------------------------------------ DR
def _cpu_env(**kw):
    import walker_env as W
    base = dict(supply_voltage=11.1, action_map="full", ext_cmd=True, domain_rand=True,
                latency_ms=0.0, backlash_deg=0.0, push_prob=0.0, push_force=0.0, imu_obs=True)
    base.update(robot_plant.robot_env_kwargs(robot_plant.ROBOT_LEGS_XML))
    base.update(kw)
    return W.BimoWalkerEnv(actuator_model="sts3215", command_mode=True, **base)


def test_cpu_roll_lash_only_on_roll_joints():
    env = _cpu_env(roll_lash_deg=1.0, roll_lash_deg_max=2.6)
    seen = []
    for sd in range(6):
        env.reset(seed=sd)
        seen.append(np.degrees(env._roll_lash_rad))
    assert min(seen) >= 1.0 - 1e-9 and max(seen) <= 2.6 + 1e-9 and np.ptp(seen) > 0.3
    roll = [env._servo_names[i] for i in np.flatnonzero(env._roll_mask)]
    assert roll == ["L_hip_roll", "L_ankle_roll", "R_hip_roll", "R_ankle_roll"]


def test_cpu_torso_dr():
    env = _cpu_env(torso_mass_range=0.4, torso_com_y=0.015)
    nom_m = env._nom_mass[env._torso_bid]
    ms, ys = [], []
    for sd in range(40):
        env.reset(seed=sd)
        ms.append(env.model.body_mass[env._torso_bid] / nom_m)
        ys.append(env.model.body_ipos[env._torso_bid, 1] - env._nom_ipos[env._torso_bid, 1])
    assert 0.6 - 1e-9 <= min(ms) and max(ms) <= 1.4 + 1e-9 and np.ptp(ms) > 0.4
    assert max(np.abs(ys)) <= 0.015 + 1e-9 and np.ptp(ys) > 0.015
    assert min(ys) < 0 < max(ys)         # both sides


def test_cpu_dr_off_by_default():
    env = _cpu_env()
    env.reset(seed=0)
    assert env.model.body_ipos[env._torso_bid, 1] == env._nom_ipos[env._torso_bid, 1]
    assert env._roll_lash_rad == 0.0


def test_mjx_roll_lash_and_torso_dr():
    import jax
    import train_mjx as T
    args = T.make_parser().parse_args(
        "--robot --xml bimo_biped_v6legs.xml --precision --family skills --out t --envs 6 "
        "--episode-seconds 2 --servo-kp-scale robot --roll-lash-deg 1.0,2.6 --torso-mass-range 0.4 "
        "--torso-com-y 0.015 --imu-mount-deg 3 --backlash-deg 0.5,1.0".split())
    kw = T.build_env_kw(args)
    assert (kw["roll_lash_deg"], kw["roll_lash_deg_max"], kw["imu_mount_deg"]) == (1.0, 2.6, 3.0)
    env = T.BimoMJXEnv(**kw)
    wrapped = T.BatchedEnv(env, env.max_steps, num_envs=6)
    st = wrapped.reset(jax.random.split(jax.random.PRNGKey(0), 6))
    lash = np.degrees(np.asarray(st.info["st"].lash))
    assert lash.shape == (6, 12)
    roll = np.flatnonzero(np.asarray(env._roll_mask))
    other = np.setdiff1d(np.arange(12), roll)
    assert lash[:, other].max() <= 1.0 + 1e-4                      # backlash only
    assert (lash[:, roll].min() >= 1.5 - 1e-4) and lash[:, roll].max() <= 3.6 + 1e-4
    assert np.allclose(lash[:, roll], lash[:, roll[:1]])           # one draw per episode
    model, axes = wrapped._batched_model(6)
    t = env._torso_bid
    ratio = np.asarray(model.body_mass)[:, t] / float(np.asarray(env._nom_mass)[t])
    assert ratio.min() >= 0.6 - 1e-5 and ratio.max() <= 1.4 + 1e-5
    dy = np.asarray(model.body_ipos)[:, t, 1] - float(np.asarray(env._nom_ipos)[t, 1])
    assert np.abs(dy).max() <= 0.015 + 1e-6 and np.ptp(dy) > 0.005
