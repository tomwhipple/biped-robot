"""bimo_biped_v4rom.xml: the ROM widening and the inter-leg contact geometry.

Run:  .venv/bin/python -m pytest tests/test_rom_contacts.py -q

v4rom is the 2026-08-03 successor to bimo_biped_v3yaw.xml. It fixes two
things the hardware/CAD session found, and this file is what stops either fix
from drifting:

  1. THE CONTACT GEOMETRY IS CALIBRATED, so it has to be pinned. The legcol
     capsules are not a CAD hull -- their radius and length were chosen so the
     plant reproduces two BENCH MEASUREMENTS at once (hip-roll adduction from
     the standing pose, and a yawed leg cross with the thighs raised). Nudging
     either number, or moving a leg segment, silently moves an onset. The
     tolerance is +-2 deg, which is about what "load onset" is worth as a
     measurement: the same sweep saw a torque-off leg come to rest at -7 deg.

  2. THE POLICY RANGE MUST NOT FOLLOW THE JOINT LIMITS. v4rom widens hip roll
     and hip pitch to the mechanical truth while holding the training range at
     v3yaw's, so a v3yaw-trained policy's actions mean the same joint angles
     on both plants. That split lives in an actuator ctrlrange, in RADIANS,
     inside a file whose compiler is angle="degree" -- exactly the kind of
     thing that gets "fixed" by someone tidying up. The degree values are
     asserted here so it cannot be.

Everything below is pure kinematics: set qpos, mj_forward, measure the
distance between the two capsules. No dynamics, no floor, no policy -- the
onsets are a property of the model, and this test is deterministic.
"""
import math
import os
import sys

import numpy as np
import pytest

import mujoco

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "sim"))

from walker_env import BimoWalkerEnv, policy_range               # noqa: E402

V4 = os.path.join(ROOT, "sim", "bimo_biped_v4rom.xml")
V3 = os.path.join(ROOT, "sim", "bimo_biped_v3yaw.xml")

COL_GEOMS = ("L_col_thigh", "L_col_shank", "R_col_thigh", "R_col_shank")

# The v3yaw policy range, in degrees, in actuator order. v4rom must train in
# exactly this box -- see the actuator block in the XML.
POLICY_DEG = {
    "hip_yaw": (-45.0, 45.0),
    "hip_roll": (-25.0, 25.0),
    "hip_pitch": (-110.0, 60.0),
    "knee": (-95.0, 5.0),
    "ankle": (-40.0, 40.0),
}
# The v4rom JOINT limits: the mechanical stop, per leg. Roll is asymmetric
# because abduction and adduction are different mechanisms -- positive roll
# moves the toe toward the robot's left, so +55 is the LEFT leg's CAD-verified
# abduction and -55 the right's.
MECH_DEG = {
    "L_hip_yaw": (-45.0, 45.0),   "R_hip_yaw": (-45.0, 45.0),
    "L_hip_roll": (-25.0, 55.0),  "R_hip_roll": (-55.0, 25.0),
    "L_hip_pitch": (-110.0, 90.0), "R_hip_pitch": (-110.0, 90.0),
    "L_knee": (-95.0, 5.0),       "R_knee": (-95.0, 5.0),
    "L_ankle": (-40.0, 40.0),     "R_ankle": (-40.0, 40.0),
}


# -- model helpers ----------------------------------------------------------
@pytest.fixture(scope="module")
def model():
    return mujoco.MjModel.from_xml_path(V4)


class Poser:
    """set a pose in degrees -> closest distance between two legcol capsules."""

    def __init__(self, m):
        self.m, self.d = m, mujoco.MjData(m)
        self.q = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j):
                  m.jnt_qposadr[j] for j in range(m.njnt)}
        self.g = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
                  for n in COL_GEOMS}

    def gaps(self, **deg):
        self.d.qpos[:] = self.m.qpos0
        for k, v in deg.items():
            self.d.qpos[self.q[k]] = math.radians(v)
        mujoco.mj_forward(self.m, self.d)
        return {(a, b): mujoco.mj_geomDistance(self.m, self.d, self.g[a],
                                               self.g[b], 0.5, None)
                for a in ("L_col_thigh", "L_col_shank")
                for b in ("R_col_thigh", "R_col_shank")}

    def min_gap(self, **deg):
        return min(self.gaps(**deg).values())

    def onset(self, pose, hi=45.0):
        """first angle (deg) at which `pose(angle)` closes a legcol pair."""
        assert self.min_gap(**pose(0.0)) > 0, "already in contact at zero"
        if self.min_gap(**pose(hi)) > 0:
            return None                       # never touches inside the sweep
        lo = 0.0
        for _ in range(50):
            mid = 0.5 * (lo + hi)
            if self.min_gap(**pose(mid)) > 0:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)


@pytest.fixture(scope="module")
def pose(model):
    return Poser(model)


# -- 1. the ranges ----------------------------------------------------------
def test_joint_limits_are_the_mechanical_truth(model):
    """Joint limits carry the measured/CAD-verified stops, not the old plant's.

    Roll abduction 55 per side: CAD buffer sweep 2026-08-03 (0.60 mm at 55,
    0.33 at 70, 0.00 at 90). Pitch +90 backward: CAD 0.70 mm, driven on both
    hips on the robot the same day. The knee stays -95..+5 -- +-95 is the
    measured envelope but hyperextension is deliberately not an operating mode.
    """
    for name, (lo, hi) in MECH_DEG.items():
        jr = np.rad2deg(model.jnt_range[model.joint(name).id])
        assert jr[0] == pytest.approx(lo, abs=1e-6), name
        assert jr[1] == pytest.approx(hi, abs=1e-6), name


def test_policy_range_is_v3yaw_unchanged(model):
    """The training box did NOT move, so v3yaw policies transfer unrescaled.

    ctrlrange is in RADIANS: the compiler applies angle="degree" to joint
    ranges and geometry, but not to actuator controls. Asserting the degree
    value here is what makes that survive the next tidy-up.
    """
    lo, hi = policy_range(model)
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        want = POLICY_DEG[name[2:]]
        assert math.degrees(lo[i]) == pytest.approx(want[0], abs=1e-4), name
        assert math.degrees(hi[i]) == pytest.approx(want[1], abs=1e-4), name
    assert model.actuator_ctrllimited.all(), "the split needs every ctrlrange"


def test_v3yaw_is_untouched_by_the_split():
    """Legacy plants declare no ctrlrange, so policy_range() is their joint
    limit and nothing about them -- or their scorecards -- moves."""
    m3 = mujoco.MjModel.from_xml_path(V3)
    assert not m3.actuator_ctrllimited.any()
    lo, hi = policy_range(m3)
    jnt = m3.actuator_trnid[:, 0]
    assert np.array_equal(lo, m3.jnt_range[jnt, 0])
    assert np.array_equal(hi, m3.jnt_range[jnt, 1])


def test_actions_mean_the_same_angles_on_both_plants():
    """The migration claim, tested: the same action produces the same joint
    targets on v3yaw and v4rom, under BOTH action maps. A warm start therefore
    begins at the old behaviour and only has to learn the new contacts."""
    for amap in ("legacy", "full"):
        e3 = BimoWalkerEnv(xml_path=V3, action_map=amap)
        e4 = BimoWalkerEnv(xml_path=V4, action_map=amap)
        assert np.allclose(e3._lo, e4._lo, atol=1e-9)
        assert np.allclose(e3._hi, e4._hi, atol=1e-9)
        assert np.allclose(e3._soft_lo, e4._soft_lo, atol=1e-9)
        assert np.allclose(e3._soft_hi, e4._soft_hi, atol=1e-9)
        rng = np.random.default_rng(0)
        for _ in range(8):
            a = rng.uniform(-1, 1, e3._nq_act)
            assert np.allclose(e3._action_to_ctrl(a), e4._action_to_ctrl(a),
                               atol=1e-9), amap


# -- 2. the measured contacts ----------------------------------------------
def test_adduction_onset_standing(pose):
    """BENCH: driving one hip roll inward from standing loads up at ~9 deg
    (docs/servo-map.md, "Model-range sweep"); a torque-off leg rests at -7.
    Both legs, because the capsules are symmetric on purpose."""
    left = pose.onset(lambda a: {"L_hip_roll": -a})
    right = pose.onset(lambda a: {"R_hip_roll": +a})
    assert left == pytest.approx(9.0, abs=2.0), f"L adduction onset {left:.2f}"
    assert right == pytest.approx(9.0, abs=2.0), f"R adduction onset {right:.2f}"
    assert left == pytest.approx(right, abs=0.1), "legs must be symmetric"


def test_adduction_contact_is_shank_on_shank(pose):
    """The pair that closes first is the one the bench saw -- if a thigh gets
    there first, the capsule geometry has drifted."""
    g = pose.gaps(L_hip_roll=-9.5)
    first = min(g, key=g.get)
    assert first == ("L_col_shank", "R_col_shank"), first
    assert g[first] < 0 <= min(v for k, v in g.items() if k != first)


def test_yaw_cross_onset_thighs_raised(pose):
    """BENCH: thighs raised 90 deg with the shanks hanging (hip pitch -90,
    knee -90), both hips yawed inward -> shank-on-shank at ~8.5 deg a side."""
    base = dict(L_hip_pitch=-90, R_hip_pitch=-90, L_knee=-90, R_knee=-90)
    onset = pose.onset(lambda a: dict(base, L_hip_yaw=-a, R_hip_yaw=+a))
    assert onset == pytest.approx(8.5, abs=2.0), f"yaw-cross onset {onset:.2f}"
    g = pose.gaps(**dict(base, L_hip_yaw=-9.5, R_hip_yaw=9.5))
    assert min(g, key=g.get) == ("L_col_shank", "R_col_shank")


# -- 3. the poses that must stay clean --------------------------------------
@pytest.mark.parametrize("label,q", [
    # standing toe-in was swept to 20 deg on the robot, clean (errata test);
    # take it to the range edge too, since the plant allows 45
    ("standing toe-in 20", dict(L_hip_yaw=-20, R_hip_yaw=20)),
    ("standing toe-in 45", dict(L_hip_yaw=-45, R_hip_yaw=45)),
    ("standing toe-out 45", dict(L_hip_yaw=45, R_hip_yaw=-45)),
    # the 90/90 sit start pose (check_sit_pose.py): waist bent, knees straight
    ("90/90 sit", dict(L_hip_pitch=-90, R_hip_pitch=-90)),
    ("90/90 sit, ankles out", dict(L_hip_pitch=-90, R_hip_pitch=-90,
                                   L_ankle=40, R_ankle=40)),
    # full knee flexion, the deepest thing the policy range allows
    ("knee flexion -95", dict(L_knee=-95, R_knee=-95)),
    ("deep crouch", dict(L_hip_pitch=-90, R_hip_pitch=-90,
                         L_knee=-95, R_knee=-95, L_ankle=40, R_ankle=40)),
    # the widened travel itself
    ("abduction 55", dict(L_hip_roll=55, R_hip_roll=-55)),
    ("hip pitch +90 back", dict(L_hip_pitch=90, R_hip_pitch=90)),
    ("hip pitch -110 fwd", dict(L_hip_pitch=-110, R_hip_pitch=-110)),
    ("standing", {}),
])
def test_pose_does_not_self_collide(pose, label, q):
    gap = pose.min_gap(**q)
    assert gap > 0.005, f"{label}: legs {gap * 1000:.1f} mm apart"


def test_the_legs_are_the_only_self_collision(model):
    """The four declared pairs are the whole story: every legcol geom is
    contype 0 / conaffinity 0, so nothing else in the model can reach them,
    and no other pair is declared. (The feet stay un-collided -- 2026-07-30.)"""
    assert model.npair == 4
    ids = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n)
           for n in COL_GEOMS}
    declared = {tuple(sorted((int(model.pair_geom1[p]),
                              int(model.pair_geom2[p]))))
                for p in range(model.npair)}
    assert {g for p in declared for g in p} == ids
    assert len(declared) == 4                       # L{thigh,shank} x R{...}
    for g in ids:
        assert model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0


def test_inertia_and_standing_height_are_v3yaw(model):
    """The collision geoms are massless and never touch the floor, so away
    from a leg-on-leg contact this plant IS v3yaw: same masses, same inertias,
    same standing height. That is the other half of the migration claim -- a
    warm start is not also absorbing a mass change."""
    m3 = mujoco.MjModel.from_xml_path(V3)
    assert np.allclose(model.body_mass, m3.body_mass, atol=0, rtol=0)
    assert np.allclose(model.body_inertia, m3.body_inertia, atol=0, rtol=0)
    assert np.allclose(model.body_ipos, m3.body_ipos, atol=0, rtol=0)
    d4, d3 = mujoco.MjData(model), mujoco.MjData(m3)
    for m, d in ((model, d4), (m3, d3)):
        mujoco.mj_forward(m, d)
    assert float(d4.qpos[2]) == pytest.approx(float(d3.qpos[2]), abs=1e-12)


# -- 4. the training engine sees the same contacts --------------------------
def test_mjx_agrees_with_the_cpu_referee_on_the_contact():
    """Training runs on MJX and scoring runs on the CPU engine, so a contact
    only counts if both engines make it. Explicit <pair> contacts are the
    happy path for MJX (one analytic capsule-capsule manifold, no box corners
    to prune -- the failure mode that broke parity gates 2g/2h in July)."""
    jax = pytest.importorskip("jax")
    os.environ.setdefault("JAX_PLATFORMS", "cpu")
    jax.config.update("jax_enable_x64", True)
    from mujoco import mjx

    m = mujoco.MjModel.from_xml_path(V4)
    mx = mjx.put_model(m)
    qadr = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j):
            m.jnt_qposadr[j] for j in range(m.njnt)}
    g_ls = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "L_col_shank")
    g_rs = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "R_col_shank")
    d = mujoco.MjData(m)
    for roll in (-6.0, -9.5, -15.0):
        q = np.array(m.qpos0)
        q[qadr["L_hip_roll"]] = math.radians(roll)
        d.qpos[:] = q
        mujoco.mj_forward(m, d)
        cpu = mujoco.mj_geomDistance(m, d, g_ls, g_rs, 0.5, None)
        dx = mjx.forward(mx, mjx.make_data(mx).replace(qpos=jax.numpy.asarray(q)))
        c = dx._impl.contact
        sel = [i for i in range(c.dist.shape[0])
               if {int(c.geom1[i]), int(c.geom2[i])} == {g_ls, g_rs}]
        assert sel, f"MJX lost the shank pair at roll {roll}"
        assert float(c.dist[sel[0]]) == pytest.approx(cpu, abs=1e-6), roll
        assert (cpu < 0) == (roll < -9.0)
