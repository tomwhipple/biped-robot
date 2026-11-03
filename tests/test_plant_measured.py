"""The measured-stops plants: sim/bimo_biped_v6legs_meas.xml and _v6ar_meas.xml.

Run:  .venv/bin/python -m pytest tests/test_plant_measured.py -q

On 2026-11-02 every v6 leg joint was swept both ways on the robot (guarded,
hanging; hw_sessions/2026-11-02/NOTES.md, "ROM pass 3") and the CAD probed at
the same angles. The plants before that carry the CAD's DESIGN ranges as
their joint stops (ankle pitch +-40, hip yaw +-45, ...), several of which the
robot simply does not have. The _meas plants carry the measured stops
(gen_plant_v6.MEASURED_STOPS) and fitted inter-leg capsules
(build_v6_inertia.MEAS_LEGCOL_R), and leave the policy range alone.

What this file pins, and why each one can drift silently:

  1. THE JOINT STOPS: the measured table, per side (ankle roll is
     asymmetric: a stop on the outer-edge-up side only).
  2. THE ACTION MAP: walker_env.policy_range on a _meas plant equals the old
     plant's under ctrl_within_stops (what walk16_act / walk17-19 and the
     get-up train with), and plain mode equals it too. A checkpoint trained on
     the old plant therefore warm-starts with identical joint targets.
     ctrl_physical is NOT preserved -- its policy range IS the joint range --
     and the test says so explicitly: physical-mode runs stay on the old plants.
  3. THE OLD PLANTS: still byte-identical to what the generator writes for
     them, with their joint and ctrl tables as they were.
  4. THE INTER-LEG CONTACTS against the CAD (cad_probe2 / check_assembly_v6):
     one hip rolled inward with the other leg at zero -> feet first, ~6-8 deg;
     the other leg pitched -70 -> thighs at ~29 (CAD 27-30).
"""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "sim")
for _p in (SIM, os.path.join(SIM, "mjx")):
    sys.path.insert(0, _p)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import mujoco  # noqa: E402

import walker_env as W  # noqa: E402

LEGS = os.path.join(SIM, "bimo_biped_v6legs.xml")
AR = os.path.join(SIM, "bimo_biped_v6ar.xml")
LEGS_MEAS = os.path.join(SIM, "bimo_biped_v6legs_meas.xml")
AR_MEAS = os.path.join(SIM, "bimo_biped_v6ar_meas.xml")
PAIRS = [(LEGS, LEGS_MEAS), (AR, AR_MEAS)]

LEG_JOINTS = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle", "ankle_roll")

# joint stops (deg), sim convention. + ankle roll lifts the LEFT foot's outer
# edge (axis +X), so the outer-edge stop is L's + and R's -.
MEAS_STOPS = {
    "hip_yaw": {"L": (-90, 90), "R": (-90, 90)},          # no stop: free to 86.7, CAD constant 1.8 mm
    "hip_roll": {"L": (-58, 58), "R": (-58, 58)},         # outward free to 57.7 (CAD 0.5 mm at 55)
    "hip_pitch": {"L": (-124, 104), "R": (-124, 104)},    # fwd STOP -125.1 / -123.6; back +104.8 (L)
    "knee": {"L": (-132, 95), "R": (-132, 95)},           # flexion >= 130.4 (CAD ~132); hyper >= 93.8 (CAD 95)
    "ankle": {"L": (-80, 80), "R": (-80, 80)},            # free to +-78 (CAD: no stop)
    "ankle_roll": {"L": (-50, 27.5), "R": (-27.5, 50)},   # outer STOP 27.9 / 26.9; inner free past 46
}
OLD_STOPS = {
    "hip_yaw": (-45, 45), "hip_roll": (-55, 55), "hip_pitch": (-120, 90),
    "knee": (-130, 95), "ankle": (-40, 40), "ankle_roll": (-25, 25),
}
# ctrlrange (deg) on the old plants; the _meas plants' differ only at the ankle (+-40)
OLD_CTRL = {
    "hip_yaw": {"L": (-45, 45), "R": (-45, 45)},
    "hip_roll": {"L": (-30, 45), "R": (-45, 30)},
    "hip_pitch": {"L": (-120, 90), "R": (-120, 90)},
    "knee": {"L": (-130, 5), "R": (-130, 5)},
    "ankle": {"L": (-45, 45), "R": (-45, 45)},
    "ankle_roll": {"L": (-25, 25), "R": (-25, 25)},
}


def _m(path):
    return mujoco.MjModel.from_xml_path(path)


def _jdeg(m, name):
    return tuple(np.degrees(m.jnt_range[m.joint(name).id]))


def _cdeg(m, name):
    return tuple(np.degrees(m.actuator_ctrlrange[m.actuator(name).id]))


# -- 3. the old plants ---------------------------------------------------------
def test_old_plants_are_still_what_the_generator_writes():
    """The generator option must not move a byte of the old plants (the
    test_asbuilt_legs / test_v6_design_gates staleness checks say the same; this
    one is here so the claim lives next to the change that could break it)."""
    import build_v6_inertia as BI
    with open(LEGS) as f:
        assert f.read() == BI.committed_legs_xml(LEGS)
    with open(AR) as f:
        assert f.read() == BI.committed_xml(AR)


@pytest.mark.parametrize("path", [LEGS, AR])
def test_old_plants_tables_unchanged(path):
    m = _m(path)
    for side in "LR":
        for j in LEG_JOINTS:
            n = f"{side}_{j}"
            assert _jdeg(m, n) == pytest.approx(OLD_STOPS[j], abs=1e-6), n
            assert _cdeg(m, n) == pytest.approx(OLD_CTRL[j][side], abs=1e-6), n


# -- 1. the measured stops -------------------------------------------------------
def test_meas_plants_are_generated():
    import build_v6_inertia as BI
    with open(LEGS_MEAS) as f:
        assert f.read() == BI.committed_meas_xml(legs=True), \
            "sim/bimo_biped_v6legs_meas.xml is stale: run sim/build_v6_inertia.py --measured-stops --asbuilt-legs --write"
    with open(AR_MEAS) as f:
        assert f.read() == BI.committed_meas_xml(legs=False), \
            "sim/bimo_biped_v6ar_meas.xml is stale: run sim/build_v6_inertia.py --measured-stops --write"


@pytest.mark.parametrize("path", [LEGS_MEAS, AR_MEAS])
def test_meas_joint_stops(path):
    m = _m(path)
    for side in "LR":
        for j in LEG_JOINTS:
            n = f"{side}_{j}"
            assert _jdeg(m, n) == pytest.approx(MEAS_STOPS[j][side], abs=1e-6), n


def test_meas_stops_cover_every_measured_free_angle():
    """Every angle the robot reached FREE lies inside the new stop, and each
    measured hard stop is within 1.2 deg of it (sim angles, NOTES.md rom3)."""
    m = _m(LEGS_MEAS)
    free = {  # joint -> angles reached without a stop
        "L_hip_yaw": (85.9, -86.7), "R_hip_yaw": (86.3, -86.5),
        "L_hip_roll": (57.7,), "R_hip_roll": (-51.3,),
        "L_hip_pitch": (104.8 - 1.0,), "R_hip_pitch": (84.0,),
        "L_knee": (-129.9, 93.8), "R_knee": (-130.4, 93.5),
        "L_ankle": (78.5, -78.4), "R_ankle": (78.0, -78.3),
        "L_ankle_roll": (-46.4,), "R_ankle_roll": (46.5,),
    }
    for n, angles in free.items():
        lo, hi = _jdeg(m, n)
        for a in angles:
            assert lo <= a <= hi, (n, a, (lo, hi))
    hard = {"L_hip_pitch": -125.1, "R_hip_pitch": -123.6, "L_ankle_roll": 27.9, "R_ankle_roll": -26.9}
    for n, a in hard.items():
        lo, hi = _jdeg(m, n)
        assert min(abs(a - lo), abs(a - hi)) <= 1.2, (n, a, (lo, hi))


def test_meas_plants_differ_from_the_old_only_where_meant():
    """Same bodies, masses, inertias, actuators and contacts; only the joint
    ranges, the ankle ctrlrange and the thigh/shank capsule radius change."""
    for old, new in PAIRS:
        a, b = _m(old), _m(new)
        for f in ("body_mass", "body_inertia", "body_ipos", "body_pos", "qpos0",
                  "actuator_gainprm", "actuator_biasprm", "actuator_forcerange",
                  "dof_damping", "dof_armature", "dof_frictionloss", "pair_geom1", "pair_geom2",
                  "geom_contype", "geom_conaffinity", "geom_pos"):
            assert np.array_equal(getattr(a, f), getattr(b, f)), (new, f)
        assert a.nu == b.nu and a.njnt == b.njnt and a.ngeom == b.ngeom
        changed = {b.geom(g).name for g in range(b.ngeom) if not np.array_equal(a.geom_size[g], b.geom_size[g])}
        assert changed == {f"{s}_col_{k}" for s in "LR" for k in ("thigh", "shank")}, changed


def test_meas_plants_are_mirror_symmetric():
    """sim/mjx/mirror.py (the mirror loss and the mirror augmentation) needs
    mirrored ranges: the asymmetric ankle roll must be L (-a, +b) / R (-b, +a)."""
    import mirror
    for path in (LEGS_MEAS, AR_MEAS):
        m = _m(path)
        mirror.joint_perm_signs(m)
        mirror.action_perm_signs(m)


# -- 2. the action map -------------------------------------------------------------
def _pr(path, **kw):
    lo, hi = W.policy_range(_m(path), **kw)
    return np.asarray(lo), np.asarray(hi)


def _same_map(a, b):
    """Identical action maps: bit-equal in float32 (what the MJX trainer holds
    _lo/_hi in) and within 1e-10 rad in float64. Not bit-equal in float64 only
    because the plants spell ctrlrange to 10 decimals: where an old ctrlrange
    equals its stop (yaw +-45, ankle roll +-25, ...) within-stops returns the
    stop's exact radians, the _meas plant its 10-decimal ctrlrange -- at most
    1.1e-11 rad (6e-10 deg) apart."""
    for x, y in zip(a, b):
        assert np.allclose(x, y, rtol=0, atol=1e-10), np.abs(np.subtract(x, y)).max()
        assert np.array_equal(np.float32(x), np.float32(y))


@pytest.mark.parametrize("old,new", PAIRS)
def test_policy_range_within_stops_is_identical(old, new):
    """THE warm-start claim: walk16_act / walk17-19_shp* / the night jobs train
    with --ctrl-within-stops; their policy range on the _meas plant is the
    old plant's (_same_map)."""
    _same_map(_pr(old, within_stops=True), _pr(new, within_stops=True))


@pytest.mark.parametrize("old,new", PAIRS)
def test_policy_range_plain_equals_the_old_within_stops(old, new):
    """Plain mode on the _meas plant == within-stops (every ctrlrange lies
    inside its new stop) == the old plant's within-stops. The ankle ctrlrange
    is +-40 outright, so the old plant's plain +-45 (5 deg into its stop) is
    the one range that is not carried over -- a plain-mode run trained on the
    old plant would see its ankle scaled 40/45 here."""
    _same_map(_pr(new), _pr(new, within_stops=True))
    _same_map(_pr(old, within_stops=True), _pr(new))
    m = _m(old)
    names = [m.joint(int(j)).name for j in m.actuator_trnid[:, 0]]
    po, pn = _pr(old), _pr(new)
    diff = {n for n, x, y, u, v in zip(names, po[0], po[1], pn[0], pn[1]) if abs(x - u) > 1e-9 or abs(y - v) > 1e-9}
    assert diff == {"L_ankle", "R_ankle"}
    i = names.index("L_ankle")
    assert np.degrees([po[0][i], po[1][i]]) == pytest.approx([-45, 45])
    assert np.degrees([pn[0][i], pn[1][i]]) == pytest.approx([-40, 40])


@pytest.mark.parametrize("old,new", PAIRS)
def test_physical_mode_is_not_preserved(old, new):
    """ctrl_physical's policy range is the joint range, which this plant
    widens: a physical-mode run would be RESCALED on the _meas plant. Pinned
    so nobody mistakes it for a drop-in there; physical-mode runs stay on the
    old plants (gen_plant_v6.measured_stops, robot_plant.ROBOT_LEGS_MEAS_XML)."""
    a, b = _pr(old, physical=True), _pr(new, physical=True)
    assert not np.allclose(a[0], b[0]) and not np.allclose(a[1], b[1])


@pytest.mark.parametrize("amap", ["legacy", "full"])
def test_actions_mean_the_same_joint_targets(amap):
    """The env-level form, with the robot preset the night recipes use
    (`--robot --xml ... --ctrl-within-stops`: held joints, servo map,
    hip_flex_deg 120): same _lo/_hi, same soft limits, same ctrl per action."""
    import robot_plant
    envs = []
    for xml in (robot_plant.ROBOT_LEGS_XML, robot_plant.ROBOT_LEGS_MEAS_XML):
        kw = robot_plant.robot_env_kwargs(xml)
        assert os.path.basename(kw["xml_path"]) == xml
        envs.append(W.BimoWalkerEnv(action_map=amap, ctrl_within_stops=True, **kw))
    e0, e1 = envs
    assert e0._act_names == e1._act_names
    for f in ("_lo", "_hi", "_soft_lo", "_soft_hi"):
        _same_map((getattr(e0, f),), (getattr(e1, f),))
    rng = np.random.default_rng(0)
    for _ in range(16):
        a = rng.uniform(-1, 1, e0._nq_act)
        assert np.allclose(e0._action_to_ctrl(a), e1._action_to_ctrl(a), rtol=0, atol=1e-10)


def test_mjx_policy_range_mirror_agrees():
    import env_mjx as M
    for path in (LEGS_MEAS, AR_MEAS):
        m = _m(path)
        for kw in ({}, {"within_stops": True}, {"physical": True}):
            a, b = W.policy_range(m, **kw), M.policy_range(m, **kw)
            assert np.array_equal(a[0], b[0]) and np.array_equal(a[1], b[1])


def test_robot_preset_recognises_the_meas_plants():
    import robot_plant
    for xml in (robot_plant.ROBOT_LEGS_MEAS_XML, robot_plant.ROBOT_MEAS_XML):
        assert robot_plant.is_robot_model(_m(robot_plant.robot_xml_path(xml)))


# -- 4. the inter-leg contacts vs the CAD ------------------------------------------
class Poser:
    """pose in degrees -> {pair: distance} over the plant's declared inter-leg pairs."""

    def __init__(self, path):
        self.m = _m(path)
        self.d = mujoco.MjData(self.m)
        self.q = {self.m.joint(j).name: self.m.jnt_qposadr[j] for j in range(self.m.njnt)}
        self.pairs = [(int(self.m.pair_geom1[p]), int(self.m.pair_geom2[p])) for p in range(self.m.npair)]

    def gaps(self, **deg):
        self.d.qpos[:] = self.m.qpos0
        for k, v in deg.items():
            self.d.qpos[self.q[k]] = math.radians(v)
        mujoco.mj_forward(self.m, self.d)
        return {(self.m.geom(a).name, self.m.geom(b).name): mujoco.mj_geomDistance(self.m, self.d, a, b, 0.2, None)
                for a, b in self.pairs}

    def first(self, **deg):
        g = self.gaps(**deg)
        k = min(g, key=g.get)
        return k, g[k]

    def onset(self, pose, hi=60.0):
        assert self.first(**pose(0.0))[1] > 0
        assert self.first(**pose(hi))[1] <= 0
        lo = 0.0
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if self.first(**pose(mid))[1] > 0:
                lo = mid
            else:
                hi = mid
        return hi, self.first(**pose(hi))[0]


@pytest.fixture(scope="module")
def pose():
    return Poser(LEGS_MEAS)


@pytest.mark.parametrize("side", "LR")
def test_inward_roll_other_leg_at_zero_feet_first(pose, side):
    """CAD: feet 1.18 mm at 6, 0.84 at 7, touch at 8; shins overlap from 9.
    The plant's foot slab (the foot + sole box) touches at 6.15 -- its gap at
    6 deg is 0.9 mm against the CAD's 1.18, i.e. ~2 deg early only because the
    CAD's exact surfaces close by under 1.2 mm between 6 and 8. Conservative."""
    sgn = -1 if side == "L" else 1
    a, pair = pose.onset(lambda x: {f"{side}_hip_roll": sgn * x})
    assert pair == ("L_col_foot", "R_col_foot")
    assert 5.5 <= a <= 8.5, a
    _, g6 = pose.first(**{f"{side}_hip_roll": sgn * 6.0})
    assert g6 * 1e3 == pytest.approx(1.18, abs=0.5)


def test_inward_roll_shins_without_the_feet(pose):
    """the leg links alone (thigh + shank capsules): CAD hulls 8.3 deg, CAD
    'shins overlap from 9'."""
    legs = {f"{s}_col_{k}" for s in "LR" for k in ("thigh", "shank")}
    keep = [(a, b) for a, b in pose.pairs if {pose.m.geom(a).name, pose.m.geom(b).name} <= legs]
    full, pose.pairs = pose.pairs, keep
    try:
        a, pair = pose.onset(lambda x: {"R_hip_roll": x})
    finally:
        pose.pairs = full
    assert pair == ("L_col_shank", "R_col_shank")
    assert a == pytest.approx(8.3, abs=1.0)


@pytest.mark.parametrize("roll,pitch", [("R_hip_roll", "L_hip_pitch"), ("L_hip_roll", "R_hip_pitch")])
def test_inward_roll_other_leg_forward_thighs(pose, roll, pitch):
    """CAD (cad_probe2, other leg at -70): thighs 27.94 mm apart at 10 deg,
    14.42 at 20, touching by 30 (0.3 mm3); the convex hulls of the same parts
    in the same MuJoCo pose touch at 29.3. The half-width capsules of the old
    plant touched at 33.7; the 25 mm ones here at 29.4."""
    sgn = -1 if roll.startswith("L") else 1
    a, pair = pose.onset(lambda x: {roll: sgn * x, pitch: -70})
    assert pair == ("L_col_thigh", "R_col_thigh")
    assert a == pytest.approx(29.3, abs=1.0)
    assert pose.first(**{roll: sgn * 10, pitch: -70})[1] * 1e3 == pytest.approx(27.94, abs=2.0)
    assert pose.first(**{roll: sgn * 20, pitch: -70})[1] * 1e3 == pytest.approx(14.42, abs=2.0)


def test_inward_roll_other_leg_at_minus_45(pose):
    """CAD probe at -45: 7.36 mm at 20, overlap at 30; hulls touch at 24.4."""
    a, pair = pose.onset(lambda x: {"R_hip_roll": x, "L_hip_pitch": -45})
    assert pair == ("L_col_thigh", "R_col_thigh")
    assert a == pytest.approx(24.4, abs=1.5)


@pytest.mark.parametrize("label,q", [
    ("standing", {}),
    # (the feet themselves cross at toe-in 20 / toe-out 45 on this plant and the old
    # one alike: unchanged foot geometry, not this file's subject)
    ("standing toe-in 10", dict(L_hip_yaw=-10, R_hip_yaw=10)),
    ("abduction 58", dict(L_hip_roll=58, R_hip_roll=-58)),
    ("ankles at the new stop", dict(L_ankle=80, R_ankle=-80)),
    ("ankle rolls inner edge up 50", dict(L_ankle_roll=-50, R_ankle_roll=50)),
    ("hip pitch -124 both", dict(L_hip_pitch=-124, R_hip_pitch=-124)),
    ("hip pitch +104 both", dict(L_hip_pitch=104, R_hip_pitch=104)),
    ("deep crouch", dict(L_hip_pitch=-90, R_hip_pitch=-90, L_knee=-130, R_knee=-130, L_ankle=40, R_ankle=40)),
    ("robot pose: L in 24, R pitched -68", dict(L_hip_roll=-24, R_hip_pitch=-68)),
])
def test_pose_does_not_self_collide(pose, label, q):
    assert pose.first(**q)[1] > 0.002, label
