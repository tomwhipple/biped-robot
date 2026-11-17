"""--footoff-sag (2026-11-16, #297): the robot's body sags toward a lifting foot, beyond the stance
encoders (hw_sessions/2026-11-16/DESIGN_moving_lift_gait.md section 1: 6.4 deg on a LEFT stance,
2.9 on a RIGHT one). The model: preloaded springs in series with the four roll servos beyond the
encoder (walker_env.footoff_sag_*), each preload scaled by its foot's share of the floor load.
Off by default, bit-exact; on, a clamped one-foot lift tilts the torso further toward the lifted
foot, more on a left stance than on a right one, and the encoders read the shaft. CPU only.
The CPU == MJX check is parity_test.py R21 (`--footoff-sag`).
"""
import inspect
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "sim")
GATE = os.path.join(HERE, "..", "experiments", "policy", "weight-transfer-gate")
sys.path.insert(0, os.path.join(SIM, "mjx"))
sys.path.insert(0, SIM)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import walker_env as W  # noqa: E402

ARGS = ["--robot", "--xml", "bimo_biped_v6legs_meas_sole.xml", "--servo-kp-scale", "asbuilt",
        "--precision", "--mjx-impl", "jax", "--family", "loco", "--quantize"]
FIT = "L=6.4,R=2.9"


def _kw(*extra):
    import train_mjx
    return train_mjx.build_env_kw(train_mjx.make_parser().parse_args(ARGS + list(extra)))


# -- parsing and plumbing -------------------------------------------------------
def test_defaults_parser_and_cli():
    from env_mjx import BimoMJXEnv
    for cls in (BimoMJXEnv, W.BimoWalkerEnv):
        assert inspect.signature(cls.__init__).parameters["footoff_sag"].default is None
    P = W.parse_footoff_sag
    assert P(None) is None and P("") is None and P({}) is None
    assert P("4") == {"L": (4.0, 4.0), "R": (4.0, 4.0)}
    assert P(FIT) == {"L": (6.4, 6.4), "R": (2.9, 2.9)}
    assert P("L=3.2:9.6,R=1.45:4.35") == {"L": (3.2, 9.6), "R": (1.45, 4.35)}
    assert P({"L": [3.0, 5.0], "R": 2.0}) == {"L": (3.0, 5.0), "R": (2.0, 2.0)}
    for bad in ("X=3", "L=5:3", "L=-1", "L=25", "L=a"):
        with pytest.raises(ValueError):
            P(bad)
    assert "footoff_sag" not in _kw()
    kw = _kw("--footoff-sag", "L=3.2:9.6,R=1.45:4.35")
    assert kw["footoff_sag"] == {"L": [3.2, 9.6], "R": [1.45, 4.35]}          # JSON-ready
    assert P(kw["footoff_sag"]) == P("L=3.2:9.6,R=1.45:4.35")


def test_exclusive_with_the_other_roll_yield_models():
    for other in (dict(roll_play_deg="hip=4,ankle=5"), dict(roll_series_k="hip=11,ankle=17.75")):
        with pytest.raises(ValueError):
            W.BimoWalkerEnv(actuator_model="sts3215", footoff_sag=FIT, **other)


def test_layout_shift_and_load_share():
    names = ["L_hip_yaw", "L_hip_roll", "L_knee", "L_ankle_roll", "R_hip_roll", "R_ankle_roll"]
    mask, side, sign, part = W.footoff_sag_layout(names)
    assert mask.tolist() == [0, 1, 0, 1, 1, 1]
    assert side.tolist() == [0, 0, 0, 0, 1, 1]
    a = W.FOOTOFF_SAG_ANKLE_SHARE
    assert part.tolist() == pytest.approx([0, 1 - a, 0, a, 1 - a, a])
    # left stance: the link angles go negative (the body rolls to its right, toward the R foot)
    th = np.radians([6.4, 2.9])
    sh = W.footoff_sag_shift(th, np.array([1.0, 0.0]), side, sign, part)
    assert sh[1] < 0 and sh[3] < 0 and np.all(sh[[0, 2, 4, 5]] == 0.0)
    assert sh[1] + sh[3] == pytest.approx(-th[0])
    sh = W.footoff_sag_shift(th, np.array([0.0, 1.0]), side, sign, part)
    assert sh[4] > 0 and sh[5] > 0 and sh[4] + sh[5] == pytest.approx(th[1])
    # shares: penetration-weighted, 0 for a foot in the air, both 0 with both feet up
    r = 0.003
    z = np.array([[0.0029, 0.0029], [0.0028, 0.0028]])          # 0.1 / 0.2 mm in
    np.testing.assert_allclose(W.footoff_load_share(z, r), [1 / 3, 2 / 3])
    np.testing.assert_allclose(W.footoff_load_share(np.array([[0.002, 0.004], [0.01, 0.01]]), r), [1, 0])
    np.testing.assert_allclose(W.footoff_load_share(np.full((2, 2), 0.01), r), [0, 0])


# -- walker_env --------------------------------------------------------------------
def _walker(**over):
    import eval_precision as ep
    cfg = dict(_kw(), **over)
    return ep.make_env(cfg, 4.0, True, cfg["xml_path"])


def _walker_roll(env, steps=8, seed=3):
    obs0, _ = env.reset(seed=seed)
    a = 0.4 * np.sin(np.arange(env.action_space.shape[0], dtype=np.float32) + 0.3)
    obs, q = [obs0], [np.array(env.data.qpos)]
    for _ in range(steps):
        o, *_ = env.step(a)
        obs.append(o)
        q.append(np.array(env.data.qpos))
    return np.array(obs), np.array(q)


def test_walker_off_is_bit_exact():
    e0 = _walker()
    assert e0.footoff_sag is None and e0._fs_theta is None
    obs0, q0 = _walker_roll(e0)
    obs1, q1 = _walker_roll(_walker(footoff_sag=None))
    np.testing.assert_array_equal(obs0, obs1)
    np.testing.assert_array_equal(q0, q1)


def test_walker_on_reads_the_shaft_and_draws_per_side():
    e = _walker(footoff_sag=FIT, dq_obs_window=2)
    obs, q = _walker_roll(e, steps=20)
    np.testing.assert_allclose(e._fs_theta, W.FOOTOFF_SAG_GAIN * np.radians([6.4, 2.9]))
    defl = e._fs_defl
    rolls = [i for i, n in enumerate(e._servo_names) if n.endswith(("hip_roll", "ankle_roll"))]
    others = [i for i in range(e._n_servo) if i not in rolls]
    assert np.abs(defl[rolls]).max() > math.radians(0.05)       # the springs are in use
    assert np.all(defl[others] == 0.0)
    nj = e._nq_act
    link = e.data.qpos[e._jqpos]
    tick = 2 * np.pi / 4096
    zo = e._p(e._zero_off) if e._zero_off is not None else 0.0
    assert np.max(np.abs(obs[-1][:nj] - (link - zo + e._p(defl)))) <= tick
    np.testing.assert_allclose(e._dq_qhist[0], link + e._p(defl))
    _, q_off = _walker_roll(_walker(dq_obs_window=2), steps=20)
    assert np.abs(q - q_off).max() > 1e-4
    e.reset(seed=5)
    assert np.all(e._fs_defl == 0.0)
    e = _walker(footoff_sag="L=3:9,R=1:4")
    th = []
    for s in range(8):
        e.reset(seed=s)
        th.append(np.degrees(e._fs_theta / W.FOOTOFF_SAG_GAIN))
    th = np.array(th)
    assert np.all((th[:, 0] >= 3.0) & (th[:, 0] <= 9.0) & (th[:, 1] >= 1.0) & (th[:, 1] <= 4.0))
    assert np.ptp(th[:, 0]) > 0.5 and np.ptp(th[:, 1]) > 0.3


def test_clamped_foot_lift_sags_toward_the_lifted_foot_per_side():
    """The stance foot welded to the world, the other lifted 30 mm (sim/roll_compliance.py's bench
    lift): the sag adds tilt toward the lifted foot, more on a left stance (6.4) than on a right one
    (2.9); a side set to 0 adds only the springs' own give."""
    import roll_compliance as RC
    import robot_plant

    def tilt(lifted, sag):
        env = RC.make_env(robot_plant.ROBOT_LEGS_XML, "L" if lifted == "R" else "R", footoff_sag=sag)
        t, _, lift = RC.run(env, lifted)
        assert lift > 15.0                                        # the foot did lift
        return t

    off = {s: tilt(s, None) for s in "LR"}
    on = {s: tilt(s, FIT) for s in "LR"}
    d_lst, d_rst = on["R"] - off["R"], on["L"] - off["L"]        # left stance lifts R, and back
    assert d_lst > 3.0 and d_rst > 1.5                            # toward the lifted foot, both sides
    assert d_lst > d_rst + 1.0                                    # the left stance sags more
    no_l = tilt("R", "L=0,R=2.9") - off["R"]                      # a left stance without its preload
    assert 0.0 < no_l < d_lst - 2.0


# -- the grading plant term ---------------------------------------------------------
def test_gate_term_replaces_the_play_and_is_live():
    sys.path.insert(0, GATE)
    import gate_common as GC
    e = _walker(roll_play_deg="hip=4,ankle=5")
    e.reset(seed=0)
    assert e._rp_h is not None
    GC.apply_terms(e, GC.parse_terms("play_L_hip=6,sag_L=6.4,sag_R=2.9"))
    assert e._rp_h is None and e.roll_play_deg is None and e._ser_c is None
    assert e.footoff_sag == {"L": (6.4, 6.4), "R": (2.9, 2.9)}
    np.testing.assert_allclose(e._fs_theta, W.FOOTOFF_SAG_GAIN * np.radians([6.4, 2.9]))
    e.reset(seed=0)                                               # survives a reset, same values
    np.testing.assert_allclose(e._fs_theta, W.FOOTOFF_SAG_GAIN * np.radians([6.4, 2.9]))
    e.step(np.zeros(e.action_space.shape[0], np.float32))
    assert np.abs(e._fs_defl).max() > 0.0
