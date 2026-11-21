"""--supply-sag-ohm (#6): the servo bus sags under load, V = V0 - R x I_bus per physics substep
(sim/supply_sag.py), with R measured on the 2026-11-03 bench supply (0.44 ohm). Off by default and
then bit-exact; on, the kernel reproduces the bench line, the relaxed update does not ring under a
saturated load, and the CPU env's standing robot sits where the bench read it (11.7 V below 1 A).
The CPU == MJX check is parity_test.py R24 (`--supply-sag`), run here in a subprocess.
"""
import inspect
import os
import subprocess
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "sim")
sys.path.insert(0, os.path.join(SIM, "mjx"))
sys.path.insert(0, SIM)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import supply_sag as S  # noqa: E402
import walker_env as W  # noqa: E402
import current_budget as CB  # noqa: E402

E3215 = S.servo_elec(["L_knee"])                 # one STS3215's (K_CU, stall A at 12 V)
E12 = S.servo_elec(["L_knee"] * 12)              # twelve of them

ARGS = ["--robot", "--xml", "bimo_biped_v6legs_meas_sole.xml", "--servo-kp-scale", "asbuilt",
        "--precision", "--mjx-impl", "jax", "--family", "loco", "--quantize",
        "--supply-voltage", "12"]


def _kw(*extra):
    import train_mjx
    return train_mjx.build_env_kw(train_mjx.make_parser().parse_args(ARGS + list(extra)))


# -- parsing and plumbing -------------------------------------------------------
def test_defaults_parser_and_cli():
    from env_mjx import BimoMJXEnv
    for cls in (BimoMJXEnv, W.BimoWalkerEnv):
        assert inspect.signature(cls.__init__).parameters["supply_sag_ohm"].default is None
    P = S.parse_sag
    assert P(None) is None and P("") is None and P(0) is None and P("0") is None
    assert P(0.44) == (0.44, 0.44) and P("0.44") == (0.44, 0.44)
    assert P("0.3:0.6") == (0.3, 0.6) and P([0.3, 0.6]) == (0.3, 0.6)
    for bad in ("-1", "0.6:0.3", "a", "1:2:3", "nan"):
        with pytest.raises(ValueError):
            P(bad)
    assert "supply_sag_ohm" not in _kw()
    kw = _kw("--supply-sag-ohm", "0.3:0.6")
    assert kw["supply_sag_ohm"] == [0.3, 0.6]                       # JSON-ready
    assert P(kw["supply_sag_ohm"]) == (0.3, 0.6)
    with pytest.raises(ValueError):
        W.BimoWalkerEnv(actuator_model="ideal", supply_sag_ohm=0.44)


# -- the kernel --------------------------------------------------------------------
def test_kernel_matches_the_datasheet_and_the_bench_line():
    # one STS3215 at its 12 V stall (2.94 N m, still) draws the datasheet's 2.7 A (the cap)
    i = S.bus_current(np.array([2.94]), np.array([0.0]), 12.0, E3215)
    assert i == pytest.approx(2.7, abs=1e-9)
    # the bench line: V = 12.02 - 0.44 I
    for amps, volts in ((1.0, 11.58), (3.3, 10.568)):
        assert 12.02 * S.sag_scale(amps, 0.44, 12.02) == pytest.approx(volts, abs=1e-3)
    assert S.sag_scale(1e3, 0.44, 12.0) == S.SAG_FLOOR and S.sag_scale(0.0, 0.44, 12.0) == 1.0
    # driving the load (mechanical power) costs current, back-driven does not add to it
    a = S.bus_current(np.array([1.0]), np.array([2.0]), 12.0, E3215)
    b = S.bus_current(np.array([1.0]), np.array([-2.0]), 12.0, E3215)
    assert a > b == pytest.approx(S.I_IDLE + CB.SERVO_ELEC["sts3215"][2] / 12.0)


def test_current_per_servo_type_and_capped_at_the_real_stall():
    """Each servo draws its own type's current (current_budget.SERVO_ELEC), and an envelope-scaled
    stall torque is priced against the servo's real stall current (#339 review): the STS3250 hip
    roll at its 4.90 N m stall draws current_budget's 4.2 A (not 7.5 A at the STS3215's K_CU), a
    knee scaled to 2x stall draws the STS3215's 2.7 A (not 10.8 A)."""
    names = ["L_hip_roll", "L_hip_pitch", "L_knee", "L_ankle"]
    k_cu, i_st = S.servo_elec(names, {"hip_roll": "sts3250", "hip_pitch": [2.0, 1.0],
                                      "knee": [2.0, 1.0]})
    assert list(k_cu) == [CB.SERVO_ELEC["sts3250"][2]] + [CB.SERVO_ELEC["sts3215"][2]] * 3
    assert list(i_st) == [CB.SERVO_ELEC["sts3250"][0]] + [CB.SERVO_ELEC["sts3215"][0]] * 3
    # the bus map alone (no servo_types) makes the hip rolls STS3250s, the rest STS3215s
    assert list(S.servo_elec(names)[1]) == list(i_st)
    # at stall, each servo's current equals current_budget's (its "motor" cap at 12 V and V)
    stall = np.array([4.90, 2.0 * 2.94, 2.0 * 2.94, 2.94])
    still = np.zeros(4)
    for v in (12.0, 10.0):
        per = [S.bus_current(stall[j:j + 1], still[j:j + 1], v, (k_cu[j:j + 1], i_st[j:j + 1]))
               for j in range(4)]
        cb = CB.servo_amps(names, stall, still, np.ones(4), v, model="motor")[0]
        np.testing.assert_allclose(per, cb, rtol=1e-12)
        np.testing.assert_allclose(per, [4.2 * v / 12, 2.7 * v / 12, 2.7 * v / 12, 2.7 * v / 12])
    # below stall the 3250's own K_CU prices it (current_budget's "bridge" model)
    tau, qd = np.array([2.0, 1.0, -1.5, 0.5]), np.array([1.0, -2.0, 0.5, 3.0])
    np.testing.assert_allclose(S.bus_current(tau, qd, 11.0, (k_cu, i_st)),
                               CB.servo_amps(names, tau, qd, np.ones(4), 11.0, "bridge").sum(),
                               rtol=1e-12)


def test_relaxed_update_does_not_ring_under_a_saturated_load():
    """Twelve servos at their cap while moving at 30 % of the no-load speed: the cap is
    stall x (s - 0.3), so the current grows faster than the supply scale s. The whole-step update
    flips period-2 between the floor and 0.83; the relaxed one settles onto the fixed point
    s = sag_scale(I(s))."""
    stall, w0, x, n = 2.94, 4.8, 0.3, 12

    def load(s):
        return np.full(n, stall * max(s - x, 0.0)), np.full(n, x * w0)

    def run(relax):
        s, out = 1.0, []
        for _ in range(80):
            tau, qd = load(s)
            s = s + relax * (S.sag_scale(S.bus_current(tau, qd, 12.0 * s, E12), 0.44, 12.0) - s)
            out.append(s)
        return np.array(out)
    whole = run(1.0)
    assert np.abs(np.diff(whole[-10:])).min() > 0.2                  # the explicit update rings
    relaxed = run(S.SAG_RELAX)
    assert np.abs(np.diff(relaxed[-10:])).max() < 1e-9               # the relaxed one settles
    s = relaxed[-1]
    tau, qd = load(s)
    assert s == pytest.approx(S.sag_scale(S.bus_current(tau, qd, 12.0 * s, E12), 0.44, 12.0), abs=1e-8)
    assert S.SAG_FLOOR < s < 1.0
    # and it is the kernel's own step
    tau, qd = load(0.8)
    s2, i2 = S.sag_step(tau, qd, 0.8, 0.44, 12.0, E12)
    assert i2 == pytest.approx(S.bus_current(tau, qd, 9.6, E12))
    assert s2 == pytest.approx(0.8 + S.SAG_RELAX * (S.sag_scale(i2, 0.44, 12.0) - 0.8))


# -- walker_env --------------------------------------------------------------------
def _walker(**over):
    import eval_precision as ep
    cfg = dict(_kw(), **over)
    return ep.make_env(cfg, 4.0, True, cfg["xml_path"])


def _roll(env, steps=10, seed=3, amp=0.4):
    obs0, _ = env.reset(seed=seed)
    a = amp * np.sin(np.arange(env.action_space.shape[0], dtype=np.float32) + 0.3)
    obs, q, s = [obs0], [np.array(env.data.qpos)], []
    for _ in range(steps):
        o, *_ = env.step(a)
        obs.append(o)
        q.append(np.array(env.data.qpos))
        s.append(env._sag_s)
    return np.array(obs), np.array(q), np.array(s)


def test_walker_off_is_bit_exact():
    e0 = _walker()
    assert e0.supply_sag_ohm is None and e0._sag_r is None
    obs0, q0, s0 = _roll(e0)
    obs1, q1, _ = _roll(_walker(supply_sag_ohm=None))
    obs2, q2, _ = _roll(_walker(supply_sag_ohm=0))
    np.testing.assert_array_equal(obs0, obs1)
    np.testing.assert_array_equal(q0, q2)
    np.testing.assert_array_equal(obs0, obs2)
    assert np.all(s0 == 1.0)


def test_walker_stand_sags_like_the_bench_and_load_sags_more():
    e = _walker(supply_sag_ohm=0.44)
    _, _, s = _roll(e, steps=50, amp=0.0)
    assert e._sag_r == 0.44
    v = 12.0 * s[-1]
    # standing: a few hundred mA of idle + holding current -> ~11.8 V (the bench: 11.7 V below 1 A)
    assert 0.2 < e._sag_i < 1.0 and 11.5 < v < 11.95
    e2 = _walker(supply_sag_ohm=0.44)
    obs_on, q_on, s2 = _roll(e2, steps=30, amp=0.8)
    assert s2.min() < s[-1]                                          # moving draws more, sags more
    obs_off, q_off, _ = _roll(_walker(), steps=30, amp=0.8)
    assert not np.array_equal(q_on, q_off)                           # and the plant feels it


def test_walker_draws_r_per_episode_inside_the_range():
    e = _walker(supply_sag_ohm="0.3:0.6", domain_rand=True)
    rs = set()
    for seed in range(6):
        e.reset(seed=seed)
        assert 0.3 <= e._sag_r <= 0.6 and e._sag_s == 1.0
        rs.add(e._sag_r)
    assert len(rs) == 6


def test_cpu_mjx_parity():
    """sim/mjx/parity_test.py R24, in a subprocess (parity_test turns jax_enable_x64 on)."""
    env = dict(os.environ, JAX_PLATFORMS="cpu", MUJOCO_GL="disabled")
    p = subprocess.run([sys.executable, "parity_test.py", "--supply-sag"], cwd=os.path.join(SIM, "mjx"),
                       env=env, capture_output=True, text=True, timeout=1200)
    assert p.returncode == 0, p.stdout[-2000:] + p.stderr[-2000:]


def test_grade_term_switches_it_on_and_off():
    """grade.py's GRADE_PLANT_TERMS=supply_ohm=<R> (weight-transfer-gate gate_common.apply_terms):
    live at once and after a reset; 0 switches it off again."""
    sys.path.insert(0, os.path.join(HERE, "..", "experiments", "policy", "weight-transfer-gate"))
    import gate_common as GC
    e = _walker()
    e.reset(seed=1)
    GC.apply_terms(e, dict(supply_ohm=0.44))
    assert e._sag_r == 0.44 and e._sag_s == 1.0
    e.step(np.zeros(e.action_space.shape, np.float32))
    assert e._sag_s < 1.0 and e._sag_i > 0.0
    e.reset(seed=2)
    assert e._sag_r == 0.44 and e._sag_s == 1.0
    GC.apply_terms(e, dict(supply_ohm=0))
    assert e._sag_r is None
    e.reset(seed=3)
    assert e._sag_r is None
