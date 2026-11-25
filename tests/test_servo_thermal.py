"""--servo-thermal (#6): each servo's temperature (register 63's sensor) as a first-order thermal node
heated by the sim's own K_CU tau^2, with the STS's 70 degC over-temperature cutoff (sim/servo_thermal.py),
fitted on the flight records (experiments/servo-thermal/). Off by default and then bit-exact; on, the
physics is untouched until a servo reaches the cutoff, and a cut-off servo is limp for the rest of the
episode. The CPU == MJX check is parity_test.py R29 (`--servo-thermal`), run here in a subprocess.
"""
import inspect
import json
import math
import os
import subprocess
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
SIM = os.path.join(ROOT, "sim")
sys.path.insert(0, os.path.join(SIM, "mjx"))
sys.path.insert(0, SIM)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import servo_thermal as TH  # noqa: E402
import walker_env as W  # noqa: E402
import current_budget as CB  # noqa: E402

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
        assert inspect.signature(cls.__init__).parameters["servo_thermal"].default is None
    P = TH.parse_thermal
    assert P(None) is None and P("") is None and P("off") is None
    assert P(29) == (29.0, 29.0) and P("29") == (29.0, 29.0) and P(0) == (0.0, 0.0)
    assert P("29:69") == (29.0, 69.0) and P([29, 69]) == (29.0, 69.0)
    for bad in ("69:29", "a", "1:2:3", "nan", "200", "-40"):
        with pytest.raises(ValueError):
            P(bad)
    assert "servo_thermal" not in _kw()
    kw = _kw("--servo-thermal", "29:69")
    assert kw["servo_thermal"] == [29.0, 69.0]                       # JSON-ready
    assert P(kw["servo_thermal"]) == (29.0, 69.0)
    with pytest.raises(ValueError):
        W.BimoWalkerEnv(actuator_model="ideal", servo_thermal=29)


# -- the kernel --------------------------------------------------------------------
def test_constants_per_type_and_from_the_fit():
    k_cu, cap, tau, rth = TH.thermal_consts(["L_hip_roll", "L_knee", "R_ankle_roll"], volts=12.0)
    assert list(k_cu) == [CB.SERVO_ELEC["sts3250"][2], CB.SERVO_ELEC["sts3215"][2],
                          CB.SERVO_ELEC["sts3215"][2]]
    assert list(tau) == [TH.THERMAL["sts3250"][0]] + [TH.THERMAL["sts3215"][0]] * 2
    assert list(rth) == [TH.THERMAL["sts3250"][1]] + [TH.THERMAL["sts3215"][1]] * 2
    # the cap is the stall power: 12 V x 4.2 A, 12 V x 2.7 A
    np.testing.assert_allclose(cap, [12.0 * 4.2, 12.0 * 2.7, 12.0 * 2.7])
    # a servo_types name wins over the bus map
    assert TH.thermal_consts(["L_knee"], {"knee": "sts3250"})[3][0] == TH.THERMAL["sts3250"][1]
    # an envelope pair is a refit of the servo that is there: same type, but its full-duty torque is
    # the pair's, so the heat at a torque is (type stall / pair stall)^2 of the type's, and full duty
    # (the pair's stall) heats exactly as the type's stall does: the stall power
    kp, capp, taup, rthp = TH.thermal_consts(["L_knee"], {"knee": [2.0, 1.3]}, volts=12.0)
    assert rthp[0] == TH.THERMAL["sts3215"][1] and taup[0] == TH.THERMAL["sts3215"][0]
    assert kp[0] == pytest.approx(CB.SERVO_ELEC["sts3215"][2] / 4.0)
    full = TH.heat(np.array([2.0 * 2.94]), (kp, capp, taup, rthp))[0]
    assert full == pytest.approx(TH.heat(np.array([2.94]), TH.thermal_consts(["L_knee"], volts=12.0))[0])
    assert full == pytest.approx(12.0 * 2.7, rel=1e-3)
    # past stall (DR above 1) the heat is capped at the stall power
    assert TH.heat(np.array([4.0]), TH.thermal_consts(["L_knee"], volts=12.0))[0] == pytest.approx(32.4)
    # the constants are the committed pooled fit's (experiments/servo-thermal/fit.json), rounded
    with open(os.path.join(ROOT, "experiments", "servo-thermal", "fit.json")) as f:
        fit = json.load(f)
    assert fit["heat"] == "sim"
    for typ, (tau_s, r) in TH.THERMAL.items():
        ft = fit["types"][typ]
        assert tau_s == pytest.approx(ft["tau_s"], rel=0.1)
        assert r == pytest.approx(ft["rth_k_per_w"], rel=0.1)
        assert ft["rth_ci"][0] <= r <= ft["rth_ci"][1]


def test_kernel_steady_state_time_constant_and_cutoff():
    cs = TH.thermal_consts(["L_knee"])
    _, _, tau_th, rth = (x[0] for x in cs)
    p = 2.0                                               # W, constant
    t, off = np.array([TH.T_IDLE]), np.array([False])
    dt = 0.02
    n_tau = int(round(tau_th / dt))
    for _ in range(n_tau):
        t, off = TH.thermal_step(t, off, np.array([p]), dt, cs)
    rise = rth * p
    assert t[0] - TH.T_IDLE == pytest.approx(rise * (1 - math.exp(-1)), rel=1e-9)   # exact per tick
    for _ in range(40 * n_tau):
        t, off = TH.thermal_step(t, off, np.array([p]), dt, cs)
    assert t[0] == pytest.approx(TH.T_IDLE + rise, abs=1e-6) and not off[0]   # 61 degC: below the cutoff
    # minutes_to_cutoff matches stepping the kernel, and "never" below the cutoff's heat
    p_hot = 5.0
    m = float(TH.minutes_to_cutoff(p_hot, tau_th, rth))
    t, off, k = np.array([TH.T_IDLE]), np.array([False]), 0
    while not off[0]:
        t, off = TH.thermal_step(t, off, np.array([p_hot]), dt, cs)
        k += 1
    assert k * dt / 60.0 == pytest.approx(m, abs=dt / 60.0 * 1.01)
    p_edge = (TH.T_CUTOFF - TH.T_IDLE) / rth
    assert np.isinf(TH.minutes_to_cutoff(0.99 * p_edge, tau_th, rth))
    assert TH.minutes_to_cutoff(p_hot, tau_th, rth, t_start=TH.T_CUTOFF) == 0.0
    # latched: cooling below the cutoff does not clear it
    t, off = TH.thermal_step(np.array([69.0]), np.array([True]), np.array([0.0]), dt, cs)
    assert off[0] and t[0] < TH.T_CUTOFF
    # the heat is the bridge model's copper term
    assert TH.heat(np.array([2.0]), cs)[0] == pytest.approx(CB.SERVO_ELEC["sts3215"][2] * 2.0 ** 2)


def test_the_fit_recovers_a_known_node_through_the_registers_quantisation():
    """experiments/servo-thermal's output-error fit on synthetic chunks: a first-order node with a known
    tau and Rth, driven by bursty heat, read through a 1 degC register -> the fit recovers both."""
    import importlib.util            # the tool shares the kernel's module name: load it by path
    spec = importlib.util.spec_from_file_location(
        "servo_thermal_tool", os.path.join(ROOT, "experiments", "servo-thermal", "servo_thermal.py"))
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    rng = np.random.default_rng(6)
    tau, rth = 480.0, 16.0
    Ts, Ps, ch = [], [], []
    k = 0
    for c in range(12):
        n = 3600
        p = np.zeros(n)
        for _ in range(8):                                 # bursts of 0.5-3 W for 20-200 s
            s = rng.integers(0, n - 200)
            p[s:s + rng.integers(20, 200)] += rng.uniform(0.5, 3.0)
        t0, tss = rng.uniform(18, 30), rng.uniform(26, 31)
        a = math.exp(-1.0 / tau)
        T = np.empty(n)
        x = t0
        for i in range(n):
            T[i] = x
            x = a * x + (1 - a) * (tss + rth * p[i])
        Ts.append(np.round(T + rng.uniform(-0.5, 0.5)))  # the register: 1 degC steps, random offset
        Ps.append(p)
        ch.append((k, k + n))
        k += n
    T, P = np.concatenate(Ts), np.concatenate(Ps)
    tau_f, rth_f, rmse, _ = tool.fit_servo(T, P, ch)
    assert tau_f == pytest.approx(tau, rel=0.15) and rth_f == pytest.approx(rth, rel=0.15)
    assert rmse < 0.35                                      # the quantisation's own 1/sqrt(12)


# -- walker_env --------------------------------------------------------------------
def _walker(**over):
    import eval_precision as ep
    cfg = dict(_kw(), **over)
    return ep.make_env(cfg, 4.0, True, cfg["xml_path"])


def _roll(env, steps=10, seed=3, amp=0.4, start=None):
    obs0, _ = env.reset(seed=seed)
    if start is not None:
        env._th_start(np.full(env._n_servo, start))
    a = amp * np.sin(np.arange(env.action_space.shape[0], dtype=np.float32) + 0.3)
    obs, q = [obs0], [np.array(env.data.qpos)]
    for _ in range(steps):
        o, *_ = env.step(a)
        obs.append(o)
        q.append(np.array(env.data.qpos))
    return np.array(obs), np.array(q)


def test_walker_off_is_bit_exact_and_on_below_the_cutoff_changes_no_physics():
    e0 = _walker()
    assert e0.servo_thermal is None and e0._th_t is None
    obs0, q0 = _roll(e0)
    obs1, q1 = _roll(_walker(servo_thermal=None))
    np.testing.assert_array_equal(obs0, obs1)
    np.testing.assert_array_equal(q0, q1)
    # on, well below the cutoff: the temperature moves, the plant does not
    e2 = _walker(servo_thermal=29.0)
    obs2, q2 = _roll(e2, amp=0.8)
    obs3, q3 = _roll(_walker(), amp=0.8)
    np.testing.assert_array_equal(obs2, obs3)
    np.testing.assert_array_equal(q2, q3)
    assert not e2._th_off.any() and (e2._th_t != 29.0).any()


def test_walker_heats_and_a_servo_at_the_cutoff_goes_limp_and_stays_limp():
    e = _walker(servo_thermal=29.0)
    _roll(e, steps=50, amp=0.8)
    rep = e.thermal_report()
    assert set(rep) == set(e._servo_names)
    heat = np.array([rep[n]["heat_w"] for n in e._servo_names])
    assert heat.max() > 0.0 and all(r["cutoff_s"] is None for r in rep.values())
    # each servo's rise over 1 s is its mean heat / heat capacity (C = tau / Rth), to first order
    _, _, tau_th, rth = e._th_consts
    rise = np.array([rep[n]["t_end"] - rep[n]["t_start"] for n in e._servo_names])
    np.testing.assert_allclose(rise, heat * 1.0 * rth / tau_th, rtol=0.02, atol=1e-6)
    # every servo a hair under the cutoff: the loaded ones trip and give no torque from then on
    e = _walker(servo_thermal=29.0)
    q_on = _roll(e, steps=60, amp=0.8, start=TH.T_CUTOFF - 1e-3)[1]
    off = e._th_off.copy()
    assert off.any()
    assert np.all(e._servo_tau[off] == 0.0)
    q_off = _roll(_walker(), steps=60, amp=0.8)[1]
    assert not np.array_equal(q_on, q_off)                           # the plant feels it
    rep = e.thermal_report()
    assert all((rep[n]["cutoff_s"] is not None) == bool(off[i]) for i, n in enumerate(e._servo_names))
    e.step(np.zeros(e.action_space.shape, np.float32))
    assert np.all(e._th_off[off])                                    # latched
    e.reset(seed=4)
    assert not e._th_off.any() and np.all(e._th_t == 29.0)          # a new episode starts fresh


def test_walker_draws_the_start_per_episode_per_servo_inside_the_range():
    e = _walker(servo_thermal="29:69", domain_rand=True)
    seen = []
    for seed in range(4):
        e.reset(seed=seed)
        assert np.all((29.0 <= e._th_t) & (e._th_t <= 69.0)) and not e._th_off.any()
        assert len(set(np.round(e._th_t, 9))) == e._n_servo
        seen.append(e._th_t.copy())
    assert not np.array_equal(seen[0], seen[1])


def test_cpu_mjx_parity():
    """sim/mjx/parity_test.py R29, in a subprocess (parity_test turns jax_enable_x64 on)."""
    env = dict(os.environ, JAX_PLATFORMS="cpu", MUJOCO_GL="disabled")
    p = subprocess.run([sys.executable, "parity_test.py", "--servo-thermal"], cwd=os.path.join(SIM, "mjx"),
                       env=env, capture_output=True, text=True, timeout=1800)
    assert p.returncode == 0, p.stdout[-2000:] + p.stderr[-2000:]


def test_grade_term_switches_it_on_and_off():
    """grade.py's GRADE_PLANT_TERMS=servo_temp=<degC> (weight-transfer-gate gate_common.apply_terms):
    live at once and after a reset; <= 0 switches it off again."""
    sys.path.insert(0, os.path.join(ROOT, "experiments", "policy", "weight-transfer-gate"))
    import gate_common as GC
    e = _walker()
    e.reset(seed=1)
    GC.apply_terms(e, dict(servo_temp=60.0))
    assert np.all(e._th_t == 60.0)
    e.step(np.zeros(e.action_space.shape, np.float32))
    assert e.thermal_report() is not None
    e.reset(seed=2)
    assert np.all(e._th_t == 60.0)
    GC.apply_terms(e, dict(servo_temp=0))
    assert e._th_t is None and e.thermal_report() is None
    e.reset(seed=3)
    assert e._th_t is None


def test_referee_thermal_summary():
    import eval_precision as ep
    r1 = {"L_knee": dict(heat_w=0.5, t_start=29.0, t_end=29.1, t_peak=29.1, cutoff_s=None,
                         min_to_cutoff=None)}
    r2 = {"L_knee": dict(heat_w=3.0, t_start=69.0, t_end=70.2, t_peak=70.2, cutoff_s=1.5,
                         min_to_cutoff=12.0)}
    s = ep.thermal_summary([r1, r2])["L_knee"]
    assert s == dict(heat_w=3.0, rise_c=1.2, t_peak=70.2, cut_off="1/2", min_to_cutoff=12.0)
