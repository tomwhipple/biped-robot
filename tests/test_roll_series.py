"""--roll-series-k (2026-11-10): a spring in series with the hip / ankle roll servos, beyond
the encoder.

The 2026-11-10 clamp test (experiments/policy/weight-transfer-gate/2026-11-10-robot/README.md,
section 4; issue #277): one foot clamped, the other lifted 30 mm, the torso tilts 9.4 deg
while the stance roll encoders move 1.5-1.6 deg (ankle) and 0.4-0.7 deg (hip). The model,
quasi-static with a massless shaft: the link angle is the servo shaft angle minus the spring's
deflection tau/k, so the link sees the roll PD in series with k (gains x 1/(1 + kp/k)) and the
encoders (q obs, the dq obs and its reg-58 window history, the firmware's goal-speed feedback)
read the shaft = link + tau/k. k is drawn per episode per role (L and R share it). Off by
default, bit-exact.
"""
import inspect
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "sim")
sys.path.insert(0, os.path.join(SIM, "mjx"))
sys.path.insert(0, SIM)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import walker_env as W  # noqa: E402

ARGS = ["--robot", "--xml", "bimo_biped_v6legs_meas_sole.xml", "--servo-kp-scale", "asbuilt",
        "--precision", "--mjx-impl", "jax", "--family", "loco", "--quantize"]
FIT = "hip=11.0,ankle=17.75"      # weight-transfer-gate Round 3, results/r3_item1_clamp_fit.md


def _kw(*extra):
    import train_mjx
    return train_mjx.build_env_kw(train_mjx.make_parser().parse_args(ARGS + list(extra)))


def _act(n):
    return 0.4 * np.sin(np.arange(n, dtype=np.float32) + 0.3)


# -- parsing and plumbing -------------------------------------------------------
def test_defaults_parser_and_cli():
    from env_mjx import BimoMJXEnv
    for cls in (BimoMJXEnv, W.BimoWalkerEnv):
        assert inspect.signature(cls.__init__).parameters["roll_series_k"].default is None
    P = W.parse_roll_series_k
    assert P(None) is None and P("") is None and P({}) is None
    assert P("hip=11") == {"hip": (11.0, 11.0)}
    assert P("hip=8:16,ankle=12:25") == {"hip": (8.0, 16.0), "ankle": (12.0, 25.0)}
    assert P({"hip": [8, 16], "ankle": 17.75}) == {"hip": (8.0, 16.0), "ankle": (17.75, 17.75)}
    for bad in ("knee=10", "hip=16:8", "hip=0", "hip=-3", "hip", "hip=1:2:3"):
        with pytest.raises(ValueError):
            P(bad)
    assert "roll_series_k" not in _kw()
    kw = _kw("--roll-series-k", "hip=8:16,ankle=12:25")
    assert kw["roll_series_k"] == {"hip": [8.0, 16.0], "ankle": [12.0, 25.0]}   # JSON-ready
    assert W.parse_roll_series_k(kw["roll_series_k"]) == P("hip=8:16,ankle=12:25")


# -- walker_env (the CPU referee / grade.py twin) ------------------------------
def _walker(**over):
    import eval_precision as ep
    cfg = dict(_kw(), **over)
    return ep.make_env(cfg, 4.0, True, cfg["xml_path"])


def _walker_roll(env, steps=8, seed=3):
    obs0, _ = env.reset(seed=seed)
    a = _act(env.action_space.shape[0])
    obs, q = [obs0], [np.array(env.data.qpos)]
    for _ in range(steps):
        o, *_ = env.step(a)
        obs.append(o)
        q.append(np.array(env.data.qpos))
    return np.array(obs), np.array(q)


def test_walker_off_is_bit_exact():
    e0 = _walker()
    assert e0.roll_series_k is None and e0._ser_c is None
    obs0, q0 = _walker_roll(e0)
    obs1, q1 = _walker_roll(_walker(roll_series_k=None))
    np.testing.assert_array_equal(obs0, obs1)
    np.testing.assert_array_equal(q0, q1)


def test_walker_on_reads_the_shaft():
    e = _walker(roll_series_k=FIT, dq_obs_window=2)
    roles = {n: e._ser_role[i] for i, n in enumerate(e._servo_names)}
    assert {n for n, r in roles.items() if r == 0} == {"L_hip_roll", "R_hip_roll"}
    assert {n for n, r in roles.items() if r == 1} == {"L_ankle_roll", "R_ankle_roll"}
    obs, q = _walker_roll(e, steps=10)
    c = e._ser_c
    for i, n in enumerate(e._servo_names):
        want = 1 / 11.0 if n.endswith("hip_roll") else 1 / 17.75 if n.endswith("ankle_roll") else 0.0
        assert c[i] == pytest.approx(want)
    # the encoder q = link + tau/k on the rolls, the link elsewhere (no quantize in make_env's
    # nominal plant unless the config asks; ARGS has --quantize, so allow one tick)
    nj = e._nq_act
    defl = e._servo_tau * c
    np.testing.assert_allclose(e._ser_defl, defl)
    link = e.data.qpos[e._jqpos]
    tick = 2 * np.pi / 4096
    assert np.max(np.abs(obs[-1][:nj] - (link + e._p(defl)))) <= tick
    assert np.abs(defl).max() > math.radians(0.05)            # the spring carries load
    # the physics differs from the rigid plant (the PD is softer)
    _, q_off = _walker_roll(_walker(dq_obs_window=2), steps=10)
    assert np.abs(q - q_off).max() > 1e-4
    # a reset zeroes the deflection
    e.reset(seed=5)
    assert np.all(e._ser_defl == 0.0)


def test_walker_draw_per_episode_and_fixed_draws_nothing():
    e = _walker(roll_series_k="hip=8:16,ankle=12:25")
    ks = []
    for s in range(12):
        e.reset(seed=s)
        c = e._ser_c
        hip = [1 / c[i] for i, n in enumerate(e._servo_names) if n.endswith("hip_roll")]
        ank = [1 / c[i] for i, n in enumerate(e._servo_names) if n.endswith("ankle_roll")]
        assert hip[0] == pytest.approx(hip[1]) and ank[0] == pytest.approx(ank[1])   # L = R
        assert 8.0 <= hip[0] <= 16.0 and 12.0 <= ank[0] <= 25.0
        ks.append((hip[0], ank[0]))
    assert len({round(h, 6) for h, _ in ks}) > 6
    # a fixed k consumes no RNG: the episode's other draws are the rigid run's
    a = _walker(roll_series_k=FIT, backlash_deg=0.0, backlash_deg_max=0.25)
    b = _walker(backlash_deg=0.0, backlash_deg_max=0.25)
    a.reset(seed=11)
    b.reset(seed=11)
    assert a._lash_rad == b._lash_rad
    assert a.np_random.uniform() == b.np_random.uniform()


# -- env_mjx -------------------------------------------------------------------
def _mjx_roll(env, steps=6, key=3):
    import jax
    st = jax.jit(env.reset)(jax.random.PRNGKey(key))
    step = jax.jit(env.step)
    a = jax.numpy.asarray(_act(env.action_size))
    obs, q = [np.asarray(st.obs)], [np.asarray(st.data.qpos)]
    for _ in range(steps):
        st = step(st, a)
        obs.append(np.asarray(st.obs))
        q.append(np.asarray(st.data.qpos))
    return st, np.array(obs), np.array(q)


@pytest.fixture(scope="module")
def mjx_off():
    from env_mjx import BimoMJXEnv
    return _mjx_roll(BimoMJXEnv(**_kw()))


def test_mjx_off_is_bit_exact(mjx_off):
    from env_mjx import BimoMJXEnv
    st0, obs0, q0 = mjx_off
    st1, obs1, q1 = _mjx_roll(BimoMJXEnv(**dict(_kw(), roll_series_k=None)))
    np.testing.assert_array_equal(obs0, obs1)
    np.testing.assert_array_equal(q0, q1)
    assert st0.ser_c.shape == (0,) and st0.ser_defl.shape == (0,)


def test_mjx_on_reads_the_shaft_and_draws(mjx_off):
    import jax
    from env_mjx import BimoMJXEnv
    env = BimoMJXEnv(**dict(_kw(), roll_series_k=FIT))
    st, obs, q = _mjx_roll(env)
    names = env._servo_names
    c = np.asarray(st.ser_c)
    for i, n in enumerate(names):
        want = 1 / 11.0 if n.endswith("hip_roll") else 1 / 17.75 if n.endswith("ankle_roll") else 0.0
        assert c[i] == pytest.approx(want, rel=1e-6)
    defl = np.asarray(st.ser_defl)
    assert np.abs(defl).max() > math.radians(0.05)
    assert np.all(defl[c == 0.0] == 0.0)
    nj = env._nq_act
    link = np.asarray(env._p(st.data.qpos[env._jq0:env._jq1]))
    tick = 2 * np.pi / 4096
    assert np.max(np.abs(obs[-1][:nj] - (link + np.asarray(env._p(defl))))) <= tick + 1e-6
    _, _, q_off = mjx_off
    assert np.abs(q - q_off).max() > 1e-4
    # per-episode draw, per role, L = R, inside the range
    env2 = BimoMJXEnv(**dict(_kw(), roll_series_k="hip=8:16,ankle=12:25"))
    sts = jax.jit(jax.vmap(env2.reset))(jax.random.split(jax.random.PRNGKey(0), 16))
    k = 1.0 / np.asarray(sts.ser_c)[:, [names.index(n) for n in
                                        ("L_hip_roll", "R_hip_roll", "L_ankle_roll", "R_ankle_roll")]]
    np.testing.assert_allclose(k[:, 0], k[:, 1], rtol=1e-6)
    np.testing.assert_allclose(k[:, 2], k[:, 3], rtol=1e-6)
    assert k[:, 0].min() >= 8.0 - 1e-4 and k[:, 0].max() <= 16.0 + 1e-4
    assert k[:, 2].min() >= 12.0 - 1e-4 and k[:, 2].max() <= 25.0 + 1e-4
    assert len(set(np.round(k[:, 0], 4))) > 8
    assert np.all(np.asarray(sts.ser_defl) == 0.0)


# -- physics: the clamp test and a zero-action stand ---------------------------
def _gate_train_env(xml, rs, lash):
    """weight-transfer-gate's `train` plant (the prec16 recipe's chain at its DR centre) with the
    roll lash at `lash` deg and roll_series_k = rs, built on `xml`."""
    import eval_precision as EP
    sys.path.insert(0, os.path.join(HERE, "..", "experiments", "policy", "weight-transfer-gate"))
    import gate_common as GC
    c = dict(GC.TRAIN_MID)
    cfg = GC.run_cfg() if os.path.exists(os.path.join(GC.run_dir(), "config.json")) else None
    if cfg is None:
        pytest.skip("the prec16 run's config.json is not on this machine (gitignored sim/runs)")
    extra = dict(roll_lash_deg=lash, roll_lash_deg_max=None, backlash_deg=c["backlash"],
                 backlash_deg_max=None, latency_ms=c["latency_ms"], joint_frictionloss=c["friction"],
                 fw_shaper_hz=0.0, push_prob=0.0, push_force=0.0, roll_series_k=rs)
    env = EP.make_env(cfg, 120.0, nominal=True, xml=xml, act_lag_hz=0.0, extra=extra)
    EP.set_contact_friction(env, c["mu"])
    env._kp_prof = np.array(env._kp_prof, float) * c["kp"]
    env._kd_prof = np.array(env._kd_prof, float) * c["kd"]
    env.reset(seed=0)
    env._zero_off = None
    env.domain_rand = False
    GC.install_chain(env, GC.TrainChain(env._servo_names, env.control_dt, c["lag_hz"],
                                        c["lag_scale_hip_roll"], c["dead15_ticks"], c["dead50_ticks"]))
    return env, GC


def _shaft_encoder(env, i):
    """what servo i's encoder reads as its error past the deadzones: target - SHAFT, where the
    shaft = link + tau/k (gate_common.encoder_err's form for the env's own spring)."""
    defl = env._ser_defl[i] if env._ser_c is not None else 0.0
    e = abs(float(env.data.ctrl[i]) - float(env.data.qpos[env._jq0 + i]) - defl)
    half = 0.5 * (env._lash_rad + env._roll_lash_rad * env._roll_mask[i])
    return max(0.0, e - half)


@pytest.mark.parametrize("side", ["L", "R"])
def test_clamp_tilt_at_the_fit(side, monkeypatch):
    """The 2026-11-10 clamp test (30 mm lift, stance foot welded; robot: tilt 9.4 deg, stance
    encoders ankle 1.5-1.6 / hip 0.4-0.7 deg) on the training plant with the Round 3 springs
    and no roll play (the fit's condition): the tilt is the robot's, the encoders stay small.
    Measured when written: 9.34 (L lifted) / 9.22 (R); encoders 1.83 / 0.73 deg. Rigid: 2.8 deg."""
    sys.path.insert(0, SIM)
    import roll_compliance as RC
    xml = RC.clamped_xml(os.path.join(SIM, "bimo_biped_v6legs_meas_sole.xml"), "R" if side == "L" else "L")
    try:
        env, GC = _gate_train_env(xml, FIT, 0.0)
        monkeypatch.setattr(GC, "encoder_err", _shaft_encoder)
        tilt, enc, lift = GC.clamp_run(env, side)
    finally:
        os.unlink(xml)
    assert 8.4 < tilt < 10.4
    assert 1.2 < enc["ankle_roll"] < 2.3 and 0.3 < enc["hip_roll"] < 1.1
    assert lift > 10.0                       # the lifted foot is off the floor


def test_zero_action_stand_at_the_soft_end():
    """A zero action (the stand pose) on the training plant at the soft end of the recipe range
    (hip 11, ankle 17.75 N m/rad) with the roll play at its DR top (2.0 deg) does not fall."""
    env, _ = _gate_train_env(os.path.join(SIM, "bimo_biped_v6legs_meas_sole.xml"), FIT, 2.0)
    env.reset(seed=0)
    z = np.zeros(env.action_space.shape[0], np.float32)
    ups = []
    for _ in range(int(4.0 / env.control_dt)):
        _, _, term, trunc, _ = env.step(z)
        assert not term
        ups.append(float(env.data.sensordata[env._up_id + 2]))
    tilt = math.degrees(math.acos(min(1.0, min(ups))))
    assert tilt < 10.0
    assert np.abs(env._ser_defl).max() > 0.0


def test_cpu_mjx_parity_airborne():
    """sim/mjx/parity_test.py R15 (airborne arithmetic with the spring and the dq window), 20
    synced steps, float64, in a subprocess (parity_test turns jax_enable_x64 on at import).
    The full block: `python sim/mjx/parity_test.py --roll-series`."""
    import subprocess
    code = (
        "import sys; sys.path.insert(0, %r); import parity_test as P\n"
        "c, g, s = P.envs_for(P.rob_kw(roll_series_k=P.ROLL_SERIES_PARITY, dq_obs_window=2),"
        " solver_tol=P.ARITH_TOL)\n"
        "ok = P.run_block('R15 quick', 20, P.air_acts(c), P.hoist, P.AIR_GATES, (c, g, s))\n"
        "ok = ok and float(abs(c._ser_defl).max()) > 1e-4\n"
        "sys.exit(0 if ok else 1)\n") % os.path.join(SIM, "mjx")
    env = dict(os.environ, JAX_PLATFORMS="cpu", MUJOCO_GL="disabled")
    p = subprocess.run([sys.executable, "-c", code], cwd=os.path.join(SIM, "mjx"), env=env,
                       capture_output=True, text=True, timeout=900)
    assert p.returncode == 0, p.stdout[-2000:] + p.stderr[-2000:]


def test_config_json_records_it(monkeypatch, tmp_path):
    """train_mjx.main writes it into config.json (and nothing else changes); without the flag
    the key is absent."""
    import json
    import train_mjx

    class Started(Exception):
        pass

    def no_training(*a, **k):
        raise Started

    monkeypatch.setattr(train_mjx, "RUNS", str(tmp_path))
    monkeypatch.setattr(train_mjx.ppo, "train", no_training)
    cfg = {}
    for name, extra in (("off", []), ("on", ["--roll-series-k", "hip=11:15,ankle=17.75:24"])):
        monkeypatch.setattr(sys, "argv", ["train_mjx", "--out", name, "--envs", "4"] + ARGS + extra)
        with pytest.raises(Started):
            train_mjx.main()
        cfg[name] = json.load(open(tmp_path / name / "config.json"))
    assert "roll_series_k" not in cfg["off"]
    assert cfg["on"]["roll_series_k"] == {"hip": [11.0, 15.0], "ankle": [17.75, 24.0]}
    drop = ("train", "git_sha", "git_dirty", "roll_series_k")
    assert ({k: v for k, v in cfg["off"].items() if k not in drop}
            == {k: v for k, v in cfg["on"].items() if k not in drop})


def test_grade_rebuilds_the_run_on_its_spring(monkeypatch, tmp_path):
    """grade.py / the RCA harness build a run's env from its config.json (eval_stand_arm._setup
    -> eval_precision.make_env): a run trained with the spring is graded with it."""
    import json
    import eval_stand_arm as ESA
    sys.path.insert(0, os.path.join(HERE, "..", "experiments", "policy", "weight-transfer-gate"))
    import gate_common as GC
    src = GC.run_dir()          # the prec16 student, GATE_MAIN's sim/runs
    if not os.path.exists(os.path.join(src, "params.pkl")):
        pytest.skip("the prec16 run is not on this machine (gitignored sim/runs)")
    run = "rs_probe"
    os.makedirs(tmp_path / run)
    os.symlink(os.path.join(src, "params.pkl"), tmp_path / run / "params.pkl")
    cfg = json.load(open(os.path.join(src, "config.json")))
    cfg["roll_series_k"] = {"hip": [11.0, 11.0], "ankle": [17.75, 17.75]}
    json.dump(cfg, open(tmp_path / run / "config.json", "w"))
    monkeypatch.setattr(ESA, "RUNS", str(tmp_path))
    for plant in ("fit", "train"):
        env, _, _ = ESA._setup(run, plant, 0.0, 2.0)
        env.reset(seed=0)
        c = env._ser_c
        assert c is not None
        assert sorted(np.round(1 / c[c > 0], 2).tolist()) == [11.0, 11.0, 17.75, 17.75]
        ESA._CACHE.pop((run, plant, 0.0, 2.0), None)
