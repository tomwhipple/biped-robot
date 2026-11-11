"""--roll-play-deg (2026-11-11): free play inside the hip / ankle roll servos, beyond the encoder.

The 2026-11-11 clamp session (hw_sessions/2026-11-11/m277/NOTES.md, issue #277): the play is inside
the roll servos, between the position sensor and the output horn. With the R foot clamped and the
L foot lifted 30 mm the torso tilts 8.45-8.7 deg while the stance ankle-roll encoder moves 1.1-1.5
deg and the hip roll 0.35-0.45; a fingertip then swings the torso 9.4 deg with the encoders moving
2.7 / 0.6. The model: the servo shaft is position-controlled and is what the encoder reads (q and
dq obs, the dq window history, the firmware's goal-speed feedback); the link floats inside
+/-play/2 of the shaft with no torque between them and is driven only at the band edges, by a
contact that pushes only. Drawn per episode per joint. Off by default, bit-exact.
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
FIT = "hip=4,ankle=5"          # fit_rollplay.py on grade + MASS_SYM, no roll lash (results/rp_fit.md)
TOP = "hip=5,ankle=6"          # the top of the recipe range hip=3:5,ankle=4:6
ROLLS = ("L_hip_roll", "R_hip_roll", "L_ankle_roll", "R_ankle_roll")


def _kw(*extra):
    import train_mjx
    return train_mjx.build_env_kw(train_mjx.make_parser().parse_args(ARGS + list(extra)))


def _act(n):
    return 0.4 * np.sin(np.arange(n, dtype=np.float32) + 0.3)


def _half_deg(env_or_h, names):
    h = np.asarray(env_or_h)
    return {n: math.degrees(2 * h[i]) for i, n in enumerate(names)}


# -- parsing and plumbing -------------------------------------------------------
def test_defaults_parser_and_cli():
    from env_mjx import BimoMJXEnv
    for cls in (BimoMJXEnv, W.BimoWalkerEnv):
        assert inspect.signature(cls.__init__).parameters["roll_play_deg"].default is None
    P = W.parse_roll_play_deg
    assert P(None) is None and P("") is None and P({}) is None and P("0") is None
    assert P("4") == {"hip": (4.0, 4.0), "ankle": (4.0, 4.0)}
    assert P("3:5") == {"hip": (3.0, 5.0), "ankle": (3.0, 5.0)}
    assert P("hip=3:5,ankle=4:6") == {"hip": (3.0, 5.0), "ankle": (4.0, 6.0)}
    assert P("ankle=5") == {"ankle": (5.0, 5.0)}
    assert P({"hip": [3, 5], "ankle": 5}) == {"hip": (3.0, 5.0), "ankle": (5.0, 5.0)}
    assert P(2.5) == {"hip": (2.5, 2.5), "ankle": (2.5, 2.5)}
    for bad in ("knee=3", "hip=5:3", "hip=-1", "hip=1:2:3", "a:b", "hip=40"):
        with pytest.raises(ValueError):
            P(bad)
    assert "roll_play_deg" not in _kw()
    kw = _kw("--roll-play-deg", "hip=3:5,ankle=4:6")
    assert kw["roll_play_deg"] == {"hip": [3.0, 5.0], "ankle": [4.0, 6.0]}   # JSON-ready
    assert P(kw["roll_play_deg"]) == P("hip=3:5,ankle=4:6")
    # tuples (distill_student.env_kwargs turns config lists into tuples) parse the same
    assert P({k: tuple(v) for k, v in kw["roll_play_deg"].items()}) == P("hip=3:5,ankle=4:6")


def test_exclusive_with_the_series_spring():
    from env_mjx import BimoMJXEnv
    kw = dict(_kw(), roll_play_deg=FIT, roll_series_k="hip=11,ankle=17.75")
    with pytest.raises(ValueError):
        BimoMJXEnv(**kw)
    with pytest.raises(ValueError):
        _walker(roll_play_deg=FIT, roll_series_k="hip=11,ankle=17.75")


def test_substep_kernel():
    """roll_play_substep: inside the band the shaft runs to the set point and the link is free;
    past an edge the shaft is held at the edge and the contact sign points from shaft to link."""
    h, lim = np.array([0.05, 0.05, 0.0]), np.array([1.0, 0.01, 1.0])
    cur, q, shaft = np.array([0.02, 0.5, 0.3]), np.zeros(3), np.zeros(3)
    eng, sg, s = W.roll_play_substep(cur, q, shaft, h, lim)
    assert not eng[0] and s[0] == pytest.approx(0.02)          # free: the shaft is at the set point
    assert not eng[1] and s[1] == pytest.approx(0.01)          # free, rate-limited
    assert eng[2] and sg[2] == 1.0 and s[2] == 0.0             # h = 0: held on the link
    eng, sg, s = W.roll_play_substep(np.array([-0.2]), np.array([0.0]), np.array([0.0]),
                                     np.array([0.05]), np.array([1.0]))
    assert eng[0] and sg[0] == -1.0 and s[0] == pytest.approx(-0.05)


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
    assert e0.roll_play_deg is None and e0._rp_h is None
    obs0, q0 = _walker_roll(e0)
    obs1, q1 = _walker_roll(_walker(roll_play_deg=None))
    np.testing.assert_array_equal(obs0, obs1)
    np.testing.assert_array_equal(q0, q1)


def test_walker_on_reads_the_shaft():
    e = _walker(roll_play_deg=FIT, dq_obs_window=2)
    obs, q = _walker_roll(e, steps=25)
    want = {n: (4.0 if n.endswith("hip_roll") else 5.0 if n.endswith("ankle_roll") else 0.0)
            for n in e._servo_names}
    got = _half_deg(e._rp_h, e._servo_names)
    for n in e._servo_names:
        assert got[n] == pytest.approx(want[n])
    off = e._rp_off
    assert np.all(np.abs(off) <= e._rp_h + 1e-12)                 # inside the band
    assert np.abs(off).max() > math.radians(0.3)                  # the play is in use
    assert np.all(off[e._rp_h == 0.0] == 0.0)                     # nothing off the rolls
    nj = e._nq_act
    link = e.data.qpos[e._jqpos]
    tick = 2 * np.pi / 4096
    assert np.max(np.abs(obs[-1][:nj] - (link + e._p(off)))) <= tick
    # the dq window reads the shaft too
    np.testing.assert_allclose(e._dq_qhist[0], link + e._p(off))
    _, q_off = _walker_roll(_walker(dq_obs_window=2), steps=25)
    assert np.abs(q - q_off).max() > 1e-4
    e.reset(seed=5)
    assert np.all(e._rp_off == 0.0)


def test_walker_draw_per_episode_per_joint_and_fixed_draws_nothing():
    e = _walker(roll_play_deg="hip=3:5,ankle=4:6")
    idx = [e._servo_names.index(n) for n in ROLLS]
    draws = []
    for s in range(12):
        e.reset(seed=s)
        pp = np.degrees(2 * e._rp_h[idx])
        assert 3.0 <= pp[0] <= 5.0 and 3.0 <= pp[1] <= 5.0
        assert 4.0 <= pp[2] <= 6.0 and 4.0 <= pp[3] <= 6.0
        draws.append(pp)
    draws = np.array(draws)
    assert np.abs(draws[:, 0] - draws[:, 1]).max() > 0.1           # L and R drawn apart
    assert len({round(x, 6) for x in draws[:, 2]}) > 8
    a = _walker(roll_play_deg=FIT, backlash_deg=0.0, backlash_deg_max=0.25)
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
    st1, obs1, q1 = _mjx_roll(BimoMJXEnv(**dict(_kw(), roll_play_deg=None)))
    np.testing.assert_array_equal(obs0, obs1)
    np.testing.assert_array_equal(q0, q1)
    assert st0.rp_h.shape == (0,) and st0.rp_off.shape == (0,)


def test_mjx_on_reads_the_shaft_and_draws(mjx_off):
    import jax
    from env_mjx import BimoMJXEnv
    env = BimoMJXEnv(**dict(_kw(), roll_play_deg=FIT))
    st, obs, q = _mjx_roll(env)
    names = env._servo_names
    h = np.asarray(st.rp_h)
    got = _half_deg(h, names)
    for n in names:
        want = 4.0 if n.endswith("hip_roll") else 5.0 if n.endswith("ankle_roll") else 0.0
        assert got[n] == pytest.approx(want, abs=1e-4)
    off = np.asarray(st.rp_off)
    assert np.abs(off).max() > math.radians(0.3)
    assert np.all(np.abs(off) <= h + 1e-6) and np.all(off[h == 0.0] == 0.0)
    nj = env._nq_act
    link = np.asarray(env._p(st.data.qpos[env._jq0:env._jq1]))
    tick = 2 * np.pi / 4096
    assert np.max(np.abs(obs[-1][:nj] - (link + np.asarray(env._p(off))))) <= tick + 1e-6
    _, _, q_off = mjx_off
    assert np.abs(q - q_off).max() > 1e-4
    env2 = BimoMJXEnv(**dict(_kw(), roll_play_deg="hip=3:5,ankle=4:6"))
    sts = jax.jit(jax.vmap(env2.reset))(jax.random.split(jax.random.PRNGKey(0), 16))
    pp = np.degrees(2 * np.asarray(sts.rp_h))[:, [names.index(n) for n in ROLLS]]
    assert pp[:, :2].min() >= 3.0 - 1e-4 and pp[:, :2].max() <= 5.0 + 1e-4
    assert pp[:, 2:].min() >= 4.0 - 1e-4 and pp[:, 2:].max() <= 6.0 + 1e-4
    assert np.abs(pp[:, 0] - pp[:, 1]).max() > 0.1
    assert len(set(np.round(pp[:, 2], 4))) > 8
    assert np.all(np.asarray(sts.rp_off) == 0.0)


def test_cpu_mjx_parity_airborne():
    """sim/mjx/parity_test.py R16 (airborne arithmetic with the play and the dq window), 20
    synced steps, float64, in a subprocess (parity_test turns jax_enable_x64 on at import).
    The full block: `python sim/mjx/parity_test.py --roll-play`."""
    import subprocess
    code = (
        "import sys; sys.path.insert(0, %r); import parity_test as P\n"
        "c, g, s = P.envs_for(P.rob_kw(roll_play_deg=P.ROLL_PLAY_PARITY, dq_obs_window=2),"
        " solver_tol=P.ARITH_TOL)\n"
        "ok = P.run_block('R16 quick', 20, P.air_acts(c), P.hoist, P.AIR_GATES, (c, g, s))\n"
        "ok = ok and float(abs(c._rp_off).max()) > 1e-4\n"
        "sys.exit(0 if ok else 1)\n") % os.path.join(SIM, "mjx")
    env = dict(os.environ, JAX_PLATFORMS="cpu", MUJOCO_GL="disabled")
    p = subprocess.run([sys.executable, "-c", code], cwd=os.path.join(SIM, "mjx"), env=env,
                       capture_output=True, text=True, timeout=900)
    assert p.returncode == 0, p.stdout[-2000:] + p.stderr[-2000:]


# -- config.json and the env rebuilds (referee, grade.py, distil) -----------------
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
    for name, extra in (("off", []), ("on", ["--roll-play-deg", "hip=3:5,ankle=4:6"])):
        monkeypatch.setattr(sys, "argv", ["train_mjx", "--out", name, "--envs", "4"] + ARGS + extra)
        with pytest.raises(Started):
            train_mjx.main()
        cfg[name] = json.load(open(tmp_path / name / "config.json"))
    assert "roll_play_deg" not in cfg["off"]
    assert cfg["on"]["roll_play_deg"] == {"hip": [3.0, 5.0], "ankle": [4.0, 6.0]}
    drop = ("train", "git_sha", "git_dirty", "roll_play_deg")
    assert ({k: v for k, v in cfg["off"].items() if k not in drop}
            == {k: v for k, v in cfg["on"].items() if k not in drop})
    # the distil rebuilds the teacher's env from this config.json (and copies it to the student)
    import distill_student as DS
    from env_mjx import BimoMJXEnv
    kw = DS.env_kwargs(cfg["on"], mjx_impl="jax")
    assert W.parse_roll_play_deg(kw["roll_play_deg"]) == {"hip": (3.0, 5.0), "ankle": (4.0, 6.0)}
    assert BimoMJXEnv(**kw).roll_play_deg == {"hip": (3.0, 5.0), "ankle": (4.0, 6.0)}
    assert "roll_play_deg" not in DS.env_kwargs(cfg["off"], mjx_impl="jax")


def test_referee_and_grade_rebuild_the_run_on_its_play(monkeypatch, tmp_path):
    """eval_precision.make_env (the referee) and eval_stand_arm._setup (grade.py's build) build a
    run's env from its config.json: a run trained with the play is graded with it."""
    import json
    import eval_precision as EP
    import eval_stand_arm as ESA
    sys.path.insert(0, GATE)
    import gate_common as GC
    src = GC.run_dir()          # the prec16 student, GATE_MAIN's sim/runs
    if not os.path.exists(os.path.join(src, "params.pkl")):
        pytest.skip("the prec16 run is not on this machine (gitignored sim/runs)")
    run = "rp_probe"
    os.makedirs(tmp_path / run)
    os.symlink(os.path.join(src, "params.pkl"), tmp_path / run / "params.pkl")
    cfg = json.load(open(os.path.join(src, "config.json")))
    cfg["roll_play_deg"] = {"hip": [4.0, 4.0], "ankle": [5.0, 5.0]}
    json.dump(cfg, open(tmp_path / run / "config.json", "w"))
    want = [4.0, 4.0, 5.0, 5.0]
    for nominal in (True, False):
        env = EP.make_env(cfg, 2.0, nominal, os.path.join(SIM, "bimo_biped_v6legs_meas_sole.xml"))
        env.reset(seed=0)
        assert sorted(np.round(np.degrees(2 * env._rp_h[env._rp_h > 0]), 4).tolist()) == want
    monkeypatch.setattr(ESA, "RUNS", str(tmp_path))
    for plant in ("fit", "train"):
        env, _, _ = ESA._setup(run, plant, 0.0, 2.0)
        env.reset(seed=0)
        assert sorted(np.round(np.degrees(2 * env._rp_h[env._rp_h > 0]), 4).tolist()) == want
        ESA._CACHE.pop((run, plant, 0.0, 2.0), None)


# -- physics: the clamp numbers at the fit, and a zero-action stand at the top --------
def _need_prec16():
    sys.path.insert(0, GATE)
    import gate_common as GC
    if not os.path.exists(os.path.join(GC.run_dir(), "config.json")):
        pytest.skip("the prec16 run's config.json is not on this machine (gitignored sim/runs)")


@pytest.fixture(scope="module")
def clamp_fit():
    """fit_rollplay's four tests on its base plant (grade + MASS_SYM) at the fit, and the lift
    without the play."""
    _need_prec16()
    sys.path.insert(0, GATE)
    import fit_rollplay as F
    base = "grade:" + F.GC.MASS_SYM.replace(",", "|")
    fit = {t: F.run_test((base, dict(play_hip=4.0, play_ankle=5.0), t)) for t in F.TESTS}
    none = F.run_test((base, {}, "lift30"))
    return fit, none


def test_clamp_numbers_at_the_fit(clamp_fit):
    """The 2026-11-11 clamp signature at the fitted play (hip 4, ankle 5 deg peak to peak). Robot:
    30 mm lift tilts 8.45-8.7 deg with the stance encoders ankle 1.1-1.5 / hip 0.35-0.45; 15 mm
    7.0 with the lifted foot barely touching; after an 81 mm shift 2.7; a fingertip swings the torso
    9.4 deg, 6.1 of it beyond the encoders. Measured when written: 7.87 (1.91 / 0.83); 7.85 (pad
    0.9 mm up); 2.45; swing 9.42 with 5.75 beyond, 0.19 N/deg. Without the play: 2.67."""
    fit, none = clamp_fit
    a, b, c, d = fit["lift30"], fit["lift15"], fit["shift"], fit["rock"]
    assert 7.3 < a["tilt"] < 9.3
    assert a["enc_ankle"] < 2.3 and a["enc_hip"] < 1.1        # the encoders do not see the play
    assert 6.5 < b["tilt"] < 9.0 and b["clear_mm"] < 3.0      # the 15 mm foot barely clears
    assert 1.8 < c["tilt"] < 3.3                              # the shift takes most of it away
    assert d["swing"] > 9.0 and 4.8 < d["beyond"] < 7.2
    assert d["n_per_deg"] < 0.4                               # free play: little push per degree
    assert none["tilt"] < 3.5                                 # the rigid plant cannot tilt 8 deg


def test_zero_action_stand_at_the_top_of_the_range():
    """A zero action (the stand pose) on the training plant (the prec16 chain at its DR centre,
    no roll lash) with the play at the top of the recipe range (hip 5, ankle 6) holds 4 s."""
    _need_prec16()
    sys.path.insert(0, GATE)
    import gate_common as GC
    env = GC.make_plant("train", terms=dict(lash=0.0, play_hip=5.0, play_ankle=6.0))
    env.reset(seed=0)
    env._zero_off = None
    z = np.zeros(env.action_space.shape[0], np.float32)
    ups = []
    for _ in range(int(4.0 / env.control_dt)):
        _, _, term, trunc, _ = env.step(z)
        assert not term
        ups.append(float(env.data.sensordata[env._up_id + 2]))
    tilt = math.degrees(math.acos(min(1.0, min(ups))))
    assert tilt < 10.0
    assert np.abs(env._rp_off).max() > math.radians(0.3)
