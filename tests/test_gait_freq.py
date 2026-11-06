"""--gait-freq lo,hi: a per-policy gait clock (precision of movement at reduced speed).

Training draws the clock per episode from the run's gait_freq_range; config.json records it,
the referee and grader read it from there, and gen_obs_spec.py emits its midpoint as
obs::kGaitFreqHz. Off by default: without the flag nothing changes, draw for draw.
"""
import json
import math
import os
import pickle
import re
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
for _p in (("sim", "mjx"), ("sim",), ("tools",), ("experiments", "policy", "shaper-train")):
    sys.path.insert(0, os.path.join(ROOT, *_p))
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

XML = "bimo_biped_v6legs_meas_sole.xml"
ARGS = ["--robot", "--xml", XML, "--servo-kp-scale", "asbuilt",
        "--precision", "--mjx-impl", "jax", "--family", "loco"]
SLOW = (0.9, 1.1)
WALK = (0.3, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)


def _kw(*extra):
    import train_mjx
    return train_mjx.build_env_kw(train_mjx.make_parser().parse_args(ARGS + list(extra)))


@pytest.fixture(scope="module")
def mjx_kw():
    return _kw()


@pytest.fixture(scope="module")
def mjx_env(mjx_kw):
    from env_mjx import BimoMJXEnv
    return BimoMJXEnv(**mjx_kw)


@pytest.fixture(scope="module")
def mjx_slow(mjx_kw):
    from env_mjx import BimoMJXEnv
    return BimoMJXEnv(**dict(mjx_kw, gait_freq_range=SLOW))


def _cpu_env(**over):
    from walker_env import BimoWalkerEnv
    kw = dict(xml_path=os.path.join(ROOT, "sim", XML), actuator_model="sts3215", command_mode=True,
              ext_cmd=True, imu_obs=True, gait_clock=True, domain_rand=False, obs_hist_len=3,
              render_mode=None)
    kw.update(over)
    return BimoWalkerEnv(**kw)


# -- the flag --------------------------------------------------------------------------

def test_flag_off_leaves_env_kw_alone(mjx_kw):
    assert "gait_freq_range" not in mjx_kw


def test_flag_parses_into_env_kw():
    assert _kw("--gait-freq", "0.9,1.1")["gait_freq_range"] == SLOW
    assert _kw("--gait-freq", "1.0,1.0")["gait_freq_range"] == (1.0, 1.0)     # a fixed clock


@pytest.mark.parametrize("bad", ["1.0", "1.1,0.9", "0.3,0.4", "a,b", "1,2,3"])
def test_flag_refuses_nonsense(bad):
    with pytest.raises(SystemExit):
        _kw("--gait-freq", bad)


def test_flag_needs_the_gait_clock():
    import train_mjx
    a = train_mjx.make_parser().parse_args(["--gait-freq", "0.9,1.1"])     # no --precision
    with pytest.raises(SystemExit):
        train_mjx.build_env_kw(a)


def test_config_json_records_it(monkeypatch, tmp_path):
    """train_mjx.main writes the range where the referee and exporter read it: not at all
    without the flag."""
    import train_mjx

    class Started(Exception):
        pass

    def no_training(*a, **k):
        raise Started

    monkeypatch.setattr(train_mjx, "RUNS", str(tmp_path))
    monkeypatch.setattr(train_mjx.ppo, "train", no_training)
    cfg = {}
    for name, extra in (("off", []), ("on", ["--gait-freq", "0.9,1.1"])):
        monkeypatch.setattr(sys, "argv", ["train_mjx", "--out", name, "--envs", "4"] + ARGS + extra)
        with pytest.raises(Started):
            train_mjx.main()
        cfg[name] = json.load(open(tmp_path / name / "config.json"))
    assert "gait_freq_range" not in cfg["off"]
    assert cfg["on"]["gait_freq_range"] == list(SLOW)
    assert {k: v for k, v in cfg["off"].items() if k not in ("train", "git_sha", "git_dirty")} == \
           {k: v for k, v in cfg["on"].items()
            if k not in ("train", "git_sha", "git_dirty", "gait_freq_range")}


# -- the training env ------------------------------------------------------------------

def test_defaults_agree_and_are_the_historical_draw():
    import inspect
    from env_mjx import BimoMJXEnv
    from walker_env import BimoWalkerEnv, GAIT_FREQ_RANGE, deploy_gait_hz
    assert GAIT_FREQ_RANGE == (1.25, 1.75)
    assert inspect.signature(BimoMJXEnv.__init__).parameters["gait_freq_range"].default == GAIT_FREQ_RANGE
    assert inspect.signature(BimoWalkerEnv.__init__).parameters["gait_freq_range"].default == GAIT_FREQ_RANGE
    assert deploy_gait_hz({}) == 1.5 and deploy_gait_hz(None) == 1.5


def test_env_refuses_a_bad_range(mjx_kw):
    from env_mjx import BimoMJXEnv
    with pytest.raises(ValueError):
        BimoMJXEnv(**dict(mjx_kw, gait_freq_range=(1.1, 0.9)))
    with pytest.raises(ValueError):
        _cpu_env(gait_freq_range=(0.2, 0.4))


def test_mjx_reset_paths_draw_inside_the_range(mjx_slow):
    import jax
    first = mjx_slow.reset(jax.random.PRNGKey(0))
    keys = jax.random.split(jax.random.PRNGKey(1), 64)
    f_reset = np.array([float(jax.jit(mjx_slow.reset)(k).gait_freq) for k in keys[:8]])
    f_reseed = np.array(jax.jit(jax.vmap(lambda k: mjx_slow.reseed(first, k).gait_freq))(keys))
    for f in (f_reset, f_reseed):
        assert f.min() >= SLOW[0] and f.max() <= SLOW[1]
    assert np.ptp(f_reseed) > 0.1                  # a draw, not a constant


def test_mjx_default_off_same_draws_and_obs(mjx_env, mjx_slow):
    """Both reset paths: the range only rescales the one uniform -- same stream, same phase, same
    frame but for nothing (the clock's frequency is not observed)."""
    import jax
    first = mjx_env.reset(jax.random.PRNGKey(0))
    first_s = mjx_slow.reset(jax.random.PRNGKey(0))
    for k in (3, 7):
        key = jax.random.PRNGKey(k)
        for a, b in ((mjx_env.reset(key), mjx_slow.reset(key)),
                     (mjx_env.reseed(first, key), mjx_slow.reseed(first_s, key))):
            u = (float(a.gait_freq) - 1.25) / 0.5
            assert float(b.gait_freq) == pytest.approx(SLOW[0] + u * (SLOW[1] - SLOW[0]), abs=1e-5)
            assert float(a.gait_phase) == float(b.gait_phase)
            np.testing.assert_array_equal(np.asarray(a.obs), np.asarray(b.obs))
            np.testing.assert_array_equal(np.asarray(a.rng), np.asarray(b.rng))


def test_mjx_phase_advances_at_the_drawn_rate(mjx_slow):
    import jax
    import jax.numpy as jp
    s = mjx_slow.reset(jax.random.PRNGKey(5))
    s = s._replace(cmd=jp.asarray(WALK, dtype=s.cmd.dtype))
    f = float(s.gait_freq)
    assert SLOW[0] <= f <= SLOW[1]
    a = jp.zeros(mjx_slow.action_size)
    p0 = float(s.gait_phase)
    for _ in range(5):
        s = mjx_slow.step(s, a)
    want = (p0 + 5 * 2 * math.pi * mjx_slow.control_dt * f + math.pi) % (2 * math.pi) - math.pi
    assert float(s.gait_phase) == pytest.approx(want, abs=1e-4)
    assert float(s.gait_freq) == f


def test_cpu_twin_default_stream_and_range():
    from walker_env import deploy_gait_hz
    a, b = _cpu_env(), _cpu_env(gait_freq_range=[0.9, 1.1])     # as config.json carries it: a list
    assert a.deploy_gait_freq == 1.5 and b.deploy_gait_freq == pytest.approx(1.0)
    assert deploy_gait_hz(dict(gait_freq_range=[0.9, 1.1])) == pytest.approx(1.0)
    for seed in range(8):
        oa, _ = a.reset(seed=seed)
        ob, _ = b.reset(seed=seed)
        assert 1.25 <= a._gait_freq <= 1.75 and SLOW[0] <= b._gait_freq <= SLOW[1]
        assert b._gait_freq == pytest.approx(0.9 + (a._gait_freq - 1.25) / 0.5 * 0.2, abs=1e-12)
        assert a._gait_phase == b._gait_phase
        np.testing.assert_array_equal(oa, ob)


def test_cpu_twin_phase_advances_at_the_drawn_rate():
    e = _cpu_env(gait_freq_range=SLOW)
    e.reset(seed=2)
    e.set_command(*WALK)
    f, p0 = e._gait_freq, e._gait_phase
    a = np.zeros(e.action_space.shape[0], dtype=np.float32)
    for _ in range(5):
        e.step(a)
    want = (p0 + 5 * 2 * math.pi * e.control_dt * f + math.pi) % (2 * math.pi) - math.pi
    assert e._gait_phase == pytest.approx(want, abs=1e-9)


def test_cpu_mimic_reference_follows_the_scaled_clock():
    """env_mjx passes the speed-scaled clock to _mimic_ref: so must the twin."""
    e = _cpu_env(gait_freq_range=SLOW, speed_clock=True, w_mimic=1.0)
    e.reset(seed=1)
    e.set_command(0.7, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    seen = []
    real = e._mimic_ref

    def spy(cmd, phase, freq):
        seen.append(float(freq))
        return real(cmd, phase, freq)
    e._mimic_ref = spy
    e.step(np.zeros(e.action_space.shape[0], dtype=np.float32))
    scale = float(np.clip(np.sqrt(0.7 / 0.35), e.speed_clock_lo, e.speed_clock_hi))
    assert seen[-1] == pytest.approx(e._gait_freq * scale)


def test_mimic_stride_follows_the_clock(mjx_slow):
    """Same phase, same speed: the slower clock's reference is the longer stride (v/f)."""
    import jax.numpy as jp
    cmd = jp.asarray(WALK, dtype=jp.float32)
    d = np.asarray(mjx_slow._default)
    i = np.asarray(mjx_slow._i_pitch)
    q15 = np.asarray(mjx_slow._mimic_ref(cmd, 0.0, 1.5))[i] - d[i]
    q10 = np.asarray(mjx_slow._mimic_ref(cmd, 0.0, 1.0))[i] - d[i]
    np.testing.assert_allclose(q10, 1.5 * q15, atol=1e-5)


# -- SIL harness -----------------------------------------------------------------------

def test_harness_clock_follows_the_env():
    sys.path.insert(0, os.path.join(ROOT, "sim", "sil"))
    import harness as H
    e = _cpu_env(gait_freq_range=SLOW)
    e.reset(seed=0)
    e.set_command(*WALK)
    w = H.pin_gait_clock(e)
    assert e._gait_freq == pytest.approx(1.0)
    assert w == pytest.approx(2 * math.pi * e.control_dt * 1.0)
    d = _cpu_env()
    d.reset(seed=0)
    d.set_command(*WALK)
    assert H.pin_gait_clock(d) == pytest.approx(2 * math.pi * d.control_dt * 1.5)
    assert H.pin_gait_clock(d, 1.25) == pytest.approx(2 * math.pi * d.control_dt * 1.25)    # explicit wins


# -- the grader ------------------------------------------------------------------------

def _grade():
    import grade
    return grade


def test_advance_clock_runs_at_the_policy_clock():
    G = _grade()
    S = G.S
    dt = 0.02
    for hz in (1.0, S.CLOCK_HZ):
        clk = dict(ph=0.0, f=hz, hz=hz, held=False)
        assert S.advance_clock(clk, "coast", dict(stand_phase=math.pi / 2), G.walk(0.3), dt) == \
            pytest.approx(2 * math.pi * hz * dt)
        clk = dict(ph=0.0, f=hz, hz=hz, held=False)
        assert S.advance_clock(clk, "run", {}, G.STAND_CMD, dt) == pytest.approx(2 * math.pi * hz * dt)
    # no "hz": the historical 1.5 Hz
    assert S.advance_clock(dict(ph=0.0, f=1.5, held=False), "run", {}, G.STAND_CMD, dt) == \
        pytest.approx(2 * math.pi * 1.5 * dt)
    # coast reaches the stand phase a cycle late at the slower clock's rate, then holds
    clk = dict(ph=0.0, f=1.0, hz=1.0, held=False)
    n = 0
    while not clk["held"] and n < 200:
        S.advance_clock(clk, "coast", dict(stand_phase=math.pi / 2), G.STAND_CMD, dt)
        n += 1
    assert n == pytest.approx(0.25 / dt, abs=1)       # a quarter cycle at 1 Hz


def test_trained_hz_reads_the_env():
    S = _grade().S
    assert S.trained_hz(_cpu_env()) == 1.5
    assert S.trained_hz(_cpu_env(gait_freq_range=SLOW)) == pytest.approx(1.0)


def test_cycle_probes_follow_the_clock():
    G = _grade()
    end = lambda name, hz: G.at_clock(G.PROBES[name], hz)["cmd_schedule"][-1][0]
    assert end("stride1 0.3 refit", 1.0) == pytest.approx(2.0 + 1.0)
    assert end("steps5 0.3 refit", 1.0) == pytest.approx(2.0 + 2.5)
    assert end("back3 -0.3 robot", 1.0) == pytest.approx(2.0 + 1.5)
    assert end("stride3 0.3 refit", 1.0) == pytest.approx(2.0 + 3.0)
    assert end("stride1 0.3 refit dqlag late40", 1.0) == pytest.approx(2.0 + 1.0 + 0.04)
    assert end("stride1 0.3 refit dqlag early40", 1.0) == pytest.approx(2.0 + 1.0 - 0.04)
    # a fixed-time probe stays put
    assert end("walk 0.3 -> stop", 1.0) == 4.0
    # half-cycle kick times
    yk = G.at_clock(G.PROBES["steps5 0.3 refit dqlag yk2"], 1.0)["yaw_kicks"]
    assert sorted(yk) == [int(round((2.0 + 0.06 + k * 0.5) / G.DT)) for k in range(5)]
    assert set(yk.values()) == {-2.0}


def test_probes_are_unchanged_at_the_default_clock():
    """Every probe's schedule at 1.5 Hz: as the module-level PROBES list always had it."""
    G = _grade()
    assert G.PROBES["stride1 0.3 refit"]["cmd_schedule"][-1][0] == 2.0 + 1.0 / 1.5
    assert G.PROBES["steps5 0.3 refit"]["cmd_schedule"][-1][0] == 2.0 + 2.5 / 1.5
    assert G.PROBES["back3 -0.3 robot"]["cmd_schedule"][-1][0] == 2.0 + 1.5 / 1.5
    assert G.PROBES["stride1 0.3 refit dqlag late40"]["cmd_schedule"][-1][0] == 2.0 + (1.0 / 1.5 + 40 / 1e3)
    assert G.PROBES["stride1 0.3 robot early40"]["cmd_schedule"][-1][0] == 2.0 + (1.0 / 1.5 - 0.04)
    for name, p in G.PROBES.items():
        q = G.at_clock(p, G.S.CLOCK_HZ)
        assert list(q.get("cmd_schedule", [])) == list(p.get("cmd_schedule", [])), name
        assert dict(q.get("yaw_kicks", {})) == dict(p.get("yaw_kicks", {})), name


def test_schedules_survive_the_worker_pool():
    G = _grade()
    for sched in (G.stride(0.3, cycles=2.5, extra_s=0.02), G.yaw_kicks(-1.0, n=5, lead=0.06)):
        back = pickle.loads(pickle.dumps(sched))
        assert back == sched and back.at(1.0) == sched.at(1.0)


# -- the referee -----------------------------------------------------------------------

def test_referee_inherits_the_range_from_config():
    import eval_precision as E
    cfg = json.loads(json.dumps(dict(_kw("--gait-freq", "0.9,1.1"), mjx_impl=None,
                                     xml_path=None, warp_envs=None), default=list))
    env = E.make_env(cfg, 5.0, True, os.path.join(ROOT, "sim", XML))
    assert env.gait_freq_range == SLOW and env.deploy_gait_freq == pytest.approx(1.0)


def test_walk_steps_lasts_the_same_cycles():
    import eval_precision as E
    assert E._registry()["walk_steps_L"][0] == pytest.approx(E.WALK_STEPS_T0 + E.WALK_STEPS_S + 3.2)
    assert E._registry(1.5)["walk_steps_L"][0] == E._registry()["walk_steps_L"][0]
    assert E._registry(1.0)["walk_steps_L"][0] == pytest.approx(E.WALK_STEPS_T0 + 1.5 * E.WALK_STEPS_S + 3.2)
    # 2 steps/s from t = 1 s, 7 swings: the window lasts as many cycles at 1.0 Hz (3.7 + 0.6 s) and
    # counts all of them; the 1.5 Hz window (2.8 + 0.4 s) stops after 5
    dt = 0.02
    h = []
    for k in range(int(6.0 / dt)):
        t = k * dt
        swing = "LR"[int((t - 1.0) / 0.5) % 2] if 1.0 <= t < 4.2 and (t - 1.0) % 0.5 < 0.3 else None
        z = 0.05 if swing else 0.0
        h.append((t, swing != "L", swing != "R", z if swing == "L" else 0.0, z if swing == "R" else 0.0))
    rows = [dict(t=0.0, x=0.0, y=0.0, height=1.0, up_z=1.0), dict(t=6.0, x=0.5, y=0.0, height=1.0, up_z=1.0)]
    for hz, steps in ((1.0, 7), (1.5, 5)):
        build, evaluate = E.scen_walk_steps("L", gait_hz=hz)
        _, ev = build()
        ev["_h"] = h
        ev["x0"], ev["y0"] = 0.0, 0.0
        r = evaluate(rows, ev, False, 1.0, {})
        assert r["metrics"]["steps"] == steps and r["success"]


# -- the firmware's constant -----------------------------------------------------------

def _spec_for(tmp_path, name, **cfg_over):
    import gen_obs_spec
    import robot_plant
    kw = robot_plant.robot_env_kwargs()
    cfg = dict(xml_path=kw["xml_path"], held_joints=kw["held_joints"],
               servo_kp_scale=kw["servo_kp_scale"], hip_flex_deg=kw["hip_flex_deg"],
               payload_mass=0.0, mimic_sole_level=True, obs_hist_len=3, action_map="full",
               imu_obs=True, gait_clock=True, **cfg_over)
    run = tmp_path / name
    run.mkdir()
    (run / "config.json").write_text(json.dumps(cfg))
    cfg2, env = gen_obs_spec.build_env(str(run))
    cfg2["_run"] = str(run)
    out = tmp_path / (name + ".h")
    gen_obs_spec.write_spec(cfg2, env, str(out))
    return out


def _freq_line(path):
    m = re.search(r"inline constexpr float kGaitFreqHz = ([0-9.]+)f;", open(path).read())
    return float(m.group(1)) if m else None


def test_gen_obs_spec_emits_the_midpoint(tmp_path):
    sys.path.insert(0, os.path.join(ROOT, "sim", "sil"))
    import harness as H
    absent = _spec_for(tmp_path, "absent")
    assert _freq_line(absent) == 1.5                          # a run with no setting: as today
    slow = _spec_for(tmp_path, "slow", gait_freq_range=[0.9, 1.1])
    assert _freq_line(slow) == pytest.approx(1.0)
    off_center = _spec_for(tmp_path, "off", gait_freq_range=[0.8, 1.0])
    assert _freq_line(off_center) == pytest.approx(0.9)
    assert H._parse_obs_spec(str(slow)).gait_freq_hz == pytest.approx(1.0)
    assert H._parse_obs_spec(str(absent)).gait_freq_hz == 1.5


def test_committed_specs_keep_the_clock():
    """The specs on main regenerate to the same behaviour: 1.5 Hz."""
    specs = [os.path.join(ROOT, "firmware", "components", "obs", "include", "obs", "obs_spec.h")]
    pol = os.path.join(ROOT, "firmware", "policies")
    specs += [os.path.join(pol, d, "obs", "obs_spec.h") for d in sorted(os.listdir(pol))]
    assert len(specs) >= 3
    for s in specs:
        assert _freq_line(s) == 1.5, s


@pytest.mark.parametrize("src", ["firmware/main/ctrl_task.cpp", "firmware/host/sil_lib.cpp"])
def test_firmware_constructs_the_clock_from_the_spec(src):
    txt = open(os.path.join(ROOT, src)).read()
    assert "g_clock(obs::kGaitFreqHz)" in txt
    assert "g_clock(1.5f)" not in txt
