"""SIL pytest suite -- items 2, 3 and 4 of the docs/sil-harness.md pyramid.

    JAX_PLATFORMS=cpu .venv/bin/pytest sim/sil -q

Item 1 (unit) lives in firmware/host ctests; the goldens it consumes are the
JSON files this suite also checks, so a drift shows up on both sides.

Everything that needs libctrl_sil skips cleanly until the library is built.
The rest -- the exported weights, the golden vectors, the whole servo-side
boundary and the inverse action map -- runs today, because those are the
layers the python side owns.
"""
import json
import math
import os
import sys

import numpy as np
import pytest

os.environ.setdefault("JAX_PLATFORMS", "cpu")

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
ROOT = os.path.dirname(SIM)
for p in (HERE, SIM, os.path.join(SIM, "mjx")):
    if p not in sys.path:
        sys.path.insert(0, p)

import harness as H                                          # noqa: E402

RUN = os.environ.get("SIL_RUN", H.DEFAULT_RUN)
# eval_precision.main() uses seed = 100 * i + 7; the SIL arm must be graded on
# exactly those episodes, so the referee's numbers are comparable.
N_SEEDS = int(os.environ.get("SIL_SEEDS", "8"))
SEEDS = [100 * i + 7 for i in range(N_SEEDS)]
QUANTUM = H.RAD_PER_STEP


# =====================================================================
#  fixtures
# =====================================================================
@pytest.fixture(scope="session")
def cfg():
    with open(os.path.join(SIM, "runs", RUN, "config.json")) as f:
        return json.load(f)


@pytest.fixture(scope="session")
def xml(cfg):
    x = cfg.get("xml_path") or "bimo_biped_v3yaw.xml"
    if not os.path.isabs(x) and not os.path.exists(x):
        x = os.path.join(SIM, os.path.basename(x))
    return x


@pytest.fixture(scope="session")
def eval_mod():
    import eval_precision
    return eval_precision


@pytest.fixture(scope="session")
def nominal_env(cfg, xml, eval_mod):
    """Clean plant: no DR, no latency, no backlash, no IMU noise -- so the
    only thing between the sim and the firmware is the boundary itself."""
    return eval_mod.make_env(cfg, 12.0, True, xml)


@pytest.fixture(scope="session")
def hw_env(cfg, xml, eval_mod):
    """Hardware-claim plant (what eval_precision scores policies on)."""
    return eval_mod.make_env(cfg, 12.0, False, xml)


@pytest.fixture(scope="session")
def silw():
    path = H.default_weights(RUN)
    if not os.path.exists(path):
        pytest.skip(f"{path} missing -- run tools/export_policy_weights.py")
    return H.read_silw(path)


@pytest.fixture(scope="session")
def cal_files():
    return H.write_cal_files()


@pytest.fixture
def lib(cal_files):
    """Function-scoped on purpose: sil_init() owns PROCESS-GLOBAL state, so a
    test that loads a different calibration would silently reconfigure every
    later test.  Re-initialising costs a millisecond."""
    if H.find_lib() is None:
        pytest.skip("libctrl_sil not built (make -C firmware/host sil)")
    try:
        return H.open_lib(RUN)
    except H.LibMissing as e:
        pytest.skip(str(e))


@pytest.fixture(scope="session")
def py_act(nominal_env, eval_mod):
    return eval_mod.load_policy(os.path.join(SIM, "runs", RUN),
                                int(nominal_env.observation_space.shape[0]),
                                int(nominal_env.action_space.shape[0]))


# =====================================================================
#  export + goldens  (feeds pyramid item 1 on the C side)
# =====================================================================
def test_obs_spec_parsed():
    assert H.SPEC.num_joints == 10
    assert H.SPEC.act_dim == 10
    assert H.SPEC.obs_dim == H.SPEC.frame_dim * H.SPEC.hist_len
    assert tuple(H.SPEC.servo_id) == (9, 1, 2, 3, 4, 10, 5, 6, 7, 8)


def test_silw_sidecar_agrees_with_blob(silw):
    with open(silw["path"] + ".json") as f:
        side = json.load(f)
    assert side["activation"] == "swish" == silw["activation"]
    assert side["obs_dim"] == silw["obs_dim"] == H.SPEC.obs_dim
    assert side["act_dim"] == silw["act_dim"] == H.SPEC.act_dim
    assert side["layer_sizes"] == silw["layer_sizes"]
    assert silw["layer_sizes"][-1] == 2 * silw["act_dim"]
    np.testing.assert_allclose(side["normalizer"]["mean"], silw["mean"],
                               rtol=0, atol=0)
    np.testing.assert_allclose(side["normalizer"]["std"], silw["std"],
                               rtol=0, atol=0)
    assert np.all(silw["std"] > 0)
    assert np.all(np.isfinite(silw["mean"]))


def test_policy_goldens_match_exported_weights(silw):
    """The exported blob must reproduce brax's deterministic action to the
    atol the firmware test uses.  This is the whole reason the normalizer
    ships with the weights."""
    g = H.load_golden("policy_vectors.json")
    assert g["run"] == RUN and len(g["cases"]) == 16
    worst_raw = worst_norm = 0.0
    for c in g["cases"]:
        a = H.silw_forward(silw, np.array(c["obs"], np.float32))
        worst_raw = max(worst_raw, float(np.max(np.abs(a - c["action"]))))
        # the same pair fed as an already-normalized vector
        z = np.array(c["obs_norm"], np.float32)
        raw = (silw["mean"] + silw["std"] * z).astype(np.float32)
        b = H.silw_forward(silw, raw)
        worst_norm = max(worst_norm, float(np.max(np.abs(b - c["action"]))))
    assert worst_raw < g["atol"], worst_raw
    assert worst_norm < g["atol"], worst_norm


def test_normalizer_frozen_channels_are_known(silw):
    """Channels the run never varied get std == std_eps (1e-6) from brax's
    running_statistics, so a deviation of d from the recorded mean arrives at
    the net as d * 1e6.  Pin the set: if it changes, every consumer that feeds
    one of these a "harmless" different value has to be re-checked.

    Live hazard as of loco_v6creep: slot 45 is cmd[3] (crouch), frozen at
    exactly 1.0 -- and ctrl_task.cpp feeds it battguard::Guard::crouch(),
    which RAMPS BELOW 1.0 on a flat pack.  crouch = 0.85 normalises to
    -1.5e5."""
    frozen = np.where(silw["std"] <= 1e-5)[0]
    fd = H.SPEC.frame_dim
    slots = sorted(set(int(i) % fd for i in frozen))
    off = H.SPEC.off
    expect = ([off["linvel"] + i for i in range(3)] + [off["height"]]
              + [off["cmd"] + i for i in range(3, H.SPEC.num_cmd)])
    assert slots == sorted(expect), (
        f"frozen obs slots changed: {slots} (expected {sorted(expect)})")
    # they repeat identically in every history frame
    assert len(frozen) == len(slots) * H.SPEC.hist_len
    crouch = off["cmd"] + 3
    assert silw["mean"][crouch] == 1.0
    amplified = abs(0.85 - silw["mean"][crouch]) / silw["std"][crouch]
    assert amplified > 1e4, amplified          # the hazard, made explicit


def test_policy_goldens_are_not_degenerate(silw):
    """A net that ignored the normalizer would still 'pass' if every golden
    action were saturated or identical -- check the vectors have spread."""
    g = H.load_golden("policy_vectors.json")
    acts = np.array([c["action"] for c in g["cases"]])
    assert acts.std() > 0.05
    assert np.abs(acts).max() < 1.0


def test_obs_goldens_assemble(silw):
    """Pure-python obs::assembleFrame over the golden Inputs must reproduce
    the walker_env frame exactly -- this is the layout contract."""
    g = H.load_golden("obs_vectors.json")
    off = H.SPEC.off
    nj, ncmd, fd = H.SPEC.num_joints, H.SPEC.num_cmd, H.SPEC.frame_dim
    assert g["frame_dim"] == fd and g["obs_dim"] == H.SPEC.obs_dim
    assert len(g["cases"]) == 8
    for c in g["cases"]:
        i = c["inputs"]
        frame = np.zeros(fd, np.float32)
        frame[off["q"]:off["q"] + nj] = i["q"]
        frame[off["dq"]:off["dq"] + nj] = i["dq"]
        frame[off["up"]:off["up"] + 3] = i["up"]
        frame[off["linvel"]:off["linvel"] + 3] = 0.0        # imu_obs=1
        frame[off["gyro"]:off["gyro"] + 3] = i["gyro"]
        frame[off["prev_action"]:off["prev_action"] + nj] = i["prev_action"]
        frame[off["height"]] = 0.0                          # imu_obs=1
        frame[off["phase"]] = math.sin(i["phase"])
        frame[off["phase"] + 1] = math.cos(i["phase"])
        frame[off["cmd"]:off["cmd"] + ncmd] = i["cmd"]
        ref = np.array(c["frame"], np.float32)
        assert np.max(np.abs(frame - ref)) < 1e-6, c["name"]
        # History::build semantics: obs = [f_t, f_t-1, f_t-2]
        obs = np.concatenate([ref] + [np.array(h, np.float32)
                                      for h in c["hist"]])
        assert np.max(np.abs(obs - np.array(c["obs"], np.float32))) < 1e-6


def test_obs_goldens_are_varied():
    g = H.load_golden("obs_vectors.json")
    ups = np.array([c["inputs"]["up"] for c in g["cases"]])
    qs = np.array([c["inputs"]["q"] for c in g["cases"]])
    assert ups[:, 2].min() < 0.999          # some torso tilt present
    assert qs.std() > 0.1                   # poses actually differ


# =====================================================================
#  pyramid item 2 -- the boundary (no library needed)
# =====================================================================
CALS = {
    "nominal": H.Calibration.nominal(),
    "perturbed": H.Calibration.perturbed(1),
    "perturbed_flipped": H.Calibration.perturbed(2, flip_dirs=True),
}


@pytest.mark.parametrize("cal_name", sorted(CALS))
def test_angle_tick_roundtrip(cal_name):
    """angle -> ticks -> angle, |err| <= one tick quantum, under every cal."""
    cal = CALS[cal_name]
    rng = np.random.default_rng(4)
    lo, hi = H.JOINT_LO, H.JOINT_HI
    for _ in range(400):
        q = lo + (hi - lo) * rng.uniform(0.0, 1.0, size=len(lo))
        ticks = H.angle_to_steps(q, cal)
        assert np.all(ticks >= 0) and np.all(ticks <= H.MAX_STEPS)
        back = H.steps_to_angle(ticks, cal)
        assert np.max(np.abs(back - q)) <= 0.5 * QUANTUM + 1e-12


@pytest.mark.parametrize("cal_name", sorted(CALS))
def test_velocity_tick_roundtrip(cal_name):
    cal = CALS[cal_name]
    rng = np.random.default_rng(5)
    for _ in range(200):
        dq = rng.uniform(-12.0, 12.0, size=H.SPEC.num_joints)
        back = H.steps_s_to_rad_s(H.rad_s_to_steps_s(dq, cal), cal)
        assert np.max(np.abs(back - dq)) <= 0.5 * QUANTUM + 1e-12


def test_calibration_files_written(cal_files):
    for p in cal_files:
        cal = H.Calibration.read(p)
        assert len(cal.zero_steps) == H.SPEC.num_joints
        assert set(np.unique(cal.dir)) <= {-1, 1}
        assert np.all(cal.zero_steps >= 0) and np.all(cal.zero_steps <= 4095)
        assert sorted(cal.bus_id.tolist()) == list(range(1, 11))
    nom = H.Calibration.read(cal_files[0])
    assert np.all(nom.zero_steps == H.CENTER_STEPS) and np.all(nom.dir == 1)
    per = H.Calibration.read(cal_files[1])
    assert not np.all(per.zero_steps == H.CENTER_STEPS)


def test_bus_permutation_is_kservoid():
    perm = H.joint_to_slot_perm()
    assert sorted(perm.tolist()) == list(range(H.SPEC.num_joints))
    assert perm.tolist() == [i - 1 for i in H.SPEC.servo_id]
    assert perm.tolist() != list(range(H.SPEC.num_joints)), \
        "kServoId is NOT an identity -- wiring.md was wrong about this"
    v = np.arange(H.SPEC.num_joints) * 3 + 1
    for order in ("joint", "ascending_id"):
        assert np.array_equal(H.from_bus(H.to_bus(v, order), order), v)
    # element k of an ascending-id array is bus id k+1
    bus = H.to_bus(v, "ascending_id")
    for i, sid in enumerate(H.SPEC.servo_id):
        assert bus[sid - 1] == v[i]


def test_action_target_map_inverts():
    rng = np.random.default_rng(6)
    for _ in range(500):
        a = rng.uniform(-1.0, 1.0, H.SPEC.act_dim)
        back = H.angles_to_action(H.action_to_angles(a))
        assert np.max(np.abs(back - a)) < 1e-9
    # action 0 is the standing pose (the action_map='full' convention)
    np.testing.assert_allclose(H.action_to_angles(np.zeros(H.SPEC.act_dim)),
                               H.JOINT_DEFAULT, atol=0)
    np.testing.assert_allclose(H.action_to_angles(np.ones(H.SPEC.act_dim)),
                               H.JOINT_HI, atol=1e-12)
    np.testing.assert_allclose(H.action_to_angles(-np.ones(H.SPEC.act_dim)),
                               H.JOINT_LO, atol=1e-12)


def test_env_joint_limits_match_obs_spec(nominal_env):
    """obs_spec.h is generated from the MJCF; if the plant moved under it the
    whole boundary is calibrated against the wrong numbers."""
    np.testing.assert_allclose(nominal_env._lo, H.JOINT_LO, atol=1e-6)
    np.testing.assert_allclose(nominal_env._hi, H.JOINT_HI, atol=1e-6)
    np.testing.assert_allclose(nominal_env._default, H.JOINT_DEFAULT,
                               atol=1e-6)
    assert abs(nominal_env.control_dt - H.SPEC.control_dt) < 1e-9


@pytest.mark.parametrize("cal_name", sorted(CALS))
def test_servo_view_roundtrips_against_walker_env(nominal_env, cal_name):
    """PYRAMID ITEM 2: the harness's servo-side view of a real rollout must
    round-trip against walker_env's joint state to within one tick quantum,
    for position AND velocity, under every calibration."""
    cal = CALS[cal_name]
    env = nominal_env
    env.reset(seed=11)
    env.set_command(0.3, 0, 0.2, 1, 0)
    rng = np.random.default_rng(12)
    worst_q = worst_dq = 0.0
    for k in range(60):
        q = np.asarray(env.data.qpos[env._jqpos], dtype=np.float64)
        dq = np.asarray(env.data.qvel[env._jqvel], dtype=np.float64)
        ticks = H.angle_to_steps(q, cal)
        vel = H.rad_s_to_steps_s(dq, cal)
        assert np.all((ticks >= 0) & (ticks <= H.MAX_STEPS))
        assert np.all(np.abs(vel) <= 32767)
        worst_q = max(worst_q, float(np.max(np.abs(
            H.steps_to_angle(ticks, cal) - np.clip(q, H.JOINT_LO,
                                                   H.JOINT_HI)))))
        worst_dq = max(worst_dq, float(np.max(np.abs(
            H.steps_s_to_rad_s(vel, cal) - dq))))
        env.step(rng.uniform(-0.25, 0.25, env.action_space.shape[0]))
    assert worst_q <= 0.5 * QUANTUM + 1e-9, worst_q
    assert worst_dq <= 0.5 * QUANTUM + 1e-9, worst_dq


def test_imu_synth_matches_env_obs_channels(hw_env):
    """PYRAMID ITEM 2 (IMU half): what the harness feeds SilSensors.up/gyro
    must be exactly the channels walker_env put in the obs -- i.e. the IMU
    mounting error, the gyro bias and the noise are all inside the loop, not
    bypassed by reading ground truth."""
    env = hw_env
    env.reset(seed=3)
    env.set_command(0, 0, 0, 1, 0)
    saw_mount_error = saw_bias = False
    for _ in range(25):
        obs = env._obs()
        f = H.split_obs(obs)
        np.testing.assert_array_equal(f["up"], obs[H.SPEC.off["up"]:
                                                   H.SPEC.off["up"] + 3])
        np.testing.assert_array_equal(f["gyro"], obs[H.SPEC.off["gyro"]:
                                                     H.SPEC.off["gyro"] + 3])
        np.testing.assert_array_equal(
            f["q"], obs[H.SPEC.off["q"]:H.SPEC.off["q"] + H.SPEC.num_joints])
        np.testing.assert_array_equal(
            f["cmd"], obs[H.SPEC.off["cmd"]:
                          H.SPEC.off["cmd"] + H.SPEC.num_cmd])
        raw_up = env.data.sensordata[env._up_id:env._up_id + 3]
        raw_gyro = env.data.qvel[3:6]
        saw_mount_error |= bool(np.max(np.abs(f["up"] - raw_up)) > 1e-6)
        saw_bias |= bool(np.max(np.abs(f["gyro"] - raw_gyro)) > 1e-6)
        # linvel + height are zeroed on the IMU-realizable plant
        assert np.all(obs[H.SPEC.off["linvel"]:H.SPEC.off["linvel"] + 3] == 0)
        assert obs[H.SPEC.off["height"]] == 0.0
        env.step(np.zeros(env.action_space.shape[0], np.float32))
    assert saw_mount_error, "IMU mounting error/noise never reached the obs"
    assert saw_bias, "gyro bias/noise never reached the obs"


def test_target_reproduction_through_env(nominal_env):
    """The inverse action map must make env.step() apply EXACTLY the angle
    the firmware commanded (that is what closes the loop)."""
    env = nominal_env
    env.reset(seed=2)
    cal = CALS["perturbed"]
    rng = np.random.default_rng(9)
    for _ in range(20):
        a0 = rng.uniform(-1.0, 1.0, env.action_space.shape[0])
        angles = H.action_to_angles(a0)
        ticks = H.angle_to_steps(angles, cal)        # firmware goal_ticks
        back_angles = H.steps_to_angle(ticks, cal)
        action = H.angles_to_action(back_angles)
        env.step(action.astype(np.float32))
        applied = np.asarray(env._last_target, dtype=np.float64)
        assert np.max(np.abs(applied - back_angles)) < 1e-6
        assert np.max(np.abs(applied - angles)) <= 0.5 * QUANTUM + 1e-9


# =====================================================================
#  library-dependent: probe + per-tick parity
# =====================================================================
def test_lib_spec(lib):
    s = lib.spec_string()
    print("sil_spec():", s)
    assert s, "sil_spec() returned nothing"
    assert str(H.SPEC.obs_dim) in s, \
        f"sil_spec() {s!r} does not report obs_dim {H.SPEC.obs_dim}"


def test_lib_abi_probe(lib):
    info = lib.probe()
    assert info["in_order"] in ("joint", "ascending_id")
    assert info["out_order"] in ("joint", "ascending_id")
    assert info["probe_pos_err_ticks"] == 0
    assert info["probe_goal_err_ticks"] <= 1
    print("SIL ABI:", info)


def test_lib_obs_matches_python(lib, nominal_env, py_act):
    """Per-tick obs/action divergence (docs/sil-harness.md item 3): with the
    gait clocks aligned, the only difference between the firmware's assembled
    obs and walker_env's must be tick quantization."""
    env = nominal_env
    env.reset(seed=21)
    H.pin_gait_clock(env, lib.gait_hz)
    env.set_command(0.3, 0, 0, 1, 0)
    ad = H.SilActAdapter(lib, env, py_act=py_act)
    for _ in range(120):
        obs = env._obs()
        a = ad(obs)
        env.step(a)
    d = ad.divergence()
    print("divergence:", json.dumps(d, indent=1, default=float))
    blk = d["obs_block_err_max"]
    # q / dq go through the tick quantiser
    assert blk["q"] <= 0.5 * QUANTUM + 1e-6, blk
    assert blk["dq"] <= 0.5 * QUANTUM + 1e-6, blk
    # these are copied verbatim -- any error at all is a layout/units bug
    for key in ("up", "gyro", "cmd", "linvel", "height"):
        assert blk[key] < 1e-6, (key, blk)
    # the gait clocks are aligned by pin_gait_clock(); float32 sin/cos only
    assert blk["phase"] < 1e-5, blk
    # prev_action differs by the quantisation expressed in action units
    span = np.minimum(H.JOINT_HI - H.JOINT_DEFAULT,
                      H.JOINT_DEFAULT - H.JOINT_LO)
    act_quantum = 0.5 * QUANTUM / span.min()
    assert blk["prev_action"] <= act_quantum + 1e-6, blk
    # obs::History holds the frames that were really produced
    assert d["hist_err_max"] <= act_quantum + 1e-6, d
    # same obs in -> same action out: firmware net vs brax, the item-1 atol
    assert d["net_err_max"] < 1e-4, d
    # end-to-end against the TRAINING obs stacking: quantisation only
    assert d["train_action_err_max"] < 0.02, d


def test_referee_driver_feeds_training_stacking(nominal_env):
    """FIXED 2026-07-30 (this harness's finding): eval_precision.Driver used
    to re-read env._obs() at the top of each tick, prepending a duplicate of
    f_t onto a ring that already held it -- the policy got [f_t, f_t, f_t-1]
    while training feeds [f_t, f_t-1, f_t-2]. The Driver now reuses the obs
    env.step() returned (training-correct stacking) and patches only the
    command channels of the head frame. This test pins the FIX: the obs the
    Driver actually feeds must equal env.step's return except in the head
    frame's cmd slice. (env._obs() itself still duplicates when called
    post-step -- that is documented env behavior, no longer on the referee
    path.)"""
    import importlib
    ep = importlib.import_module("eval_precision")
    env = nominal_env
    fd = H.SPEC.frame_dim
    obs0, _ = env.reset(seed=31)

    fed = {}

    def act(obs):
        fed["obs"] = np.asarray(obs, dtype=np.float64).copy()
        return np.zeros(env.action_space.shape[0], np.float32)

    drv = ep.Driver(env, act, 31, record=False)
    stepret = np.asarray(drv.obs, dtype=np.float64).copy()
    drv.step((0.1, 0, 0, 1, 0))
    nc = env._ncmd
    # outside the head frame's cmd slice, fed == env.step's return exactly
    mask = np.ones(len(stepret), bool)
    mask[fd - nc:fd] = False
    assert np.array_equal(fed["obs"][mask], stepret[mask]), \
        "Driver no longer feeds the training stacking"
    # and the cmd slice carries the newly set command
    assert abs(fed["obs"][fd - nc] - 0.1) < 1e-6


def test_lib_dropout_holds_target(lib, hw_env):
    """PYRAMID ITEM 4: three consecutive skipped ticks (simulated overrun)
    must hold the last commanded target and must not drop the robot."""
    env = hw_env
    skip = (200, 201, 202)
    ad = H.SilActAdapter(lib, env, skip_ticks=skip)
    env.reset(seed=5)
    env.set_command(0, 0, 0, 1, 0)
    fell = False
    x0 = y0 = None
    for k in range(500):
        obs = env._obs()
        a = ad(obs)
        _, _, term, _, info = env.step(a)
        if x0 is None:
            x0, y0 = float(env.data.qpos[0]), float(env.data.qpos[1])
        if term:
            fell = True
            break
    assert not fell, "the 3-tick dropout dropped the robot"
    drift = math.hypot(float(env.data.qpos[0]) - x0,
                       float(env.data.qpos[1]) - y0)
    assert drift < 0.20, f"drift {drift:.3f} m after dropout"
    held = [r for r in ad.rows if r["tick"] in skip]
    assert len(held) == 3 and all(r["skipped"] for r in held)
    ref = [r for r in ad.rows if r["tick"] == skip[0] - 1][0]["action"]
    for r in held:
        np.testing.assert_array_equal(r["action"], ref)


def test_lib_survives_perturbed_calibration(cal_files, nominal_env):
    """The calibration must be a pure change of coordinates: with the SAME
    perturbed zeros on both sides of the boundary the robot still stands.
    A cal handled inconsistently shows up here as a face-plant."""
    if H.find_lib() is None:
        pytest.skip("libctrl_sil not built")
    lib = H.open_lib(RUN, cal="perturbed")
    assert not np.all(lib.cal.zero_steps == H.CENTER_STEPS)
    env = nominal_env
    env.reset(seed=41)
    H.pin_gait_clock(env, lib.gait_hz)
    env.set_command(0, 0, 0, 1, 0)
    ad = H.SilActAdapter(lib, env)
    for _ in range(250):
        obs = env._obs()
        _, _, term, _, _ = env.step(ad(obs))
        assert not term, "perturbed calibration dropped the robot"
    d = ad.divergence()
    assert d["obs_block_err_max"]["q"] <= 0.5 * QUANTUM + 1e-6, d


# =====================================================================
#  library-dependent: closed loop (pyramid item 3)
# =====================================================================
CLOSED_LOOP = ["stand_10s", "line_1m", "goal_home"]


@pytest.mark.slow
@pytest.mark.parametrize("scenario", CLOSED_LOOP)
def test_closed_loop_matches_python_referee(scenario, lib, cfg, xml, eval_mod,
                                            py_act):
    """PYRAMID ITEM 3: the SIL stack scored by the same referee, same seeds,
    same plant.  Pass rates must agree within one seed."""
    reg = eval_mod._registry()
    secs, factory, _ = reg[scenario]
    env = eval_mod.make_env(cfg, secs, False, xml,
                            extra=eval_mod.ENV_EXTRA.get(scenario))
    mass = float(np.sum(env.model.body_mass))
    nominal_h = env._nominal_h

    sil = H.SilActAdapter(lib, env, log=False)
    py_pass, sil_pass = 0, 0
    for seed in SEEDS:
        r = eval_mod.run_one(env, py_act, factory, seed, False, nominal_h, mass)
        py_pass += bool(r["success"])
        r = eval_mod.run_one(env, sil, factory, seed, False, nominal_h, mass)
        sil_pass += bool(r["success"])
    print(f"{scenario}: python {py_pass}/{len(SEEDS)}  "
          f"SIL {sil_pass}/{len(SEEDS)}")
    assert abs(py_pass - sil_pass) <= 1, (
        f"{scenario}: python {py_pass}/{len(SEEDS)} vs SIL "
        f"{sil_pass}/{len(SEEDS)} -- more than one seed apart")
