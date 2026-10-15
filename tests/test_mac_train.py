"""sim/mac_train.py, the Mac trainer (CPU physics in worker processes, the
networks on MPS or CPU): a run it writes must be one the CPU referee and the
firmware exporter read unchanged, and the CPU env's training-only additions
must leave the referee's env as it was."""
import json
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "sim")
sys.path[:0] = [SIM, os.path.join(SIM, "mjx")]

ROBOT = ["--robot", "--precision", "--family", "loco", "--walk-submix", "0,0",
         "--cmd-v-range", "0.05,0.35", "--w-mimic", "4.0", "--servo-kp-scale", "robot",
         "--act-lag", "2,12"]


@pytest.fixture(scope="module")
def mac_run(tmp_path_factory):
    import mac_train as MT
    runs = tmp_path_factory.mktemp("runs")
    MT.RUNS = str(runs)
    MT.main(ROBOT + ["--out", "smoke", "--workers", "2", "--envs", "4", "--batch-size", "8",
                     "--minibatches", "2", "--unroll", "4", "--updates", "1", "--steps", "64",
                     "--num-evals", "1", "--device", "cpu"])
    return os.path.join(str(runs), "smoke")


def test_run_layout(mac_run):
    for f in ("config.json", "params.pkl", "progress.jsonl", "mac_state.pt"):
        assert os.path.exists(os.path.join(mac_run, f)), f
    cfg = json.load(open(os.path.join(mac_run, "config.json")))
    assert cfg["trainer"] == "mac_ppo" and cfg["xml_path"] == "bimo_biped_v6ar.xml"
    assert cfg["w_mimic"] == 4.0 and cfg["act_lag_hz_max"] == 12.0
    # the stiffness the env resolved, per actuator (train_mjx's rule)
    assert cfg["servo_kp_scale"]["L_hip_roll"] == 4.0 and cfg["servo_kp_scale"]["L_knee"] == 2.8
    rec = json.loads(open(os.path.join(mac_run, "progress.jsonl")).readline())
    assert rec["steps"] > 0 and "training/sps" in rec


def test_referee_loads_and_runs_it(mac_run):
    import eval_ref as ER
    cfg = json.load(open(os.path.join(mac_run, "config.json")))
    env = ER.make_env(cfg)
    act, _ = ER.load_policy(mac_run, env.observation_space.shape[0], env.action_space.shape[0])
    obs, _ = env.reset(seed=0)
    for _ in range(5):
        a = act(obs.astype(np.float32))
        assert a.shape == (12,) and np.all(np.isfinite(a)) and np.all(np.abs(a) <= 1.0)
        obs, *_ = env.step(a)


def test_cpu_env_kwargs_map_the_training_extras():
    import mac_train as MT
    import train_mjx as TM
    kw = TM.build_env_kw(TM.make_parser().parse_args(ROBOT))
    ckw = MT.cpu_env_kwargs(kw)
    assert ckw["act_lag_dr_max"] == 12.0 and "act_lag_hz_max" not in ckw
    assert ckw["foot_cross_term"] and ckw["command_mode"] and ckw["actuator_model"] == "sts3215"


def test_referee_env_keeps_one_pinned_lag_pole(mac_run):
    """The CPU env's per-episode lag DR is named apart from the config key, so
    a referee rebuilding from a run's config.json never randomizes the pole."""
    import eval_ref as ER
    cfg = json.load(open(os.path.join(mac_run, "config.json")))
    env = ER.make_env(cfg, act_lag_hz=2.0)
    assert env.act_lag_dr_max is None and not env.foot_cross_term
    for s in range(3):
        env.reset(seed=s)
        assert env._act_lag_ep == 2.0


def test_act_lag_dr_draws_per_episode():
    import robot_plant
    from walker_env import BimoWalkerEnv
    kw = robot_plant.robot_env_kwargs()
    env = BimoWalkerEnv(actuator_model="sts3215", command_mode=True, act_lag_hz=2.0,
                        act_lag_dr_max=12.0, **kw)
    poles = []
    for s in range(6):
        env.reset(seed=s)
        poles.append(env._act_lag_ep)
    assert all(2.0 <= p <= 12.0 for p in poles) and len(set(poles)) == 6
