"""distill_student.py on the robot's plant: the teacher's config.json rebuilds
the legs env (12 joints with the ankle rolls), and the physics override that
lets a Warp teacher distill on the CPU leaves the plant untouched.

The run itself (DAgger rounds, export) is exercised end-to-end by
infra/night_distill.sh; this pins the config plumbing it depends on."""
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "sim", "mjx"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")

ds = pytest.importorskip("distill_student")

# the keys train_mjx.py --robot --xml bimo_biped_v6legs.xml --mjx-impl warp
# writes that matter here (robot_legs_walk9's config.json, trimmed)
LEGS_CFG = {
    "xml_path": "bimo_biped_v6legs.xml", "action_map": "full",
    "imu_obs": True, "ext_cmd": True, "gait_clock": True, "obs_hist_len": 3,
    "held_joints": {}, "floor_contacts": "feet", "domain_rand": True,
    "mjx_impl": "warp", "warp_envs": 2048,
    "cmd_v_range": [0.05, 0.7], "recover_mix": 0.0,
    "train": {"out": "t"}, "git_sha": "x", "git_dirty": False,
    "distill": {"teacher": "older"}, "distill_physics": "jax",
}


def test_env_kwargs_keeps_warp_and_resizes_its_pool():
    kw = ds.env_kwargs(LEGS_CFG, None, 512)
    assert kw["mjx_impl"] == "warp" and kw["warp_envs"] == 512
    assert kw["xml_path"].endswith(os.path.join("sim", "bimo_biped_v6legs.xml"))
    assert kw["cmd_v_range"] == (0.05, 0.7)
    for k in ("train", "git_sha", "git_dirty", "distill", "distill_physics"):
        assert k not in kw


def test_env_kwargs_jax_override_drops_warp_keys():
    kw = ds.env_kwargs(LEGS_CFG, "jax", 512)
    assert "mjx_impl" not in kw and "warp_envs" not in kw


def test_legs_env_builds_from_teacher_config_on_cpu():
    from env_mjx import BimoMJXEnv
    env = BimoMJXEnv(**ds.env_kwargs(LEGS_CFG, "jax", 64))
    # 12 policy joints (6 per leg with the ankle roll): frame 55, history 3
    assert env.action_size == 12
    assert env.obs_size == 165
    assert env.action_map == "full"
