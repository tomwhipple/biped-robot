"""Push-stress grade (get-up campaign, 2026-10-27).

The referee pins pushes to the hardware claim, 5 N torso shoves held one control
step at 1 % per tick, whatever the run trained with. The get-up policies pass
that at 47/48 - 48/48, yet stand only 8-12/16 at 15 N @ 2 %, so the campaign
grades push robustness on its own card: eval_precision.py --push-force/--push-prob
replace the pinned pushes, and train_mjx.py --push-force trains with them.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "sim")
sys.path.insert(0, os.path.join(SIM, "mjx"))
sys.path.insert(0, SIM)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")


def _env(cfg_extra, nominal=False):
    import eval_precision as ep
    import robot_plant
    kw = robot_plant.robot_env_kwargs()
    cfg = dict(kw, precision=True, action_map="full", push_force=40.0, push_prob=0.5,
               **cfg_extra)
    return ep.make_env(cfg, 10.0, nominal, kw["xml_path"])


def test_referee_pins_the_hardware_claim_pushes_whatever_the_run_trained_with():
    env = _env({})
    assert (env.push_force, env.push_prob) == (5.0, 0.01)


def test_push_stress_replaces_the_pinned_pushes():
    env = _env({"_referee_push_force": 15.0, "_referee_push_prob": 0.02})
    assert (env.push_force, env.push_prob) == (15.0, 0.02)


def test_nominal_grade_has_no_pushes_even_under_stress_flags():
    env = _env({"_referee_push_force": 15.0, "_referee_push_prob": 0.02}, nominal=True)
    assert (env.push_force, env.push_prob) == (0.0, 0.0)


def test_trainer_passes_push_force_to_the_env():
    import train_mjx as T
    base = ["--robot", "--robot-arms", "--precision", "--family", "getup"]
    assert "push_force" not in T.build_env_kw(T.make_parser().parse_args(base))
    kw = T.build_env_kw(T.make_parser().parse_args(
        base + ["--push-prob", "0.02", "--push-force", "15"]))
    assert (kw["push_force"], kw["push_prob"]) == (15.0, 0.02)
