#!/usr/bin/env python3
"""Make an UNTRAINED, deterministic policy run for a plant that has no
deployable trained net yet, so the whole on-ESP32 path can be built and
golden-tested end to end before one exists.

    CUDA_VISIBLE_DEVICES= JAX_PLATFORMS=cpu .venv/bin/python \
        tools/make_test_policy_run.py --out v6legs_fullmap_testnet

writes sim/runs/<out>/ (gitignored, like every run):

  config.json   the env kwargs a REAL run of the recipe would record:
                sim/mjx/train_mjx.py's own build_env_kw() over the recipe's
                flags (below, or after `--`), so the observation layout, the
                joint table and the action map are the trainer's, not a copy
  params.pkl    (normalizer, policy, value) in brax's PPO format: flax's own
                initialisation of a make_ppo_networks net under a fixed seed,
                and a fixed-seed normalizer. Random numbers, not a policy.

config.json carries "test_net": {...}; tools/gen_policy_weights.py compiles
such a run with kWeightsArePlaceholder = true, and the firmware refuses to arm
on placeholder weights (cli.cpp armRefused). From there the real deployment
tools run unchanged: export_policy_weights.py -> gen_obs_spec.py ->
gen_policy_weights.py (with --profile, docs/training.md section 9).

The default recipe is the robot's legs-only walk (the robot_legs_walk* runs:
--robot on sim/bimo_biped_v6legs.xml, the 7-channel precision command set,
the loco family) with the deployable action map `full`. Only the flags that
shape the observation and the action map matter to the firmware; the rest
make config.json read like a run of that recipe.

--servo-kp-scale stock: the test net's spec claims the stiffness the robot's
servos are configured for TODAY (main/servo_gains.h, every row factory), so the
image links. A real run records the stiffness it trained with, and
servo_gains.h's static_assert then demands the matching gain table.
"""
import argparse
import json
import os
import pickle
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "sim"))
sys.path.insert(0, os.path.join(ROOT, "sim", "mjx"))

RUNS = os.path.join(ROOT, "sim", "runs")

DEFAULT_RECIPE = [
    "--precision", "--robot", "--xml", "bimo_biped_v6legs.xml",
    "--family", "loco", "--action-map", "full",
    "--servo-kp-scale", "stock",
    "--floor-contacts", "feet", "--mimic-lift", "0.07",
]


def build_config(recipe, out):
    """config.json exactly as train_mjx.main() would write it (minus git)."""
    import train_mjx
    from walker_env import servo_kp_scale_vector
    import mujoco

    args = train_mjx.make_parser().parse_args(recipe + ["--out", out])
    env_kw = train_mjx.build_env_kw(args)
    if env_kw.get("servo_kp_scale") is not None:
        # train_mjx.main pins the RESOLVED stiffness per actuator
        m = mujoco.MjModel.from_xml_path(env_kw["xml_path"])
        names = [m.joint(int(j)).name for j in m.actuator_trnid[:, 0]]
        scale = servo_kp_scale_vector(names, env_kw["servo_kp_scale"])
        env_kw["servo_kp_scale"] = {n: float(f) for n, f in zip(names, scale)
                                    if f != 1.0}
    cfg = {k: (list(v) if isinstance(v, tuple) else v)
           for k, v in env_kw.items()}
    cfg["xml_path"] = os.path.basename(cfg["xml_path"])
    cfg["train"] = vars(args)
    return cfg


def frame_layout(nj, ncmd):
    """walker_env._obs()'s frame (tools/gen_obs_spec.py writes the same)."""
    off, out = 0, {}
    for label, size in (("q", nj), ("dq", nj), ("up", 3), ("linvel", 3),
                        ("gyro", 3), ("prev", nj), ("height", 1),
                        ("phase", 2), ("cmd", ncmd)):
        out[label] = (off, size)
        off += size
    return out, off


def make_params(obs_dim, act_dim, frame, layout, hidden, seed):
    import jax
    from brax.training.acme import running_statistics, specs
    from brax.training.agents.ppo import networks as ppo_networks

    net = ppo_networks.make_ppo_networks(
        observation_size=obs_dim, action_size=act_dim,
        preprocess_observations_fn=running_statistics.normalize,
        policy_hidden_layer_sizes=tuple(hidden),
        value_hidden_layer_sizes=(256, 256))
    kp, kv = jax.random.split(jax.random.PRNGKey(seed))
    policy = jax.device_get(net.policy_network.init(kp))
    value = jax.device_get(net.value_network.init(kv))

    # A normalizer with the shape a trained one has: ordinary channels get
    # non-unit means and scales (so a firmware that skips normalisation fails
    # loudly), and the channels the robot cannot sense -- linvel and height,
    # which the firmware feeds as exact zeros (imu_obs) -- are frozen at mean 0,
    # std = brax's std_eps, as every trained run's are.
    rng = np.random.default_rng(seed)
    mean = rng.uniform(-0.3, 0.3, obs_dim).astype(np.float32)
    std = rng.uniform(0.1, 1.5, obs_dim).astype(np.float32)
    for h in range(obs_dim // frame):
        for key in ("linvel", "height"):
            o, n = layout[key]
            mean[h * frame + o:h * frame + o + n] = 0.0
            std[h * frame + o:h * frame + o + n] = 1e-6
    state = running_statistics.init_state(
        specs.Array((obs_dim,), np.dtype("float32")))
    state = state.replace(mean=mean, std=std,
                          summed_variance=(std ** 2) * 1e6,
                          count=np.float32(1e6))
    return (jax.device_get(state), policy, value)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n\n")[0],
        epilog="train_mjx flags after `--` replace the default recipe")
    ap.add_argument("--out", required=True, help="run name under sim/runs/")
    ap.add_argument("--hidden", default="128,128",
                    help="policy hidden widths (the deployed student's)")
    ap.add_argument("--seed", type=int, default=20261030)
    argv = sys.argv[1:]
    recipe = DEFAULT_RECIPE
    if "--" in argv:
        i = argv.index("--")
        argv, recipe = argv[:i], argv[i + 1:]
    args = ap.parse_args(argv)
    hidden = [int(x) for x in args.hidden.split(",") if x.strip()]

    cfg = build_config(recipe, args.out)
    if cfg.get("action_map") != "full":
        raise SystemExit("the firmware implements action_map 'full' only "
                         "(obs/actuation.h); add --action-map full")

    from eval_precision import make_env
    env = make_env(cfg, 12.0, True,
                   os.path.join(ROOT, "sim", cfg["xml_path"]))
    obs_dim = int(env.observation_space.shape[0])
    act_dim = int(env.action_space.shape[0])
    hist = int(cfg.get("obs_hist_len", 1))
    frame = obs_dim // hist
    layout, fsum = frame_layout(env._nq_act, env._ncmd)
    if fsum != frame or frame * hist != obs_dim:
        raise SystemExit(f"obs {obs_dim} is not {hist} frames of the "
                         f"{fsum}-wide layout")
    cfg["test_net"] = dict(
        seed=args.seed, hidden=hidden, recipe=recipe,
        note="UNTRAINED: flax-initialised weights and a synthetic "
             "normalizer under a fixed seed. Exists to golden-test the "
             "firmware path; never to drive the robot.")

    params = make_params(obs_dim, act_dim, frame, layout, hidden, args.seed)
    out = os.path.join(RUNS, args.out)
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "config.json"), "w") as f:
        json.dump(cfg, f, indent=2)
    with open(os.path.join(out, "params.pkl"), "wb") as f:
        pickle.dump(params, f)
    print(f"{args.out}: {cfg['xml_path']}, action_map {cfg['action_map']}, "
          f"{act_dim} joints {env._act_names}")
    print(f"  obs {hist} x {frame} = {obs_dim}; net "
          f"{' -> '.join(str(s) for s in [obs_dim] + hidden + [2 * act_dim])}"
          f" (UNTRAINED, seed {args.seed})")
    print(f"  wrote {out}/config.json, params.pkl")


if __name__ == "__main__":
    main()
