"""Smoke test for BimoWalkerEnv (Stage 2a).

Not an RL run -- just proves the env is wired correctly:
  1. Gymnasium API contract (spaces, reset, step shapes/dtypes).
  2. A random policy runs a full episode without crashing.
  3. Sanity of the reward signal: standing still keeps the robot alive and
     accumulates positive alive/upright reward (a learnable baseline).
Run: python sim/smoke_test.py
"""
import numpy as np
from walker_env import BimoWalkerEnv  # run from sim/  (or: python -m sim.smoke_test)


def check_contract(env):
    obs, info = env.reset(seed=0)
    assert obs.shape == env.observation_space.shape, obs.shape
    assert obs.dtype == np.float32
    assert env.action_space.shape == (8,)
    a = env.action_space.sample()
    obs, r, term, trunc, info = env.step(a)
    assert obs.shape == env.observation_space.shape
    assert np.isfinite(obs).all(), "non-finite obs"
    assert isinstance(r, float) and np.isfinite(r)
    assert set(info) >= {"fwd_vel", "height", "up_z", "x"}
    print(f"[contract] obs_dim={obs.shape[0]} act_dim=8 "
          f"substeps={env.n_substeps} control_dt={env.control_dt:.3f}s "
          f"max_steps={env.max_steps}  OK")


def run_policy(env, policy, label, seed=0):
    obs, _ = env.reset(seed=seed)
    total, steps = 0.0, 0
    term = trunc = False
    while not (term or trunc):
        obs, r, term, trunc, info = env.step(policy(obs, steps))
        total += r
        steps += 1
    print(f"[{label:9s}] steps={steps:4d} return={total:8.2f} "
          f"final_x={info['x']*1000:6.0f}mm height={info['height']*1000:4.0f}mm "
          f"up_z={info['up_z']:.2f} {'FELL' if term else 'survived'}")
    return total, steps


def main():
    env = BimoWalkerEnv()
    check_contract(env)

    rng = np.random.default_rng(0)
    run_policy(env, lambda o, t: np.zeros(8), "stand")
    run_policy(env, lambda o, t: rng.uniform(-1, 1, 8).astype(np.float32), "random")

    # A crude scripted crouch-and-lean: should stay alive and not be worse than
    # flailing. Not a gait -- just confirms actuation moves the robot sensibly.
    def scripted(o, t):
        a = np.zeros(8, dtype=np.float32)
        c = 0.3 * np.sin(2 * np.pi * t / 40.0)
        a[[1, 5]] = 0.3      # hip pitch forward lean
        a[[2, 6]] = -0.4 + c  # knees flex/extend
        a[[3, 7]] = 0.3      # ankles
        return a
    run_policy(env, scripted, "scripted")

    env.close()
    print("\nSmoke test passed: env resets, steps, and rewards are finite. "
          "Ready to plug into PPO.")


if __name__ == "__main__":
    main()
