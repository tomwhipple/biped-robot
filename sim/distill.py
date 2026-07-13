"""Distill per-skill experts into one command-conditioned student (DAgger).

Why: five from-scratch attempts showed this plant learns single behaviors
well (dash, stand) but not a conditional skill family jointly at CPU-PPO
scale. So: train experts per command region, then teach one student to
imitate whichever expert matches the active command.

Run:  .venv/bin/python sim/distill.py --out cmd_distill \
          --walk exp_walk --stand cmd_11v2 --pivotl exp_pivot_l --pivotr exp_pivot_r

Experts were trained with the command channels pinned (cmd_fixed) but share
the student's 38-dim observation space; each expert reads observations
normalized by ITS OWN VecNormalize stats. The student is saved as a normal
SB3 run dir (model.zip + vecnormalize.pkl + env_config.json), so
eval_policy.py / eval_commands.py work on it unchanged.
"""
import argparse
import json
import os

import numpy as np
import torch
import torch.nn as nn

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from walker_env import BimoWalkerEnv

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "runs")
XML = os.path.join(HERE, "bimo_biped_v2.xml")

ENV_KW = dict(xml_path=XML, actuator_model="sts3215", supply_voltage=11.1,
              command_mode=True, imu_obs=True, backlash_deg=0.5,
              cmd_v_range=(0.3, 0.9), cmd_w_range=0.6,
              cmd_resample_s=(3.0, 6.0))


class Expert:
    def __init__(self, run_name):
        d = os.path.join(RUNS, run_name)
        self.model = PPO.load(os.path.join(d, "model"), device="cpu")
        import pickle
        with open(os.path.join(d, "vecnormalize.pkl"), "rb") as f:
            vn = pickle.load(f)
        self.mean = vn.obs_rms.mean
        self.std = np.sqrt(vn.obs_rms.var + 1e-8)

    def act(self, raw_obs):
        o = np.clip((raw_obs - self.mean) / self.std, -10, 10).astype(np.float32)
        a, _ = self.model.predict(o[None], deterministic=True)
        return a[0]


def pick_expert(experts, cmd):
    v, w = cmd
    if abs(v) >= 0.25:
        return experts["walk"]
    if w >= 0.2:
        return experts["pivotl"]
    if w <= -0.2:
        return experts["pivotr"]
    return experts["stand"]


class Student(nn.Module):
    """Matches the project's SB3 actor: obs -> 128 tanh -> 128 tanh -> 8."""
    def __init__(self, n_obs, n_act=8):
        super().__init__()
        # matches train_ppo's policy_kwargs net_arch=[128, 128]
        self.net = nn.Sequential(nn.Linear(n_obs, 128), nn.Tanh(),
                                 nn.Linear(128, 128), nn.Tanh(),
                                 nn.Linear(128, n_act))

    def forward(self, x):
        return self.net(x)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--out", default="cmd_distill")
    p.add_argument("--walk", default="exp_walk")
    p.add_argument("--stand", default="cmd_11v2")
    p.add_argument("--pivotl", default="exp_pivot_l")
    p.add_argument("--pivotr", default="exp_pivot_r")
    p.add_argument("--iters", type=int, default=6)
    p.add_argument("--steps-per-iter", type=int, default=20000)
    p.add_argument("--epochs", type=int, default=10)
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args()

    torch.manual_seed(args.seed)
    experts = {k: Expert(getattr(args, k))
               for k in ("walk", "stand", "pivotl", "pivotr")}
    env = BimoWalkerEnv(**ENV_KW)
    n_obs = env.observation_space.shape[0]
    student = Student(n_obs)
    opt = torch.optim.Adam(student.parameters(), lr=3e-4)

    # dataset (aggregated across iterations, the point of DAgger)
    X, Y = [], []
    s_mean = np.zeros(n_obs)
    s_std = np.ones(n_obs)

    rng = np.random.default_rng(args.seed)
    for it in range(args.iters):
        beta = max(0.0, 1.0 - it / 3.0)     # teacher-driven early, student late
        obs, _ = env.reset(seed=args.seed * 1000 + it)
        falls, steps_alive, ep_len = 0, [], 0
        for t in range(args.steps_per_iter):
            teacher = pick_expert(experts, env._cmd)
            a_teacher = teacher.act(obs)
            X.append(obs.copy())
            Y.append(a_teacher.copy())
            if rng.uniform() < beta:
                a = a_teacher
            else:
                o = np.clip((obs - s_mean) / s_std, -10, 10)
                with torch.no_grad():
                    a = student(torch.as_tensor(o, dtype=torch.float32)).numpy()
                a = np.clip(a, -1, 1)
            obs, _, term, trunc, _ = env.step(np.asarray(a, dtype=np.float32))
            ep_len += 1
            if term or trunc:
                falls += int(term)
                steps_alive.append(ep_len)
                ep_len = 0
                obs, _ = env.reset()
        # refit input standardization on all data, then BC-train
        Xa = np.asarray(X, dtype=np.float32)
        Ya = np.asarray(Y, dtype=np.float32)
        s_mean = Xa.mean(axis=0)
        s_std = Xa.std(axis=0) + 1e-6
        Xn = torch.as_tensor(np.clip((Xa - s_mean) / s_std, -10, 10))
        Yt = torch.as_tensor(Ya)
        for ep in range(args.epochs):
            perm = torch.randperm(len(Xn))
            tot = 0.0
            for i in range(0, len(Xn), 1024):
                idx = perm[i:i + 1024]
                loss = ((student(Xn[idx]) - Yt[idx]) ** 2).mean()
                opt.zero_grad()
                loss.backward()
                opt.step()
                tot += float(loss) * len(idx)
        print(f"iter {it}: beta {beta:.2f}  dataset {len(Xn)}  "
              f"BC loss {tot / len(Xn):.4f}  falls {falls}  "
              f"mean ep {np.mean(steps_alive) * 0.02 if steps_alive else 10:.1f}s",
              flush=True)

    # ---- export as a normal SB3 run dir ------------------------------------
    out = os.path.join(RUNS, args.out)
    os.makedirs(out, exist_ok=True)
    venv = DummyVecEnv([lambda: BimoWalkerEnv(**ENV_KW)])
    venv = VecNormalize(venv, norm_obs=True, norm_reward=False, clip_obs=10.0)
    venv.obs_rms.mean = s_mean.astype(np.float64)
    venv.obs_rms.var = (s_std.astype(np.float64)) ** 2
    venv.obs_rms.count = len(X)
    model = PPO("MlpPolicy", venv, device="cpu", seed=args.seed,
                policy_kwargs=dict(net_arch=[128, 128]))
    with torch.no_grad():  # copy student weights into the SB3 actor
        sb3_layers = [model.policy.mlp_extractor.policy_net[0],
                      model.policy.mlp_extractor.policy_net[2],
                      model.policy.action_net]
        my_layers = [student.net[0], student.net[2], student.net[4]]
        for dst, src in zip(sb3_layers, my_layers):
            dst.weight.copy_(src.weight)
            dst.bias.copy_(src.bias)
        model.policy.log_std.fill_(-3.0)    # near-deterministic
    model.save(os.path.join(out, "model"))
    venv.training = False
    venv.save(os.path.join(out, "vecnormalize.pkl"))
    cfg = {k: (list(v) if isinstance(v, tuple) else v) for k, v in ENV_KW.items()}
    cfg.update(dash=False, cmd_stand_prob=0.3)
    with open(os.path.join(out, "env_config.json"), "w") as f:
        json.dump(cfg, f, indent=2)
    print(f"student saved -> {out} (SB3-compatible: eval tools work as-is)")


if __name__ == "__main__":
    main()
