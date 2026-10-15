"""PPO on the Mac: CPU MuJoCo physics in worker processes, networks on the GPU.

The physics runs in sim/walker_env.BimoWalkerEnv -- the CPU referee's own
env -- in N worker processes; the policy and value networks train on the M5
Max GPU through PyTorch's Metal backend (MPS, falling back to CPU). Training
in the referee's env removes the MJX -> CPU gap every MJX policy has to cross
before it is believed (docs/training.md section 8).

Same command line as sim/mjx/train_mjx.py (its make_parser / build_env_kw, so
one set of flags means one env config whichever trainer runs it), plus
--workers, --device and --hours. Same run layout under sim/runs/<out>/:
  config.json     train_mjx's keys + "trainer": "mac_ppo"
  params.pkl      brax's (normalizer, policy, value) pytrees, so
                  eval_precision.py, eval_ref.py, distill_student.py and
                  tools/export_policy_weights.py read a Mac run unchanged
  progress.jsonl  one line per iteration ("eval/episode_reward" and
                  "eval/avg_episode_length" are the training episodes that
                  ended in that iteration, stochastic policy)
  mac_state.pt    torch weights + Adam state, for --resume

The PPO is brax 0.14's, term for term: tanh-normal policy (softplus scale +
0.001), swish MLPs (512, 256, 128) with lecun-uniform kernels, the running-
statistics observation normalizer (brax's own code, updated on each batch
before the SGD), GAE (lambda 0.95) recomputed per minibatch with the current
value net, clip 0.3, normalized advantages, value loss 0.25 * mean(err^2),
entropy with the tanh log-det term, Adam without grad clipping. One iteration
collects batch_size x minibatches trajectories of unroll steps with one set
of params, then runs `updates` epochs over trajectory-shuffled minibatches.

    .venv/bin/python sim/mac_train.py --out robot_walk_c0_mac --robot --precision \\
        --family loco --walk-submix 0,0 --cmd-v-range 0.05,0.35 --w-mimic 1.5 \\
        --servo-kp-scale robot --act-lag 2,12 --steps 400000000 --hours 6
"""
from __future__ import annotations

import inspect
import json
import math
import multiprocessing as mp
import os
import pickle
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(HERE, "mjx")]
RUNS = os.path.join(HERE, "runs")
LOG2PI = math.log(2.0 * math.pi)


# --------------------------------------------------------------- env kwargs
def cpu_env_kwargs(env_kw: dict) -> dict:
    """train_mjx's env kwargs -> BimoWalkerEnv kwargs for TRAINING: the keys
    the CPU env shares, the constructor settings MJX implies (command mode,
    the STS3215 actuator model), the per-episode lag DR under the CPU env's
    own name, and the feet-crossing reward term."""
    from walker_env import BimoWalkerEnv
    params = set(inspect.signature(BimoWalkerEnv.__init__).parameters)
    kw = {k: v for k, v in env_kw.items() if k in params}
    kw.update(actuator_model="sts3215", command_mode=True, foot_cross_term=True)
    if env_kw.get("act_lag_hz_max") is not None:
        kw["act_lag_dr_max"] = float(env_kw["act_lag_hz_max"])
    if "xml_path" in kw and not os.path.isabs(kw["xml_path"]) and not os.path.exists(kw["xml_path"]):
        kw["xml_path"] = os.path.join(HERE, os.path.basename(kw["xml_path"]))
    return kw


# --------------------------------------------------------------- networks
def build_nets(obs_size, act_size, hidden):
    import torch
    from torch import nn

    def mlp(sizes):
        layers = []
        for i in range(len(sizes) - 1):
            lin = nn.Linear(sizes[i], sizes[i + 1])
            # flax lecun_uniform: U(-sqrt(3/fan_in), +sqrt(3/fan_in)); bias 0
            lim = math.sqrt(3.0 / sizes[i])
            nn.init.uniform_(lin.weight, -lim, lim)
            nn.init.zeros_(lin.bias)
            layers.append(lin)
            if i < len(sizes) - 2:
                layers.append(nn.SiLU())        # linen.swish
        return nn.Sequential(*layers)
    pol = mlp([obs_size, *hidden, 2 * act_size])
    val = mlp([obs_size, *hidden, 1])
    return pol, val, torch


def linears(net):
    return [m for m in net if m.__class__.__name__ == "Linear"]


def tanh_fldj(x):
    """brax TanhBijector.forward_log_det_jacobian, elementwise."""
    import torch.nn.functional as F
    return 2.0 * (math.log(2.0) - x - F.softplus(-2.0 * x))


# --------------------------------------------------------------- workers
def _worker(conn, env_kwargs, n_envs, seed, obs_size, act_size, hidden):
    import torch
    torch.set_num_threads(1)
    from walker_env import BimoWalkerEnv
    envs = [BimoWalkerEnv(**env_kwargs) for _ in range(n_envs)]
    obs = np.stack([e.reset(seed=seed + i)[0] for i, e in enumerate(envs)]).astype(np.float32)
    ep_ret = np.zeros(n_envs)
    ep_len = np.zeros(n_envs, dtype=np.int64)
    pol, _, _ = build_nets(obs_size, act_size, hidden)
    pol.eval()
    gen = torch.Generator().manual_seed(seed)
    mean = np.zeros(obs_size, np.float32)
    std = np.ones(obs_size, np.float32)
    while True:
        msg = conn.recv()
        if msg[0] == "close":
            conn.close()
            return
        if msg[0] == "params":
            _, sd, mean, std = msg
            pol.load_state_dict({k: torch.from_numpy(v) for k, v in sd.items()})
            continue
        _, T, deterministic = msg                        # "rollout"
        O = np.empty((T + 1, n_envs, obs_size), np.float32)
        A = np.empty((T, n_envs, act_size), np.float32)  # raw (pre-tanh)
        LP = np.empty((T, n_envs), np.float32)
        R = np.empty((T, n_envs), np.float32)
        TERM = np.zeros((T, n_envs), np.float32)
        TRUNC = np.zeros((T, n_envs), np.float32)
        done_eps = []
        O[0] = obs
        t_phys = 0.0
        for t in range(T):
            with torch.no_grad():
                x = torch.from_numpy((obs - mean) / std)
                out = pol(x)
                loc, raw_s = out[:, :act_size], out[:, act_size:]
                scale = torch.nn.functional.softplus(raw_s) + 0.001
                if deterministic:
                    raw = loc
                else:
                    raw = loc + scale * torch.randn(loc.shape, generator=gen)
                lp = (-0.5 * ((raw - loc) / scale) ** 2 - torch.log(scale) - 0.5 * LOG2PI
                      - tanh_fldj(raw)).sum(-1)
                act = torch.tanh(raw).numpy()
            A[t] = raw.numpy()
            LP[t] = lp.numpy()
            t0 = time.perf_counter()
            for i, e in enumerate(envs):
                o, r, term, trunc, _ = e.step(act[i])
                ep_ret[i] += r
                ep_len[i] += 1
                R[t, i] = r
                TERM[t, i] = float(term)
                TRUNC[t, i] = float(trunc and not term)
                if term or trunc:
                    done_eps.append((float(ep_ret[i]), int(ep_len[i])))
                    ep_ret[i] = 0.0
                    ep_len[i] = 0
                    o, _ = e.reset()
                obs[i] = o
            t_phys += time.perf_counter() - t0
            O[t + 1] = obs
        conn.send((O, A, LP, R, TERM, TRUNC, done_eps, t_phys))


class Workers:
    def __init__(self, env_kwargs, n_workers, envs_per_worker, seed, obs_size, act_size, hidden):
        ctx = mp.get_context("spawn")
        self.conns, self.procs = [], []
        for w in range(n_workers):
            a, b = ctx.Pipe()
            p = ctx.Process(target=_worker, args=(b, env_kwargs, envs_per_worker,
                                                  seed + 100_000 * w, obs_size, act_size, hidden),
                            daemon=True)
            p.start()
            self.conns.append(a)
            self.procs.append(p)

    def set_params(self, sd, mean, std):
        for c in self.conns:
            c.send(("params", sd, mean, std))

    def rollout(self, T, deterministic=False):
        for c in self.conns:
            c.send(("rollout", T, deterministic))
        parts = [c.recv() for c in self.conns]
        cat = [np.concatenate([p[k] for p in parts], axis=1) for k in range(6)]
        eps = [e for p in parts for e in p[6]]
        t_phys = max(p[7] for p in parts)
        return cat, eps, t_phys

    def close(self):
        for c in self.conns:
            try:
                c.send(("close",))
            except Exception:  # noqa: BLE001
                pass
        for p in self.procs:
            p.join(timeout=5)


# --------------------------------------------------------------- brax params
class BraxIO:
    """Templates of brax's param pytrees, so a checkpoint is exactly what
    brax ppo.train pickles: (RunningStatisticsState, policy, value)."""

    def __init__(self, obs_size, act_size, hidden):
        import jax
        import jax.numpy as jnp
        from brax.training.acme import running_statistics, specs
        from brax.training.agents.ppo import networks as ppo_networks
        self.jax, self.jnp, self.rs = jax, jnp, running_statistics
        nets = ppo_networks.make_ppo_networks(
            observation_size=obs_size, action_size=act_size,
            preprocess_observations_fn=running_statistics.normalize,
            policy_hidden_layer_sizes=hidden, value_hidden_layer_sizes=hidden)
        key = jax.random.PRNGKey(0)
        self.pol_t = nets.policy_network.init(key)
        self.val_t = nets.value_network.init(key)
        self.norm = running_statistics.init_state(specs.Array((obs_size,), jnp.dtype("float32")))
        self._update = jax.jit(running_statistics.update)

    def update_norm(self, obs2d):
        self.norm = self._update(self.norm, self.jnp.asarray(obs2d))

    def mean_std(self):
        return (np.asarray(self.norm.mean, np.float32), np.asarray(self.norm.std, np.float32))

    def _fill(self, template, net):
        lins = linears(net)
        layers = dict(template["params"])
        new = {}
        for i in range(len(lins)):
            k = f"hidden_{i}"
            new[k] = {"kernel": self.jnp.asarray(lins[i].weight.detach().cpu().numpy().T),
                      "bias": self.jnp.asarray(lins[i].bias.detach().cpu().numpy())}
        assert set(new) == set(layers), (sorted(new), sorted(layers))
        out = dict(template)
        out["params"] = new
        return out

    def params(self, pol, val):
        return (self.norm, self._fill(self.pol_t, pol), self._fill(self.val_t, val))

    @staticmethod
    def load_into(params, pol, val, torch):
        """brax params.pkl -> torch nets (warm start); returns the normalizer."""
        for net, tree in ((pol, params[1]), (val, params[2])):
            lins = linears(net)
            for i, lin in enumerate(lins):
                p = tree["params"][f"hidden_{i}"]
                lin.weight.data = torch.as_tensor(np.asarray(p["kernel"]).T.copy())
                lin.bias.data = torch.as_tensor(np.asarray(p["bias"]).copy())
        return params[0]


# --------------------------------------------------------------- PPO loss
def ppo_loss(pol, val, torch, obs, raw, logp_b, rew, term, trunc, cfg):
    """brax compute_ppo_loss on a [B, T] minibatch (obs is [B, T+1, D],
    already normalized). Returns (loss, metrics)."""
    B, T1, D = obs.shape
    T = T1 - 1
    out = pol(obs[:, :T].reshape(-1, D))
    A = raw.shape[-1]
    loc, raw_s = out[:, :A], out[:, A:]
    scale = torch.nn.functional.softplus(raw_s) + 0.001
    r = raw.reshape(-1, A)
    logp = (-0.5 * ((r - loc) / scale) ** 2 - torch.log(scale) - 0.5 * LOG2PI - tanh_fldj(r)).sum(-1)
    logp = logp.reshape(B, T)
    values = val(obs.reshape(-1, D)).reshape(B, T1)
    v, boot = values[:, :T], values[:, T]
    with torch.no_grad():
        disc = cfg["discounting"]
        lam = cfg["gae_lambda"]
        tmask = 1.0 - trunc
        vnext = torch.cat([v[:, 1:], boot[:, None]], dim=1)
        deltas = (rew + disc * (1.0 - term) * vnext - v) * tmask
        acc = torch.zeros_like(boot)
        vs_minus = torch.empty_like(v)
        for t in range(T - 1, -1, -1):
            acc = deltas[:, t] + disc * (1.0 - term[:, t]) * tmask[:, t] * lam * acc
            vs_minus[:, t] = acc
        vs = vs_minus + v
        vs_next = torch.cat([vs[:, 1:], boot[:, None]], dim=1)
        adv = (rew + disc * (1.0 - term) * vs_next - v) * tmask
        adv = (adv - adv.mean()) / (adv.std(unbiased=False) + 1e-8)
    rho = torch.exp(logp - logp_b)
    eps = cfg["clip"]
    policy_loss = -torch.mean(torch.minimum(rho * adv, torch.clamp(rho, 1 - eps, 1 + eps) * adv))
    v_loss = torch.mean((vs - v) ** 2) * 0.5 * cfg["vf_coef"]
    sample = loc + scale * torch.randn_like(loc)
    ent = (0.5 + 0.5 * LOG2PI + torch.log(scale) + tanh_fldj(sample)).sum(-1).mean()
    ent_loss = -cfg["entropy"] * ent
    loss = policy_loss + v_loss + ent_loss
    return loss, dict(policy_loss=policy_loss.detach(), v_loss=v_loss.detach(),
                      entropy_loss=ent_loss.detach(), mean_std=scale.detach().mean())


# --------------------------------------------------------------- main
def main(argv=None):
    import train_mjx as TM
    p = TM.make_parser()
    p.description = __doc__
    p.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 4) - 2),
                   help="physics worker processes (default: cores - 2)")
    p.add_argument("--device", default=None, help="mps (default when available) or cpu")
    p.add_argument("--hours", type=float, default=None, help="stop (and checkpoint) after this long")
    p.add_argument("--resume", action="store_true", help="continue <out> from its mac_state.pt")
    args = p.parse_args(argv)
    env_kw = TM.build_env_kw(args)

    import torch
    dev = args.device or ("mps" if torch.backends.mps.is_available() else "cpu")
    torch.manual_seed(args.seed)
    ckw = cpu_env_kwargs(env_kw)
    from walker_env import BimoWalkerEnv
    probe = BimoWalkerEnv(**ckw)
    obs_size, act_size = probe.observation_space.shape[0], probe.action_space.shape[0]
    if env_kw.get("servo_kp_scale") is not None:
        # pin the stiffness the env RESOLVED, per actuator (train_mjx's rule)
        env_kw["servo_kp_scale"] = {n: float(f) for n, f in zip(probe._servo_names, probe.servo_kp_scale)
                                    if f != 1.0}
        ckw["servo_kp_scale"] = env_kw["servo_kp_scale"]
    episode_length = probe.max_steps
    del probe
    hidden = (512, 256, 128) if args.precision else (128, 128)
    if not args.precision:
        print("note: brax's non-precision value net is (256, 256); this trainer uses one size for both")

    out = os.path.join(RUNS, args.out)
    os.makedirs(out, exist_ok=True)
    cfg = {k: (list(v) if isinstance(v, tuple) else v) for k, v in env_kw.items()}
    if "xml_path" in cfg:
        cfg["xml_path"] = os.path.basename(cfg["xml_path"])
    cfg.update(train=vars(args), trainer="mac_ppo")
    try:
        sha = subprocess.run(["git", "rev-parse", "HEAD"], cwd=HERE, text=True, capture_output=True,
                             timeout=10).stdout.strip() or None
        dirty = bool(subprocess.run(["git", "status", "--porcelain", "--untracked-files=no"], cwd=HERE,
                                    text=True, capture_output=True, timeout=10).stdout.strip())
    except Exception:  # noqa: BLE001
        sha, dirty = None, None
    cfg.update(git_sha=sha, git_dirty=dirty)
    with open(os.path.join(out, "config.json"), "w") as f:
        json.dump(cfg, f, indent=2)

    pol, val, _ = build_nets(obs_size, act_size, hidden)
    bio = BraxIO(obs_size, act_size, hidden)
    if args.init_from:
        with open(os.path.join(RUNS, args.init_from, "params.pkl"), "rb") as f:
            bio.norm = BraxIO.load_into(pickle.load(f), pol, val, torch)
        print(f"warm-starting from {args.init_from}/params.pkl")
    pol.to(dev)
    val.to(dev)
    opt = torch.optim.Adam(list(pol.parameters()) + list(val.parameters()), lr=args.lr)
    steps_done = 0
    state_path = os.path.join(out, "mac_state.pt")
    if args.resume and os.path.exists(state_path):
        st = torch.load(state_path, map_location=dev, weights_only=False)
        pol.load_state_dict(st["pol"])
        val.load_state_dict(st["val"])
        opt.load_state_dict(st["opt"])
        bio.norm = st["norm"]
        steps_done = int(st["steps"])
        print(f"resumed {args.out} at {steps_done:,} steps")

    n_traj = args.batch_size * args.minibatches
    T = args.unroll
    n_envs = max(args.workers, min(args.envs, n_traj))
    epw = max(1, n_envs // args.workers)
    n_envs = epw * args.workers
    rounds = max(1, math.ceil(n_traj / n_envs))
    n_traj = rounds * n_envs                       # rounded up to whole rounds
    mb = n_traj // args.minibatches
    steps_per_it = n_traj * T
    n_it = max(1, math.ceil((args.steps - steps_done) / steps_per_it))
    eval_every = max(1, n_it // max(1, args.num_evals))
    ppo_cfg = dict(discounting=args.discounting, gae_lambda=0.95, clip=0.3, vf_coef=0.5, entropy=args.entropy)
    print(f"mac_ppo: {args.workers} workers x {epw} envs = {n_envs} envs; {rounds} rounds x {T} steps "
          f"= {n_traj} trajectories ({steps_per_it:,} steps) per iteration, {args.minibatches} minibatches "
          f"x {args.updates} epochs; device {dev}; {n_it} iterations; obs {obs_size}, act {act_size}", flush=True)

    workers = Workers(ckw, args.workers, epw, args.seed, obs_size, act_size, hidden)
    log_path = os.path.join(out, "progress.jsonl")
    t0 = time.time()
    deadline = t0 + 3600.0 * args.hours if args.hours else None
    last_eps = []

    def sd_cpu():
        return {k: v.detach().cpu().numpy() for k, v in pol.state_dict().items()}

    def checkpoint():
        with open(os.path.join(out, "params.pkl"), "wb") as f:
            pickle.dump(bio.params(pol, val), f)
        torch.save(dict(pol=pol.state_dict(), val=val.state_dict(), opt=opt.state_dict(), norm=bio.norm,
                        steps=steps_done), state_path)

    try:
        for it in range(n_it):
            ti = time.time()
            mean, std = bio.mean_std()
            workers.set_params(sd_cpu(), mean, std)
            chunks, eps_it, t_phys = [], [], 0.0
            for _ in range(rounds):
                c, e, tp = workers.rollout(T)
                chunks.append(c)
                eps_it += e
                t_phys += tp
            # [T(+1), n_envs, ...] per round -> [n_traj, T(+1), ...]
            O, A, LP, R, TERM, TRUNC = (np.concatenate([np.swapaxes(c[k], 0, 1) for c in chunks], axis=0)
                                        for k in range(6))
            t_roll = time.time() - ti
            # brax: the normalizer is updated with this batch's observations
            # BEFORE the SGD, which then normalizes with the updated stats
            bio.update_norm(O[:, :T].reshape(-1, obs_size))
            mean, std = bio.mean_std()
            tu = time.time()
            On = torch.as_tensor((O - mean) / std, device=dev)
            At = torch.as_tensor(A, device=dev)
            LPt = torch.as_tensor(LP, device=dev)
            Rt = torch.as_tensor(R, device=dev)
            TEt = torch.as_tensor(TERM, device=dev)
            TRt = torch.as_tensor(TRUNC, device=dev)
            m_acc = {}
            for _ in range(args.updates):
                perm = torch.randperm(n_traj, device=dev)
                for j in range(args.minibatches):
                    idx = perm[j * mb:(j + 1) * mb]
                    loss, m = ppo_loss(pol, val, torch, On[idx], At[idx], LPt[idx], Rt[idx], TEt[idx],
                                       TRt[idx], ppo_cfg)
                    opt.zero_grad(set_to_none=True)
                    loss.backward()
                    opt.step()
                    for k, v in m.items():          # summed on the device, read once
                        m_acc[k] = m_acc[k] + v if k in m_acc else v
            if dev == "mps":
                torch.mps.synchronize()
            t_upd = time.time() - tu
            steps_done += steps_per_it
            if eps_it:
                last_eps = eps_it
            n_upd = args.updates * args.minibatches
            rec = {"t": round(time.time() - t0, 1), "steps": int(steps_done)}
            if last_eps:
                rec["eval/episode_reward"] = float(np.mean([e[0] for e in last_eps]))
                rec["eval/avg_episode_length"] = float(np.mean([e[1] for e in last_eps]))
                rec["eval/episodes"] = len(last_eps)
            rec.update({f"training/{k}": float(v) / n_upd for k, v in m_acc.items()})
            dt_it = time.time() - ti
            rec.update({"training/sps": steps_per_it / dt_it, "mac/rollout_s": round(t_roll, 2),
                        "mac/physics_s": round(t_phys, 2), "mac/update_s": round(t_upd, 2)})
            with open(log_path, "a") as f:
                f.write(json.dumps(rec) + "\n")
            print(f"[{rec['t']:8.1f}s] {steps_done:>12,d} steps  ep_len "
                  f"{rec.get('eval/avg_episode_length', float('nan')):6.1f}/{episode_length}  reward "
                  f"{rec.get('eval/episode_reward', float('nan')):9.1f}  {rec['training/sps']:7.0f} sps "
                  f"(rollout {t_roll:.1f}s, update {t_upd:.1f}s)", flush=True)
            if (it + 1) % eval_every == 0 or it == n_it - 1:
                checkpoint()
            if deadline and time.time() >= deadline:
                print("time limit reached")
                break
    finally:
        checkpoint()
        workers.close()
    print(f"saved -> {out}  ({time.time() - t0:.0f}s total)")


if __name__ == "__main__":
    main()
