#!/usr/bin/env python3
"""An int8 profile's net as a brax run dir, so every float grader (eval_precision.py,
rca/candidates.py, stand-still/stand_candidates.py) can grade the int8 net with no shim:

    .venv/bin/python experiments/policy/int8-256/q8_proxy.py <run> <profile> [out.json]

reads firmware/policies/<profile>/policy/weights.h (the compiled-in int8 net; its
kWeightsRun must be <run>), writes sim/runs/<run>_q8/ = <run>'s params.pkl with every
policy kernel/bias replaced by the dequantized int8 one (wq * scale; the head's 12 mean
logits; the unused log-std half stays float) and <run>'s config.json.

What the proxy leaves out is the firmware's per-tensor int16 activation rounding (< 2e-4
in action, docs/design-v6/2026-11-02-esp32-int8-policy.md); it is checked here against the
bit-exact mirror tools/policy_q8.forward_q8 on CLOSED-LOOP ROLLOUTS OF THE INT8 NET ITSELF
(held-out seeds and commands, the referee's CPU env), which also gives the int8-vs-float
action agreement on the int8 net's own state distribution."""
from __future__ import annotations
import json, os, pickle, re, shutil, sys
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))
for d in (os.path.join(ROOT, "sim"), os.path.join(ROOT, "sim", "mjx")):
    sys.path.insert(0, d)
import policy_q8 as Q  # noqa: E402

F = np.float32
SEEDS = tuple(range(2000, 2016))          # held out: calibration used 1000-1011
CMDS = ((0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
        (0.15, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
        (0.3, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0))
STEPS = 500


def float_layers(policy_params):
    p = policy_params["params"]
    ks = sorted((k for k in p if k.startswith("hidden_")), key=lambda k: int(k.split("_")[1]))
    return ks, [(np.asarray(p[k]["kernel"], F), np.asarray(p[k]["bias"], F)) for k in ks]


def main():
    run, profile = sys.argv[1], sys.argv[2]
    out_json = sys.argv[3] if len(sys.argv) > 3 else None
    hdr = os.path.join(ROOT, "firmware", "policies", profile, "policy", "weights.h")
    txt = open(hdr).read()
    wrun = re.search(r'kWeightsRun = "([^"]+)"', txt).group(1)
    if wrun != run:
        raise SystemExit(f"{hdr} carries {wrun}, not {run}")
    mean, std, qlayers, act_dim = Q.read_q8_header(hdr)
    rdir = os.path.join(ROOT, "sim", "runs", run)
    params = pickle.load(open(os.path.join(rdir, "params.pkl"), "rb"))
    norm, pol = params[0], params[1]
    ks, layers = float_layers(pol)
    assert len(ks) == len(qlayers)
    m0 = np.asarray(norm.mean, F).reshape(-1); s0 = np.asarray(norm.std, F).reshape(-1)
    assert np.array_equal(m0, mean) and np.array_equal(s0, std), "header normalizer != run's"

    # dequantized policy params
    newp = {k: dict(v) for k, v in pol["params"].items()}
    deq_layers = []
    for k, (w, b), (wq, sc, qb, n_in) in zip(ks, layers, qlayers):
        n_out = wq.shape[0]
        deq = (wq[:, :n_in].astype(F) * sc[:, None]).T.astype(F)      # (in, n_out)
        kern = w.copy(); kern[:, :n_out] = deq
        bias = b.copy(); bias[:n_out] = qb
        newp[k]["kernel"] = kern; newp[k]["bias"] = bias
        deq_layers.append((kern, bias))
    pol_q = dict(pol); pol_q["params"] = newp
    out_name = run + "_q8"
    odir = os.path.join(ROOT, "sim", "runs", out_name)
    os.makedirs(odir, exist_ok=True)
    with open(os.path.join(odir, "params.pkl"), "wb") as f:
        pickle.dump((norm, pol_q) + tuple(params[2:]), f)
    shutil.copyfile(os.path.join(rdir, "config.json"), os.path.join(odir, "config.json"))

    # closed-loop rollouts of the int8 net (bit-exact mirror) on the referee's CPU env
    import eval_precision as E
    cfg = json.load(open(os.path.join(rdir, "config.json")))
    env = E.make_env(cfg, 12.0, True, os.path.join(ROOT, "sim", cfg["xml_path"]))
    eq, ep, steps, falls = [], [], 0, 0
    for k, seed in enumerate(SEEDS):
        o, _ = env.reset(seed=seed)
        env.set_command(*CMDS[k % len(CMDS)])
        for _ in range(STEPS):
            a_q = Q.forward_q8(mean, std, qlayers, o, act_dim)
            a_f = Q.float_forward(mean, std, layers, o, act_dim)
            a_p = Q.float_forward(mean, std, deq_layers, o, act_dim)
            eq.append(np.abs(a_q - a_f)); ep.append(np.abs(a_q - a_p))
            o, _, term, _, _ = env.step(a_q)
            steps += 1
            if term:
                falls += 1
                break
    eq, ep = np.array(eq), np.array(ep)

    def st(e):
        return dict(rms=float(np.sqrt(np.mean(e ** 2))), p99=float(np.percentile(e, 99)),
                    max=float(e.max()))
    res = dict(run=run, profile=profile, proxy=out_name, obs=int(steps), rollouts=len(SEEDS),
               rollout_terminations=falls, int8_vs_float=st(eq), proxy_vs_int8=st(ep))
    with open(os.path.join(odir, "q8_proxy.json"), "w") as f:
        json.dump(res, f, indent=2)
    if out_json:
        with open(out_json, "w") as f:
            json.dump(res, f, indent=2)
    print(json.dumps(res, indent=2))


if __name__ == "__main__":
    main()
