"""Linearise the student around the recorded stand: the policy's own recursion
through prev_action (3 frames) and its frequency response from gyro / up / q / dq."""
import os, sys, pickle
import numpy as np
os.environ.setdefault("JAX_PLATFORMS", "cpu")
import jax, jax.numpy as jnp
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import rec, fidelity as FI
sys.path.insert(0, os.path.join(rec.ROOT, "sim", "mjx"))
from brax.training.agents.ppo import networks as ppo_networks
from brax.training.acme import running_statistics
import eval_precision as EP

RUN = FI.RUN


def make_mean_fn(run=RUN):
    params = pickle.load(open(os.path.join(run, "params.pkl"), "rb"))
    net = ppo_networks.make_ppo_networks(165, 12, preprocess_observations_fn=running_statistics.normalize,
                                         policy_hidden_layer_sizes=EP._hidden_sizes(params[1], (128, 128)))
    def f(o):
        logits = net.policy_network.apply(params[0], params[1], o[None])[0]
        return jnp.tanh(logits[:12])
    return jax.jit(f)


def freq_resp(J, slot, w, nj=12):
    """H(e^jw) from a 3-wide sensor slot (per frame offset `slot`, width k) to actions,
    including the prev_action recursion."""
    z = np.exp(1j * w)
    P = sum(J[:, h * 55 + 33:h * 55 + 45] * z ** (-(1 + h)) for h in range(3))
    off, k = slot
    G = sum(J[:, h * 55 + off:h * 55 + off + k] * z ** (-h) for h in range(3))
    return np.linalg.solve(np.eye(nj) - P, G)


def main(run=RUN, label=""):
    f = make_mean_fn(run)
    jac = jax.jit(jax.jacfwd(f))
    r = rec.load(); n = 498
    a_fw = rec.angles_to_action(r["nom"][:n]); prev = np.vstack([np.zeros((1, 12)), a_fw[:-1]])
    O = FI.stack(FI.frames(r, n, prev))
    Js = np.array([np.asarray(jac(jnp.asarray(O[i]))) for i in range(25, 75, 2)])   # 0.5-1.5 s, before the growth
    J = Js.mean(0)
    # policy-only recursion (plant frozen): companion matrix of a_t = sum_h P_h a_{t-1-h}
    Ph = [J[:, h * 55 + 33:h * 55 + 45] for h in range(3)]
    C = np.zeros((36, 36)); C[:12, :12], C[:12, 12:24], C[:12, 24:] = Ph
    C[12:24, :12] = np.eye(12); C[24:, 12:24] = np.eye(12)
    ev = np.linalg.eigvals(C)
    k = np.argsort(-np.abs(ev))[:6]
    print(label, "prev_action recursion, largest |eig|:", [f"{abs(e):.2f}@{abs(np.angle(e)) / (2 * np.pi) * 50:.1f}Hz" for e in ev[k]])
    out = {}
    for name, slot in (("gyro", (30, 3)), ("up", (24, 3)), ("q", (0, 12)), ("dq", (12, 12))):
        row = []
        for fh in (1, 3, 6, 9, 12, 20):
            Hm = freq_resp(J, slot, 2 * np.pi * fh / 50)
            row.append(np.linalg.norm(Hm, 2))
        out[name] = row
        print(f"  {label} |H| {name:5s} (sigma_max, action per unit) @1,3,6,9,12,20 Hz:", np.round(row, 2))
    # gyro x specifically -> roll joints
    for fh in (3, 9):
        Hm = freq_resp(J, (30, 3), 2 * np.pi * fh / 50)
        print(f"  {label} gyro_x -> actions @{fh}Hz |.|:", np.round(np.abs(Hm[:, 0]), 2))
    return J, ev, out


if __name__ == "__main__":
    main()
