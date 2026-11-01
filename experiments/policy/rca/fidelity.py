"""FIDELITY: rebuild the v6legs observation from the flight recording exactly as
ctrl_task.cpp does, run the brax student on it, compare with the firmware's action
(recorded as the per-joint `nominal` = actionToAngles(action), in steps)."""
import os, sys, json, pickle
import numpy as np
os.environ.setdefault("JAX_PLATFORMS", "cpu")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import rec
sys.path.insert(0, os.path.join(rec.ROOT, "sim", "mjx")); sys.path.insert(0, os.path.join(rec.ROOT, "sim"))
RUN = os.path.join(rec.MAIN, "sim/runs/robot_legs_walk14_s1_s128r24")
NJ, FW, H = 12, 55, 3


def frames(r, n, prev_actions, variant=None):
    """One 55-wide frame per tick (obs/assembler.cpp)."""
    F = np.zeros((n, FW), np.float32)
    F[:, 0:12] = r["q"][:n]
    F[:, 12:24] = r["dq_reg"][:n]
    F[:, 24:27] = r["up"][:n]
    F[:, 30:33] = r["gyro"][:n]
    F[:, 33:45] = prev_actions
    F[:, 46] = np.sin(r["phase"][:n]); F[:, 47] = np.cos(r["phase"][:n])
    F[:, 48:55] = r["cmd"][:n]
    if variant:
        variant(F)
    return F


def stack(F):
    n = len(F)
    O = np.zeros((n, FW * H), np.float32)
    for i in range(n):
        for h in range(H):
            O[i, h * FW:(h + 1) * FW] = F[max(i - h, 0)]
    return O


def load_act():
    import eval_precision as EP
    return EP.load_policy(RUN, FW * H, NJ)


def main():
    r = rec.load()
    n = int(np.sum(r["link"] == 1))
    a_fw = rec.angles_to_action(r["nom"][:n])
    prev = np.vstack([np.zeros((1, NJ)), a_fw[:-1]])
    act = load_act()
    O = stack(frames(r, n, prev))
    a_py = np.array([act(o) for o in O])
    # what the nominal quantisation alone allows: half a step over the span
    span = np.where(a_fw >= 0, rec.HI, -rec.LO)
    qtol = 0.5 * rec.RAD / span
    err = np.abs(np.clip(a_py, -1, 1) - a_fw)
    t = r["t"][:n] - r["t"][0]
    print(f"ticks {n}; |python - firmware| action: max {err.max():.2e} mean {err.mean():.2e}; "
          f"quantisation bound max {qtol.max():.2e}")
    print("worst tick", np.unravel_index(err.argmax(), err.shape), "per-joint max", np.round(err.max(0), 4))
    bad = (err > qtol * 1.01 + 1e-4).sum()
    print("entries above the tick-quantisation bound:", bad, "of", err.size)
    np.savez(os.path.join(HERE, "fidelity.npz"), t=t, a_fw=a_fw, a_py=a_py, err=err)
    # sensitivity: how different would the action be under plausible wiring bugs?
    vars_ = {
        "dq = finite difference (alpha .5)": None,
        "gyro sign flipped": lambda F: F.__setitem__((slice(None), slice(30, 33)), -F[:, 30:33]),
        "up x,y flipped": lambda F: F.__setitem__((slice(None), slice(24, 26)), -F[:, 24:26]),
        "dq zeroed": lambda F: F.__setitem__((slice(None), slice(12, 24)), 0),
    }
    out = {}
    for k, v in vars_.items():
        if v is None:
            fd = np.zeros((n, NJ)); est = np.zeros(NJ)
            for i in range(1, n):
                est += 0.5 * ((r["q"][i] - r["q"][i - 1]) / 0.02 - est); fd[i] = est
            def v(F, fd=fd): F[:, 12:24] = fd
        a2 = np.array([act(o) for o in stack(frames(r, n, prev, v))])
        d = np.abs(a2 - a_py)
        out[k] = (float(d[:150].mean()), float(d[:150].max()))
        print(f"  {k:36s} open-loop action change in the first 3 s: mean {d[:150].mean():.3f} max {d[:150].max():.3f}")
    json.dump(dict(n=n, max_err=float(err.max()), mean_err=float(err.mean()), qtol=float(qtol.max()),
                   above_bound=int(bad), sensitivity=out), open(os.path.join(HERE, "fidelity.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
