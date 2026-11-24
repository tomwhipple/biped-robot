"""x1.5 stand-probe traces (#263 margin diagnosis, 2026-11-24). Runs grade.py's episode() for the
x1.5 probe (or any out_gain) and keeps the per-tick traces grade.py throws away.

  python trace_x15.py OUT.npz RUN GAIN SEED [RUN_S]

XGROUP=yaw|roll|pitch|noyaw|noroll|... applies GAIN to that joint group only (the rest at 1);
GYRO_SCALE=z=0.5 (x=, y=) scales that gyro axis in every obs frame the policy reads.

Env as grade.py (GRADE_XML, GRADE_PLANT_TERMS, SHAPE_HZ). Read-only on the runs."""
import os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "experiments", "policy", "shaper-train"))
import grade as G  # noqa: E402

_CAP = {}
_loop0 = G._episode_loop


def _loop(env, *a, **k):
    res = _loop0(env, *a, **k)
    GO, GY, PW, TL, SQ, X, fell, ext = res
    _CAP.update(GO=np.array(GO), GY=np.array(GY), TL=np.array(TL), X=np.array(X), fell=fell,
                **{n: np.array(ext[n]) for n in ("YW", "QM", "QD", "AC", "Y", "FT", "RL")})
    _CAP["default"] = np.array(env._default, float)
    return res


G._episode_loop = _loop

# GYRO_SCALE="z=0.5" (or x=, y=): scale that gyro axis in every obs frame the policy reads (a
# firmware-side observation gain, as obsfreeze gain=; the plant and the guard are untouched)
_build0 = G.simh.build


def _build(*a, **k):
    env, act, zoff = _build0(*a, **k)
    spec = os.environ.get("GYRO_SCALE")
    if not spec:
        return env, act, zoff
    nj = env._nq_act
    F = env.obs_frame
    ax, _, val = spec.partition("=")
    idx = 2 * nj + 6 + "xyz".index(ax)

    def act_s(obs):
        o = np.array(obs, dtype=np.float32, copy=True)
        o[idx::F] *= float(val)
        return act(o)
    return env, act_s, zoff


G.simh.build = _build


GROUPS = dict(yaw=(0, 6), roll=(1, 5, 7, 11), pitch=(2, 3, 4, 8, 9, 10),
              noyaw=(1, 2, 3, 4, 5, 7, 8, 9, 10, 11), noroll=(0, 2, 3, 4, 6, 8, 9, 10),
              hiproll=(1, 7), ankleroll=(5, 11), all=tuple(range(12)))


class VecGain(np.ndarray):
    """A per-joint out_gain that grade's `out_gain != 1.0` test reads as True."""
    def __ne__(self, other):
        return True


def gain_of(g):
    grp = os.environ.get("XGROUP")
    if not grp:
        return g
    v = np.ones(12); v[list(GROUPS[grp])] = g
    return v.view(VecGain)


def main():
    out, run, gain, seed = sys.argv[1], sys.argv[2], float(sys.argv[3]), int(sys.argv[4])
    run_s = float(sys.argv[5]) if len(sys.argv) > 5 else 20.0
    hz = float(os.environ.get("SHAPE_HZ", "8"))
    spec = dict(run=run, shaper_hz=hz, stand_phase=G.P2, clock="coast", out_gain=gain_of(gain))
    m = G.episode(spec, seed=seed, run_s=run_s)
    _CAP["AC"] = np.asarray(_CAP["AC"])
    np.savez_compressed(out, run=run, gain=gain, seed=seed, names=np.array(G.rec.POL),
                        trip=str(m.get("trip")), t_end=m["t_end"], **_CAP)
    print(out, "fell", m["fell"], "t_end", m["t_end"], "trip", m.get("trip"), flush=True)


if __name__ == "__main__":
    main()
