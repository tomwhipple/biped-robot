"""Fit an actuator model to the 2026-11-01 double-support roll sweep, joint level AND body
level jointly: replay the robot's recorded goals into the fitted plant with the servo's
position loop as a dynamic element (stiffness kp and damping kd scaled from the 'asbuilt'
model, Coulomb friction, play as a deadzone on the error, extra latency for the speed-limited
goal write, first-order set-point lag), and score the complex present/goal ratios of the four
roll joints and the body-roll/goal ratio, 0.5-4 Hz.

  CUDA_VISIBLE_DEVICES= JAX_PLATFORMS=cpu python actuator_fit.py [n_random] [n_refine]
"""
import json, math, os, sys
from concurrent.futures import ProcessPoolExecutor
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)

# name: (lo, hi, log)
SPACE = {
    "kp": (0.08, 1.2, True),        # x the asbuilt stiffness (34 N*m/rad STS3215, 48 STS3250)
    "kd": (0.03, 1.5, True),        # x the asbuilt damping
    "fl": (0.02, 0.6, True),        # joint Coulomb friction, N*m (training: 0.35)
    "lash": (0.0, 2.5, False),      # roll-joint play, deg total (deadzone on the error)
    "lat": (0.0, 30.0, False),      # ms of target latency (speed-limited goal write)
}
ALPHAS = [0.0, 0.2, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
ROLLS = ["L_hip_roll", "R_hip_roll", "L_ankle_roll", "R_ankle_roll"]


def model_of(p):
    return dict(kp_scale=p["kp"], kd_scale=p["kd"], frictionloss=p["fl"],
                roll_lash_deg=p["lash"], backlash_deg=0.0, latency_ms=p["lat"])


def job(p):
    import sweep_sim as SS, simh
    try:
        rs, t0 = SS.run("fit", simh.Servo(), model_of(p))
        real = json.load(open(os.path.join(HERE, "sweep_real.json")))
        best = None
        for a in ALPHAS:
            r = SS.response(rs, t0, a)
            sc = score(r, real)
            if best is None or sc < best[0]:
                best = (sc, a, r)
        p = dict(p, alpha=best[1])
        return p, best[2]
    except Exception as e:      # noqa: BLE001
        return p, None


def cplx(g, ph):
    return g * np.exp(1j * np.radians(ph))


def score(res, real, w_body=1.0):
    if res is None:
        return 99.0
    e, n = 0.0, 0
    for s, r in zip(res, real):
        for k in ROLLS:
            if r["f"] == 1.5 and k == "R_ankle_roll":
                continue                 # the robot's 3.09 outlier (a slipped read)
            a = math.log(max(s[k]["gain"], 1e-3) / r[k]["gain"])
            d = math.radians(((s[k]["phase_deg"] - r[k]["phase_deg"] + 180) % 360) - 180)
            e += a * a + 0.5 * d * d; n += 1
        a = math.log(max(s["body_roll"]["per_goal"], 1e-3) / r["body_roll"]["per_goal"])
        d = math.radians(((s["body_roll"]["phase_deg"] - r["body_roll"]["phase_deg"] + 180) % 360) - 180)
        e += w_body * 4 * (a * a + 0.5 * d * d); n += w_body * 4
    return e / n


def sample(rng, center=None, scale=1.0):
    p = {}
    for k, (lo, hi, lg) in SPACE.items():
        if center is None:
            u = rng.uniform()
        else:
            c = center[k]
            c = min(max(c, lo), hi)
            u0 = (math.log(c / lo) / math.log(hi / lo)) if lg else (c - lo) / (hi - lo)
            u = float(np.clip(u0 + rng.normal(0, 0.12 * scale), 0, 1))
        p[k] = float(lo * (hi / lo) ** u) if lg else float(lo + u * (hi - lo))
    return p


def main():
    real = json.load(open(os.path.join(HERE, "sweep_real.json")))
    n0 = int(sys.argv[1]) if len(sys.argv) > 1 else 84
    n1 = int(sys.argv[2]) if len(sys.argv) > 2 else 42
    rng = np.random.default_rng(7)
    base = [dict(kp=1.0, kd=1.0, fl=0.35, lash=0.0, lat=0.0), dict(kp=1.0, kd=1.0, fl=0.35, lash=1.55, lat=0.0)]
    bkeys = [tuple(sorted(b.items())) for b in base]
    pts = base + [sample(rng) for _ in range(n0)]
    allr = []
    with ProcessPoolExecutor(int(os.environ.get("JOBS", 14))) as ex:
        allr += [(p, r, score(r, real)) for p, r in ex.map(job, pts)]
        for rnd, sc in ((1, 1.0), (2, 0.5), (3, 0.25)):
            allr.sort(key=lambda x: x[2])
            top = [x[0] for x in allr[:6]]
            pts = [sample(rng, top[i % len(top)], sc) for i in range(n1)]
            allr += [(p, r, score(r, real)) for p, r in ex.map(job, pts)]
            print(f"round {rnd}: best {allr[0][2]:.3f} {allr[0][0]}", flush=True)
    allr.sort(key=lambda x: x[2])
    out = [dict(params=p, score=s, response=r) for p, r, s in allr]
    json.dump(dict(best=out[0], top=out[:20], baseline=[x for x in out if tuple(sorted((k, v) for k, v in x["params"].items() if k != "alpha")) in bkeys]),
              open(os.path.join(HERE, "actuator_fit.json"), "w"), indent=1)
    for x in out[:10]:
        p = x["params"]; r = x["response"]
        print(f"score {x['score']:.3f} kp {p['kp']:.2f} kd {p['kd']:.2f} fl {p['fl']:.2f} lash {p['lash']:.2f} lat {p['lat']:.0f} a {p.get('alpha',0):.1f} | "
              f"roll/goal {[round(v['body_roll']['per_goal'], 2) for v in r]} | Lhip {[round(v['L_hip_roll']['gain'], 2) for v in r]}")
    for x in out:
        if tuple(sorted((k, v) for k, v in x["params"].items() if k != "alpha")) in bkeys:
            print("baseline", x["params"], f"score {x['score']:.3f}")


if __name__ == "__main__":
    main()
