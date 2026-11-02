"""Joint actuator fit: the robot's loaded step responses (stepresp) AND the double-support roll
sweep (joint + body level), one model: plant stiffness/damping/friction scales + per-servo-type
dead time and set-point lag (TypedServo). Random search + 3 shrinking refinement rounds."""
import json, math, os, sys
from concurrent.futures import ProcessPoolExecutor
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import actuator_fit as AF

SPACE = {
    "kp": (0.2, 1.5, True), "kd": (0.02, 1.5, True), "fl": (0.02, 0.5, True),
    "dead15": (20.0, 100.0, False), "tau15": (2.0, 80.0, True),
    "dead50": (0.0, 30.0, False), "tau50": (2.0, 40.0, True),
}
AF.SPACE = SPACE


def servo_kw(p):
    return dict(dead15=p["dead15"], tau15=p["tau15"], dead50=p["dead50"], tau50=p["tau50"])


def model_of(p):
    return dict(kp_scale=p["kp"], kd_scale=p["kd"], frictionloss=p["fl"], roll_lash_deg=0.0, backlash_deg=0.0)


def step_score(st):
    import sim_stepresp as SR
    e = 0.0
    for j, (t10, t90, fin) in SR.TARGET.items():
        s = st[j]
        a = (s["t10_ms"] - t10) / 20.0 if math.isfinite(s["t10_ms"]) else 5.0
        b = (s["t90_ms"] - t90) / 40.0 if math.isfinite(s["t90_ms"]) else 5.0
        c = math.log(max(s["final"], 0.05) / fin) / 0.2
        e += a * a + b * b + c * c
    return e / (3 * len(SR.TARGET))


def job(p):
    import sweep_sim as SS, sim_stepresp as SR, simh
    try:
        st = {j: SR.run(j, servo_kw(p), model_of(p)) for j in SR.STEP_JOINTS}
        rs, t0 = SS.run("fit", simh.TypedServo(**servo_kw(p)), model_of(p))
        real = json.load(open(os.path.join(HERE, "sweep_real.json")))
        r = SS.response(rs, t0)
        return p, dict(step=st, sweep=r, s_step=step_score(st), s_sweep=AF.score(r, real))
    except Exception as e:   # noqa: BLE001
        return p, None


def total(r):
    return 99.0 if r is None else r["s_step"] + r["s_sweep"]


def main():
    n0 = int(sys.argv[1]) if len(sys.argv) > 1 else 84
    n1 = int(sys.argv[2]) if len(sys.argv) > 2 else 42
    rng = np.random.default_rng(11)
    base = [dict(kp=1.0, kd=1.0, fl=0.35, dead15=0.0, tau15=2.0, dead50=0.0, tau50=2.0)]
    pts = base + [AF.sample(rng) for _ in range(n0)]
    allr = []
    with ProcessPoolExecutor(int(os.environ.get("JOBS", 14))) as ex:
        allr += [(p, r, total(r)) for p, r in ex.map(job, pts)]
        for rnd, sc in ((1, 1.0), (2, 0.5), (3, 0.25)):
            allr.sort(key=lambda x: x[2])
            top = [x[0] for x in allr[:6]]
            pts = [AF.sample(rng, top[i % len(top)], sc) for i in range(n1)]
            allr += [(p, r, total(r)) for p, r in ex.map(job, pts)]
            print(f"round {rnd}: best {allr[0][2]:.3f} {allr[0][0]}", flush=True)
    allr.sort(key=lambda x: x[2])
    json.dump(dict(best=dict(params=allr[0][0], result=allr[0][1], score=allr[0][2]),
                   top=[dict(params=p, result=r, score=s) for p, r, s in allr[:20]],
                   baseline=[dict(params=p, result=r, score=s) for p, r, s in allr if p in base]),
              open(os.path.join(HERE, "joint_fit.json"), "w"), indent=1, default=float)
    for p, r, s in allr[:8]:
        print(f"total {s:.3f} (step {r['s_step']:.2f} sweep {r['s_sweep']:.2f}) " +
              " ".join(f"{k} {v:.3g}" for k, v in p.items()) +
              f" | roll/goal {[round(v['body_roll']['per_goal'], 2) for v in r['sweep']]}")
    for p, r, s in allr:
        if p in base:
            print(f"BASELINE total {s:.3f} (step {r['s_step']:.2f} sweep {r['s_sweep']:.2f})")


if __name__ == "__main__":
    main()
