"""Quick floor-contact grid against the hardware roll sweep: which floor (contact time
constant, damping ratio) and roll-play model reproduce the robot's ~3 Hz body-roll
resonance? Score = log-amplitude + phase error of body roll per hip goal, 0.5-4 Hz."""
import json, os, sys, itertools
from concurrent.futures import ProcessPoolExecutor
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)


def job(a):
    tc, dr, lash = a
    import sweep_sim as SS, simh
    model = dict(floor_tc=tc, floor_dr=dr)
    if lash == "none":
        model.update(roll_lash_deg=0.0, backlash_deg=0.0)
    rs, t0 = SS.run("fit", simh.Servo(lag_hz=0), model)
    return a, SS.response(rs, t0)


def score(res, real):
    e = 0.0
    for s, r in zip(res, real):
        a = np.log(max(s["body_roll"]["per_goal"], 1e-3) / r["body_roll"]["per_goal"])
        p = np.radians(((s["body_roll"]["phase_deg"] - r["body_roll"]["phase_deg"] + 180) % 360) - 180)
        e += a * a + 0.5 * p * p
    return float(e / len(res))


if __name__ == "__main__":
    real = json.load(open(os.path.join(HERE, "sweep_real.json")))
    grid = list(itertools.product([0.004, 0.01, 0.02, 0.0334], [1.0, 0.3, 0.1], ["fit", "none"]))
    with ProcessPoolExecutor(12) as ex:
        out = list(ex.map(job, grid))
    rows = []
    for (tc, dr, lash), res in out:
        rows.append(dict(tc=tc, dr=dr, lash=lash, score=score(res, real),
                         roll=[round(x["body_roll"]["per_goal"], 2) for x in res],
                         phase=[round(x["body_roll"]["phase_deg"]) for x in res],
                         hip_gain=[round(x["L_hip_roll"]["gain"], 2) for x in res]))
    rows.sort(key=lambda r: r["score"])
    print("real roll/goal", [round(x["body_roll"]["per_goal"], 2) for x in real], [round(x["body_roll"]["phase_deg"]) for x in real])
    for r in rows:
        print(f"tc {r['tc']:.4f} dr {r['dr']:.1f} lash {r['lash']:4s} score {r['score']:.2f} roll {r['roll']} ph {r['phase']} hipgain {r['hip_gain']}")
    json.dump(rows, open(os.path.join(HERE, "floor_grid.json"), "w"), indent=1)
