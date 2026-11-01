"""Replay the hardware roll sweep's recorded goals into the sim plant(s) and compare the
present/goal response and the body roll with the robot's (sweep_fit.py)."""
import json, math, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import rec, simh, sweep_fit as SF


def run(plant="fit", servo=None, model=None, quiet=False):
    r = rec.load(rec.SWEEP, source="pose")
    _, t0 = SF.analyse(r)
    t = r["t"]
    k0 = np.searchsorted(t, t0) - 25
    k1 = np.searchsorted(t, t0 + sum(max(3.0, 4.0 / f) + 0.8 for f in SF.FREQS) + 0.5)
    env, act, zoff = simh.build(plant, ep_s=50.0)
    if plant.startswith("fit"):
        env.domain_rand = False
    env.reset(seed=0)
    if plant.startswith("fit"):
        env._zero_off = zoff.copy()
    if model:
        simh.apply_model(env, model)
    simh.place_and_hold(env, r["goal"][k0])
    sv = servo or simh.Servo()
    sv.reset(r["goal"][k0])
    cur = {"g": r["goal"][k0]}
    env._action_to_ctrl = lambda a: cur["g"]
    out_q, out_roll = [], []
    for k in range(k0, k1):
        cur["g"] = sv(r["goal"][k])
        env.step(np.zeros(12, np.float32))
        d = env.data
        q = np.asarray(d.qpos[env._jqpos]) - (env._zero_off if env._zero_off is not None else 0)
        out_q.append(q); out_roll.append(simh.sim_roll_pitch(d)[0])
    rs = dict(t=t[k0:k1], goal=r["goal"][k0:k1], q=np.array(out_q), roll=np.degrees(np.array(out_roll)))
    res, _ = SF.analyse_arrays(rs, t0) if hasattr(SF, "analyse_arrays") else (None, None)
    return rs, t0


def response(rs, t0, alpha=0.0):
    """alpha: series compliance downstream of the encoder -- the encoder reads the servo
    shaft, q_enc = q_link + alpha * (target - q_link) (alpha = kp_servo / (kp_servo + k_struct))."""
    j = [rec.POL.index(n) for n in SF.ROLLS]
    if alpha:
        rs = dict(rs, q=rs["q"] + alpha * (rs["goal"] - rs["q"]))
    out = []
    for f, a, b in SF.segments():
        m = (rs["t"] >= t0 + a) & (rs["t"] <= t0 + b)
        tt = rs["t"][m] - t0
        row = dict(f=f)
        for n, jj in zip(SF.ROLLS, j):
            G = SF.fit_sine(tt, rs["goal"][m, jj], f); P = SF.fit_sine(tt, rs["q"][m, jj], f)
            row[n] = dict(gain=float(abs(P / G)), phase_deg=float(np.degrees(np.angle(P / G))))
        Gm = np.mean([SF.fit_sine(tt, rs["goal"][m, jj], f) for jj in j[:2]])
        R = SF.fit_sine(tt, np.radians(rs["roll"][m]), f)
        row["body_roll"] = dict(deg=float(np.degrees(abs(R))), per_goal=float(abs(R / Gm)),
                                phase_deg=float(np.degrees(np.angle(R / Gm))))
        out.append(row)
    return out


def show(label, res):
    print(label)
    for row in res:
        s = f"  {row['f']:3.1f} Hz "
        for n in SF.ROLLS:
            v = row[n]; s += f"| {n[:6]}{n[-4:]} {v['gain']:.2f} {v['phase_deg']:5.0f} "
        v = row["body_roll"]; s += f"| roll {v['deg']:.2f} deg ph {v['phase_deg']:5.0f}"
        print(s)


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--plant", default="fit")
    p.add_argument("--delay", type=int, default=0)
    p.add_argument("--lag", type=float, default=0.0)
    p.add_argument("--db", type=float, default=0.0)
    p.add_argument("--floor-tc", type=float, default=None)
    p.add_argument("--latency", type=float, default=None)
    p.add_argument("--roll-lash", type=float, default=None)
    p.add_argument("--backlash", type=float, default=None)
    a = p.parse_args()
    rs, t0 = run(a.plant, simh.Servo(a.delay, a.lag, a.db),
                 dict(floor_tc=a.floor_tc, latency_ms=a.latency, roll_lash_deg=a.roll_lash, backlash_deg=a.backlash))
    show(f"sim {a.plant} delay {a.delay} lag {a.lag} db {a.db} floor_tc {a.floor_tc} rlash {a.roll_lash} lash {a.backlash}", response(rs, t0))
