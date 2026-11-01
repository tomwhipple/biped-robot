"""Open-loop plant check: feed the firmware's RECORDED goals (after shaper + ramp) of the
policy stand into the sim plant, tick for tick, and compare the sim's encoders / gyro with
the robot's. Same goals in -> does the plant answer the same?"""
import json, math, os, sys
import numpy as np
from scipy.signal import butter, filtfilt
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import rec, simh

BANDS = [(0.3, 2.5), (2.5, 7.0), (7.0, 13.0)]


def replay(plant="fit", model=None, servo=None, k1=165, seed=0):
    r = rec.load()
    env, act, zoff = simh.build(plant, ep_s=8.0)
    if plant.startswith("fit"):
        env.domain_rand = False
    env.push_prob = 0.0
    env.reset(seed=seed)
    if plant.startswith("fit"):
        env._zero_off = zoff.copy()
    env.domain_rand = False
    if model:
        simh.apply_model(env, model)
    simh.place_and_hold(env, r["q"][0], 1.5)
    sv = servo or simh.Servo(); sv.reset(r["q"][0])
    cur = {"g": r["q"][0]}
    env._action_to_ctrl = lambda a: cur["g"]
    Q, GY, TL = [], [], []
    for k in range(k1):
        cur["g"] = sv(r["goal"][k])
        obs, _, term, _, info = env.step(np.zeros(12, np.float32))
        d = env.data
        Q.append(np.asarray(d.qpos[env._jqpos]) - (env._zero_off if env._zero_off is not None else 0))
        GY.append(np.asarray(d.qvel[3:6]).copy())
        TL.append(math.degrees(math.acos(max(-1, min(1, float(info["up_z"]))))))
        if term:
            break
    return dict(q=np.array(Q), gyro=np.array(GY), tilt=np.array(TL)), r


def band_rms(x, lo, hi, s):
    b, a = butter(2, [lo / 25, min(hi / 25, 0.99)], "band")
    return np.sqrt(np.mean(filtfilt(b, a, x, axis=0)[s] ** 2, axis=0))


def compare(sim, r, lo=50, hi=160):
    n = min(len(sim["q"]), hi)
    s = slice(lo, n)
    out = {}
    for bl, bh in BANDS:
        gr = band_rms(r["gyro"][:n], bl, bh, s); gs = band_rms(sim["gyro"][:n], bl, bh, s)
        qr = np.degrees(band_rms(r["q"][:n], bl, bh, s)); qs = np.degrees(band_rms(sim["q"][:n], bl, bh, s))
        out[f"{bl}-{bh}Hz"] = dict(gyro_real=gr.tolist(), gyro_sim=gs.tolist(),
                                   q_real_mean=float(qr.mean()), q_sim_mean=float(qs.mean()))
    return out


def show(label, c):
    print(label)
    for k, v in c.items():
        print(f"  {k:9s} gyro rms x/y/z real {np.round(v['gyro_real'], 3)} sim {np.round(v['gyro_sim'], 3)} | "
              f"q rms deg real {v['q_real_mean']:.2f} sim {v['q_sim_mean']:.2f}")


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--plant", default="fit")
    p.add_argument("--floor-tc", type=float); p.add_argument("--friction", type=float)
    p.add_argument("--roll-lash", type=float); p.add_argument("--backlash", type=float)
    p.add_argument("--lag", type=float, default=12.0); p.add_argument("--delay", type=int, default=0)
    a = p.parse_args()
    sim, r = replay(a.plant, dict(floor_tc=a.floor_tc, friction=a.friction, roll_lash_deg=a.roll_lash,
                                  backlash_deg=a.backlash), simh.Servo(a.delay, a.lag))
    show(f"{a.plant} floor_tc {a.floor_tc} mu {a.friction} rlash {a.roll_lash} lash {a.backlash} lag {a.lag} delay {a.delay}"
         f" (1.0-3.2 s)", compare(sim, r))
