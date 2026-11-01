"""Which ingredient makes the student flail? One plant knob at a time on the fitted and
training plants, seeds in parallel."""
import json, os, sys
from concurrent.futures import ProcessPoolExecutor
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)

VARIANTS = {
    "base (shaper 10 + servo lag 12 Hz)": dict(servo=dict(lag_hz=12)),
    "servo dead time 1 tick": dict(servo=dict(lag_hz=12, delay=1)),
    "servo dead time 2 ticks": dict(servo=dict(lag_hz=12, delay=2)),
    "servo dead time 3 ticks": dict(servo=dict(lag_hz=12, delay=3)),
    "servo lag 4 Hz": dict(servo=dict(lag_hz=4)),
    "servo lag 2 Hz": dict(servo=dict(lag_hz=2)),
    "set-point hysteresis 1 deg": dict(servo=dict(lag_hz=12, db_deg=1.0)),
    "set-point hysteresis 2 deg": dict(servo=dict(lag_hz=12, db_deg=2.0)),
    "no roll lash (encoder follows)": dict(servo=dict(lag_hz=12), model=dict(roll_lash_deg=0.0, backlash_deg=0.0)),
    "roll lash 3 deg": dict(servo=dict(lag_hz=12), model=dict(roll_lash_deg=3.0)),
    "hard floor tc 5 ms": dict(servo=dict(lag_hz=12), model=dict(floor_tc=0.005)),
    "hard floor tc 2 ms": dict(servo=dict(lag_hz=12), model=dict(floor_tc=0.002)),
    "hard floor 2 ms + no lash": dict(servo=dict(lag_hz=12), model=dict(floor_tc=0.002, roll_lash_deg=0.0, backlash_deg=0.0)),
    "sensor 1 tick stale": dict(servo=dict(lag_hz=12), sensor_delay=1),
    "IMU noise x3": dict(servo=dict(lag_hz=12), imu_noise_x=3.0),
    "shaper off": dict(servo=dict(lag_hz=12), shaper_hz=0.0),
}


DQ_Q = 50 * 2 * np.pi / 4096          # register 58's real quantum: 50 steps/s


def obs_hook(spec):
    """Sensor-side models applied to the policy's obs (all 3 frames)."""
    if not spec:
        return None
    def h(o, env):
        for k in range(3):
            b = k * 55
            if spec.get("dq_quant"):
                o[b + 12:b + 24] = np.round(o[b + 12:b + 24] / DQ_Q) * DQ_Q
            if spec.get("gyro_gain"):
                o[b + 30:b + 33] *= spec["gyro_gain"]
    return h


def job(args):
    name, plant, seed, v = args
    import takeover as T, simh
    sv = simh.TypedServo(**v["typed"]) if "typed" in v else simh.Servo(**v.get("servo", {}))
    m = T.episode(plant, seed, sv, v.get("model"), shaper_hz=v.get("shaper_hz", 10.0),
                  sensor_delay=v.get("sensor_delay", 0), imu_noise_x=v.get("imu_noise_x", 1.0),
                  obs_hook=obs_hook(v.get("obs")), run=v.get("run", "robot_legs_walk14_s1_s128r24"),
                  out_gain=v.get("out_gain", 1.0), push_mps=v.get("push", 0.0))
    m["variant"] = name
    return m


def run(variants, plants, seeds, jobs=8, out=None):
    js = [(k, p, s, v) for k, v in variants.items() for p in plants for s in range(seeds)]
    with ProcessPoolExecutor(jobs) as ex:
        res = list(ex.map(job, js))
    rows = []
    for k in variants:
        for p in plants:
            e = [r for r in res if r["variant"] == k and r["plant"] == p]
            if not e:
                continue
            f = lambda key: float(np.mean([x[key] for x in e]))
            rows.append(dict(variant=k, plant=p, n=len(e), flail=sum(x["flail"] for x in e),
                             falls=sum(x["fell"] for x in e), arate_1_2=f("arate_1_2"), arate_2_3=f("arate_2_3"),
                             arate_all=f("arate_all"), sat=f("sat_frac"), gx_hf_2_3=f("gx_hf_2_3"),
                             tilt=f("tilt_max")))
    if out:
        json.dump(dict(rows=rows, episodes=res), open(out, "w"), indent=1, default=float)
    return rows


def table(rows, real=None):
    L = ["| variant | plant | flail | falls | act-rate 1-2 s | 2-3 s | whole run | sat frac | gyro-x 6-13 Hz 2-3 s | tilt max |",
         "|---|---|---|---|---|---|---|---|---|---|"]
    if real:
        L.append(f"| **robot, 2026-11-01** | real | yes | (tripped) | {real['arate_1_2']:.3f} | {real['arate_2_3']:.3f} | "
                 f"{real['arate_all']:.3f} | {real['sat_frac']:.3f} | {real['gx_hf_2_3']:.3f} | {real['tilt_max']:.0f} |")
    for r in rows:
        L.append(f"| {r['variant']} | {r['plant']} | {r['flail']}/{r['n']} | {r['falls']}/{r['n']} | {r['arate_1_2']:.3f} | "
                 f"{r['arate_2_3']:.3f} | {r['arate_all']:.3f} | {r['sat']:.3f} | {r['gx_hf_2_3']:.3f} | {r['tilt']:.1f} |")
    return "\n".join(L)


if __name__ == "__main__":
    import takeover as T
    real, _ = T.real_metrics()
    seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 4
    rows = run(VARIANTS, ["fit", "train"], seeds, jobs=int(os.environ.get("JOBS", 10)),
               out=os.path.join(HERE, "ingredients.json"))
    print(table(rows, real))
