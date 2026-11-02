"""Stand-still metrics, computed the same way on the robot recording and in sim.

jitter    per-joint std of the WRITTEN goal (deg) after a 0.5 Hz high-pass, and its
          band rms at the gait clock (1.5 Hz) and its harmonic (3 Hz)
body      body angular-rate std per axis (rad/s; x roll, y pitch, z yaw) and band rms
power     the robot: sum over servos of |present current| x bus voltage (W, motor
          current only -- the register is a magnitude, 6.5 mA/LSB). The sim: walker_env's
          info["power_w"] proxy (positive mechanical work + copper loss K_CU tau^2).
"""
from __future__ import annotations
import os, sys
import numpy as np
from scipy.signal import butter, filtfilt, welch

HERE = os.path.dirname(os.path.abspath(__file__))
RCA = os.path.join(HERE, "..", "rca")
sys.path.insert(0, RCA)
import rec  # noqa: E402

STAND2 = os.path.join(rec.MAIN, "hw_sessions/2026-11-02/flight_20261102T160708Z_policy_stand2.frec")
FS = 50.0
BANDS = (1.5, 3.0)


def _band_rms(x, fc, half=0.2):
    n = min(len(x), 1024)
    f, P = welch(x, fs=FS, nperseg=n, axis=0)
    m = (f > fc - half) & (f < fc + half)
    return np.sqrt(P[m].sum(0) * (f[1] - f[0]))


def jitter(goal, gyro):
    """goal (n,12) rad policy order; gyro (n,3) rad/s body frame. Returns a flat dict."""
    b, a = butter(2, 0.5 / (FS / 2), "high")
    gd = np.degrees(goal)
    hp = filtfilt(b, a, gd, axis=0) if len(gd) > 30 else gd - gd.mean(0)
    sd = hp.std(0)
    out = dict(goal_rms_mean=float(sd.mean()), goal_rms_max=float(sd.max()),
               goal_rms_max_joint=rec.POL[int(np.argmax(sd))],
               goal_rms=dict(zip(rec.POL, np.round(sd, 3).tolist())))
    for fc in BANDS:
        out[f"goal_{fc:g}hz_mean"] = float(_band_rms(gd - gd.mean(0), fc).mean())
    gy = np.asarray(gyro) - np.mean(gyro, 0)
    for i, ax in enumerate("xyz"):
        out[f"rate_{ax}"] = float(gy[:, i].std())
        for fc in BANDS:
            out[f"rate_{ax}_{fc:g}hz"] = float(_band_rms(gy[:, i], fc))
    return out


# the armed stand: `run` 16:07:41.04, soft STOP 16:09:46.93 UTC (boot 6bfabd9f)
T_ARM, T_STOP = 1793635661.04, 1793635786.93
BUS_V = 12.0          # `volt` before the run: 12.0 V, mean of 12 servos


def _window(r, t0, t1):
    k = (r["t"] >= t0) & (r["t"] <= t1)
    return {key: v[k] for key, v in r.items() if len(v) == len(k)}


def armed_segment(path, t_after):
    """(t_first, t_last) of the contiguous armed (source=ctrl) run starting after t_after."""
    r = rec.load(path)
    t = r["t"][r["t"] >= t_after]
    if not len(t):
        raise ValueError(f"no armed ticks after {t_after} in {path}")
    gap = np.where(np.diff(t) > 0.5)[0]
    return float(t[0]), float(t[gap[0]] if len(gap) else t[-1])


def robot(path=STAND2, skip_s=5.0, t_arm=T_ARM, t_stop=T_STOP, last_s=None):
    """A recorded armed stand: jitter + power over [t_arm + skip_s, t_stop] (or its last last_s)."""
    r = _window(rec.load(path), t_arm, t_stop)
    s = slice(int(skip_s * FS), None) if last_s is None else slice(-int(last_s * FS), None)
    m = jitter(r["goal"][s], r["gyro"][s])
    m.update(robot_power(path, skip_s, t_arm, t_stop, last_s))
    m["duration_s"] = float(r["t"][-1] - r["t"][0])
    return m


def robot_power(path=STAND2, skip_s=5.0, t_arm=T_ARM, t_stop=T_STOP, last_s=None):
    """Motor power, W: armed, the soft STOP's static HOLD right after (same pose, torque on,
    policy off -- 2 s, until autohome) and limp (released, 45-55 s after the stop)."""
    P = lambda r: np.abs(r["current"]) * 6.5e-3 * BUS_V
    t0 = t_arm + skip_s if last_s is None else t_stop - last_s
    arm = _window(rec.load(path), t0, t_stop)
    idle = rec.load(path, source="idle")
    hold = _window(idle, t_stop + 0.02, t_stop + 2.05)
    limp = _window(idle, t_stop + 45.0, t_stop + 55.0)
    pa = P(arm)
    mean = lambda x: float(P(x).sum(1).mean()) if len(x["t"]) else float("nan")
    return dict(power_w=float(pa.sum(1).mean()), power_hold_w=mean(hold), power_limp_w=mean(limp),
                power_by_joint=dict(zip(rec.POL, np.round(pa.mean(0), 3).tolist())),
                power_hold_by_joint=dict(zip(rec.POL, np.round(P(hold).mean(0), 3).tolist()))
                if len(hold["t"]) else {}, n_hold=int(len(hold["t"])))


STAND3_FAST = os.path.join(rec.MAIN, "hw_sessions/2026-11-02/flight_20261102T162925Z_policy_stand3_fast.frec")
T3_ARM = 1793637078.9      # `run` with obsfreeze gain=0.5 + shape 6, fast STS3215 (reg 85 = 0, 81 = 1)


if __name__ == "__main__":
    import json
    out = {"stand2 (factory servos, knobs)": robot()}
    a, b = armed_segment(STAND3_FAST, T3_ARM)
    out["stand3_fast (fast servos, knobs), last 10 s"] = robot(STAND3_FAST, t_arm=a, t_stop=b, last_s=10.0)
    out["stand3_fast (fast servos, knobs), from 5 s"] = robot(STAND3_FAST, t_arm=a, t_stop=b)
    out["stand2 (factory servos, knobs), last 10 s"] = robot(last_s=10.0)
    json.dump(out, open(os.path.join(HERE, "robot_baseline.json"), "w"), indent=1)
    for k, m in out.items():
        print(f"{k}: jitter {m['goal_rms_mean']:.3f}/{m['goal_rms_max']:.3f} deg, 1.5/3 Hz {m['goal_1.5hz_mean']:.3f}/"
              f"{m['goal_3hz_mean']:.3f}, rate x/y/z {m['rate_x']:.3f}/{m['rate_y']:.3f}/{m['rate_z']:.3f}, "
              f"power {m['power_w']:.2f} W, hold {m['power_hold_w']:.2f} W, limp {m['power_limp_w']:.2f} W, "
              f"{m['duration_s']:.0f} s armed")
