"""Load the 2026-11-01 first-policy-stand flight recording into policy-order arrays."""
from __future__ import annotations
import os, sys
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
MAIN = "/home/claw/code/robot"           # recordings + runs live in the main checkout
sys.path.insert(0, os.path.join(ROOT, "tools"))
import flight_reader as FR  # noqa: E402

SESS = os.path.join(MAIN, "hw_sessions/2026-11-01/flight_20261101T213631Z_policy_stand1.frec")
SWEEP = os.path.join(MAIN, "hw_sessions/2026-11-01/flight_20261101T213236Z_ilim_stands.frec")
POL = ["L_hip_yaw", "L_hip_roll", "L_hip_pitch", "L_knee", "L_ankle", "L_ankle_roll",
       "R_hip_yaw", "R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle", "R_ankle_roll"]
LO = np.array([-0.785398163, -0.523598776, -2.0943951, -2.26892803, -0.698131701, -0.436332313,
               -0.785398163, -0.785398163, -2.0943951, -2.26892803, -0.698131701, -0.436332313])
HI = np.array([0.785398163, 0.785398163, 1.57079633, 0.0872664626, 0.698131701, 0.436332313,
               0.785398163, 0.523598776, 1.57079633, 0.0872664626, 0.698131701, 0.436332313])
RAD = 2 * np.pi / 4096


def angles_to_action(a):
    return np.clip(np.where(a >= 0, a / HI, -a / LO), -1, 1)


def action_to_angles(x):
    x = np.clip(x, -1, 1)
    return np.where(x >= 0, HI * x, -LO * x)


def load(path=SESS, source="ctrl"):
    s = FR.load(path)
    cal = FR.calibration(s)
    out = {k: [] for k in ("t", "tick", "flags", "link", "q", "dq_reg", "nom", "goal", "goal_speed",
                           "load", "current", "up", "gyro", "accel", "quat", "roll", "pitch",
                           "phase", "cmd", "fb_off", "imu_age", "torque")}
    for p in s.ticks:
        if F_SRC(p) != source:
            continue
        bc = cal[p["boot_id"]]
        js = {FR.F.BUS_JOINT_NAMES[j["bus"]]: j for j in p["joints"]}
        if not all(n in js for n in POL):
            continue
        z = [bc[FR.F.BUS_JOINT_NAMES.index(n)][0] for n in POL]
        d = np.array([bc[FR.F.BUS_JOINT_NAMES.index(n)][1] for n in POL], float)
        g = lambda k: np.array([js[n][k] for n in POL], float)
        out["t"].append(p["utc_us"] / 1e6)
        out["tick"].append(p["tick"])
        out["flags"].append(p["flags"])
        out["link"].append(p["link_state"])
        out["torque"].append(bool(p["flags"] & FR.F.TF_TORQUE))
        out["q"].append((g("pos") - z) * RAD * d)
        out["dq_reg"].append(g("speed") * RAD * d)
        out["nom"].append((g("nominal") - z) * RAD * d)
        out["goal"].append((g("goal") - z) * RAD * d)
        out["goal_speed"].append(g("goal_speed"))
        out["load"].append(g("load") * d)
        out["current"].append(g("current"))
        out["up"].append(p["up"]); out["gyro"].append(p["gyro"]); out["accel"].append(p["accel"])
        out["quat"].append(p["quat"]); out["roll"].append(p["roll"]); out["pitch"].append(p["pitch"])
        out["phase"].append(p["ctrl"]["phase"]); out["cmd"].append(p["ctrl"]["cmd"])
        out["fb_off"].append(p["fb_off_us"]); out["imu_age"].append(p["imu_age_us"])
    return {k: np.asarray(v) for k, v in out.items()}


def F_SRC(p):
    return FR.F.SOURCE_NAMES.get(p["source"], p["source"])
