"""Margin probes per candidate policy (fitted plant, the firmware arm path): how much extra
loop gain / gyro-path gain each student or teacher survives. The robot needed >= ~4x."""
import os, sys, json
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import ingredients as I

import simh
L = dict(lag_hz=12)
# the robot's measured actuator (stepresp + sweep fit, 2026-11-01): the probes that matter now
MEAS = dict(typed=simh.MEAS_SERVO, model=simh.JF_PLANT)
PROBES_MEAS = {
    "measured actuator": dict(MEAS),
    "measured, loop gain x1.25": dict(MEAS, out_gain=1.25),
    "measured, loop gain x1.5": dict(MEAS, out_gain=1.5),
    "measured, gyro-path x1.5": dict(MEAS, obs=dict(gyro_gain=1.5)),
    "measured, push 0.4": dict(MEAS, push=0.4),
}
PROBES_HARD = {
    "measured, loop gain x2": dict(MEAS, out_gain=2.0),
    "measured, loop gain x3": dict(MEAS, out_gain=3.0),
    "measured, gyro-path x2.5": dict(MEAS, obs=dict(gyro_gain=2.5)),
    "measured, push 0.6": dict(MEAS, push=0.6),
    "measured + reg-58 dq": dict(MEAS, obs=dict(dq_delay=1, dq_quant=1)),
    "measured, STS3215 dead 100 tau 90": dict(typed=dict(simh.MEAS_SERVO, dead15=100, tau15=90), model=simh.JF_PLANT),
    "sweep-only fit actuator": dict(servo=dict(), model=dict(simh.FITTED_ACTUATOR)),
    "as-trained plant (lag 12)": dict(servo=L),
    "as-trained plant, push 0.4": dict(servo=L, push=0.4),
}
PROBES = {
    "nominal": dict(servo=L),
    "gyro-path x3": dict(servo=L, obs=dict(gyro_gain=3.0)),
    "gyro-path x4": dict(servo=L, obs=dict(gyro_gain=4.0)),
    "gyro-path x6": dict(servo=L, obs=dict(gyro_gain=6.0)),
    "loop gain x2.5": dict(servo=L, out_gain=2.5),
    "loop gain x3": dict(servo=L, out_gain=3.0),
    "push 0.4": dict(servo=L, push=0.4),
}


def variants(runs, probes=PROBES):
    V = {}
    for r in runs:
        for k, v in probes.items():
            V[f"{r} :: {k}"] = dict(v, run=r)
    return V


if __name__ == "__main__":
    runs = sys.argv[1].split(",")
    plants = (sys.argv[2] if len(sys.argv) > 2 else "fit").split(",")
    seeds = int(sys.argv[3]) if len(sys.argv) > 3 else 4
    out = sys.argv[4] if len(sys.argv) > 4 else os.path.join(HERE, "candidates.json")
    probes = {"meas": PROBES_MEAS, "hard": PROBES_HARD}.get(os.environ.get("ACT"), PROBES)
    rows = I.run(variants(runs, probes), plants, seeds, jobs=int(os.environ.get("JOBS", 14)), out=out)
    print(I.table(rows))
