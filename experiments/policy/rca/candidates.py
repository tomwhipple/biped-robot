"""Margin probes per candidate policy (fitted plant, the firmware arm path): how much extra
loop gain / gyro-path gain each student or teacher survives. The robot needed >= ~4x."""
import os, sys, json
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import ingredients as I

L = dict(lag_hz=12)
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
    rows = I.run(variants(runs), plants, seeds, jobs=int(os.environ.get("JOBS", 14)), out=out)
    print(I.table(rows))
