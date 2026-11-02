#!/usr/bin/env python3
"""Students x probes x seeds on the FAST measured actuator, run the way the robot runs them
now: `standclock coast` (PR #197, default target pi/2), the 12 Hz command shaper, gyro x1
(no obsfreeze). Probes are stand-still/stand_candidates.py's (stand jitter + power,
x1.25 / x1.5 loop gain, push 0.4, walk 0.15 -> stop) plus its hard set (reg-58 dq, the
slow servo, gyro path x1.5, x2, push 0.5).

  CUDA_VISIBLE_DEVICES= SERVO=fast HARD=1 JOBS=16 \
    .venv/bin/python experiments/policy/int8-256/grade_stand.py <run,run,...> [seeds] [out.json]

A `<run>_q8` is the int8 net (q8_proxy.py)."""
from __future__ import annotations
import math, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "stand-still"))
os.environ.setdefault("SERVO", "fast")
import stand_candidates as C  # noqa: E402

HW = dict(gyro_obs=1.0, shaper_hz=12.0, clock="coast", stand_phase=math.pi / 2)

if __name__ == "__main__":
    import jitter as J
    runs = sys.argv[1].split(",")
    seeds = int(sys.argv[2]) if len(sys.argv) > 2 else 8
    out = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, "stand_fast.json")
    variants = {r: dict(HW, run=r) for r in runs}
    probes = dict(C.PROBES, **C.PROBES_HARD) if os.environ.get("HARD") else C.PROBES
    rows = C.run(variants, probes, seeds, int(os.environ.get("JOBS", 12)), out)
    md = (f"servo: {C.SERVO} ({C.SERVOS[C.SERVO]}); firmware: coast -> pi/2, shaper 12 Hz, gyro x1; "
          f"{seeds} seeds, fitted plant\n\n## Compact\n\n" + C.compact(rows, list(probes))
          + "\n\n## Full\n\n" + C.table(rows, J.robot()) + "\n")
    open(out.replace(".json", ".md"), "w").write(md)
    print(md)
