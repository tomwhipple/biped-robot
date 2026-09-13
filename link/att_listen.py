#!/usr/bin/env python3
"""Log the robot's attitude over the link WITHOUT arming: ARM=0|ATT frames for
--duration s, every beacon's up vector to CSV (t, up_x, up_y, up_z, state).
Used while the tether drives open-loop poses (2026-09-13: does the real torso
sway during a hip-roll rock?)."""
import argparse, csv, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from arm_script import Driver  # noqa: E402
from protocol import FLAG_ATT  # noqa: E402
p = argparse.ArgumentParser(); p.add_argument("--host", required=True); p.add_argument("--duration", type=float, default=10.0)
p.add_argument("--csv", required=True); a = p.parse_args()
d = Driver(a.host); rows = []; t0 = time.monotonic(); last = None
def fn(t): return (0.0, 0.0, FLAG_ATT)
start = time.monotonic()
while time.monotonic() - start < a.duration:
    d.run_for(0.1, fn, "att")
    tl = d.tlm
    if tl is not None and tl is not last:
        last = tl
        ux, uy = (tl.up_xy if len(tl.up_xy) == 2 else (float("nan"), float("nan")))
        rows.append((round(time.monotonic() - t0, 3), ux, uy, tl.up_z, str(tl.state)))
d.close()
with open(a.csv, "w", newline="") as f:
    w = csv.writer(f); w.writerow(["t", "up_x", "up_y", "up_z", "state"]); w.writerows(rows)
print(f"att_listen: {len(rows)} beacons -> {a.csv}")
