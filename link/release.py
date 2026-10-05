#!/usr/bin/env python3
"""Release the robot's torque over the link: ARM|ESTOP frames for 1 s, then
silence. The ESTOP latch is the link-side torque release (Tom 2026-09-13:
"always release torque ... zeroing the servos implies released torque").
The next non-ESTOP frame clears the latch, so this tool sends nothing after.

    .venv/bin/python link/release.py --host <robot-ip>
"""
import argparse, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from arm_script import Driver  # noqa: E402
from protocol import FLAG_ARM, FLAG_ESTOP  # noqa: E402

p = argparse.ArgumentParser(); p.add_argument("--host", required=True)
p.add_argument("--port", type=int); p.add_argument("--tlm-port", type=int)
a = p.parse_args()
kw = {k: v for k, v in (("cmd_port", a.port), ("tlm_port", a.tlm_port)) if v}
d = Driver(a.host, **kw)
d.run_for(1.0, lambda t: (0.0, 0.0, FLAG_ARM | FLAG_ESTOP), "ESTOP: torque release")
d.close()
print("released -- link silent; the ESTOP latch holds until the next non-ESTOP frame")
