"""Send one RESET SERVOS edge over the link and print every verdict the robot
reports until it settles (HOME_PENDING -> DISARMED_HOME / HOME_NOT_REACHED).
Run from the repo root: .venv/bin/python tools/home_verify.py <robot-ip>
(or set $ROBOT_HOST)."""
import sys, time; import os; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'link'))
from arm_script import Driver, arm_off
from protocol import FLAG_HOME, ArmResult, diag_arm_result, diag_reason
host = sys.argv[1] if len(sys.argv) > 1 else os.environ.get('ROBOT_HOST')
if not host: sys.exit('usage: tools/home_verify.py <robot-ip>  (or set $ROBOT_HOST)')
d = Driver(host)
d.run_for(0.5, arm_off, "pre: ARM=0 frames")
t0 = d.now(); d.run_for(0.4, lambda t: (0.0, 0.0, FLAG_HOME), "RESET SERVOS: FLAG_HOME edge")
last = None
for _ in range(100):
    d.run_for(0.1, arm_off, "poll") if False else None
    d.send(0.0, 0.0, 0); d.pump(0.0, 0.0, 0); time.sleep(0.05)
    r = diag_arm_result(d.tlm.diag) if d.tlm else None
    if r != last:
        print(f"t={d.now()-t0:5.2f}s verdict {r} -- {diag_reason(d.tlm.diag) if d.tlm else '-'}", flush=True); last = r
    if r in (ArmResult.DISARMED_HOME, ArmResult.HOME_NOT_REACHED): break
d.close()
