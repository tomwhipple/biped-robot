"""Instrumented crouch on hardware: the crouch1 ramp through the ext channel
with ARM|POSE|ATT frames (40 B pose beacons -> every joint angle logged), the
osc_probe guards (torso tilt, joint peak-to-peak swing), the home gate before
and the RESET SERVOS edge (still armed) after.

2026-09-11: the first crouch1 on hardware (arm_script, 20 B beacons) turned
the toes inward during the hold and nobody could say whether that was the
policy's commanded hip yaw or the mechanics -- this writes the angles down.

    .venv/bin/python link/crouch_probe.py --host 192.168.2.90 \
        --csv hw_sessions/2026-09-11/crouch2_beacons.csv
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from arm_script import arm_off  # noqa: E402
from osc_probe import OscDriver  # noqa: E402
from protocol import (FLAG_ARM, FLAG_ATT, FLAG_ENABLE, FLAG_ESTOP, FLAG_HOME, FLAG_POSE,  # noqa: E402
                      ArmResult, LinkState, diag_arm_result, diag_reason)
from sources import SCRIPTS  # noqa: E402

POSE = FLAG_ARM | FLAG_POSE | FLAG_ATT


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--host", required=True)
    p.add_argument("--script", default="crouch1")
    p.add_argument("--settle", type=float, default=3.0)
    p.add_argument("--duration", type=float, default=8.0)
    p.add_argument("--tail", type=float, default=3.0)
    p.add_argument("--tilt", type=float, default=0.90)
    p.add_argument("--pp-rad", type=float, default=0.5)
    p.add_argument("--csv", required=True)
    p.add_argument("--port", type=int)
    p.add_argument("--tlm-port", type=int)
    a = p.parse_args()
    fn = SCRIPTS[a.script]
    kw = {k: v for k, v in (("cmd_port", a.port), ("tlm_port", a.tlm_port)) if v is not None}
    d = OscDriver(a.host, a.csv, a.tilt, a.pp_rad, **kw)

    def script(t):
        out = fn(t)
        ext = {k: v for k, v in out.items() if k != "vx" and k != "wz"} if isinstance(out, dict) else {}
        active = any(ext.values())
        return 0.0, 0.0, POSE | (FLAG_ENABLE if active else 0), ext

    verdict = None
    try:
        verdict = d.run_for(0.5, arm_off, "pre-arm: ARM=0 frames (latch level)")
        if not verdict:
            d.run_for(0.4, lambda t: (0.0, 0.0, FLAG_HOME), "RESET SERVOS: FLAG_HOME edge")
            r = None
            for _ in range(16):   # the zeroing takes ~8 s since the hip wiggle (2026-09-13)
                d.run_for(1.0, arm_off, "home settle: ARM=0 frames")
                r = diag_arm_result(d.tlm.diag) if d.tlm is not None else None
                if r is not None and r is not ArmResult.HOME_PENDING:
                    break
            print(f"== home verdict: {r} -- {diag_reason(d.tlm.diag) if d.tlm else 'no beacon'}", flush=True)
            if r is not ArmResult.DISARMED_HOME:
                verdict = f"home not confirmed ({r}); not arming"
                d.run_for(0.5, arm_off, "abort: ARM=0 frames")
        if not verdict:
            verdict = d.run_for(a.settle, lambda t: (0.0, 0.0, POSE), "settle: ARM|POSE stand")
        if not verdict:
            verdict = d.run_for(a.duration, script, f"script {a.script} (ARM|POSE)")
        if not verdict:
            verdict = d.run_for(a.tail, lambda t: (0.0, 0.0, POSE), "tail: ARM|POSE stand")
        if verdict and not verdict.startswith("home not confirmed"):
            d.emergency(verdict)
        # end of run: zero the servos (firmware rolls the hips out/back for
        # play and RELEASES torque itself); ESTOP only if the reset failed
        d.home_and_release(FLAG_ARM | FLAG_HOME, "end: RESET SERVOS (still armed)")
    except KeyboardInterrupt:
        d.emergency("operator ctrl-C")
        verdict = "interrupted"
    finally:
        print(f"== done: {d.seq - d.seq0} frames sent, {d.n_tlm} beacons ({d.n_logged} logged), "
              f"max joint p-p {d.max_pp:.3f} rad, min up_z {d.min_upz:+.2f}; states: {d.states_seen}", flush=True)
        if d.tlm is not None and d.tlm.state == LinkState.BENCH:
            print(f"== benched: {diag_reason(d.tlm.seq_echo)}", flush=True)
        d.close()
    sys.exit(2 if verdict else 0)


if __name__ == "__main__":
    main()
