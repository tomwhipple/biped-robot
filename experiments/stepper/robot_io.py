"""The robot over one persistent Wi-Fi CLI session: poses out, IMU and servo readings back.

Everything the stepper sends is a bench-CLI line (`pose`, `torque`, `home`, `release`, `imu`, `pos`),
so the firmware's own guards (calibrated range clamps, the bench fall guard) stay in force.
"""
from __future__ import annotations

import math
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "link"))
import wcliproto as W  # noqa: E402

BUS = ["R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle", "L_hip_roll", "L_hip_pitch", "L_knee",
       "L_ankle", "R_hip_yaw", "L_hip_yaw", "R_ankle_roll", "L_ankle_roll"]
N_BUS = 17
_NUM = r"[-+]?\d+\.\d+"


class Fallen(RuntimeError):
    """The firmware's fall guard has latched: it is limp and refuses motion until it has been picked up
    and has homed itself. Send nothing that moves it."""


class Robot:
    def __init__(self, host="192.168.2.90", key_file=None, log=None):
        key = W.load_key(key_file or os.path.expanduser("~/.config/bimo/wcli.key"))
        self.c = W.Client(host, key)
        self.c.connect()
        self.log = log
        self.t0 = time.monotonic()
        self.cal = self._cal()

    # -- plumbing ---------------------------------------------------------
    def cmd(self, line, idle=8.0):
        t = time.monotonic()
        r = self.c.command(line, idle_timeout=idle)
        if self.log:
            self.log(dict(t=round(t - self.t0, 3), dt=round(time.monotonic() - t, 3), cmd=line[:120],
                          status=r.status, text=r.text.strip()[-300:]))
        if "FALLEN" in r.text:
            raise Fallen(r.text.strip()[-160:])
        if r.status != "done" or r.refused or r.text.lstrip().startswith("refused"):
            raise RuntimeError(f"{line[:60]!r}: {r.status} {r.text.strip()[-200:]}")
        return r.text

    def _cal(self):
        cal = {}
        for m in re.finditer(r"^\s+(\w+)\s+id\s+(\d+)\s+zero\s+(\d+)\s+dir\s+([-+]\d)\s+fitted\s*$",
                             self.cmd("cal show"), re.M):
            cal[m.group(1)] = (int(m.group(2)), int(m.group(3)), int(m.group(4)))
        missing = [n for n in BUS if n not in cal]
        if missing:
            raise RuntimeError(f"cal show: not fitted {missing}")
        return cal

    # -- reads ------------------------------------------------------------
    def imu(self):
        """up (body z in the yaw-stripped frame) and gyro (rad/s). roll/pitch in deg:
        roll + = leaning to the robot's RIGHT (the firmware's `balance` sign), pitch + = leaning forward."""
        txt = self.cmd("imu")
        up = [float(x) for x in re.search(rf"^up\s+({_NUM})\s+({_NUM})\s+({_NUM})", txt, re.M).groups()]
        gy = [float(x) for x in re.search(rf"^gyro\s+({_NUM})\s+({_NUM})\s+({_NUM})", txt, re.M).groups()]
        return dict(up=up, gyro=gy, roll=-math.degrees(math.asin(max(-1, min(1, up[1])))),
                    pitch=math.degrees(math.atan2(up[0], up[2])), t=time.monotonic() - self.t0)

    def pos(self, name):
        sid = self.cal[name][0]
        m = re.search(r"pos\s+(\d+)\s+spd\s+(-?\d+)\s+load\s+(-?\d+)\s+([\d.]+) V\s+(\d+) C", self.cmd(f"pos {sid}"))
        return dict(pos=int(m.group(1)), spd=int(m.group(2)), load=int(m.group(3)), volt=float(m.group(4)),
                    temp=int(m.group(5)))

    # -- writes -----------------------------------------------------------
    def ticks(self, q):
        """{joint: rad} -> the 17 `pose` targets (bus order; unfitted slots 2048)."""
        out = []
        for n in BUS:
            _, z, s = self.cal[n]
            out.append(int(round(z + s * q[n] * 4096 / (2 * math.pi))))
        return out + [2048] * (N_BUS - len(BUS))

    def pose(self, q, ms, wait=True):
        """One smooth move to q ({joint: rad}) over ms. Returns when the firmware reports it done."""
        line = "pose " + " ".join(map(str, self.ticks(q))) + f" s{int(ms)}"
        self.cmd(line)
        if wait:
            self.c.idle(ms / 1000.0 + 0.15)
        return line

    def torque(self):
        return self.cmd("torque")

    def release(self):
        return self.cmd("release")

    def home(self):
        self.cmd("home")
        self.c.idle(9.0)

    def fallen(self):
        """The firmware's own verdict (`fall`): True while its fall latch is set."""
        r = self.c.command("fall", idle_timeout=5.0)
        return "not fallen" not in r.text

    def wait_recovered(self, tmax=300.0, tilt_deg=25.0, gyro=1.0, hold_s=1.0, say=print):
        """After a fall: send nothing that moves until the robot is back upright, then home it.
        The firmware homes itself once it has been within 10 deg and still (0.35 rad/s) for 2 s, a rule a
        robot held in the hand or set down leaning does not meet (2026-11-11, third fall: 48 s upright
        and never homed). So this also homes it, with a typed `home`, once it has been within `tilt_deg`
        of upright and under `gyro` rad/s for `hold_s`. Returns True once it is homed (torque released)."""
        t = time.monotonic()
        since = None
        while time.monotonic() - t < tmax:
            if not self.fallen():                       # the firmware saw the pick-up first: let its home finish
                say("  the firmware homed it")
                self.c.idle(11.0)
                return True
            try:
                s = self.imu()
            except Exception:
                since = None
                time.sleep(0.5)
                continue
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, s["up"][2]))))
            w = math.sqrt(sum(x * x for x in s["gyro"]))
            if tilt < tilt_deg and w < gyro:
                since = since or time.monotonic()
                if time.monotonic() - since >= hold_s:
                    say(f"  upright ({tilt:.0f} deg) and still for {hold_s:.0f} s: homing")
                    self.c.command("home", idle_timeout=5.0)
                    self.c.idle(12.0)
                    return not self.fallen()
            else:
                since = None
            time.sleep(0.1)
        return False

    def close(self):
        self.c.close()
