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
from wcliproto import LinkDown  # noqa: E402,F401

BUS = ["R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle", "L_hip_roll", "L_hip_pitch", "L_knee",
       "L_ankle", "R_hip_yaw", "L_hip_yaw", "R_ankle_roll", "L_ankle_roll"]
N_BUS = 17
_NUM = r"[-+]?\d+\.\d+"


class Fallen(RuntimeError):
    """The firmware's fall guard has latched: it is limp and refuses motion until it has been picked up
    and has homed itself. Send nothing that moves it."""


class Robot:
    def __init__(self, host="192.168.2.90", key_file=None, log=None, link_retries=3, link_wait=2.0):
        self.key = W.load_key(key_file or os.path.expanduser("~/.config/bimo/wcli.key"))
        self.host = host
        self.c = W.Client(host, self.key)
        self.c.connect()
        self.log = log
        self.t0 = time.monotonic()
        self.link_retries, self.link_wait = link_retries, link_wait
        self.on_link = None                     # callable(event dict), set by the stepper to log `link` events
        self.yaw0 = None                        # heading bookkeeping (see heading())
        self.yaw_rate = 0.0
        self.cal = self._cal()

    # -- plumbing ---------------------------------------------------------
    def _link(self, **kw):
        if self.on_link:
            self.on_link(kw)

    def reconnect(self, fresh=False):
        """A new session with the robot (fresh: a new client object too)."""
        try:
            self.c.close()
        except Exception:
            pass
        if fresh:
            self.c = W.Client(self.host, self.key)
        self.c.connect()

    def fresh(self):
        self.reconnect(fresh=True)

    def cmd(self, line, idle=8.0):
        """One CLI line. If the link drops (LinkDown: the robot did not answer; the session was lost after sending,
        so the line may have run) it reconnects and sends the line again, up to link_retries times, link_wait s
        apart. Every line the stepper sends is safe to repeat: poses are absolute, the rest are idempotent. After a
        `pose` of unknown outcome the IMU is read first, to know the robot answers and is upright."""
        t = time.monotonic()
        for attempt in range(self.link_retries + 1):
            try:
                r = self.c.command(line, idle_timeout=idle)
                if r.status == "unknown":
                    raise W.LinkDown(f"command {r.seq} outcome unknown (session lost)")
                break
            except W.LinkDown as e:
                self._link(ev="down", line=line[:40], attempt=attempt, err=str(e))
                if attempt >= self.link_retries:
                    raise
                time.sleep(self.link_wait)
                try:
                    self.reconnect()
                    if line.startswith("pose"):
                        self.c.command("imu", idle_timeout=idle)
                    self._link(ev="reconnected", attempt=attempt)
                except W.LinkDown as e2:
                    self._link(ev="reconnect_failed", attempt=attempt, err=str(e2))
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
        out = dict(up=up, gyro=gy, roll=-math.degrees(math.asin(max(-1, min(1, up[1])))),
                   pitch=math.degrees(math.atan2(up[0], up[2])), t=time.monotonic() - self.t0)
        m = re.search(rf"^quat\s+({_NUM})\s+({_NUM})\s+({_NUM})\s+({_NUM})", txt, re.M)
        if m:                                           # the fused quaternion's yaw, deg (+ = counter-clockwise from above)
            w, x, y, z = (float(v) for v in m.groups())
            out["yaw"] = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
        return out

    # -- heading -----------------------------------------------------------
    # The fused quaternion's yaw drifts (about -1 deg/s, a gyro bias). heading() is that yaw less the heading at the
    # start less the drift estimated while standing still: heading_still() fits yaw against time over a still window
    # and keeps the slope as the drift rate (blended with the earlier estimate after the first).
    def heading_start(self, samples):
        """samples: [(t, yaw_deg)] of a still stand. Zero the heading here; the drift rate is the slope."""
        self.yaw_rate = self._slope(samples)
        self.yaw0, self.t_ref, self.drift = samples[-1][1], samples[-1][0], 0.0

    @staticmethod
    def _slope(samples):
        n = len(samples)
        mt = sum(t for t, _ in samples) / n
        my = sum(y for _, y in samples) / n
        den = sum((t - mt) ** 2 for t, _ in samples)
        return sum((t - mt) * (y - my) for t, y in samples) / den if den > 1e-9 else 0.0

    def heading_still(self, samples, blend=0.3):
        """A still window (robot settled, both feet down): the yaw slope over it is drift, not turning."""
        if self.yaw0 is not None and len(samples) >= 4 and samples[-1][0] - samples[0][0] >= 0.3:
            now = samples[-1][0]
            self.drift += self.yaw_rate * (now - self.t_ref)          # what the old rate removed so far stays removed
            self.t_ref = now
            self.yaw_rate = (1 - blend) * self.yaw_rate + blend * self._slope(samples)

    def heading(self, yaw, t):
        """Heading (deg, + = CCW) from a raw fused yaw read at time t (s), or None before heading_start."""
        if self.yaw0 is None or yaw is None:
            return None
        h = yaw - self.yaw0 - (self.drift + self.yaw_rate * (t - self.t_ref))
        return (h + 180.0) % 360.0 - 180.0

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

    def pose(self, q, ms, wait=True, margin=0.15):
        """One smooth move to q ({joint: rad}) over ms. Returns when the firmware reports it done."""
        line = "pose " + " ".join(map(str, self.ticks(q))) + f" s{int(ms)}"
        self.cmd(line)
        if wait:
            try:
                self.c.idle(ms / 1000.0 + margin)
            except W.LinkDown:
                self._link(ev="down", line="idle after pose")
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

    def wait_recovered(self, tmax=1200.0, say=print, after=None):
        """After a fall: send nothing that moves and wait. The firmware does the recovering: it homes itself
        once the robot is within 25 deg of upright with |gyro| < 1.0 rad/s for 1 s (picked up and held still).
        This only polls its `fall` status, about once a second, and returns True once it reports "not fallen"
        and its own home has had time to finish; False after `tmax` s.
        after=(seconds, callable): `callable()` is called once, if it is still down after that many seconds
        (a notification; it must not block, and an exception from it is ignored)."""
        t = time.monotonic()
        while time.monotonic() - t < tmax:
            if after is not None and time.monotonic() - t >= after[0]:
                try:
                    after[1]()
                except Exception:
                    pass
                after = None
            try:
                if not self.fallen():
                    say("  the firmware homed it")
                    self.c.idle(11.0)
                    return True
            except Exception:
                pass
            time.sleep(1.0)
        return False

    def close(self):
        self.c.close()
