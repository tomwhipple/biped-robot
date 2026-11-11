"""A stand-in for the robot, for dry runs of stepper.py: a body whose roll follows the lifted foot
through `play` degrees of unseen roll play, less whatever pelvis roll is commanded."""
import math
import random
import time

from nool import sag_deg
from robot_io import Robot as _R

CLOCK = [1000.0]      # a virtual clock (s): the fake robot's own round trips advance it, so does a patched sleep


class FakeRobot:
    cal = {}

    def __init__(self, kin, play=(9.0, 5.0), noise=0.15):
        self.kin, self.play, self.noise = kin, dict(L=play[0], R=play[1]), noise
        self.h = dict(L=0.0, R=0.0); self.hike = dict(L=0.0, R=0.0); self.t = 0.0
        self.temp, self.volt = 40, 11.8             # what `pos` reports
        self.psi_deg = 0.0                          # the pelvis heading as commanded (deg)
        self.slip = 0.0                             # a yaw the body has taken on its own (deg), set by a test
        self.yaw_drift = -1.0                       # the fused yaw's drift, deg/s of fake time
        self.yaw0 = None; self.yaw_rate = 0.0; self.t_ref = 0.0; self.drift = 0.0

    heading_start = _R.heading_start
    heading_still = _R.heading_still
    heading = _R.heading
    _slope = staticmethod(_R._slope)

    def tell(self, feet, hike, ankle=None, psi_deg=0.0):     # the stepper's own state, handed over by the test
        self.psi_deg = psi_deg
        # hip and ankle trim both act on the body roll through the same play
        self.h = dict(L=feet["L"][2], R=feet["R"][2])
        self.hike = {k: hike[k] + (ankle or {}).get(k, 0.0) for k in hike}

    def imu(self):
        CLOCK[0] += 0.094                            # one IMU round trip
        self.t = CLOCK[0]
        up = "R" if self.h["R"] > self.h["L"] else "L"
        st = "L" if up == "R" else "R"
        h = self.h[up]
        # on the floor the short leg pins the tilt; once lift + hip trim have used up the play it hangs at play - trim
        sag = min(sag_deg(h), self.play[st] - self.hike[st])
        roll = (sag if up == "R" else -sag) + random.gauss(0, self.noise)
        yaw = self.psi_deg + self.slip + self.yaw_drift * self.t + random.gauss(0, 0.05)
        return dict(roll=roll, pitch=random.gauss(0, self.noise), gyro=[0.01, 0.01, 0.0], t=self.t, up=[0, 0, 1], yaw=yaw)

    def pose(self, q, ms, wait=True, margin=0.0):
        assert all(math.isfinite(v) for v in q.values())
        CLOCK[0] += 0.1 + (ms / 1000.0 + margin if wait else 0.0)        # the command's round trip, then the move

    def pos(self, name):
        return dict(pos=2048, spd=0, load=0, volt=self.volt, temp=self.temp)

    def fresh(self): pass

    def torque(self): pass
    def release(self): pass
    def fallen(self): return False
    def wait_recovered(self, tmax=0, say=print, after=None): return True
    def close(self): pass
