"""A stand-in for the robot, for dry runs of stepper.py: a body whose roll follows the lifted foot
through `play` degrees of unseen roll play, less whatever pelvis roll is commanded."""
import math
import random
import time

from nool import sag_deg


class FakeRobot:
    cal = {}

    def __init__(self, kin, play=(9.0, 5.0), noise=0.15):
        self.kin, self.play, self.noise = kin, dict(L=play[0], R=play[1]), noise
        self.h = dict(L=0.0, R=0.0); self.hike = dict(L=0.0, R=0.0); self.t = 0.0

    def tell(self, feet, hike):                 # the stepper's own state, handed over by the test
        self.h = dict(L=feet["L"][2], R=feet["R"][2]); self.hike = dict(hike)

    def imu(self):
        self.t += 0.094
        up = "R" if self.h["R"] > self.h["L"] else "L"
        st = "L" if up == "R" else "R"
        h = self.h[up]
        # on the floor the short leg pins the tilt; once lift + hip trim have used up the play it hangs at play - trim
        sag = min(sag_deg(h), self.play[st] - self.hike[st])
        roll = (sag if up == "R" else -sag) + random.gauss(0, self.noise)
        return dict(roll=roll, pitch=random.gauss(0, self.noise), gyro=[0.01, 0.01, 0.0], t=self.t, up=[0, 0, 1])

    def pose(self, q, ms, wait=True):
        assert all(math.isfinite(v) for v in q.values())

    def torque(self): pass
    def release(self): pass
    def fallen(self): return False
    def wait_recovered(self, tmax=0): return True
    def close(self): pass
