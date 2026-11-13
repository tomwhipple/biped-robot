"""The stance-hip roll trim (#297 step 2): the Python twin of firmware/components/balance/stance_trim.
Same states, same arithmetic; firmware/host/test_stance_trim.cpp pins the C++ to the numbers this file
prints (`python sim/stance_trim.py`).

    e    = roll - roll_ref                      (rad; roll + = leaning to the robot's right)
    s    = +1 on L stance, -1 on R
    hike = clip(r * T[side] + kp * s * e, +-lim) (rad, the stepper's convention: + lifts the other hip)
    T[side] += ki * s * e * dt                   while CLOSED, |unclipped hike| < lim and |e| <= learn; |T| <= t_lim
    T is provisional within a chain: a chain that ends (release, abort, stop) with its peak |e| past keep,
    an abort, or the hike at its clamp puts T of its side back to its value at the chain's begin
    stance hip roll joint += s * hike

CLOSED -> HOLD (frozen) -> FADE (linear to 0 over fade_s) -> OFF; |e| > abort while CLOSED, or a
non-finite roll, freezes the correction and fades it the same way (ABORT); T learns only while CLOSED.
"""
from __future__ import annotations

import math

D2R = math.pi / 180.0
OFF, CLOSED, HOLD, FADE, ABORT_TIP, ABORT_IMU = 0, 5, 6, 7, 8, 9


class StanceTrim:
    def __init__(self, kp=0.25, ki=1.0, lim_deg=15.0, t_lim_deg=14.0, fade_s=0.6, abort_deg=15.0, learn_deg=6.0,
                 keep_deg=8.0):
        self.kp, self.ki = kp, ki
        self.lim, self.t_lim = lim_deg * D2R, t_lim_deg * D2R
        self.fade_s, self.abort = fade_s, abort_deg * D2R
        self.T = {"L": 0.0, "R": 0.0}
        self.state, self.side = OFF, None
        self.ref = self.err = self.hike = self.frozen = self.since = 0.0
        self.learn, self.keep = learn_deg * D2R, keep_deg * D2R
        self.t_begin, self.peak, self.clamped, self.reverted = 0.0, 0.0, False, False

    def sign(self):
        return -1.0 if self.side == "R" else 1.0

    def begin(self, side, roll_ref):
        self.side = "R" if side == "R" else "L"
        self.ref, self.err, self.hike, self.frozen, self.since = roll_ref, 0.0, 0.0, 0.0, 0.0
        self.state = CLOSED
        self.t_begin, self.peak, self.clamped = self.T[self.side], 0.0, False

    def end_chain(self, aborting):
        if self.side not in ("L", "R"):
            return
        self.reverted = aborting or self.clamped or self.peak > self.keep
        if self.reverted:
            self.T[self.side] = self.t_begin

    def fade_left(self):
        if self.state not in (FADE, ABORT_TIP, ABORT_IMU):
            return 1.0
        return max(0.0, 1.0 - self.since / self.fade_s)

    def update(self, roll, dt, ramp):
        dt = dt if (math.isfinite(dt) and dt > 0) else 0.0
        if self.state == OFF:
            return self.state
        if self.state == HOLD:
            if math.isfinite(roll):
                self.err = roll - self.ref
                self.peak = max(self.peak, abs(self.err))     # the whole single support counts (as the C++)
            return self.state
        if self.state in (FADE, ABORT_TIP, ABORT_IMU):
            self.since += dt
            if math.isfinite(roll):
                self.err = roll - self.ref
            k = self.fade_left()
            self.hike = self.frozen * k
            if k <= 1e-4:      # the C++ float time sum lands a hair short of fade_s
                self.hike, self.state, self.side = 0.0, OFF, None
            return self.state
        if not math.isfinite(roll):
            self.imu_fault()
            return self.state
        e = roll - self.ref
        self.err = e
        self.peak = max(self.peak, abs(e))
        if abs(e) > self.abort:
            self.frozen, self.since = self.hike, 0.0
            self.end_chain(True)
            self.state = ABORT_TIP
            return self.state
        r = min(max(ramp if math.isfinite(ramp) else 1.0, 0.0), 1.0)
        s = self.sign()
        unsat = r * self.T[self.side] + self.kp * s * e
        if abs(unsat) >= self.lim:
            self.clamped = True
        elif abs(e) <= self.learn:
            self.T[self.side] = min(max(self.T[self.side] + self.ki * s * e * dt, -self.t_lim), self.t_lim)
        self.hike = min(max(unsat, -self.lim), self.lim)
        return self.state

    def hold(self):
        if self.state == CLOSED:
            self.state = HOLD

    def resume(self):
        if self.state == HOLD:
            self.state = CLOSED

    def release(self):
        if self.state in (CLOSED, HOLD):
            self.frozen, self.since = self.hike, 0.0
            self.end_chain(False)
            self.state = FADE

    def imu_fault(self):
        if self.state in (CLOSED, HOLD):
            self.frozen, self.since = self.hike, 0.0
            self.end_chain(True)
            self.state = ABORT_IMU

    def hip_joint(self):
        return self.sign() * self.hike


def golden():
    """The sequence test_stance_trim.cpp replays: (event, roll rad, ramp) per 20 ms tick -> hike, T."""
    tr = StanceTrim()
    rows = []
    tr.begin("L", 0.01)
    seq = ([("u", 0.01 + 0.002 * k, min(1.0, k / 10)) for k in range(15)]
           + [("hold", 0.06, 1.0)] + [("u", 0.07, 1.0)] * 3
           + [("release", 0.06, 1.0)] + [("u", 0.05, 1.0)] * 32)
    for ev, roll, ramp in seq:
        if ev == "hold":
            tr.hold()
        elif ev == "release":
            tr.release()
        tr.update(roll, 0.02, ramp)
        rows.append((ev, roll, ramp, tr.state, tr.hike, tr.T["L"]))
    # R stance with a learned trim, then an abort
    tr.T["R"] = 3.0 * D2R
    tr.begin("R", -0.02)
    for k in range(6):
        roll = -0.02 - 0.03 * k
        tr.update(roll, 0.02, 1.0 if k else 0.5)
        rows.append(("u", roll, 1.0 if k else 0.5, tr.state, tr.hike, tr.T["R"]))
    for _ in range(4):
        tr.update(-0.4, 0.02, 1.0)
        rows.append(("u", -0.4, 1.0, tr.state, tr.hike, tr.T["R"]))
    for _ in range(31):                      # the abort fades out
        tr.update(-0.4, 0.02, 1.0)
        rows.append(("u", -0.4, 1.0, tr.state, tr.hike, tr.T["R"]))
    # R again: a lean to 9 deg (learning stops past 6), released without an abort -> T reverts at release
    tr.begin("R", 0.0)
    for k in range(10):
        roll = -0.016 * k
        tr.update(roll, 0.02, 1.0)
        rows.append(("u", roll, 1.0, tr.state, tr.hike, tr.T["R"]))
    tr.release()
    tr.update(-0.1, 0.02, 1.0)
    rows.append(("release", -0.1, 1.0, tr.state, tr.hike, tr.T["R"]))
    return rows


if __name__ == "__main__":
    for ev, roll, ramp, st, hike, t in golden():
        print(f'{{"{ev}", {roll:.6f}f, {ramp:.6f}f, {st}, {hike:.6f}f, {t:.6f}f}},')
