"""Stance-leg IMU roll balance for the one-leg stand: the controller as the sim
runs it, and the run metrics.

Mirrors firmware/components/balance/roll_pi.h (the pure PI the bench `pose ...
B<L|R>` streamer calls every poseTick), tick for tick: same error definition,
same anti-windup, same clamp, same output mixing. Keep the two in step -- the
host test pins the C++ side against numbers this file produced
(`balance.py --golden`).

Structure
    err   = roll - roll_ref                 roll_ref captured when feedback engages
    rate  = body roll rate from the gyro (rad/s, + = roll increasing), 1 tick late, noisy
    u     = clamp(kp*err + ki*I + kd*rate, +-clamp)   I += err*dt unless saturated and err pushes further
    stance ankle_roll target += ka * u      (ka, kh signed mixes; hip roll and
    stance hip_roll   target += kh * u       ankle roll of the STANCE leg only)
    the swing leg is untouched.

On exit (the next pose) the correction fades to zero along the next move's
min-jerk profile, as the firmware does by starting that move from the
corrected targets.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "sim"))
from design_gates import JN, minjerk  # noqa: E402


class RollPI:
    def __init__(self, kp, ki, clamp, kd=0.0):
        self.kp, self.ki, self.clamp, self.kd = kp, ki, clamp, kd
        self.i = 0.0
        self.u = 0.0

    def step(self, err, dt, rate=0.0):
        """rate = body roll rate (rad/s, + = roll increasing = d err/dt), straight from the
        gyro. u is the stance correction with the mix signs of the caller, in which a +err
        gets a +u; so the damping term is +kd*rate (it opposes the motion of the tip)."""
        unsat = self.kp * err + self.ki * self.i + self.kd * rate
        u = max(-self.clamp, min(self.clamp, unsat))
        if not (u != unsat and (err > 0) == (u > 0)):      # anti-windup: freeze I while pushing into the clamp
            self.i += err * dt
        self.u = u
        return u


class Balance:
    """hook(t, env, qc) for static_gait.run_walk."""

    def __init__(self, stance, kp, ki, ka, kh, clamp, t_on, t_off, t_fade, t_hold0, t_hold1,
                 delay_ticks=0, noise=0.0, seed=0, abort=math.radians(12.0),
                 kd=0.0, gyro_noise=0.0075, gyro_bias=0.0, gyro_delay=1):
        self.s = stance
        self.pi = RollPI(kp, ki, clamp, kd)
        self.gyro_noise, self.gyro_bias, self.gyro_delay = gyro_noise, gyro_bias, gyro_delay
        self.ghist = []
        self.ka, self.kh = ka, kh
        self.t_on, self.t_off, self.t_fade = t_on, t_off, t_fade
        self.h0, self.h1 = t_hold0, t_hold1
        self.delay = delay_ticks
        self.noise = noise
        self.rng = np.random.default_rng(seed)
        self.abort = abort
        self.ref = None
        self.hist = []
        self.u_off = None
        self.tripped = False
        self.u_frozen = 0.0
        self.rec = []                      # (t, roll, u)
        self.air = 0.0
        self.pads = None
        self.dt = None

    @staticmethod
    def roll_of(env):
        Rm = env.data.xmat[env._torso_bid].reshape(3, 3)
        return math.atan2(Rm[2, 1], Rm[2, 2])

    @staticmethod
    def rate_of(env):
        """Body-frame angular velocity about the torso x axis: the freejoint's qvel[3:6] is
        body-local in MuJoCo, and a + rotation about x (forward) tips the top toward -y =
        the robot's right = the same sign as roll_of. This is what the gyro reports."""
        return float(env.data.qvel[3])

    def __call__(self, t, env, qc):
        import mujoco
        dt = env.control_dt
        self.dt = dt
        if self.pads is None:
            m = env.model
            self.pads = [g for g in range(m.ngeom)
                         if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith(
                             ("R_pad_" if self.s == "L" else "L_pad_"))]
        roll = self.roll_of(env)
        self.hist.append(roll)
        meas = self.hist[max(0, len(self.hist) - 1 - self.delay)]
        if self.noise:
            meas += float(self.rng.normal(0.0, self.noise))
        self.ghist.append(self.rate_of(env))
        rate = self.ghist[max(0, len(self.ghist) - 1 - self.gyro_delay)] + self.gyro_bias
        if self.gyro_noise:
            rate += float(self.rng.normal(0.0, self.gyro_noise))
        m = env.model
        d = env.data
        if self.h0 <= t < self.h1:
            h = min(d.geom_xpos[g][2] - m.geom_size[g][0] for g in self.pads)
            if h > 0.005:
                self.air += dt
        if t < self.t_on:
            self.rec.append((t, roll, 0.0))
            return qc
        if self.ref is None:
            self.ref = meas                    # captured when feedback engages
        if t < self.t_off:
            err = meas - self.ref
            if not self.tripped and abs(err) > self.abort:
                self.tripped = True            # safety abort: freeze the correction, keep torque
                self.u_frozen = self.pi.u
            u = self.u_frozen if self.tripped else self.pi.step(err, dt, rate)
            self.u_off = u
        else:
            u = self.u_off * (1.0 - minjerk(min(1.0, (t - self.t_off) / self.t_fade)))
        self.rec.append((t, roll, u))
        qc = qc.copy()
        i_a, i_h = JN.index(f"{self.s}_ankle_roll"), JN.index(f"{self.s}_hip_roll")
        qc[i_a] += self.ka * u
        qc[i_h] += self.kh * u
        return qc

    def metrics(self):
        r = np.array([(t, ro, u) for t, ro, u in self.rec]).reshape(-1, 3)
        hold = r[(r[:, 0] >= self.h0) & (r[:, 0] < self.h1)]
        if not len(hold):                       # fell before the hold
            return dict(roll_rms_deg=float("nan"), roll_rms_last5_deg=float("nan"), roll_err_mean_deg=float("nan"), roll_err_max_deg=float("nan"),
                        u_max_deg=float("nan"), u_end_deg=float("nan"), foot_off_s=self.air, tripped=self.tripped,
                        err_peak_deg=float("inf"), settle_s=float("inf"))
        e = hold[:, 1] - (self.ref if self.ref is not None else 0.0)
        # oscillation: RMS about the hold mean, and sign changes of the detrended
        # roll over the second half of the hold (a limit cycle shows as many)
        dev = hold[:, 1] - hold[:, 1].mean()
        h2 = hold[len(hold) // 2:, 1]
        h2 = h2 - h2.mean()
        last = hold[hold[:, 0] >= self.h1 - 5.0][:, 1]
        rms5 = math.degrees(float(np.sqrt(np.mean((last - last.mean()) ** 2)))) if len(last) else float("nan")
        # transient: the whole engaged window (lift start .. hold end); settling = the first time
        # after the lift ends from which |err| stays under 1 deg to the end of the hold
        eng = r[(r[:, 0] >= self.t_on) & (r[:, 0] < self.h1)]
        ee = np.abs(eng[:, 1] - self.ref)
        late = (eng[:, 0] >= self.h0)
        bad = np.where(late & (ee > math.radians(1.0)))[0]
        settle = float(eng[bad[-1] + 1, 0] - self.h0) if len(bad) and bad[-1] + 1 < len(eng) else (float("inf") if len(bad) else 0.0)
        return dict(roll_rms_deg=math.degrees(float(np.sqrt(np.mean(dev ** 2)))), roll_rms_last5_deg=rms5,
                    err_peak_deg=math.degrees(float(ee.max())), settle_s=settle,
                    roll_err_mean_deg=math.degrees(float(e.mean())),
                    roll_err_max_deg=math.degrees(float(np.abs(e).max())),
                    u_max_deg=math.degrees(float(np.abs(hold[:, 2]).max())),
                    u_end_deg=math.degrees(float(hold[-1, 2])),
                    foot_off_s=self.air, tripped=self.tripped)


if __name__ == "__main__":
    # golden vectors for firmware/host/test_roll_pi.cpp
    p = RollPI(1.0, 0.5, math.radians(6))
    for e in (0.02, 0.05, 0.2, 0.2, 0.2, -0.1, -0.1):
        print(f"{e:.3f} -> u {p.step(e, 0.02):.6f}  i {p.i:.6f}")
    # with the gyro term: RollPI(kp 1.0, ki 0.5, clamp 6 deg, kd 0.1); (err, rate) -> u, i
    p = RollPI(1.0, 0.5, math.radians(6), kd=0.1)
    for e, w in ((0.02, 0.0), (0.02, 0.5), (0.02, -0.5), (0.0, 0.3), (0.1, 0.4), (0.1, 0.4), (-0.05, -2.0)):
        print(f"{e:.3f} {w:.3f} -> u {p.step(e, 0.02, w):.6f}  i {p.i:.6f}")
