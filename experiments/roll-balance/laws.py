"""Candidate balance laws for the testbed (testbed.run(law=...)). Each is called once per 20 ms tick:

    law.tick(t, meas, phase) -> {joint: rad}     added to the commanded targets
    meas  = (roll, pitch, roll_rate, pitch_rate), rad and rad/s, IMU-delayed
    phase = None (both feet down) or {stance, swing, t0 (raise began), raise_end, t1 (lowered)}

Sign convention (derived from experiments/stepper/kin.py and checked in the testbed): with
e = roll - roll_ref (roll + = leaning to the robot's right), a stance HIP roll joint change of +K e (sim
joint frame, either side) rolls the pelvis back toward level: that is the stepper's own trim (`hike`). The
same on the stance ANKLE roll rolls the whole body back over the stance foot. The old firmware loop's
default mix (ankle +u, hip -u) is a lateral pelvis translation with no net torso tilt.
"""
from __future__ import annotations

import math

D2R = math.pi / 180.0


def _clip(x, lim):
    return max(-lim, min(lim, x))


class Law:
    name = "off"

    def reset(self):
        self.ref = None
        self.ph = None
        self.u = 0.0           # last hip correction, rad
        self.ua = 0.0          # last ankle correction, rad
        self.fade_from = None
        self.last = (0.0, 0.0)

    def joints(self, stance, hip, ankle, pitch=0.0):
        d = {}
        if stance is not None:
            d[f"{stance}_hip_roll"] = hip
            d[f"{stance}_ankle_roll"] = ankle
            if pitch:
                d[f"{stance}_ankle"] = pitch
        self.last = (hip, ankle)
        return d


class OldPI(Law):
    """firmware/components/balance/roll_pi (kp 0.15, ki 1.0, kd 0, ankle +1 / hip -1, clamp 10 deg, I share
    3 deg, abort 12 deg). mode "through": engaged when the raise begins (reference = the roll then) and run
    through the lift, the way that fell outward 3/3 on the bench; mode "armed": as built (u held at 0 until
    the raise move ends or |err| > 4 deg, then the reference is taken there). Disengaged when the foot is
    down; an abort fades u to 0 over 0.3 s (the firmware's landing is not modelled)."""

    def __init__(self, mode="through", kp=0.15, ki=1.0, kd=0.0, ka=1.0, kh=-1.0, clamp=10.0, i_lim=3.0,
                 abort=12.0, arm_start=4.0):
        self.mode, self.kp, self.ki, self.kd, self.ka, self.kh = mode, kp, ki, kd, ka, kh
        self.clamp, self.i_lim, self.abort, self.arm_start = clamp * D2R, i_lim * D2R, abort * D2R, arm_start * D2R
        self.name = f"oldPI-{mode}"

    def reset(self):
        super().reset()
        self.i = 0.0
        self.state = "off"
        self.aborts = 0
        self.t_abort = None

    def tick(self, t, meas, ph):
        roll, _, rate, _ = meas
        if ph is None:
            if self.state != "off":
                self.state, self.u, self.i = "off", 0.0, 0.0
            return {}
        if self.ph is not ph:
            self.ph, self.i, self.u = ph, 0.0, 0.0
            self.ref = roll
            self.state = "armed" if self.mode == "armed" else "active"
        e = roll - self.ref
        if self.state == "abort":
            k = max(0.0, 1.0 - (t - self.t_abort) / 0.3)
            return self.joints(ph["stance"], self.kh * self.u_abort * k, self.ka * self.u_abort * k)
        if abs(e) > self.abort:
            self.state, self.t_abort, self.u_abort = "abort", t, self.u
            self.aborts += 1
            return self.joints(ph["stance"], self.kh * self.u, self.ka * self.u)
        if self.state == "armed":
            if t >= ph["raise_end"] or abs(e) > self.arm_start:
                self.state, self.ref, self.i = "active", roll, 0.0
            return self.joints(ph["stance"], 0.0, 0.0)
        unsat = self.kp * e + self.ki * self.i + self.kd * rate
        sat = _clip(unsat, self.clamp)
        if not (sat != unsat and (e > 0) == (sat > 0)):
            self.i += e * 0.02
        if self.i_lim > 0 and self.ki > 0:
            self.i = _clip(self.i, self.i_lim / self.ki)
        self.u = sat
        return self.joints(ph["stance"], self.kh * self.u, self.ka * self.u)


class HipLevel(Law):
    """Hold the pre-lift body roll with the STANCE hip roll (the stepper's trim, on the tick):
        e  = roll - roll(raise began)
        uh = clip(kp e + kd roll_rate + ki I, lim)        stance hip roll += uh
        ua = clip(ka e + kad roll_rate, lim_a)            stance ankle roll += ua   (ka = 0: hip only)
    I integrates e only while |uh| < lim (anti-windup), capped at i_lim. Outward rule (out_rate deg/s, 0 =
    off): the part of uh that rolls the body toward the stance side may grow at most this fast. After the
    foot is down the corrections fade to 0 over `fade` s (the next shift starts from a level pelvis)."""

    def __init__(self, kp=1.0, kd=0.0, ki=0.0, ka=0.0, kad=0.0, lim=15.0, lim_a=8.0, i_lim=5.0, out_rate=0.0,
                 fade=0.6, name=None):
        self.kp, self.kd, self.ki, self.ka, self.kad = kp, kd, ki, ka, kad
        self.lim, self.lim_a, self.i_lim, self.out_rate, self.fade = lim * D2R, lim_a * D2R, i_lim * D2R, out_rate * D2R, fade
        self.name = name or f"hip kp{kp:g} kd{kd:g} ki{ki:g} ka{ka:g}" + (f" out{out_rate:g}" if out_rate else "")

    def reset(self):
        super().reset()
        self.i = 0.0
        self.t_end = None
        self.stance = None

    def tick(self, t, meas, ph):
        roll, _, rate, _ = meas
        if ph is None:
            if self.stance is None:
                return {}
            if self.t_end is None:
                self.t_end = t
            k = max(0.0, 1.0 - (t - self.t_end) / self.fade) if self.fade > 0 else 0.0
            if k == 0.0:
                st, self.stance = self.stance, None
                return self.joints(st, 0.0, 0.0)
            return self.joints(self.stance, self.u * k, self.ua * k)
        if self.ph is not ph:
            self.ph, self.ref, self.i, self.t_end, self.stance = ph, roll, 0.0, None, ph["stance"]
            self.u0, self.ua0 = self.u, self.ua      # a correction still fading from the last phase carries over
            self.u, self.ua = 0.0, 0.0
        e = roll - self.ref
        s = 1.0 if ph["stance"] == "L" else -1.0       # s*uh > 0: the hip rolls the body toward the stance side
        uh = self.kp * e + self.kd * rate + self.ki * self.i
        uh_c = _clip(uh, self.lim)
        if self.ki and abs(uh) < self.lim:
            self.i = _clip(self.i + e * 0.02, self.i_lim / max(self.ki, 1e-9))
        if self.out_rate > 0:
            out_now = max(0.0, s * uh_c)
            out_prev = max(0.0, s * self.u)
            if out_now > out_prev + self.out_rate * 0.02:
                uh_c = s * (out_prev + self.out_rate * 0.02) + (uh_c - s * out_now) * 0.0
        self.u = uh_c
        self.ua = _clip(self.ka * e + self.kad * rate, self.lim_a)
        return self.joints(ph["stance"], self.u, self.ua)


class Trim(Law):
    """The stepper's stance-hip trim, closed on the tick, with a per-side learned trim:
        e  = roll - roll(raise began)                  (rad; + = leaning toward the robot's right)
        uh = r(t) * T[stance] + kp e + kd roll_rate     stance hip roll += uh   (clip lim)
        T[stance] += ki e dt                            while closed (|uh| < lim), within +-t_lim
        ua = ka e                                       stance ankle roll += ua (clip lim_a)
    r(t) ramps 0 -> 1 over the raise move (min-jerk), so the learned trim T is applied as the foot is
    unloaded; T persists per stance side from one step to the next (the stepper's learned `hike`), so a
    faster gait does not need a fast integrator. Closed from the raise to the start of the lowering; held
    through the lowering and the touch-down; faded to 0 over `fade` s after the foot is down."""

    def __init__(self, kp=1.0, kd=0.0, ki=1.0, ka=0.0, lim=15.0, lim_a=8.0, t_lim=14.0, t0=None, fade=0.6,
                 out_rate=0.0, kt=0.0, t_lim_t=6.0, name=None):
        self.kp, self.kd, self.ki, self.ka = kp, kd, ki, ka
        # outward catch (kt, s): while the body rolls OUTWARD (toward the stance side), a pelvis translation
        # inboard, ut = kt * roll_rate (stance ankle += ut, hip -= ut: no net torso tilt), one-sided, within
        # +-t_lim_t deg; it decays (0.3 s) as soon as the outward roll stops
        self.kt, self.t_lim_t = kt, t_lim_t * D2R
        self.lim, self.lim_a, self.t_lim, self.fade = lim * D2R, lim_a * D2R, t_lim * D2R, fade
        self.out_rate = out_rate * D2R
        self.T0 = {k: v * D2R for k, v in (t0 or {"L": 0.0, "R": 0.0}).items()}
        self.name = name or f"trim kp{kp:g} kd{kd:g} ki{ki:g} ka{ka:g}" + (f" out{out_rate:g}" if out_rate else "") + \
            (f" kt{kt:g}" if kt else "") + \
            (f" T0 {t0['L']:g}/{t0['R']:g}" if t0 else "")

    def reset(self):
        super().reset()
        self.T = dict(self.T0)
        self.stance = None
        self.t_end = None
        self.hold = None
        self.ut = 0.0

    def tick(self, t, meas, ph):
        roll, _, rate, _ = meas
        if ph is None:
            if self.stance is None:
                return {}
            if self.t_end is None:
                self.t_end = t
            k = max(0.0, 1.0 - (t - self.t_end) / self.fade) if self.fade > 0 else 0.0
            st = self.stance
            if k == 0.0:
                self.stance = None
            return self.joints(st, self.u * k, self.ua * k)
        if self.ph is not ph:
            self.ph, self.ref, self.t_end, self.stance, self.hold = ph, roll, None, ph["stance"], None
            self.u, self.ua = 0.0, 0.0
        if t >= ph.get("lower_start", 1e9):
            return self.joints(ph["stance"], self.u, self.ua)        # held through the lowering
        st = ph["stance"]
        e = roll - self.ref
        dur = max(ph["raise_end"] - ph["t0"], 0.05)
        u_ = min(max((t - ph["t0"]) / dur, 0.0), 1.0)
        r = u_ * u_ * u_ * (10 - 15 * u_ + 6 * u_ * u_)
        uh = r * self.T[st] + self.kp * e + self.kd * rate
        uh_c = _clip(uh, self.lim)
        if self.ki and abs(uh) < self.lim:
            self.T[st] = _clip(self.T[st] + self.ki * e * 0.02, self.t_lim)
        if self.out_rate > 0:
            s = 1.0 if st == "L" else -1.0
            if s * uh_c > s * self.u + self.out_rate * 0.02 and s * uh_c > 0:
                uh_c = self.u + s * self.out_rate * 0.02
        self.u = uh_c
        self.ua = _clip(self.ka * e, self.lim_a)
        if self.kt:
            s = 1.0 if st == "L" else -1.0
            want = _clip(self.kt * rate, self.t_lim_t) if s * rate < 0 else 0.0
            if abs(want) >= abs(self.ut):
                self.ut = want
            else:
                self.ut *= math.exp(-0.02 / 0.3)
            return self.joints(st, self.u - self.ut, self.ua + self.ut)
        return self.joints(st, self.u, self.ua)
