"""STS3215 (ST-3215-C018, 12 V) protection and supply-current model for the
open-loop studies (get-up, gate walk). Every constant below is either the
manufacturer's documented number, with its source, or marked FITTED.

Sources (manufacturer documents):
  [DS]  ST-3215-C018 product specification, Feetech, ed. A/0 2023-07-20,
        docs/datasheets/st3215/ST-3215-C018-datasheet-12V-30kg.pdf
        section 5 (electrical), 7-11 (electronic protection)
  [MT]  ST3215 memory table V3.7 (Feetech, the xls Waveshare links from
        https://www.waveshare.com/wiki/ST3215_Servo ; the same file
        firmware/components/scsbus/include/scsbus/registers.h cites):
        https://files.waveshare.com/upload/2/27/ST3215%20memory%20register%20map-EN.xls

Protection, as documented:
  overload   [MT] reg 36 "overload torque" 80 (%): the load threshold that
             starts the protection timer; reg 35 "protection time" 200
             (x 10 ms = 2.0 s): how long the load output must stay above it;
             reg 34 "protection torque" 20 (%): "output torque after
             entering overload protection ... 20 % of the maximum torque".
             [DS] 7-11: "stalled above 80 % of stall for 2 s -> protection;
             sending a new position command clears the overload flag".
             Reg 19 (unloading conditions) default 44 = bits 2, 3, 5:
             temperature, current and OVERLOAD protection are ON by default.
  overcurrent [DS] 7-11: "running current above 2 A for 2 s -> output OFF;
             a new position command clears it". [MT] reg 28 "protection
             current" default 500 x 6.5 mA = 3.25 A, reg 38 "overcurrent
             protection time" 200 x 10 ms = 2.0 s. The two documents
             DISAGREE on the threshold (2 A vs 3.25 A); both are modelled.
             At 11.1 V the model's stall current is 2.50 A, so the memory-
             table default can never fire; the datasheet's 2 A can.

What "load" means: [MT] reg 60 "present load" is "the voltage duty cycle of
the current control output driving the motor", and reg 35 times "the current
load output" against reg 36. So the protection compares PWM DUTY, as a
fraction of the maximum (reg 16/48 = 1000 = 100 % of locked-rotor torque).
In the envelope model (walker_env: cap = stall * (1 - |w|/w0), a DC motor at
full duty) the duty that produces torque tau at speed w is
    duty = tau / stall + w / w0          (signed, joint convention)
so a quasi-static hold reads duty = |tau| / stall, and a joint moving with
its load reads higher. `load="torque"` uses |tau| / stall alone.

What is NOT documented and is therefore a stated modelling choice:
  * how the servo LEAVES protection other than by a new position command.
    The firmware streams a goal every 20 ms tick; whether an unchanged goal
    counts as "new" is a bench question (#83). Default here: LATCHED for
    the rest of the run (the worst case); `release="goal_change"` clears
    it on the next tick whose commanded goal differs by >= 1 tick.
  * whether the timer resets when the load dips below the threshold.
    [MT] reg 35's wording ("exceeds ... and remains") is a CONTINUOUS
    timer, the default here; `timer="cumulative"` never resets (an upper
    bound). Both totals are always reported.

Supply current, per servo (the bus current is the sum):
  documented [DS] 5: stall 30 kg.cm (2.94 N.m) at 2.7 A at 12 V; Kt
  11 kg.cm/A (1.079 N.m/A); no-load running current 180 mA; idle (stopped)
  30 mA; terminal resistance 1.0 ohm; the [DS] section 10 curve draws I
  linear in torque from ~0.18 A to 2.7 A along the full-duty line.
  Two models bracket the truth, because the documents give the motor's
  current along the FULL-DUTY line, and the bus current of a PWM H-bridge
  at partial duty is lower:
    motor   ("upper"): I = I_idle + (I_noload - I_idle) * |w| / w0
                           + |tau| / Kt, clamped to the stall current at
            the supply voltage (2.7 A * V / 12). Exactly the [DS] curve at
            full duty; an upper bound for a servo holding a load at low
            speed. Constants: all documented; the |w| / w0 interpolation of
            the no-load current between idle and full speed is FITTED
            (a straight line between two documented points).
    bridge  ("lower"): the lossless-bridge power balance sim/current_budget.py
            and walker_env use: I = I_idle + (max(tau * w, 0) + K_CU tau^2) / V,
            K_CU = 3.75 W/(N.m)^2 FITTED so that the 2.94 N.m stall draws
            2.7 A at 12 V (an effective 4.4 ohm, not the documented 1.0 ohm
            terminal resistance; the gap is not explained by the documents).
  The truth for the averaged supply current lies between them; the pack-lead
  shunt on the first powered run (#77) settles it.
"""
from __future__ import annotations

import numpy as np

# ------------------------------------------------------------------ documented
KGCM = 0.0980665                        # N.m per kg.cm
STALL_NM_12V = 30.0 * KGCM              # [DS] 5-4: 2.94 N.m
I_STALL_12V = 2.7                       # [DS] 5-5, A
I_NOLOAD = 0.180                        # [DS] 5-3, A (running, no load, 12 V)
I_IDLE = 0.030                          # [DS] 5-6, A (stopped)
KT = 11.0 * KGCM                        # [DS] 5-11: 11 kg.cm/A = 1.079 N.m/A
R_TERMINAL = 1.0                        # [DS] 5-10, ohm (not used: see K_CU)

OVERLOAD_FRAC = 0.80                    # [MT] reg 36 default 80 (%)
PROTECT_TIME_S = 2.0                    # [MT] reg 35 default 200 x 10 ms
PROTECT_FRAC = 0.20                     # [MT] reg 34 default 20 (%)
OVERCURRENT_A_DS = 2.0                  # [DS] 7-11
OVERCURRENT_A_MT = 500 * 0.0065         # [MT] reg 28 default: 3.25 A
OVERCURRENT_TIME_S = 2.0                # [MT] reg 38 default 200 x 10 ms, [DS] 7-11

# ------------------------------------------------------------------ fitted
K_CU = 3.75                             # W/(N.m)^2, walker_env._K_CU: 2.94 N.m -> 2.7 A at 12 V


def supply_current(tau, qd, w0, volts, model="motor"):
    """per-servo supply current (A) from the joint torque tau (N.m) and speed
    qd (rad/s); arrays broadcast. w0 = that servo's no-load speed (rad/s)."""
    tau = np.asarray(tau, dtype=float)
    qd = np.asarray(qd, dtype=float)
    if model == "motor":
        i_stall = I_STALL_12V * volts / 12.0
        i = I_IDLE + (I_NOLOAD - I_IDLE) * np.clip(np.abs(qd) / w0, 0.0, 1.0) + np.abs(tau) / KT
        return np.minimum(i, i_stall)
    if model == "bridge":
        return I_IDLE + (np.maximum(tau * qd, 0.0) + K_CU * tau ** 2) / volts
    raise ValueError(model)


class STSProtection:
    """Per-servo overload (and optional overcurrent) protection on top of the
    walker_env "sts3215" envelope, applied at the 50 Hz control tick.

    Attach AFTER the caller has written its per-joint env._servo arrays
    (kp, kd, stall, w0); call tick(env, dt) after every env.step(). With
    enforce=False it only measures (the report is identical, the plant is
    untouched). When a servo trips, its stall entry in env._servo is set to
    PROTECT_FRAC x its own stall (overload) or 0 (overcurrent, "output
    off"), which the envelope clamp then applies from the next substep on.

    Per servo it reports: peak load (fraction of that servo's own maximum),
    the longest CONTINUOUS time above the threshold, the CUMULATIVE time
    above it, and the trip time (None if it never tripped)."""

    def __init__(self, env, names, enforce=True, load="duty", timer="continuous",
                 release="latch", overcurrent_a=None, volts=11.1):
        kp, kd, stall, w0 = env._servo
        na = env._nq_act
        self.names = list(names)[:na]
        self.stall0 = np.array(np.broadcast_to(np.asarray(stall, dtype=float), (na,)), dtype=float)
        self.w0 = np.array(np.broadcast_to(np.asarray(w0, dtype=float), (na,)), dtype=float)
        # env._servo may hold scalars; make stall a per-joint array we can edit in place
        env._servo = np.array([kp, kd, self.stall0.copy(), w0], dtype=object)
        self.enforce, self.load, self.timer, self.release = enforce, load, timer, release
        self.overcurrent_a, self.volts = overcurrent_a, volts
        self.t = 0.0
        self.cont = np.zeros(na)          # current continuous stretch above the threshold
        self.cont_max = np.zeros(na)
        self.cum = np.zeros(na)
        self.peak = np.zeros(na)
        self.trip_t = [None] * na
        self.tripped = np.zeros(na, bool)
        self.oc_cont = np.zeros(na)
        self.oc_cont_max = np.zeros(na)
        self.oc_trip_t = [None] * na
        self.peak_i = np.zeros(na)
        self._last_goal = None
        # both readings of "load" are always measured (the enforced one is self.load)
        self.alt = {k: dict(peak=np.zeros(na), cont=np.zeros(na), cont_max=np.zeros(na), cum=np.zeros(na))
                    for k in ("torque", "duty")}

    def _loads(self, tau, qd):
        return dict(torque=np.abs(tau) / self.stall0, duty=np.abs(tau / self.stall0 + qd / self.w0))

    def tick(self, env, dt):
        na = len(self.stall0)
        tau = np.asarray(env._servo_tau, dtype=float)[:na]
        qd = np.asarray(env.data.qvel[env._jqvel], dtype=float)[:na]
        lds = self._loads(tau, qd)
        for k, a in self.alt.items():
            o = lds[k] > OVERLOAD_FRAC
            a["peak"] = np.maximum(a["peak"], lds[k])
            a["cum"] += np.where(o, dt, 0.0)
            a["cont"] = np.where(o, a["cont"] + dt, 0.0)
            a["cont_max"] = np.maximum(a["cont_max"], a["cont"])
        ld = lds[self.load]
        self.peak = np.maximum(self.peak, ld)
        over = ld > OVERLOAD_FRAC
        self.cum += np.where(over, dt, 0.0)
        if self.timer == "cumulative":
            self.cont = self.cum.copy()
        else:
            self.cont = np.where(over, self.cont + dt, 0.0)
        self.cont_max = np.maximum(self.cont_max, self.cont)
        i_m = supply_current(tau, qd, self.w0, self.volts, "motor")
        self.peak_i = np.maximum(self.peak_i, i_m)
        if self.overcurrent_a is not None:
            oc = i_m > self.overcurrent_a
            self.oc_cont = np.where(oc, self.oc_cont + dt, 0.0)
            self.oc_cont_max = np.maximum(self.oc_cont_max, self.oc_cont)
        self.t += dt
        stall = env._servo[2]
        goal = np.asarray(env.data.ctrl, dtype=float)[:na]
        if self.release == "goal_change" and self._last_goal is not None:
            moved = np.abs(goal - self._last_goal) >= (2 * np.pi / 4096)
            for i in np.nonzero(self.tripped & moved)[0]:
                self.tripped[i] = False
                self.cont[i] = 0.0
                if self.enforce:
                    stall[i] = self.stall0[i]
        self._last_goal = goal.copy()
        for i in range(na):
            if not self.tripped[i] and self.cont[i] >= PROTECT_TIME_S:
                self.tripped[i] = True
                if self.trip_t[i] is None:
                    self.trip_t[i] = round(self.t, 2)
                if self.enforce:
                    stall[i] = PROTECT_FRAC * self.stall0[i]
            if (self.overcurrent_a is not None and self.oc_trip_t[i] is None
                    and self.oc_cont[i] >= OVERCURRENT_TIME_S):
                self.oc_trip_t[i] = round(self.t, 2)
                if self.enforce:
                    stall[i] = 0.0

    def report(self):
        """{joint: dict(peak, cont_max, cum, trip_t, peak_i, oc_cont_max, oc_trip_t,
        and per reading: peak_torque, cont_max_torque, cum_torque, peak_duty, ...)}"""
        out = {}
        for i, n in enumerate(self.names):
            r = dict(peak=float(self.peak[i]), cont_max=float(self.cont_max[i]), cum=float(self.cum[i]),
                     trip_t=self.trip_t[i], peak_i=float(self.peak_i[i]),
                     oc_cont_max=float(self.oc_cont_max[i]), oc_trip_t=self.oc_trip_t[i])
            for k, a in self.alt.items():
                r[f"peak_{k}"] = float(a["peak"][i]); r[f"cont_max_{k}"] = float(a["cont_max"][i])
                r[f"cum_{k}"] = float(a["cum"][i])
            out[n] = r
        return out
