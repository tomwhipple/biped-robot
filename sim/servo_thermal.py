"""The servos' temperature and their over-temperature cutoff (#6), shared by the training envs.

What is measured (docs/design-v6/2026-11-25-servo-thermal.md, experiments/servo-thermal/): every
flight record on mira carries each leg servo's temperature register (63, 1 degC) at 50 Hz, 40 h of
powered robot over 2026-10-30..11-17. A first-order model of that register fits every servo to the
register's own 1 degC quantisation (rmse 0.29-0.36 degC):

    tau_th dT/dt = T_idle + Rth P - T,        P = K_CU tau^2      (W; the sim's own heat)

with P the copper term of the sim's bridge current model (sts_servo_model / supply_sag: K_CU per
physical type), priced from the logged PWM duty and speed through the sim's envelope, so Rth pairs
with the heat the sim computes. Per type, pooled over its servos:
    STS3215 (10 servos)   tau_th 8 min,  Rth 16 K/W  (heat capacity ~30 J/K)
    STS3250 (2 hip rolls) tau_th 14 min, Rth 30 K/W  (its K_CU rests on an unverified 4.2 A stall)
T_idle is the powered, unloaded servo's equilibrium: the room plus the electronics' own standby
heating, 6-9 degC on every cold start on record (the room 17-25 degC). The cutoff is the STS's
register 13 default, 70 degC (servo-map section 4: above it torque goes off). Register 13 has not
been read on the robot.

What is not measured, so not modelled: any torque derating below the cutoff (the motor's copper and
magnets do lose torque warm, but no bench record shows how much at this sensor's reading); what the
servo does after it trips (recovery temperature, hysteresis) -- here a tripped servo stays limp for
the rest of the episode, as it would within any episode-length cool-down (tau_th is minutes); the
winding's own fast hot spot (the register sits on the board, not in the winding).

The sim, per control tick, in both envs:
    P   = mean over the tick's substeps of min(k tau^2, the stall power)   (a tripped servo: no heat)
          k = K_CU of the physical type; under an envelope PAIR (a refit of the servo that is there)
          k = K_CU (type stall / pair stall)^2, so a duty heats as it did in the fitted data
    T'  = T + (1 - exp(-dt / tau_th)) (T_idle + Rth P - T)
    off'= off | (T' >= T_CUTOFF)                        latched for the episode
and a servo that is `off` gives no torque from the next tick (its PD output is zeroed, as torque
release does). The start temperature is drawn per episode per servo, uniform in the option's range
(`servo_thermal="LO:HI"` degC; one value = fixed): within a 10-40 s episode a servo heats at most
a few degC (an STS3215 at 80 % of its 12 V stall: +0.7 degC/s), so what an episode can feel is a
robot that STARTS warm. Off by default (None), and then neither env computes anything here: the
default path is bit-identical.

Every function uses array operators only, so the same code runs under numpy (sim/walker_env.py) and
jax.numpy (sim/mjx/env_mjx.py) -- the pattern of supply_sag.py.
"""
from __future__ import annotations

import math

import numpy as np

import supply_sag  # physical_type: which servo is where (the bus map, servo_types)
import current_budget as CB  # SERVO_ELEC: K_CU per physical type

# per physical type: (thermal time constant s, thermal resistance K/W against K_CU tau^2), the pooled
# fit of experiments/servo-thermal/fit.json ("types")
THERMAL = {"sts3215": (480.0, 16.0),
           "sts3250": (840.0, 30.0)}
T_IDLE = 29.0               # degC: a powered, unloaded servo's equilibrium in a ~22 degC room
T_CUTOFF = 70.0             # degC: register 13's default; above it the STS turns torque off


def parse_thermal(spec):
    """servo_thermal from a kwarg, config.json or the CLI -> None (off) or (lo, hi) degC, the per
    episode start temperature's range. None / "" / "off" -> None; a number or "T" -> (T, T) fixed;
    "LO:HI" or a 2-sequence -> drawn per episode per servo, uniform."""
    if spec is None or (isinstance(spec, str) and spec.strip().lower() in ("", "off", "none")):
        return None
    if isinstance(spec, str):
        parts = [float(x) for x in spec.split(":")]
    elif isinstance(spec, (list, tuple)):
        parts = [float(x) for x in spec]
    else:
        parts = [float(spec)]
    if len(parts) == 1:
        parts = parts * 2
    lo, hi = parts if len(parts) == 2 else (None, None)
    if (lo is None or not (math.isfinite(lo) and math.isfinite(hi)) or hi < lo
            or lo < -20.0 or hi > 150.0):
        raise ValueError(f"servo_thermal wants T or LO:HI degC (start temperature), -20 <= LO <= HI "
                         f"<= 150; got {spec!r}")
    return (lo, hi)


def thermal_consts(names, servo_types=None, volts=12.0):
    """Per actuator of `names`: (k W/(N m)^2, the heat's cap W, tau_th s, Rth K/W), float64 numpy arrays,
    by each servo's physical type (supply_sag.physical_type: a named servo_types type, else the bus
    map's). The heat is the fit's: duty^2 x the stall power, duty = tau / the servo's full-duty torque,
    so k = K_CU of the physical type, except under an envelope PAIR (`--servo-envelope`, a refit of the
    servo that is there: its full-duty torque is the pair's), where k = K_CU x (type stall / pair
    stall)^2. The cap is the stall power at `volts` (the supply), I_stall12 x volts^2 / 12."""
    import walker_env as W        # lazy: walker_env imports this module
    k, cap, tau, rth = [], [], [], []
    for n in names:
        t = supply_sag.physical_type(n, servo_types)
        if t not in THERMAL:
            raise ValueError(f"servo type {t!r} has no thermal constants (have {sorted(THERMAL)})")
        i_st, _, k_cu = CB.SERVO_ELEC[t]
        spec = (servo_types or {}).get(n, (servo_types or {}).get(W.joint_role(n)))
        if spec is not None and not isinstance(spec, str):
            k_cu = k_cu * (W.SERVO_TYPES[t][0] / W._envelope(spec)[0]) ** 2
        k.append(k_cu)
        cap.append(i_st * volts * volts / 12.0)
        tau.append(THERMAL[t][0])
        rth.append(THERMAL[t][1])
    return tuple(np.array(x, dtype=np.float64) for x in (k, cap, tau, rth))


def heat(tau, consts, xp=np):
    """W per servo: the copper term of the bridge model, k tau^2, capped at the stall power."""
    return xp.minimum(consts[0] * tau * tau, consts[1])


def thermal_step(temp, off, p_mean, dt, consts, xp=np):
    """One control tick of dt seconds at mean heat p_mean (W per servo): (the new temperature, the
    new latched cutoff mask). consts = thermal_consts(...). Exact for a constant p over the tick."""
    tau_th, rth = consts[2], consts[3]
    g = 1.0 - xp.exp(-dt / tau_th)
    t2 = temp + g * (T_IDLE + rth * p_mean - temp)
    return t2, xp.logical_or(off, t2 >= T_CUTOFF)


def minutes_to_cutoff(p_mean, tau_th, rth, t_start=T_IDLE):
    """Minutes for a servo held at mean heat p_mean (W) from t_start (degC) to reach T_CUTOFF;
    inf if its steady temperature stays below. numpy only (reports, not the envs)."""
    p = np.asarray(p_mean, dtype=np.float64)
    tss = T_IDLE + rth * p
    with np.errstate(divide="ignore", invalid="ignore"):
        m = tau_th / 60.0 * np.log((tss - t_start) / (tss - T_CUTOFF))
    return np.where(tss > T_CUTOFF, np.where(t_start >= T_CUTOFF, 0.0, m), np.inf)
