"""The servo bus's supply sag under load (#6), shared by the training envs.

The bench supply measured 2026-11-03 sags as V = 12.02 - 0.44 ohm x I (11.7 V below 1 A, 11.0 V at
2-3 A, 10.7 V at the 3.3 A first-step kick; hw_sessions/2026-11-03, notes hw_sessions/2026-11-04/
NOTES.md). The 0.44 ohm is the leads and connectors, not the pack (a 3S pack is ~0.05-0.1 ohm).
Without this term the sim's supply is a fixed `supply_voltage`, so a servo can hold stall torque
from a supply that never droops.

The model, per physics substep (2 ms), in both envs:

    V  = V0 x s,                    s = the supply scale the servos run on this substep
    I  = bus_current(tau, qd, V)    the summed current of every servo on the bus (held ones too)
    s' = s + SAG_RELAX x (sag_scale(I, R, V0) - s)      the next substep's scale,
         sag_scale(I, R, V0) = clip(1 - R I / V0, SAG_FLOOR, 1)

and each servo's stall torque and no-load speed are multiplied by s, exactly as `supply_voltage`
scales them (~V/12) -- the envelope is a DC motor's, linear in the supply. V0 is the env's
`supply_voltage` (the open-circuit voltage). The current is the lossless-bridge power balance
walker_env / env_mjx already price as `power_w` and sim/current_budget.py budgets with
(sts_servo_model.supply_current "bridge"): I = sum(I_IDLE + (max(tau qd, 0) + K_CU tau^2) / V),
the datasheet's 2.7 A at 12 V stall. That is the LOWER of sts_servo_model's two current models,
so the sag it gives is the smaller one.

Why one substep late, and relaxed: tau depends on the supply through the envelope's cap and the
supply on tau through the current, so this substep's current sets the NEXT substep's supply (the
explicit form: no fixed-point solve inside the physics loop). Taken whole, that update rings
period-2 under a saturated load: a servo at its cap draws a current proportional to the supply,
so a high supply gives a big current gives a low supply, and back (seen on random actions: the
scale flipping 0.73 / 0.50 every substep). Moving halfway to the loaded value each substep
(SAG_RELAX = 0.5, a ~3 ms time constant, a stand-in for the bus capacitance) damps it: it settles
onto the loaded supply in a few substeps (tests/test_supply_sag.py pins a case that rings whole and
settles relaxed). Both lags are far below the 20 ms control
tick and what the bench's 50 Hz records resolve. The scale is carried across control ticks and
starts at 1 (no load) at reset. SAG_FLOOR (half the supply) only keeps the arithmetic finite: the
servos would brown out long before (the bench never read below 10.7 V).

R is fixed (`supply_sag_ohm=0.44`) or drawn per episode, uniform (`"LO:HI"`). Off by default
(None), and then neither env computes anything here: the default path is bit-identical.

Every function uses array operators only, so the same code runs under numpy (sim/walker_env.py)
and jax.numpy (sim/mjx/env_mjx.py) -- the pattern of sim/trim_kernel.py and sim/cmd_kernel.py.
"""
from __future__ import annotations

import math

import numpy as np

BENCH_OHM = 0.44            # the 2026-11-03 bench supply's lead + connector resistance
BENCH_V0 = 12.02            # its open-circuit voltage (the env's supply_voltage stands in)
I_IDLE = 0.030              # A per servo, stopped ([DS] 5-6; sts_servo_model.I_IDLE)
K_CU = 3.75                 # W/(N m)^2: 2.94 N m stall draws 2.7 A at 12 V (walker_env._K_CU)
SAG_FLOOR = 0.5             # the scale's floor: an arithmetic guard, not a brown-out model
SAG_RELAX = 0.5             # fraction of the way to the loaded supply per substep (see above)


def parse_sag(spec):
    """supply_sag_ohm from a kwarg, config.json or the CLI -> None (off) or (lo, hi) ohms.
    None / "" / 0 -> None; a number or "R" -> (R, R) fixed; "LO:HI" or a 2-sequence -> drawn per
    episode, uniform."""
    if spec is None or (isinstance(spec, str) and not spec.strip()):
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
    if lo is None or not (math.isfinite(lo) and math.isfinite(hi)) or lo < 0.0 or hi < lo:
        raise ValueError(f"supply_sag_ohm wants R or LO:HI ohms, 0 <= LO <= HI; got {spec!r}")
    if hi == 0.0:
        return None
    return (lo, hi)


def bus_current(tau, qd, volts, xp=np):
    """The bus current (A): every servo's lossless-bridge draw at supply `volts`, summed over the
    last axis. tau (N m), qd (rad/s): the servos' torque and joint speed this substep."""
    return xp.sum(I_IDLE + (xp.maximum(tau * qd, 0.0) + K_CU * tau * tau) / volts, axis=-1)


def sag_scale(i_bus, r_ohm, v0, xp=np):
    """V / V0 at bus current i_bus (A) through r_ohm from an open-circuit v0 (V), floored."""
    return xp.clip(1.0 - r_ohm * i_bus / v0, SAG_FLOOR, 1.0)


def sag_step(tau, qd, scale, r_ohm, v0, xp=np):
    """One substep: the servos ran on V = v0 x scale with torques tau at speeds qd ->
    (the next substep's scale, the bus current). The scale moves SAG_RELAX of the way to the
    loaded supply's."""
    i_bus = bus_current(tau, qd, v0 * scale, xp)
    return scale + SAG_RELAX * (sag_scale(i_bus, r_ohm, v0, xp) - scale), i_bus
