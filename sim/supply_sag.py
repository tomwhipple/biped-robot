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
sim/current_budget.py budgets with (`servo_amps` "bridge"), each servo with its own PHYSICAL type's
constants (current_budget.SERVO_ELEC: K_CU and the stall current at 12 V), and capped at that
servo's real stall current at the supply:

    I = sum_j min(I_IDLE + (max(tau_j qd_j, 0) + K_CU_j tau_j^2) / V,  I_STALL12_j x V / 12)

An STS3215 (K_CU 3.75) draws the datasheet's 2.7 A at its 12 V stall, an STS3250 (K_CU ~2.10) the
listed 4.2 A at its 4.90 N m. The physical type (`servo_elec`) is the one a servo_types value
names ("sts3250"); a servo with no named type, or with an envelope PAIR (`--servo-envelope
hip_pitch=2.0,...`: a refit or a what-if scaling of the servo that is there, not a bigger servo),
is the type the bus map puts there (current_budget.servo_type: the STS3250 hip rolls, STS3215
elsewhere). The cap is what keeps an envelope-scaled torque from being priced as real current: a
knee scaled to 2x stall still draws at most the STS3215's 2.7 A (not the 10.8 A that K_CU tau^2
gives at 5.9 N m). The bridge form is the LOWER of sts_servo_model's two current models, so the sag
it gives is the smaller one.

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

import current_budget as CB  # per servo type: SERVO_ELEC (stall A at 12 V, Kt, K_CU), the bus map

BENCH_OHM = 0.44            # the 2026-11-03 bench supply's lead + connector resistance
BENCH_V0 = 12.02            # its open-circuit voltage (the env's supply_voltage stands in)
I_IDLE = CB.SM.I_IDLE       # A per servo, stopped ([DS] 5-6; sts_servo_model.I_IDLE)
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


def physical_type(name, servo_types=None):
    """The servo type actuator `name` physically is, for its current: a servo_types value (keyed by
    actuator or role, the actuator's winning) that names a type is that type; unset, or an envelope
    pair (stall ratio, no-load ratio), is the bus map's type there (current_budget.servo_type),
    STS3215 for an actuator not on the map."""
    if servo_types:
        from walker_env import joint_role
        t = servo_types.get(name, servo_types.get(joint_role(name)))
        if isinstance(t, str):
            if t not in CB.SERVO_ELEC:
                raise ValueError(f"servo type {t!r} has no electrical constants in "
                                 f"current_budget.SERVO_ELEC (have {sorted(CB.SERVO_ELEC)})")
            return t
    return CB.servo_type(name) if name in CB.NAME_TO_ID else "sts3215"


def servo_elec(names, servo_types=None):
    """Per actuator of `names`: (K_CU W/(N m)^2, stall current A at 12 V), float64 numpy arrays,
    from current_budget.SERVO_ELEC by each servo's physical type (`physical_type`)."""
    e = [CB.SERVO_ELEC[physical_type(n, servo_types)] for n in names]
    return (np.array([x[2] for x in e], dtype=np.float64),
            np.array([x[0] for x in e], dtype=np.float64))


def bus_current(tau, qd, volts, elec, xp=np):
    """The bus current (A): every servo's lossless-bridge draw at supply `volts`, each with its own
    type's K_CU and capped at its real stall current at `volts`, summed over the last axis.
    tau (N m), qd (rad/s): the servos' torque and joint speed this substep; elec = (k_cu, i_stall12)
    per servo (`servo_elec`)."""
    k_cu, i_stall12 = elec
    i = I_IDLE + (xp.maximum(tau * qd, 0.0) + k_cu * tau * tau) / volts
    return xp.sum(xp.minimum(i, i_stall12 * (volts / 12.0)), axis=-1)


def sag_scale(i_bus, r_ohm, v0, xp=np):
    """V / V0 at bus current i_bus (A) through r_ohm from an open-circuit v0 (V), floored."""
    return xp.clip(1.0 - r_ohm * i_bus / v0, SAG_FLOOR, 1.0)


def sag_step(tau, qd, scale, r_ohm, v0, elec, xp=np):
    """One substep: the servos ran on V = v0 x scale with torques tau at speeds qd ->
    (the next substep's scale, the bus current). The scale moves SAG_RELAX of the way to the
    loaded supply's. elec: (k_cu, i_stall12) per servo (`servo_elec`)."""
    i_bus = bus_current(tau, qd, v0 * scale, elec, xp)
    return scale + SAG_RELAX * (sag_scale(i_bus, r_ohm, v0, xp) - scale), i_bus
