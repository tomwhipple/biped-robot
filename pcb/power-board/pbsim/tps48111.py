"""Behavioural model of the TPS48111-Q1 high-side servo-rail switch.

Scope: the EN/UVLO divider, the INP input, the circuit-breaker timer, the
short-circuit fast trip, the gate-slew soft start, and the IMON current
mirror. NOT modelled: the internal charge pump details, the overtemperature
path (Q5's diode), I²C — this IC has none.

Datasheet numbers (TPS4811-Q1 SLUSEE5E, confirmed against schematic values
in test_schematic_intact and TI's calculator in C3):

    V(UVLOR)            1.18 V  — EN/UVLO above it (with INP high) drives
                                  the gate; × (R30+R31)/R31 = 7.79 V of PACK+
    V(UVLOF)            1.11 V  — below it the FET is pulled off regardless
                                  of INP; × 6.6 = 7.33 V of PACK+ (C3: 7.39)
    V(ENF)              0.3 V min — below it the part shuts down; a latched
                                  fault resets ("Toggle INP or EN/UVLO
                                  (below ENF) ... to reset the latch", §8.3.3.2)
    V(INP) high         2 V max — 3.3 V logic through R32/R33 (≈3.27 V) passes
    I_OC  (circuit breaker)   Eq. 6: 11.9 × R25 / (R24 × R34) = 19.8 A
    t_OC  (latch-off timer)   Eq. 10: C19 × R35 × ln(1 / (1 − 1.2 V /
                              (R35 × 82 µA))) = 10.8 ms; TI's calculator
                              (C3) gives 10.5 ms, which the model uses
    I_SC  (short circuit)     Eq. 11: (R26 + 464 Ω) × 15.6 µA / R24 =
                              39.8 A; the calculator (C3) gives 39.3 A,
                              which the model uses
    t_SC                      ≤ 1.6 µs trip + PD-at-1.34 Ω pull-down
    test_rail_switch_protections derives each from the parsed schematic.

Soft start: at enable the gate driver charges C16 (330 nF) at ~10 µA,
taking ≈ 40 ms to reach V(GS) ≥ Vth — C4 of the verification record gives
rail-at-90 % in 40.2 ms into a 2 mF servo load, 40.5 ms into 4 mF. We
model the rail rise as exponential with τ chosen so v(rail) reaches 90 %
of final at 40 ms. The soft-start inrush peak (≈1.4–2 A) is what C4's
ngspice runs measured; we don't re-do that here — the
``test_servo_branches.py`` assertions check that the soft start keeps the
inrush below the F1 fuse's "no-melt" band and below the TPS48111's own
19.8 A trip with order-of-magnitude margin.

This model exists to answer the SEQUENCE and THRESHOLD questions — what
happens when a button is pressed, whether a faulted rail recovers, whether
one stuck NC contact defeats the stop. It does not try to reproduce
C4–C6's waveform shapes.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto


class RailState(Enum):
    OFF = auto()             # no gate drive; SERVO+ at 0 V (minus leakage)
    STARTING = auto()        # gate-slew ramp in progress
    ON = auto()              # conducting, gate fully on
    LATCHED_CB = auto()      # circuit breaker tripped and latched off
    LATCHED_SC = auto()      # short-circuit tripped, latched


# Datasheet + C3 + schematic-verified values. R/C numbers here are PINNED to
# what the verification record computed *from the schematic*; the test
# schematic_intact_test asserts the schematic itself carries them.
VU_VLO_RISING = 1.18     # V(UVLOR) typ
VU_VLO_FALLING = 1.11    # V(UVLOF) typ
V_ENF_MIN = 0.3          # V(ENF) min: EN below it is a shutdown for sure
VINP_H = 2.0

CB_TRIP_A = 19.8         # at R24=2mΩ, R25=100, R34=30.1k (C3)
CB_DELAY_S = 10.5e-3     # C3: "10.5 ms (FLT at 9.7 ms)"
SC_TRIP_A = 39.3         # at R26=4.64 kΩ (C3)
SC_DELAY_S = 1.6e-6      # short-circuit trip + PD time; sub-µs; modeled as
                         # next-tick because our timescale is 1 ms

SOFT_START_T90_S = 40.0e-3   # C4: rail at 90 % in ~40 ms (servo cap 2–4 mF)
# τ such that 1 - exp(-t/τ) = 0.9 at t=40 ms: τ = t/ln(10) ≈ 17.4 ms
_SOFT_START_TAU_S = SOFT_START_T90_S / 2.302585


@dataclass
class Tps48111:
    """Servo-rail switch + its Q3 pass FET + R24 sense + EN divider.

    All currents in amps (positive = rail sourcing load). All times in
    seconds.

    Inputs to update():
      t_s          clock
      dt_s         tick length
      en_pin_v     the EN/UVLO node — output of R30/R31 fed by ESTOP_FEED
                   through the two NC contacts in **series** (design §5.3:
                   "The E-stop's normally-closed contact sits in series with
                   that pin's pull-up."). The harness.md J11 mapping puts
                   contact A on pins 1–2 (the EN feed) and contact B on
                   pins 3–4 (ESTOP_N to the ESP32). A third implied branch
                   — the E-stop's case where a wire breaks — is the same
                   EN pull-down.
      inp_pin_v    the INP node — SERVO_EN through R32/R33 from the ESP32
      pack_v       PACK+ level (upstream of R24, downstream of Q2)
      rail_i_a     current into the servo load, through R24
    """
    state: RailState = RailState.OFF
    rail_v: float = 0.0            # SERVO+ relative to GND
    cb_since: float | None = None
    sc_since: float | None = None
    _rising_started_at: float | None = None
    trip_log: list[tuple[str, float, float]] = field(default_factory=list)

    @property
    def enabled(self) -> bool:
        return self.state == RailState.ON

    @property
    def commanded_off(self) -> bool:
        return self.state in (
            RailState.OFF, RailState.LATCHED_CB, RailState.LATCHED_SC)

    def update(self, t_s: float, dt_s: float,
               en_pin_v: float, inp_pin_v: float,
               pack_v: float, rail_i_a: float):
        # -- EN/UVLO pin: the button state in series with R30/R31. When both
        # contacts of J11 are closed, the pin sees PACK+ × R31 / (R30 + R31)
        # ≈ PACK+ × 0.1515: on above 7.79 V of PACK+, off below 7.33 V (C3).
        # A pressed E-stop or a broken wire takes it to 0 V.
        en_hi = en_pin_v >= VU_VLO_RISING
        en_lo = en_pin_v < VU_VLO_FALLING

        # -- INP pin: SERVO_EN driven by the ESP32 through R32/R33. ESP32
        # high is 3.3 V → 3.27 V at INP (matches C3's V(INP_H) ≤ 2 V).
        inp_hi = inp_pin_v >= VINP_H

        # -- Latched states: the latch clears when INP goes low (SERVO_EN
        #    dropped), when EN/UVLO goes below ENF (E-stop pressed), or on
        #    a VCC recycle (§8.3.3.2). A latch with EN and INP held high
        #    stays latched.
        if self.state in (RailState.LATCHED_CB, RailState.LATCHED_SC):
            if en_pin_v < V_ENF_MIN or not inp_hi:
                self.state = RailState.OFF
                self.cb_since = self.sc_since = None
                self._rising_started_at = None
                self.rail_v = 0.0
                return
            self.rail_v = 0.0
            return

        # -- OFF: EN low or INP low both hold it off. Hardware E-stop
        #    cannot be overridden by INP (design §5.3: "The ESP32 can open
        #    the rail, but it cannot close it while the button is
        #    latched.").
        if self.state == RailState.OFF:
            self.rail_v = 0.0
            if en_hi and inp_hi:
                self.state = RailState.STARTING
                self._rising_started_at = t_s
            return

        # -- Gate drive up: rail voltage rises exponentially to pack level.
        #    EN/UVLO below V(UVLOF) or INP low at ANY point in the ramp
        #    pulls the FET off, no firmware involved — the STARTING state
        #    has to honour EN/INP just like ON does, otherwise a latched
        #    E-STOP during soft start lets the rail reach ON anyway (P1-2).
        if self.state == RailState.STARTING:
            if en_lo or not inp_hi:
                self.state = RailState.OFF
                self.rail_v = 0.0
                self.cb_since = self.sc_since = None
                self._rising_started_at = None
                return
            # `_rising_started_at` is always set when STARTING is entered
            # from OFF above. `is not None`, not `or` — a transition at
            # t=0.0 exactly must not hang the ramp on the falsy zero.
            started = self._rising_started_at
            if started is None:      # defensive; unreachable in the suite
                started = t_s
                self._rising_started_at = t_s
            elapsed = t_s - started
            target = pack_v
            self.rail_v = target * (1.0 - np.exp(-elapsed / _SOFT_START_TAU_S)) \
                if elapsed > 0 else 0.0
            if elapsed >= SOFT_START_T90_S * 1.05:
                self.state = RailState.ON
                self.rail_v = pack_v
                self._rising_started_at = None
            return

        # -- ON: watch for EN falling or INP low → off; overcurrents → trip
        if en_lo or not inp_hi:
            self.state = RailState.OFF
            self.rail_v = 0.0
            self.cb_since = self.sc_since = None
            return

        if rail_i_a >= SC_TRIP_A:
            if self.sc_since is None:
                self.sc_since = t_s
            elif t_s - self.sc_since >= SC_DELAY_S:
                self.state = RailState.LATCHED_SC
                self.rail_v = 0.0
                self.trip_log.append(("SC", t_s, rail_i_a))
                return

        if rail_i_a >= CB_TRIP_A:
            if self.cb_since is None:
                self.cb_since = t_s
            elif t_s - self.cb_since >= CB_DELAY_S:
                self.state = RailState.LATCHED_CB
                self.rail_v = 0.0
                self.trip_log.append(("CB", t_s, rail_i_a))
                return
        else:
            self.cb_since = None

        # -- conduction: rail tracks pack minus the switch drop. CSD17556Q5B
        #    at 10 A is ~10 mV of drop (Rds_on 1.2 mΩ + Kelvin). Sense R24
        #    drops its 2 mΩ × I.
        r24_drop_v = rail_i_a * 2e-3
        self.rail_v = max(0.0, pack_v - r24_drop_v - rail_i_a * 1.2e-3)


# --- module import guard --------------------------------------------------
import numpy as np  # noqa: E402  — used above for exp()
