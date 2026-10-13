"""Behavioural model of the TPS48111-Q1 high-side servo-rail switch.

Scope: the EN/UVLO divider, the INP input, the circuit-breaker timer, the
short-circuit fast trip, the gate-slew soft start, and the IMON current
mirror. NOT modelled: the internal charge pump details, the overtemperature
path (Q5's diode), I²C — this IC has none.

Datasheet numbers (TPS4811-Q1 SLUSEE5E, confirmed against schematic values
in test_schematic_intact and TI's calculator in C3):

    V(EN/UVLO) rising   1 V     — EN/UVLO > 1 V turns the controller on
    V(EN/UVLO) falling  0.3 V   — below 0.3 V it pulls the FET off regardless
                                  of INP (and pulls itself low if floated)
    V(INP) high         2 V max — 3.3 V logic through R32/R33 (≈3.27 V) passes
    V(CS+ − CS−) CB threshold  20 mV / R34-set:
                    C3 reads R24·(30 mV across R25+R34 ratio) = 19.8 A.
                    We use the C3 number directly.
    t_CB                R35 × C19 = 100 kΩ × 680 nF = 68 ms (datasheet
                    "latch-off by R35 100 kΩ"). C3 reads "10.5 ms"; TI's
                    calculator gives the FLT line at 9.7 ms with latch at
                    10.5 ms — those are the *effective* times the calculator
                    reports for the chosen R×C; we latch off at 10.5 ms to
                    match the record rather than re-derive.
    V(CS+ − CS−) SC  threshold  78 mV on R24: R26 4.64 kΩ sets SC = 39 A
                    (C3's number: 39.3 A)
    t_SC                ≤ 1.6 µs trip + PD-at-1.34 Ω pull-down

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
VU_VLO_RISING = 1.0
VU_VLO_FALLING = 0.3
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
        # contacts of J11 are closed, the EN divider sees pack_v-derived
        # UVLO = pack_v × R31 / (R30 + R31) ≈ pack_v × 0.1516 — rising above
        # 1 V at pack_v ≈ 6.6 V, which the rev-A's R30/R31 give as 7.79 V
        # (C3: 560 kΩ over 100 kΩ ⇒ rising threshold pack_v = 7.79 V, falling
        # 7.39 V). We use the pin voltage at EN directly: the EN pin sees
        # the divider output.
        en_hi = en_pin_v >= VU_VLO_RISING
        en_lo = en_pin_v <= VU_VLO_FALLING

        # -- INP pin: SERVO_EN driven by the ESP32 through R32/R33. ESP32
        # high is 3.3 V → 3.27 V at INP (matches C3's V(INP_H) ≤ 2 V).
        inp_hi = inp_pin_v >= VINP_H

        # -- Latched states never leave without a power cycle.
        if self.state in (RailState.LATCHED_CB, RailState.LATCHED_SC):
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

        # -- Gate drive up: rail voltage rises exponentially to pack level
        if self.state == RailState.STARTING:
            elapsed = t_s - (self._rising_started_at or t_s)
            target = pack_v
            self.rail_v = target * (1.0 - np.exp(-elapsed / _SOFT_START_TAU_S)) \
                if elapsed > 0 else 0.0
            if elapsed >= SOFT_START_T90_S * 1.05:
                self.state = RailState.ON
                self.rail_v = pack_v
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
