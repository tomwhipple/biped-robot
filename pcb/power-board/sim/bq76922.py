"""Behavioural model of the TI BQ76922 cell-protection IC, rev A board
configuration.

Only the behaviours that this suite exercises are modelled:

* power states              SHUTDOWN → NORMAL (via TS2 wake pulse)
                            NORMAL → SHUTDOWN (via RST_SHUT ≥ 1 s, I²C
                            Shutdown subcommand, or the OTP-set autonomous
                            SUCD "minimum-cell or stack-voltage" shutdown)
* SHUTDOWN current          ~1 µA (datasheet §8.5: I_SHUTDOWN 1 µA typ,
                            3.1 µA max)
* protections               cell UV (CUV)   — per-cell
                            cell OV (COV)   — per-cell
                            discharge OC  (OCD)  — pack-current, delay-set
                            short-circuit (SCD)  — pack-current, fastest delay
                            auto-off on SUCD     — OTP-set
* FETs                      CHG + DSG high-side N-FETs (Q1, Q2); on in
                            NORMAL, off in SHUTDOWN or on the tripped
                            protection while latched.
* TS2 wake                  POWER button pulls TS2 low through R9 + D2;
                            ≥ 0.7 V margin below the REG18-driven TS2
                            float level wakes the part (datasheet 8.5;
                            C7 of the verification record checks the
                            resistor math against the real board).
* I²C telemetry             cell voltages, stack, current (coulomb
                            counter and instantaneous), protection flags.

NOT modelled: temperature channels, the ALERT pin's maskable sources, the
internal charge pump, I²C framing (we model the *contents* delivered).

OTP thresholds (rev A, per design §5.5 + verification C3/C5):
    COV            4.20 V/cell (LiPo max charge; not exercised in discharge
                   tests here but included for completeness)
    CUV            3.00 V/cell (under battguard's 3.3 V/cell landing; the
                   firmware-independent backstop, design §5.5 §10 V2)
    OCD            20 A   (design §5.5: clears get-up peaks 9.4–11.8 A)
    SCD            150 A  (verification record C5: above pack-side wiring
                   L/R settling so a servo-rail short clears on the servo
                   switch alone; below the 150 A threshold ±35 % at the
                   lowest is 97.5 A, and the servo switch's 39 A trip is
                   well under that)
    SUCD           9.0 V stack (≈3.0 V/cell) — the autonomous shutdown
                   floor, chosen below battguard's VLAND so the firmware
                   gets the first word

The model's state machine is driven by external stimuli (button presses,
loads, ESP32 commands) via the orchestrating sequencer (`sequencer_sim.py`).

Numbers that came from the design record (and that the tests should be able
to defend) are in CAPS-lock constants near the top; everything else is a
parameter.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto


class BqState(Enum):
    SHUTDOWN = auto()       # FETs off, I²C dead, < 4 µA into TS2
    NORMAL = auto()         # FETs on, monitoring active
    # BQ76922 doesn't have a SHIPPING state that's distinct from SHUTDOWN
    # for our purposes; treat them as the same off-state.


class Protection(Enum):
    NONE = auto()
    CUV = auto()   # cell under-voltage
    COV = auto()   # cell over-voltage
    OCD = auto()   # discharge over-current
    SCD = auto()   # short-circuit discharge
    SUCD = auto()  # autonomous shutdown (stack or per-cell floor)


# What the OTP is programmed to on rev A (design §5.5 + C3/C5). These are
# the "intended" values; test_pack_voltage asserts the model is in
# agreement with these and would catch a misprogrammed OTP.
OTP_COV_V = 4.20          # per-cell
OTP_CUV_V = 3.00          # per-cell
OTP_OCD_A = 20.0          # with delay
OTP_OCD_DELAY_S = 0.050   # conservative: design says "with delay"; 50 ms
                          # matches BQ76922's 17 ms / 33 ms / 66 ms family
OTP_SCD_A = 150.0
OTP_SCD_DELAY_S = 30e-6   # "fastest delay" per design; BQ76922's SCD is
                          # ~15 µs typ / 30 µs max at SCDL=0
OTP_SUCD_STACK_V = 9.0    # autonomous-shutdown stack threshold


# Resistor-network constants lifted from the schematic via
# `schematicsnapshot` in tests, but exposed here for the model's use:
RSENSE_PACK_MOHM = 1.0    # R1, the pack-current sense (1 mΩ)
TS2_WAKE_MIN_V = 0.7      # TS2 wake threshold (datasheet V_WAKEONTS2 min)
TS2_FLOAT_LEVEL_V = 5.5   # REG18 domain, 4.5–6 V typ; conservative: 5.5
BQ76922_I2C_ADDR = 0x08


@dataclass
class ProtectionEvent:
    kind: Protection
    at_s: float
    value: float  # cell volts for CUV/COV; amps for OCD/SCD; stack volts for SUCD


@dataclass
class Bq76922:
    """The protection IC + its two pass FETs, behaviourally.

    All currents are amps at the pack port (positive = discharge). All
    voltages are volts at the named node w.r.t. BAT−.
    """
    # OTP-set thresholds
    cov_v: float = OTP_COV_V
    cuv_v: float = OTP_CUV_V
    ocd_a: float = OTP_OCD_A
    ocd_delay_s: float = OTP_OCD_DELAY_S
    scd_a: float = OTP_SCD_A
    scd_delay_s: float = OTP_SCD_DELAY_S
    sucd_stack_v: float = OTP_SUCD_STACK_V

    state: BqState = BqState.SHUTDOWN
    latched_faults: list[ProtectionEvent] = field(default_factory=list)
    _ts2_pulled_below_wake_at: float | None = None
    _ts2_wake_armed_at: float | None = None
    _rst_shut_high_at: float | None = None
    _ocd_since: float | None = None
    _scd_since: float | None = None

    # ------------- the public surface used by sequencer_sim -----------------

    @property
    def fets_closed(self) -> bool:
        """Q1 (CHG) and Q2 (DSG) conduct only in NORMAL with no latched
        protection that opens them. For our purposes a latched fault opens
        the FETs; a CUV latch releases only on a charger detection, and we
        never model a charger, so any latch holds until `kill()` reboots.
        """
        return self.state == BqState.NORMAL and not self.latched_faults

    @property
    def powered_up(self) -> bool:
        return self.state == BqState.NORMAL

    @property
    def shutdown_current_ma(self) -> float:
        return 0.001 if self.state == BqState.SHUTDOWN else 0.25

    # ------------- time-stepped behavioural update -----------------------------

    def update(self, t_s: float, dt_s: float,
               cell_taps_v: tuple[float, float, float, float],
               pack_v: float, pack_i_a: float,
               ts2_pin_v: float, rst_shut_pin_v: float):
        """One tick of the BQ76922's internal monitor state machine.

        - SHUTDOWN:  wake on TS2 pulled below TS2_WAKE_MIN_V (datasheet,
                     "wake on TS2"); a transition takes ~280 ms.
        - NORMAL:    latch faults with their per-fault delay; on SUCD
                     transition state to SHUTDOWN.
        - KILL pin:  high for ≥ 1 s in NORMAL → SHUTDOWN, unconditional
                     (datasheet 8.5, design §7 "Power on" sequence).
        """
        # -- state transitions first
        if self.state == BqState.SHUTDOWN:
            if ts2_pin_v <= TS2_WAKE_MIN_V:
                # button pressed (or held): mark first wake edge
                if self._ts2_pulled_below_wake_at is None:
                    self._ts2_pulled_below_wake_at = t_s
                # 280 ms from first TS2 low → first FETs
                if (t_s - self._ts2_pulled_below_wake_at) >= 0.280:
                    self.state = BqState.NORMAL
                    self._ts2_wake_armed_at = t_s
                    self._ts2_pulled_below_wake_at = None
            else:
                self._ts2_pulled_below_wake_at = None
            return  # no protections in SHUTDOWN

        # NORMAL:
        if rst_shut_pin_v >= 1.2:   # V(IH) 0.67 × REG18 ≈ 1.2 V
            if self._rst_shut_high_at is None:
                self._rst_shut_high_at = t_s
            if t_s - self._rst_shut_high_at >= 1.0:
                self._die(t_s, "KILL pin held high ≥ 1 s")
                return
        else:
            self._rst_shut_high_at = None

        # -- protections (per-cell first)
        cells = (cell_taps_v[1] - cell_taps_v[0],
                 cell_taps_v[2] - cell_taps_v[1],
                 cell_taps_v[3] - cell_taps_v[2])
        for i, cv in enumerate(cells):
            if cv <= self.cuv_v and all(
                    f.kind != Protection.CUV for f in self.latched_faults
            ):
                self.latched_faults.append(
                    ProtectionEvent(Protection.CUV, t_s, cv))
            if cv >= self.cov_v and all(
                    f.kind != Protection.COV for f in self.latched_faults
            ):
                self.latched_faults.append(
                    ProtectionEvent(Protection.COV, t_s, cv))

        # -- over-current with delay
        if pack_i_a >= self.ocd_a:
            self._ocd_since = self._ocd_since or t_s
            if t_s - self._ocd_since >= self.ocd_delay_s:
                if all(f.kind != Protection.OCD for f in self.latched_faults):
                    self.latched_faults.append(
                        ProtectionEvent(Protection.OCD, t_s, pack_i_a))
        else:
            self._ocd_since = None

        if pack_i_a >= self.scd_a:
            self._scd_since = self._scd_since or t_s
            if t_s - self._scd_since >= self.scd_delay_s:
                if all(f.kind != Protection.SCD for f in self.latched_faults):
                    self.latched_faults.append(
                        ProtectionEvent(Protection.SCD, t_s, pack_i_a))
        else:
            self._scd_since = None

        # -- autonomous shutdown (SUCD): unconditional once stack is below
        # the OTP floor — "no firmware required".
        if pack_v <= self.sucd_stack_v:
            if all(f.kind != Protection.SUCD for f in self.latched_faults):
                self.latched_faults.append(
                    ProtectionEvent(Protection.SUCD, t_s, pack_v))
                self._die(t_s, "SUCD floor crossed")

    # ------------- host-visible API -------------------------------------------

    def cell_voltages(self, taps):
        return tuple(taps[i+1] - taps[i] for i in range(3))

    def stack_v(self, taps) -> float:
        return taps[3]

    def current_a(self) -> float:
        # The coulomb counter is not implemented here (it's an integrator);
        # instantaneous current is the reading path the design uses for
        # protections, and that's the one the tests exercise.
        raise NotImplementedError(
            "host calls sequencer_sim's pack_i_a instead; this IC's view "
            "is modelled via update().")

    def protection_report(self) -> list[tuple[str, float]]:
        return [(f.kind.name, f.value) for f in self.latched_faults]

    def _die(self, t_s: float, why: str):
        self.state = BqState.SHUTDOWN
        # Q1/Q2 charge-pump caps decay ~ms later; we model the FETs as
        # opening on the same tick (the RC fall time is not in the scope of
        # any assertion here).
        self._ts2_pulled_below_wake_at = None
        self._ts2_wake_armed_at = None
        self._rst_shut_high_at = None

    # ------------- convenience ---------------------------------------------------

    def any_latch(self) -> bool:
        return bool(self.latched_faults)

    def clear_latches(self):
        """Host-side clear; only the firmware would do this (a bench-side
        I²C write). Pack state and FET state unchanged."""
        self.latched_faults.clear()

    def kill(self, t_s: float):
        """The I²C-shutdown path AND the external RST_SHUT path collapse to
        the same state transition."""
        if self.state == BqState.NORMAL:
            self._die(t_s, "host KILL")
        # in SHUTDOWN this is a no-op: RST_SHUT high on a dead part does
        # nothing
