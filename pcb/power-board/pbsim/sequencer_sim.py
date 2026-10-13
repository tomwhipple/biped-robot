"""High-level power-board sequencer combining pack + BQ76922 + TPS48111 +
buttons + ESP32 + loads. This is the §7 machinery wired through.

The orchestrator does not implement its own physics: it stitches the three
component models (each of which lives close to its datasheet), feeds them
the inputs the board actually sees, and applies the harness-side wiring —
the ESTOP_FEED → EN series path through J11 pins 1-2, the R38/R39 RAIL_V
divider, the R32/R33 SERVO_EN divider, the KILL path through R12/R11.

The simulation loop is deliberately synchronous and simple; each tick runs:

    1. ESP32 driver decisions (what the firmware would write this tick)
    2. BQ76922.update(...)
    3. TPS48111.update(...)
    4. pack.step(...)

State lives on the components themselves; this class just wires them.

Tests in this package should reach for one of the pre-staged scenarios in
`scenarios.py` rather than assembling the stack by hand.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from pbsim.bq76922 import (BQ76922_I2C_ADDR, Bq76922, BqState, ProtectionEvent)
from pbsim.pack import Pack3S
from pbsim.tps48111 import RailState, Tps48111


# R30/R31 on EN/UVLO (C3): rising 7.79 V, falling 7.39 V
R30_OHM = 560e3
R31_OHM = 100e3

# R32/R33 on INP (C7's divider check: 3.27 V at INP from 3.3 V SERVO_EN)
R32_OHM = 1e3
R33_OHM = 100e3

# R38/R39 on RAIL_V (C7: 12.6 V → 2.21 V at GPIO 34)
R38_OHM = 47e3
R39_OHM = 10e3

# R12/R11 on KILL: 3.3 V ESP32 high → ≈ 3.0 V at RST_SHUT
R12_OHM = 1e3
R11_OHM = 10e3

# E-stop feed chain: ESTOP_FEED (PACK+ through 1 kΩ) — J11 contact A.
ESTOP_FEED_SERIES_OHM = 1000.0

# TS2 wake button: pulls TS2 low through R9 (10 kΩ) and D2 (BAT54C, ~0.25 V)
R9_OHM = 10e3

TICK_S = 1e-3     # 1 ms tick, matching the servo tick the firmware runs


@dataclass
class BoardState:
    """One tick's external view. What the test sees."""
    t_s: float
    pack_terminal_v: float
    pack_internal_i_a: float       # pack current source (discharge positive)
    packplus_v: float              # the PACK+ rail (downstream of Q1+Q2)
    logic_v: float                 # LOGIC_OUT: PACK+ through F2
    rail_v: float                  # SERVO+ (downstream of Q3)
    rail_i_a: float                # current into the servo branches
    esp32_ts2_v: float             # pin state going to BQ76922's TS2
    esp32_rst_shut_v: float        # ESP32's KILL output at RST_SHUT pin
    esp32_servo_en_v: float        # ESP32's SERVO_EN output at INP
    esp32_sda_v: float             # pulled low only when ESP32 drives START
    pwr_int_n_v: float             # INTO the ESP32 (B button contact via D2)
    estop_n_v: float               # INTO the ESP32 (contact B)
    rail_v_at_gpio34: float        # R38/R39 divider into GPIO 34
    bq_state: str
    rail_state: str
    bq_fets_closed: bool
    rail_closed: bool
    latched_faults: list[str]


class Esp32Stub:
    """A scripted ESP32: what the firmware would output this tick.

    Subclassed by tests to make the firmware behave differently (release
    SERVO_EN on schedule, assert KILL on schedule, snoop rail_v, etc.).
    """

    def tick(self, t_s: float, board: BoardState) -> dict[str, float]:
        """Return the ESP32's output pins for this tick.

        keys:
            servo_en_v   0.0/3.3
            kill_v       0.0/3.3
            i2c_start_v  3.3 (idle) or 0.0 (driven START)
        """
        return {"servo_en_v": 0.0, "kill_v": 0.0, "i2c_start_v": 3.3}


@dataclass
class Buttons:
    power_pressed: bool = False       # J10 momentary NO
    estop_latched: bool = False       # J11 latching NC × 2 — both contacts
    estop_a_stuck_closed: bool = False  # fault injection: contact A welded
    estop_b_stuck_closed: bool = False  # fault injection: contact B welded
    estop_broken_wire: bool = False     # fault injection: J11 unplugged


class PowerBoard:
    """The rev-A board."""

    def __init__(self, pack: Pack3S | None = None,
                 bq: Bq76922 | None = None,
                 rail_sw: Tps48111 | None = None):
        self.pack = pack if pack is not None else Pack3S.fresh()
        self.bq = bq if bq is not None else Bq76922()
        self.rail_sw = rail_sw if rail_sw is not None else Tps48111()
        self.t_s = 0.0

    def _ts2_pin_v(self, buttons: Buttons, esp: Esp32Stub) -> float:
        """TS2 pin: floats to ~5.5 V (REG18 domain) in SHUTDOWN; the POWER
        button holds it low through R9 + D2 (BAT54C) when pressed. With the
        button unpressed and the ESP32 not driving TS2 (it's a wake pin,
        not a driven pin in rev A), the pin returns to its float state.
        """
        if buttons.power_pressed:
            # D2 drops ~0.25 V; R9 pulls down to well below the 0.7 V
            # wake threshold (C7 verified 0.15 V at the pin).
            return 0.15
        return 5.5

    def _rst_shut_v(self, esp: Esp32Stub) -> float:
        # The ESP32 drives KILL high to request shutdown. R12 in series +
        # R11 pulldown gives ≈ 3.0 V at the pin for a 3.3 V drive.
        return 0.0

    def _en_divider_v(self, buttons: Buttons, packplus_v: float) -> float:
        """EN/UVLO pin voltage.

        The EN path: +BATT (PACK+) → ESTOP_FEED (1 kΩ) → J11 contact A
        (normally closed) → R30 (560 kΩ) → EN pin → R31 (100 kΩ) → GND.

        Unpressed: pin = packplus × (R31 / (R30+R31)) ≈ pack × 0.1515.
        Pressed (contact A latched open): R30 sees no feed; the pin falls
        to 0 V through the R31 pulldown.
        Contact A stuck-closed: pin follows the divider regardless.
        Broken wire (J11 unplugged): feed interrupted; pin = 0.
        """
        if buttons.estop_broken_wire or (
                buttons.estop_latched and not buttons.estop_a_stuck_closed):
            return 0.0
        return packplus_v * R31_OHM / (R30_OHM + R31_OHM)

    def _estop_n_v(self, buttons: Buttons) -> float:
        """Contact B: ESTOP_N to ESP32 IO4. Pulled-up via R37 (10k) to
        +3V3_HOST; contact B closes to GND normally, opens on press.
        """
        if buttons.estop_broken_wire or (
                buttons.estop_latched and not buttons.estop_b_stuck_closed):
            return 3.3  # pull-up: reads as "E-STOP pressed"
        return 0.0      # contact closed: ESP32 sees GND

    def _pwr_int_n_v(self, buttons: Buttons) -> float:
        """PWR_INT_N to ESP32 GPIO 35. Wired through D2 (BAT54C): pressed
        pulls it low, unpressed floats to 3.3 V pull-up."""
        return 0.25 if buttons.power_pressed else 3.3

    def _inp_pin_v(self, esp32_servo_en_v: float) -> float:
        """SERVO_EN through R32 (series 1 kΩ) / R33 (pulldown 100 kΩ) at
        INP. 3.3 V logic from the ESP32 → 3.27 V at INP (C7)."""
        return esp32_servo_en_v * R33_OHM / (R32_OHM + R33_OHM)

    def _rail_v_at_gpio34(self) -> float:
        return self.rail_sw.rail_v * R39_OHM / (R38_OHM + R39_OHM)

    # ---------------- the tick --------------------------------------------

    def tick(self, buttons: Buttons, esp: Esp32Stub,
             rail_load_a: float = 0.0, board: BoardState | None = None
             ) -> BoardState:
        """Advance one 1 ms tick.

        `rail_load_a` is the servo branches' aggregate pull at SERVO+;
        `board` is an optional prior state the ESP32 stub might be reading
        (tests pass it in when their stub reads IO4 etc.).
        """
        dt = TICK_S

        esp_out = esp.tick(self.t_s, board) if board is not None else \
            esp.tick(self.t_s, None)  # type: ignore[arg-type]

        # Electrical state at tick start
        pack_v = self.pack.terminal_v(self._last_pack_i_a)
        taps = self.pack.balance_taps(self._last_pack_i_a)

        # BQ76922: what does it see?
        self.bq.update(
            self.t_s, dt,
            cell_taps_v=taps,
            pack_v=pack_v,
            pack_i_a=self._last_pack_i_a,
            ts2_pin_v=self._ts2_pin_v(buttons, esp),
            rst_shut_pin_v=esp_out.get("kill_v", 0.0),
        )

        # PACK+ = pack output if the FETs are closed; otherwise 0
        packplus_v = pack_v if self.bq.fets_closed else 0.0

        # EN pin and INP pin
        en_v = self._en_divider_v(buttons, packplus_v)
        inp_v = self._inp_pin_v(esp_out.get("servo_en_v", 0.0))

        # Servo load current: while the rail is closed and delivering
        # packplus_v, the branches pull `rail_load_a` at packplus_v. While
        # the rail is OFF/STARTING, the load is whatever the rail is passing.
        rail_v_now = self.rail_sw.rail_v
        if self.rail_sw.enabled and packplus_v > 0:
            rail_i = rail_load_a
        elif self.rail_sw.state == RailState.STARTING:
            # during soft start the load current is the bulk-cap charge
            rail_i = rail_load_a * (rail_v_now / max(pack_v, 0.1))
        else:
            rail_i = 0.0

        self.rail_sw.update(self.t_s, dt,
                            en_pin_v=en_v, inp_pin_v=inp_v,
                            pack_v=packplus_v, rail_i_a=rail_i)

        # Pack integration: total pack current = servo current + logic
        # (~0.5 A when the ESP32 + Pi are up).
        logic_a = 0.5 if packplus_v > 0 else 0.0
        pack_i = rail_i + logic_a
        self._last_pack_i_a = pack_i
        self.pack.step(pack_i, dt)

        self.t_s += dt

        faults = [f.kind.name for f in self.bq.latched_faults]
        return BoardState(
            t_s=self.t_s,
            pack_terminal_v=pack_v,
            pack_internal_i_a=pack_i,
            packplus_v=packplus_v,
            logic_v=packplus_v,       # F2 < 3 A: drop negligible
            rail_v=self.rail_sw.rail_v,
            rail_i_a=rail_i,
            esp32_ts2_v=self._ts2_pin_v(buttons, esp),
            esp32_rst_shut_v=esp_out.get("kill_v", 0.0),
            esp32_servo_en_v=esp_out.get("servo_en_v", 0.0),
            esp32_sda_v=esp_out.get("i2c_start_v", 3.3),
            pwr_int_n_v=self._pwr_int_n_v(buttons),
            estop_n_v=self._estop_n_v(buttons),
            rail_v_at_gpio34=self._rail_v_at_gpio34(),
            bq_state=self.bq.state.name,
            rail_state=self.rail_sw.state.name,
            bq_fets_closed=self.bq.fets_closed,
            rail_closed=self.rail_sw.enabled,
            latched_faults=faults,
        )

    _last_pack_i_a: float = 0.0


# ----------------- convenience runner --------------------------------------


def run_sim(buttons_seq, esp: Esp32Stub, rail_load: float = 0.0,
            n_ticks: int = 2000, pack: Pack3S | None = None) -> list[BoardState]:
    """Run the board under a buttons sequence. `buttons_seq` is a callable:
    t_s -> Buttons. Rail load is a constant servo current (the §2 figures).

    Returns a per-tick list of BoardState.
    """
    board = PowerBoard(pack=pack)
    out: list[BoardState] = []
    last: BoardState | None = None
    for _ in range(n_ticks):
        b = buttons_seq(board.t_s)
        last = board.tick(b, esp, rail_load_a=rail_load, board=last)
        out.append(last)
    return out
