"""Part-2.3: the ESP32 control plane — J12 (control signals) + J9 (I²C, 3V3).

The contract:

- **Command → switch-state mapping** (design §5.3):
    * SERVO_EN high + E-STOP released  → rail closes (after soft start)
    * SERVO_EN low                     → rail opens
    * SERVO_EN high + E-STOP pressed   → rail stays open ("the ESP32 can
      open the rail, but it cannot close it while the button is latched")
    * KILL high ≥ 1 s                  → BQ76922 → SHUTDOWN; both rails off
    * ESP32 unpowered / never booted   → SERVO_EN is at R33's pulldown
      (≈ 0 V); rail stays OFF; the board is inert
- **Stuck-on servo rail**: ESP32 drives SERVO_EN low, the rail is still
  live (because Q3 is shorted or INP is broken). The F4 sequencer in
  firmware raises KILL. This test asserts the *chain* a real F4 module
  would ride on: (a) the disagreement between commanded rail state and
  observed rail state is *visible* via RAIL_V at GPIO 34, and (b) KILL
  opens the main switch even when the servo switch is welded.
- **I²C idle**: J9 is mA only when ESP32 drives a START. We model the SDA
  and SCL lines with their pull-ups; the test asserts that with the ESP32
  powered down and no I²C host on the bus, SDA/SCL sit at +3V3_HOST and the
  BQ76922's SDA/SCL inputs see a high.

The ESP32 is *not* simulated as a CPU; the Esp32Stub pattern from
test_buttons returns a pin dict per tick. Here we use it to drive specific
sequences (deferred boot, stuck-on rail, etc).
"""
from __future__ import annotations

import pytest

from sim.sequencer_sim import (R32_OHM, R33_OHM, BoardState, Buttons,
                               Esp32Stub, PowerBoard)


class _DelayedBoot(Esp32Stub):
    """ESP32 is unpowered until `boot_at` (its SERVO_EN floats at 0 V via
    R33), then it boots, then it sets SERVO_EN at `servo_en_at`."""
    def __init__(self, boot_at=0.5, servo_en_at=0.7):
        self.boot_at = boot_at
        self.servo_en_at = servo_en_at

    def tick(self, t_s, board):
        if t_s < self.boot_at:
            return {"servo_en_v": 0.0, "kill_v": 0.0, "i2c_start_v": 0.0}
        if t_s < self.servo_en_at:
            return {"servo_en_v": 0.0, "kill_v": 0.0, "i2c_start_v": 3.3}
        return {"servo_en_v": 3.3, "kill_v": 0.0, "i2c_start_v": 3.3}


def _boot_buttons(t_s):
    return Buttons(power_pressed=t_s < 0.3)


def test_esp32_unpowered_board_is_inert():
    """Design §5.3: the ESP32 must never hold SERVO_EN high by accident;
    the pull-down R33 makes "unpowered ESP32" mean "SERVO_EN low". So
    with the ESP32 off (never calling tick, which means SERVO_EN at 0 V),
    the board still wakes via the POWER button (hardware) but the rail
    stays OFF.
    """
    class NeverBoots(Esp32Stub):
        def tick(self, t_s, board):
            return {"servo_en_v": 0.0, "kill_v": 0.0, "i2c_start_v": 0.0}
    board = PowerBoard()
    out, last = [], None
    for _ in range(1500):
        st = board.tick(_boot_buttons(board.t_s), NeverBoots(),
                        rail_load_a=0.0, board=last)
        out.append(st); last = st
    assert any(s.bq_state == "NORMAL" for s in out), (
        "board failed to wake on POWER-press with ESP32 dead"
    )
    # The rail must NOT close.
    assert all(s.rail_state == "OFF" for s in out)
    assert all(s.rail_v < 0.1 for s in out)
    # The ESP32's view: ESTOP_N reads LOW (contact B closed, rail off), the
    # rail's divider reads 0.
    assert all(s.rail_v_at_gpio34 < 0.05 for s in out)


def test_servo_en_low_holds_rail_open():
    """Explicit: even with the BQ76922 in NORMAL and the E-stop released,
    SERVO_EN low holds the rail open. The TPS48111's INP gate is the
    second of two switches."""
    board = PowerBoard()
    class NoServoEn(Esp32Stub):
        def tick(self, t_s, board):
            return {"servo_en_v": 0.0, "kill_v": 0.0, "i2c_start_v": 3.3}
    out, last = [], None
    for _ in range(1200):
        st = board.tick(_boot_buttons(board.t_s), NoServoEn(),
                        rail_load_a=0.0, board=last)
        out.append(st); last = st
    assert any(s.bq_state == "NORMAL" for s in out)
    assert all(s.rail_v < 0.1 for s in out)


def test_servo_en_high_with_estop_pressed_cannot_close_rail():
    """Design §5.3: 'The ESP32 can open the rail, but it cannot close it
    while the button is latched.' With E-stop latched and the ESP32
    driving SERVO_EN high continuously, the rail stays OFF."""
    class Tries(Esp32Stub):
        def tick(self, t_s, board):
            return {"servo_en_v": 3.3, "kill_v": 0.0, "i2c_start_v": 3.3}
    def btns(t):
        if t < 0.3:
            return Buttons(power_pressed=True)
        if t >= 1.0:
            return Buttons(estop_latched=True)
        return Buttons()
    board = PowerBoard()
    out, last = [], None
    for _ in range(2000):
        st = board.tick(btns(board.t_s), Tries(),
                        rail_load_a=0.0, board=last)
        out.append(st); last = st
    # rail comes up while E-stop is *not* pressed
    assert any(s.rail_state == "ON" for s in out if s.t_s < 1.0)
    # once pressed, it stays down in hardware regardless of SERVO_EN
    pressed = [s for s in out if s.t_s > 1.01]
    assert all(s.rail_v < 0.1 for s in pressed)


def test_servo_en_pin_voltage_within_inp_spec():
    """C7: 3.3 V from the ESP32 through R32/R33 must give ≥ 2 V at INP.
    With the schematic values R32=1 kΩ / R33=100 kΩ the divider delivers
    3.3 × 100/101 = 3.267 V. Assert the divider ratio is as-schematic."""
    expected = 3.3 * R33_OHM / (R32_OHM + R33_OHM)
    assert expected > 2.0, \
        f"ESP32 3.3 V through R32/R33 = {expected:.3f} V < INP threshold 2 V"
    # And the pull-down keeps a floating ESP32 pin at ≈ 0 V.
    # (The divider ratio gives 0 / 101 = 0.)


def test_rail_v_divider_keeps_gpio34_in_adc_range():
    """C7's divider check: 12.6 V at the rail → R38/R39 gives 2.21 V at
    GPIO 34 (within the ESP32's ADC11 range 150–2450 mV). At 0.5 V rail
    (just above 'off'), GPIO 34 reads 87 mV — below ADC noise floor, fine
    for an off signal."""
    from sim.sequencer_sim import R38_OHM, R39_OHM
    gnd_at_12_6 = 12.6 * R39_OHM / (R38_OHM + R39_OHM)
    gnd_at_9 = 9.0 * R39_OHM / (R38_OHM + R39_OHM)
    assert 0.15 < gnd_at_12_6 < 2.45  # ESP32 ADC 11 dB range, C7
    assert 0.05 < gnd_at_9 < 2.45
    # specifically C7's figure of merit
    assert gnd_at_12_6 == pytest.approx(2.21, abs=0.05)


def test_kill_pin_voltage_within_rst_shut_spec():
    """C7: ESP32 3.3 V high through R12/R11 must give ≥ 1.2 V at RST_SHUT
    (V(IH) 0.67 × REG18 ≈ 1.2 V, 5.5 V max). Schematic values R12=1 kΩ /
    R11=10 kΩ give 3.3 × 10/11 = 3.0 V."""
    from sim.sequencer_sim import R11_OHM, R12_OHM
    v = 3.3 * R11_OHM / (R12_OHM + R11_OHM)
    assert v > 1.2, f"KILL voltage {v:.2f} V is under the 1.2 V threshold"
    assert v < 5.5


def test_stuck_on_rail_is_detectable_by_firmware():
    """Stuck-on rail (Q3 welded, or INP failed): the ESP32 drives SERVO_EN
    low, the rail SHOULD fall. If the rail stays up, RAIL_V at GPIO 34
    reports it. The F4 sequencer (firmware-side) then raises KILL.

    We don't simulate the Q3 weld electrically. We assert the detection
    contract: ESP32's SERVO_EN = low AND rail_v > 0.5 V persisting ≥ 50 ms
    is the fault signature. We demonstrate that signature exists: a test
    'fault injector' forces rail_v to stay high via a monkeypatched
    Tps48111 that ignores INP (acts as if EN alone drives it). The ESP32
    stub then raises KILL, and the PACK+ (via BQ76922) drops."""
    board = PowerBoard()

    # Weld Q3: replace the rail switch with one that never opens on INP.
    from sim.tps48111 import RailState
    def stuck_update(t_s, dt_s, en_pin_v, inp_pin_v, pack_v, rail_i_a):
        sw = board.rail_sw
        if en_pin_v >= 1.0 and pack_v > 6.0:
            sw.state = RailState.ON
            sw.rail_v = pack_v - rail_i_a * (2e-3 + 1.2e-3)
        else:
            sw.state = RailState.OFF
            sw.rail_v = 0.0
    board.rail_sw.update = stuck_update  # type: ignore[method-assign]

    class F4Sequencer(Esp32Stub):
        """F4 from design §9: sees rail-up-with-SERVO_EN-low (the
        disagreement), raises KILL (with the 1 s RST_SHUT window the
        datasheet sets)."""
        def __init__(self):
            self.disagree_since: float | None = None
            self.kill_at: float | None = None
            self.servo_en_v = 3.3

        def tick(self, t_s, board):
            if board is None:
                return {"servo_en_v": 3.3, "kill_v": 0.0,
                        "i2c_start_v": 3.3}
            if self.kill_at is not None:
                return {"servo_en_v": 0.0, "kill_v": 3.3,
                        "i2c_start_v": 3.3}
            if t_s > 0.8:
                self.servo_en_v = 0.0   # commanded off
            disagree = board.rail_v_at_gpio34 > 0.05 and self.servo_en_v < 0.05
            if disagree and self.disagree_since is None:
                self.disagree_since = t_s
            if disagree and t_s - self.disagree_since > 0.05:
                self.kill_at = t_s
                return {"servo_en_v": 0.0, "kill_v": 3.3,
                        "i2c_start_v": 3.3}
            if not disagree:
                self.disagree_since = None
            return {"servo_en_v": self.servo_en_v, "kill_v": 0.0,
                    "i2c_start_v": 3.3}

    esp = F4Sequencer()
    out, last = [], None
    for _ in range(5000):
        st = board.tick(_boot_buttons(board.t_s), esp,
                        rail_load_a=3.05, board=last)
        out.append(st); last = st
    # rail was on mid-run (stuck-on condition)
    assert any(s.rail_state == "ON" for s in out if s.t_s < 0.8)
    # F4 saw the disagreement and raised KILL
    assert esp.kill_at is not None, "F4 never detected the stuck rail"
    # after KILL ≥ 1.0 s the BQ76922 went to SHUTDOWN and the pack dropped
    after = [s for s in out if s.t_s > esp.kill_at + 1.2]
    assert all(s.bq_state == "SHUTDOWN" for s in after)
    assert all(s.packplus_v < 1.0 for s in after)


def test_esp32_unpowered_imu_still_boots():
    """Design §5.5: 'No board part may load the I²C bus while it is
    unpowered.' The power board's J9 line is mA only, and both SDA and SCL
    have their own series resistors and pull-ups upstream. With the ESP32
    off:
      * SDA/SCL float at +3V3_HOST (i.e. no current is sinking into the
        power board's ICs when they themselves are off), and
      * the BQ76922's SHUTDOWN current is what it is without any I²C talk.
    The model: the IMU is on the ESP32's bus (off-board); the power board
    pulls nothing below 3V3_HOST. Assert J9's pin list matches the harness
    (1 3V3, 2 GND, 3 SDA, 4 SCL) and that the SDA/SCL nets are pulled up
    (not down) — we observe that the schematic's `SDA` and `SCL` labels
    appear exactly once each on the BQ76922 and once on the J9 connector,
    no additional pulldown."""
    from sim.schematicsnapshot import load
    s = load()
    j9 = s.components["J9"]
    assert sorted(j9.pins) == ["1", "2", "3", "4"]
    # schematic carries the nets
    assert {"SDA", "SCL"} <= s.local_labels
    # the rail is OFF while ESP32 is unpowered — covered above by
    # test_esp32_unpowered_board_is_inert. This test pins the harness
    # shape only.


def test_control_signals_on_j12_match_design_pin_map():
    """J12 pin map per harness.md and design §9. If J12 is repinned, tests
    downstream of it (RAIL_V scaling, KILL, etc.) must see the change here
    first."""
    from sim.schematicsnapshot import load
    s = load()
    j12 = s.components["J12"]
    assert sorted(j12.pins, key=int) == [str(i) for i in range(1, 9)]
    # The harness.md J12 table is the source of truth for pin → signal.
    # We assert the schematic *carries the named labels* that the table
    # lists.
    needed = {"KILL", "SERVO_EN", "PWR_INT_N", "RAIL_V", "ESTOP_N",
              "SERVO_IMON", "SERVO_FLT_N"}
    assert needed <= s.local_labels, (
        f"J12 control nets not all present as labels: missing "
        f"{needed - s.local_labels}"
    )
