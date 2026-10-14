"""Pre-staged board scenarios for tests (referenced from sequencer_sim.py's
docstring; P3-6 of round 2).

These are thin conveniences over `PowerBoard` + `Esp32Stub` so tests share
one definition of "boot and walk at 3.05 A", "boot, walk, then E-stop at
1.5 s", and so on.
"""
from __future__ import annotations

from pbsim.sequencer_sim import BoardState, Buttons, Esp32Stub, PowerBoard


class BootAndHoldOn(Esp32Stub):
    """Standard firmware: SERVO_EN high from boot, never KILL."""
    def tick(self, t_s, board):
        return {"servo_en_v": 3.3, "kill_v": 0.0, "i2c_start_v": 3.3}


def press_power_until(t_until: float):
    """Buttons callable: POWER held from t=0 to `t_until`, then released."""
    def at(t_s):
        return Buttons(power_pressed=t_s < t_until)
    return at


def press_power_then_estop_latched(power_until: float, estop_from: float):
    """Buttons callable: boot-press POWER, then latch E-STOP at
    `estop_from`."""
    def at(t_s):
        if t_s < power_until:
            return Buttons(power_pressed=True)
        if t_s >= estop_from:
            return Buttons(estop_latched=True)
        return Buttons()
    return at


def run(board: PowerBoard, buttons_at, esp: Esp32Stub, rail_load: float,
        n_ticks: int) -> list[BoardState]:
    """Advance the board under `buttons_at` for `n_ticks` at a constant
    servo load, returning the per-tick BoardState list."""
    out, last = [], None
    for _ in range(n_ticks):
        st = board.tick(buttons_at(board.t_s), esp,
                        rail_load_a=rail_load, board=last)
        out.append(st)
        last = st
    return out
