"""Guards for powered joint sweeps on the robot (tools/rom_sweep.py, tools/joint_goto.py).

2026-11-02: a sweep stepped the right hip roll inward to +30 deg (the policy's limit)
with the robot hanging on the stand. The legs touch at ~9 deg; the hip rolls (ids 51/52)
are not on the firmware's clamp map, so nothing capped it, and the right leg drove into
the left and broke the hip. The sweep's only force check was a |load| ceiling of 400-450
read AFTER each step: the load rose 0 -> 8 -> 64 -> 144 and then FELL to 56 and 8 while
the joint kept following -- the part yielding. Nothing fired.

So every powered step is now judged on:
  power     |present current| x bus voltage, sampled DURING the move as well as after
            it. Abort when it rises above what the sweep has seen so far by more than a
            margin (a step change on top of the smooth gravity trend = a contact), or above
            the at-rest baseline by more than --rise-w, or above an absolute ceiling.
  load drop |load| falling by more than half from >= 100 while the joint still follows:
            something gave way.
  torque    the swept servo's RAM torque limit (register 48) is lowered for the sweep, so
            a contact stalls the servo instead of breaking a part; restored afterwards.
  contact   inward hip roll is capped short of leg-leg contact unless --allow-contact.
"""
from __future__ import annotations

import re
import time
from dataclasses import dataclass, field

MA_PER_LSB = 6.5                     # STS present current, register 69 (servo-map.md 5)
REG_CURRENT, REG_TORQUE_LIMIT = 69, 48

# Sim-angle direction that moves each hip roll TOWARD THE OTHER LEG (servo-map.md 3.2:
# sim-positive hip roll moves the toe to the robot's left), and how far it may go with
# the other leg at zero. Leg-leg contact is ~9 deg (mech_envelope.h).
INWARD = {"R_hip_roll": +1, "L_hip_roll": -1}
INWARD_CAP_DEG = 6.0


def current_ma(raw: int) -> float:
    """Register 69 value -> signed mA (magnitude in bits 0-14, sign in bit 15)."""
    mag = (raw & 0x7FFF) * MA_PER_LSB
    return -mag if raw & 0x8000 else mag


def inward_limit(name: str, allow_contact: bool = False):
    """(sign, cap_deg) of the direction that points at the other leg, or None."""
    if allow_contact or name not in INWARD:
        return None
    return INWARD[name], INWARD_CAP_DEG


def cap_angle(name: str, angle_deg: float, allow_contact: bool = False) -> float:
    """The sim angle a joint may be commanded to, inward hip roll capped."""
    lim = inward_limit(name, allow_contact)
    if lim is None:
        return angle_deg
    sgn, cap = lim
    return min(angle_deg, cap) if sgn > 0 else max(angle_deg, -cap)


@dataclass
class PowerGuard:
    """Judges one sweep direction, step by step. Feed it the at-rest baseline first."""
    rise_w: float = 1.5          # above the at-rest baseline (gravity on a hanging leg ~< 1 W)
    step_w: float = 0.35         # above the last accepted step's power plus its trend
    abs_w: float = 4.0           # absolute ceiling, any joint
    drop_frac: float = 0.5       # |load| falling by more than this fraction ...
    drop_min_load: int = 100     # ... from at least this much, while still following
    base_w: float | None = None
    hist: list = field(default_factory=list)      # accepted (power W, |load|)

    def baseline(self, samples_w):
        s = sorted(samples_w)
        self.base_w = s[len(s) // 2]
        self.hist = [(self.base_w, 0)]

    def expected_w(self) -> float:
        """What the next step may draw: the last accepted power plus its rising trend
        (a hanging leg's gravity load changes smoothly with angle) plus step_w."""
        p = [h[0] for h in self.hist]
        trend = max(0.0, p[-1] - p[-2]) if len(p) >= 2 else 0.0
        return min(p[-1] + trend + self.step_w, self.base_w + self.rise_w, self.abs_w)

    def check(self, power_w: float, load: int, following: bool = True):
        """None if the step is fine (and it is accepted), else the abort reason."""
        if self.base_w is None:
            raise RuntimeError("PowerGuard.check before baseline")
        exp = self.expected_w()
        if power_w > exp:
            why = ("absolute ceiling" if exp >= self.abs_w else
                   "rise over baseline" if exp >= self.base_w + self.rise_w else "spike over trend")
            return f"power {power_w:.2f} W > expected {exp:.2f} W ({why}; baseline {self.base_w:.2f} W)"
        prev = self.hist[-1][1]
        if following and prev >= self.drop_min_load and abs(load) < (1 - self.drop_frac) * prev:
            return f"load fell {prev} -> {abs(load)} while following (a part giving way?)"
        self.hist.append((power_w, abs(load)))
        return None


# -- Port helpers (tools/rom_sweep.Port: .cmd(line, until_regex, timeout)) ----------

def read_current_ma(p, sid: int) -> float:
    out = p.cmd(f"reg {sid} {REG_CURRENT} 2", rf"reg {REG_CURRENT}( = \d+|: \S+)|timeout")
    m = re.search(rf"reg {REG_CURRENT} = (\d+)", out)
    if not m:
        raise RuntimeError(f"id {sid}: no reply to reg {REG_CURRENT}")
    return current_ma(int(m.group(1)))


def read_torque_limit(p, sid: int) -> int:
    out = p.cmd(f"reg {sid} {REG_TORQUE_LIMIT} 2", rf"reg {REG_TORQUE_LIMIT}( = \d+|: \S+)|timeout")
    m = re.search(rf"reg {REG_TORQUE_LIMIT} = (\d+)", out)
    if not m:
        raise RuntimeError(f"id {sid}: no reply to reg {REG_TORQUE_LIMIT}")
    return int(m.group(1))


def set_torque_limit(p, sid: int, permille: int) -> int:
    """Write the RAM torque limit (0-1000) byte by byte with `regw` -- lowering writes
    the high byte first, raising the low byte first, so the value in between never
    exceeds the larger of old and new. Returns the value read back."""
    permille = max(0, min(1000, int(permille)))
    old = read_torque_limit(p, sid)
    hi, lo = permille >> 8, permille & 0xFF
    order = [(REG_TORQUE_LIMIT + 1, hi), (REG_TORQUE_LIMIT, lo)] if permille < old else \
            [(REG_TORQUE_LIMIT, lo), (REG_TORQUE_LIMIT + 1, hi)]
    for addr, v in order:
        out = p.cmd(f"regw {sid} {addr} {v}", r"regw.*\n|refused|usage|\? \(try")
        if any(w in out for w in ("refused", "bad", "usage", "? (try")):
            raise RuntimeError(f"regw {sid} {addr} {v}: {out.strip()[-90:]!r}")
    got = read_torque_limit(p, sid)
    if got != permille:
        raise RuntimeError(f"id {sid}: torque limit reads {got}, wanted {permille}")
    return got


def sample_power(p, sid: int, volts: float, seconds: float, every: float = 0.08):
    """|current| x volts, sampled for `seconds` (during a move). At least one sample."""
    out, t_end = [], time.monotonic() + seconds
    while True:
        out.append(abs(read_current_ma(p, sid)) / 1000.0 * volts)
        if time.monotonic() >= t_end:
            return out
        time.sleep(every)
