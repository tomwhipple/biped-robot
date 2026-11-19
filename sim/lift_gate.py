"""The lift gate (#297): the Python twin of firmware/components/gaitplay/lift_gate. Same states, same
arithmetic; firmware/host/test_lift_gate.cpp pins the C++ to the rows this file prints
(`python sim/lift_gate.py`), tests/test_lift_gate.py checks the two agree.

The track player looks at the body when its clock reaches a frame flagged gaitplay::kGate (the last double
support frame before a lift-off): level  |roll - roll_ds[side]| <= lvl  and quiet  |roll rate| <= rate. Both:
the clock goes on. Else it is parked at the frame's time until both have held for dwell_ms in a row; a hold
past timeout_ms is a timeout (the caller ends the track from this double support). roll_ds per stance side:
seeded with the roll of the track's first tick, the first acceptance of a side takes its own roll, later ones
move it by alpha of the difference; only accepted rolls move it.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

D2R = math.pi / 180.0
NONE, PASS, HOLD_START, HOLD_END, TIMEOUT = 0, 1, 2, 3, 4


@dataclass
class Config:
    enabled: bool = True
    lvl: float = 3.0 * D2R
    rate: float = 15.0 * D2R
    dwell_ms: int = 150
    timeout_ms: int = 3000
    alpha: float = 0.25


def validate(c: Config):
    """None when usable, else why not."""
    if not (0.00873 <= c.lvl <= 0.34907):
        return "lvl must be 0.5-20 deg"
    if not (0.01745 <= c.rate <= 3.4907):
        return "rate must be 1-200 deg/s"
    if not (0 <= c.dwell_ms <= 2000):
        return "dwell must be 0-2000 ms"
    if not (c.dwell_ms + 100 <= c.timeout_ms <= 30000):
        return "timeout must be dwell + 100 ms .. 30000 ms"
    if not (0.0 <= c.alpha <= 1.0):
        return "ref_alpha must be 0-1"
    return None


@dataclass
class Out:
    t_ms: int = 0
    holding: bool = False
    ev: int = NONE
    hold_ms: int = 0
    dev: float = 0.0
    rate: float = 0.0
    side: str = ""


@dataclass
class Stats:
    gates: int = 0
    holds: int = 0
    held_ms: int = 0
    timeouts: int = 0


class LiftGate:
    def __init__(self, cfg: Config | None = None):
        self.cfg = cfg or Config()
        self.begin()

    def begin(self):
        self.st = Stats()
        self.start = None
        self.refs = {"L": None, "R": None}
        self.holding = self.quiet = False
        self.held = self.hold_t0 = self.quiet_t0 = self.last_hold_ms = self.last_gate_t = self.t_gate = 0
        self.side = ""
        self.seen = False

    def ref(self, side):
        r = self.refs["R" if side == "R" else "L"]
        return r if r is not None else (self.start if self.start is not None else math.nan)

    def _good(self, roll, rate, side):
        r = self.ref(side)
        d = roll - r
        if not (math.isfinite(roll) and math.isfinite(rate) and math.isfinite(r)):
            return False, d
        return abs(d) <= self.cfg.lvl and abs(rate) <= self.cfg.rate, d

    def _accept(self, side, roll):
        k = "R" if side == "R" else "L"
        if self.refs[k] is None:
            self.refs[k] = roll
        else:
            self.refs[k] += self.cfg.alpha * (roll - self.refs[k])

    def tick(self, wall_ms, t_raw_ms, have_gate, t_gate_ms, side, roll, rate) -> Out:
        o = Out()
        if self.start is None and math.isfinite(roll):
            self.start = roll
        t = t_raw_ms - self.held if t_raw_ms > self.held else 0
        o.t_ms, o.rate = t, rate
        c = self.cfg
        if self.holding:
            o.side = self.side
            ok, d = self._good(roll, rate, self.side)
            ok = ok and c.enabled
            o.dev = d
            if ok:
                if not self.quiet:
                    self.quiet, self.quiet_t0 = True, wall_ms
            else:
                self.quiet = False
            dur = wall_ms - self.hold_t0
            self.held = t_raw_ms - self.t_gate if t_raw_ms > self.t_gate else 0
            o.t_ms = self.t_gate
            if not c.enabled or (self.quiet and wall_ms - self.quiet_t0 >= c.dwell_ms):
                self.holding = False
                o.ev, o.hold_ms = HOLD_END, dur
                self.st.held_ms += dur
                self.last_hold_ms = dur
                if c.enabled:
                    self._accept(self.side, roll)
            elif dur >= c.timeout_ms:
                self.holding = False
                o.ev, o.hold_ms = TIMEOUT, dur
                self.st.held_ms += dur
                self.st.timeouts += 1
                self.last_hold_ms = dur
            else:
                o.holding = True
            return o
        if not c.enabled or not have_gate:
            return o
        if self.seen and t_gate_ms <= self.last_gate_t:
            return o
        if t_gate_ms > t:
            return o
        self.seen, self.last_gate_t = True, t_gate_ms
        o.side = side
        self.st.gates += 1
        ok, d = self._good(roll, rate, side)
        o.dev = d
        if ok:
            self._accept(side, roll)
            self.last_hold_ms = 0
            o.ev = PASS
            return o
        self.holding, self.quiet, self.side, self.t_gate, self.hold_t0 = True, False, side, t_gate_ms, wall_ms
        self.held = t_raw_ms - t_gate_ms if t_raw_ms > t_gate_ms else 0
        self.st.holds += 1
        o.ev, o.holding, o.t_ms = HOLD_START, True, t_gate_ms
        return o


def _r(x):
    return round(x, 6)


def golden():
    """The sequence test_lift_gate.cpp replays, one row per 20 ms tick:
    (wall, t_raw, have_gate, t_gate, side, roll, rate, t_out, holding, ev, dev, cfg_change) -> the gate's output.
    The track clock is the caller's: t_raw = wall + 1000 (a 1 s lead-in), the gate frame times are on the
    track timeline. cfg_change: 0 none, 1 disable at this tick, 2 enable."""
    g = LiftGate()
    rows = []
    wall = 0
    gates = []                     # (t_gate, side) the frames the "queue" holds

    def run(n, roll_f, rate_f, gate_f, change=0):
        nonlocal wall
        for k in range(n):
            if change and k == 0:
                g.cfg.enabled = change == 2
            roll, rate = _r(roll_f(k)), _r(rate_f(k))
            t_raw = wall + 1000
            hg, tg, sd = gate_f(k, t_raw - g.held)
            o = g.tick(wall, t_raw, hg, tg, sd, roll, rate)
            rows.append((wall, t_raw, int(hg), tg, sd or "-", roll, rate, o.t_ms, int(o.holding), o.ev,
                         _r(o.dev) if math.isfinite(o.dev) else 0.0, change if k == 0 else 0))
            wall += 20

    zero, quiet = (lambda k: 0.0), (lambda k: 0.0)

    def gate_at(tg, sd):
        # the queue still holds the frame until the clock has passed it by one tick
        return lambda k, t_nat: (t_nat < tg + 20, tg, sd)

    nog = lambda k, t: (False, 0, "")  # noqa: E731
    # 1. the track starts level 1.0 deg (the seed); 10 ticks without a gate
    run(10, lambda k: 0.017, quiet, nog)
    # 2. a gate on L-stance lifts at track time 1400: level (+0.5 deg off the seed) and quiet -> pass, no hold
    run(25, lambda k: 0.0175 + 0.009, lambda k: 0.05, gate_at(1400, "L"))
    # 3. R stance gate at 2600: the body leans 5 deg off the seed -> hold; settles after 0.3 s, then dwell 150 ms
    run(30, lambda k: 0.017 + 0.087 * max(0.0, 1.0 - k / 15.0), lambda k: 0.0, nog)   # (no gate: just rolls)
    run(60, lambda k: 0.017 + 0.087 * max(0.0, 1.0 - k / 15.0), lambda k: 0.3 if k < 15 else 0.0,
        gate_at(2300, "R"))
    # 4. a noisy settle: level but a rate spike resets the dwell
    run(5, lambda k: 0.02, quiet, nog)
    run(40, lambda k: 0.02 + 0.07 * (1 if k < 10 else 0), lambda k: 0.4 if k in (13, 14) else 0.0,
        gate_at(3400, "L"))
    # 5. never settles: timeout after 3 s
    run(5, lambda k: 0.02, quiet, nog)
    run(170, lambda k: 0.02 + 0.12, lambda k: 0.0, gate_at(4200, "R"))
    # 6. a non-finite roll while holding does not count as level; a hold released by `liftgate off`
    g.begin()
    wall = 0
    run(6, lambda k: 0.0, quiet, nog)
    run(15, lambda k: math.nan if k > 3 else 0.0, quiet, gate_at(1200, "L"))
    run(8, lambda k: 0.15, quiet, gate_at(1200, "L"), change=1)
    run(8, lambda k: 0.15, quiet, nog, change=2)
    return rows


if __name__ == "__main__":
    for r in golden():
        wall, t_raw, hg, tg, sd, roll, rate, t_out, hold, ev, dev, ch = r
        rolls = "NAN" if not math.isfinite(roll) else f"{roll:.6f}f"
        print(f"{{{wall}, {t_raw}, {hg}, {tg}, '{sd}', {rolls}, {rate:.6f}f, {t_out}, {hold}, {ev}, {dev:.6f}f, {ch}}},")
