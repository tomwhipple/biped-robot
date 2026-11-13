"""The trim's learned T on the recorded 2026-11-12 track runs, through the Python twin (sim/stance_trim.py), with the
keep rule as flashed (peak |e| over CLOSED only) and as fixed (CLOSED + HOLD: the whole single support).

Each chain is replayed from the flight record: it begins where the record's trim state turns CLOSED (with the
record's reference roll), holds and releases at the record's transitions, and sees the record's own error
each tick (roll = ref + e). The robot's roll does not depend on the replayed T (open loop), so this shows
what each rule does with what the robot actually did. The "as flashed" column reproduces the record's keep /
put-back decisions; its values run ~0.2-0.5 deg under the record (the replay ramps T in at once, the firmware
over the raise).

    python replay_trim.py      # the T 3.4 / 3.2 marches (16:38:03, 16:38:32 MST)
"""
from __future__ import annotations

import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "sim"))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import stance_trim as ST  # noqa: E402

FREC = "/home/claw/code/robot/hw_sessions/2026-11-09/flight_20261112T232515Z_prec16.frec"
RUNS = {"16:38:03": "march T 3.4", "16:38:32": "march T 3.2"}
D = math.degrees


class Flashed(ST.StanceTrim):
    """The rule as flashed in 8f3ad92: the HOLD did not count toward the peak."""

    def update(self, roll, dt, ramp):
        if self.state == ST.HOLD:
            if math.isfinite(roll):
                self.err = roll - self.ref
            return self.state
        return super().update(roll, dt, ramp)


def main():
    import flight_reader as FR
    S = FR.load(FREC)
    ticks = FR.tick_rows(S)
    evs = FR.event_rows(S)
    tt = np.array([r["t_utc"] for r in ticks])
    plays = [e for e in evs if e["kind"] == "cmd" and e["text"].startswith("track play")]
    for k, p in enumerate(plays):
        name = RUNS.get(time.strftime("%H:%M:%S", time.localtime(p["t_utc"])))
        if not name:
            continue
        t_end = plays[k + 1]["t_utc"] if k + 1 < len(plays) else p["t_utc"] + 60
        rows = [ticks[i] for i in np.where((tt >= p["t_utc"]) & (tt < t_end))[0]]
        tw = {"flashed": Flashed(), "fixed": ST.StanceTrim()}
        for t in tw.values():
            t.T = {"L": 5.2 * ST.D2R, "R": 3.4 * ST.D2R}
        prev = 0
        print(f"== {name}: per chain, T of the stance side at the chain's begin -> after its release (deg)")
        rec = None
        for r in rows:
            st, side = r["bal_state"], (chr(r["bal_side"]) if r["bal_side"] else "")
            if st == 5 and prev not in (5, 6):
                rec = dict(side=side, rec_begin=D(r["bal_integral"]), peak=0.0, peak_cl=0.0,
                           begin={k: D(t.T[side]) for k, t in tw.items()})
                for t in tw.values():
                    t.begin(side, r["bal_ref"])
            if rec is not None and st in (5, 6):
                if st == 6 and prev == 5:
                    for t in tw.values():
                        t.hold()
                roll = r["bal_ref"] + r["bal_err"]
                for t in tw.values():
                    t.update(roll, 0.02, 1.0)
                rec["peak"] = max(rec["peak"], abs(D(r["bal_err"])))
                if st == 5:
                    rec["peak_cl"] = max(rec["peak_cl"], abs(D(r["bal_err"])))
            if rec is not None and st == 7 and prev in (5, 6):
                for t in tw.values():
                    t.release()
                s = rec["side"]
                print(f"  {s}  peak |e| {rec['peak']:4.1f} (closed {rec['peak_cl']:4.1f})   record {rec['rec_begin']:5.2f} -> "
                      f"{D(r['bal_integral']):5.2f}   twin as flashed {rec['begin']['flashed']:5.2f} -> "
                      f"{D(tw['flashed'].T[s]):5.2f}   fixed {rec['begin']['fixed']:5.2f} -> {D(tw['fixed'].T[s]):5.2f}")
                rec = None
            prev = st
    return 0


if __name__ == "__main__":
    sys.exit(main())
