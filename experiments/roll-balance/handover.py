"""B2: what the fade of one stance side does while the next side's trim ramps in (the handover), in the
SIL (sil_gait.py: the firmware's track + TrimDriver + StanceTrim against the sim plant).

At T = 2 s the double support is 0.5 s and the fade 0.6 s, so the last chain's correction is still
fading when the next single support begins. Variants:
  residual   the driver's residual (the firmware now): the old hip finishes its fade while the new one ramps
  drop       the old behaviour of the track / policy path: the old hip's remaining correction dropped at once
  fade 0.4 / 0.3   residual, shorter fades (the fade ends inside the double support at 0.4 s at T = 2 s)
Per handover (a new single support beginning): the jump of the old hip's correction on that tick, and over
[-0.1, +0.3] s around it the peak torso roll rate and the roll excursion from the value at the handover.
Per run: falls, lifts, the firmware's TS lean toward the swing foot.

    python handover.py      # -> results/handover.json + a table
"""
from __future__ import annotations

import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sil_gait as G  # noqa: E402

VARIANTS = {"residual": dict(fade=0.6), "drop": dict(fade=0.6, drop_residual=True),
            "fade0.4": dict(fade=0.4), "fade0.3": dict(fade=0.3)}


def handovers(tr):
    t = np.array([r[0] for r in tr])
    roll = np.array([r[1] for r in tr])
    rate = np.array([r[2] for r in tr])
    side = [r[3] for r in tr]
    hip = {76: np.array([r[4] for r in tr]), 82: np.array([r[5] for r in tr])}     # 'L', 'R'
    out, last = [], 0
    for i in range(1, len(tr)):
        if side[i] and side[i] != last and last:
            old = last
            jump = abs(hip[old][i] - hip[old][i - 1])
            w = (t >= t[i] - 0.1) & (t <= t[i] + 0.3)
            out.append(dict(t=float(t[i]), jump=float(jump), rate=float(np.max(np.abs(rate[w]))),
                            exc=float(np.max(np.abs(roll[w] - roll[i])))))
        if side[i]:
            last = side[i]
    return out


def one(a):
    T, d, plant, var = a
    try:
        r = G.run(T, d, plant, 0.0, 8, trace=True, **VARIANTS[var])
        hs = handovers(r.pop("trace"))
        return dict(T=T, dir=d, plant=plant, var=var, fell=r["fell"], lifts=r["lifts"], n=r["n_steps"],
                    lean_in=max([x["lean_in"] for x in r["reports"]] or [0.0]), handovers=hs,
                    reverted=sum(x.get("reverted", False) for x in r["reports"]))
    except Exception as e:  # noqa: BLE001
        return dict(args=list(a), error=repr(e)[:300])


def main():
    jobs = [(T, d, p, v) for T in (2.0, 2.5, 3.0) for d in ("march", "fwd", "sideL") for p in ("fit", "hi")
            for v in VARIANTS]
    with ProcessPoolExecutor(16) as ex:
        rows = list(ex.map(one, jobs))
    with open(os.path.join(HERE, "results", "handover.json"), "w") as f:
        json.dump(rows, f)
    errs = [r for r in rows if "error" in r]
    print(f"{len(rows) - len(errs)} runs, {len(errs)} errors", errs[:1])
    print("T    variant    falls lifts   TS lean max | per handover: old-hip jump deg (median / max), "
          "roll rate peak deg/s (median / max), roll excursion deg (median / max)")
    for T in (2.0, 2.5, 3.0):
        for v in VARIANTS:
            rs = [r for r in rows if "error" not in r and r["T"] == T and r["var"] == v]
            hs = [h for r in rs for h in r["handovers"]]
            if not hs:
                continue
            j = [h["jump"] for h in hs]
            rt = [h["rate"] for h in hs]
            ex_ = [h["exc"] for h in hs]
            print(f"{T:3.1f}  {v:9s}  {sum(r['fell'] for r in rs):3d}  {sum(r['lifts'] for r in rs):3d}/"
                  f"{sum(r['n'] for r in rs):3d}  {max(r['lean_in'] for r in rs):5.1f}     | "
                  f"{np.median(j):4.2f} / {max(j):4.2f}    {np.median(rt):5.1f} / {max(rt):5.1f}    "
                  f"{np.median(ex_):4.2f} / {max(ex_):4.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
