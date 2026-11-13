"""The lift gate (sim/lift_gate.py, the twin of gaitplay::LiftGate) against the recorded track runs: what would it
have seen at every lift-off, and which of the falls would it have held?

Each `track play` in the flight records starts a gate (seeded by the roll at the end of its lead-in, like the
firmware's first track tick); every stance-trim chain that begins (balance state CLOSED after anything but
CLOSED / HOLD) is a lift-off, and the gate is asked at that tick (--lag ticks earlier too: the firmware checks at
the last gate frame before lift-off, up to ~100 ms = 5 ticks before). The record is open loop (the robot did not
wait), so this says what the gate would have SEEN and whether it would have held, not what the hold would have
led to. Roll is atan2(-up_y, up_z) (the firmware's rollFromUp), the rate the gyro x, both of the recorded tick.

    python lift_gate_replay.py --from "2026-11-12 23:30" --to "2026-11-13 16:00" frec1 frec2 ...    (UTC)
"""
from __future__ import annotations

import argparse
import datetime as dt
import math
import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "sim"))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import flightrec as F  # noqa: E402
import lift_gate as LG  # noqa: E402

REC = struct.Struct("<HQ")
D = math.degrees
MST = dt.timezone(dt.timedelta(hours=-7))


def stream(path):
    """(ticks, events): ticks (boot, t_utc_us, t_boot_us, roll, gyro_x, bal_state, bal_side), events (.., code, a, b, text)."""
    ticks, events = [], []
    with open(path, "rb") as f:
        f.read(len(b"FRECSESS1\n"))
        f.readline()
        while True:
            h = f.read(REC.size)
            if len(h) < REC.size:
                break
            n, _rx = REC.unpack(h)
            d = f.read(n)
            if len(d) < n:
                break
            try:
                p = F.decode(d)
            except F.DecodeError:
                continue
            if p["type"] == F.T_TICK:
                up, b = p["up"], p["bal"]
                ticks.append((p["boot_id"], p["t_utc_us"], p["t_boot_us"], math.atan2(-up[1], up[2]), p["gyro"][0],
                              b["state"], b["side"]))
            elif p["type"] == F.T_EVENT:
                events.append((p["boot_id"], p["t_utc_us"], p["t_boot_us"], p["code"], p["a"], p["b"], p["text"]))
    return ticks, events


def with_utc(ticks, events):
    off = {}
    for r in ticks + events:
        if r[1]:
            off.setdefault(r[0], []).append(r[1] - r[2])
    med = {b: sorted(v)[len(v) // 2] for b, v in off.items()}
    u = lambda r: (r[1] if r[1] else r[2] + med.get(r[0], 0)) / 1e6  # noqa: E731
    return sorted(((u(r),) + r[3:] for r in ticks)), sorted(((u(r),) + r[3:] for r in events))


def plays_of(ticks, events, t0, t1):
    t = np.array([x[0] for x in ticks])
    plays = [e for e in events if e[1] == F.EV_CMD and e[4].startswith("track play") and t0 <= e[0] < t1]
    falls = [e[0] for e in events if e[1] == F.EV_FALL and e[3] == 1]
    out = []
    for k, p in enumerate(plays):
        nxt = plays[k + 1][0] if k + 1 < len(plays) else p[0] + 60
        fl = [f for f in falls if p[0] < f < nxt + 1]
        end = min([nxt] + fl)
        a, b = np.searchsorted(t, p[0]), np.searchsorted(t, end)
        seg = ticks[a:b]
        if len(seg) < 100:
            continue
        chains, prev = [], 0
        for i, x in enumerate(seg):
            if x[3] == 5 and prev not in (5, 6):
                chains.append(i)
            prev = x[3]
        out.append(dict(t=p[0], fell=bool(fl), t_fall=fl[0] if fl else None, seg=seg, chains=chains,
                        lead_end=p[0] + 2.5))
    return out


def gate_run(pl, lag, cfg=None):
    """The twin over one play: [(chain tick, side, dev deg, rate deg/s, held?)]; no hold is modelled."""
    g = LG.LiftGate(cfg or LG.Config())
    seg = pl["seg"]
    k0 = next((i for i, x in enumerate(seg) if x[0] >= pl["lead_end"]), 0)
    g.tick(0, 0, False, 0, "", seg[k0][1], 0.0)             # the seed
    rows = []
    wall = 1000
    for i in pl["chains"]:
        j = max(0, i - lag)
        side = chr(seg[i][4]) if seg[i][4] else "L"
        roll, rate = seg[j][1], seg[j][2]
        o = g.tick(wall, wall, True, wall, side, roll, rate)
        rows.append((i, side, D(o.dev), D(rate), o.ev == LG.HOLD_START))
        if o.ev == LG.HOLD_START:                           # the record carries on: drop the hold, keep the gate
            g.holding = False
        wall += 5000
    return rows


def mst(t):
    return dt.datetime.fromtimestamp(t, MST).strftime("%m-%d %H:%M:%S")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frec", nargs="+")
    ap.add_argument("--from", dest="t0", default="2026-11-12 23:30", help="UTC")
    ap.add_argument("--to", dest="t1", default="2026-11-13 16:00", help="UTC")
    ap.add_argument("--lags", default="0,2,5", help="ticks before the lift-off at which the gate looks")
    a = ap.parse_args(argv)
    p = lambda s: dt.datetime.strptime(s, "%Y-%m-%d %H:%M").replace(tzinfo=dt.timezone.utc).timestamp()  # noqa: E731
    t0, t1 = p(a.t0), p(a.t1)
    ticks, events = [], []
    for path in a.frec:
        t, e = stream(path)
        ticks += t
        events += e
    ticks, events = with_utc(ticks, events)
    plays = plays_of(ticks, events, t0, t1)
    lags = [int(x) for x in a.lags.split(",")]
    print(f"{len(plays)} track plays, {sum(len(x['chains']) for x in plays)} lift-offs, "
          f"{sum(x['fell'] for x in plays)} plays that ended in a fall ({mst(t0)} - {mst(t1)} MST)")
    print("\nclean steps (every lift-off except the last of a play that fell):")
    for lag in lags:
        n = held = 0
        devs, rates = [], []
        for pl in plays:
            rows = gate_run(pl, lag)
            for k, r in enumerate(rows):
                if pl["fell"] and k == len(rows) - 1:
                    continue
                n += 1
                held += r[4]
                if k:
                    devs.append(abs(r[2]))
                    rates.append(abs(r[3]))
        print(f"  gate {lag * 20:3d} ms before lift-off: {held} of {n} held ({100 * held / max(n, 1):.2f} %); "
              f"|roll - roll_ds| p50 {np.percentile(devs, 50):.2f} p95 {np.percentile(devs, 95):.2f} max {max(devs):.2f} deg, "
              f"|rate| p50 {np.percentile(rates, 50):.1f} p95 {np.percentile(rates, 95):.1f} max {max(rates):.1f} deg/s")
    print("\nthe plays that ended in a fall: the gate's look at the last lift-off (lvl 3 deg, rate 15 deg/s):")
    print("  play (MST)   fall (MST)   last lift-off  side  dev deg  rate deg/s  gate   the lift-off before it: dev / rate")
    for pl in plays:
        if not pl["fell"]:
            continue
        per_lag = {lag: gate_run(pl, lag) for lag in lags}
        r0 = per_lag[lags[0]]
        if not r0:
            print(f"  {mst(pl['t'])}  {mst(pl['t_fall'])}  no lift-off before the fall")
            continue
        last, prev = r0[-1], (r0[-2] if len(r0) > 1 else None)
        seg = pl["seg"]
        t_lo = seg[last[0]][0]
        verdicts = "/".join("HOLD" if per_lag[l][-1][4] else "pass" for l in lags)
        print(f"  {mst(pl['t'])}  {mst(pl['t_fall'])}  {mst(t_lo)} ({pl['t_fall'] - t_lo:5.1f} s before the fall)  {last[1]}  "
              f"{last[2]:+7.2f}  {last[3]:+8.1f}  {verdicts}  "
              + (f"{prev[2]:+.2f} / {prev[3]:+.1f}" if prev else "-"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
