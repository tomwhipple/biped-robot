#!/usr/bin/env python3
"""Replay a bimo_gui session recording (hw_sessions/<stamp>-<host>.jsonl) as
the command stream it was: same flags, velocities and extended channels, at
the recorded times, to the sim twin or to the robot.

Why: "what did the operator actually command" is the half of a hardware take
nobody could reproduce. The GUI recorder writes every frame it put on the wire
(link/recorder.h). This tool puts the same frames back on the wire, so a drive
Tom did from the GUI can be run against `sim/sil_twin.py` (same policy, sim
plant) and against the robot again, and the beacons compared.

    # twin: start it first (no --boot-armed; the recording carries the ARM edges)
    (cd sim && ../.venv/bin/python sil_twin.py --run-name loco_v31home_s128r24 \
        --act-lag-hz 2 --duration 60 --obs-csv /tmp/twin_replay.csv) &
    .venv/bin/python tools/replay_session.py hw_sessions/2026-09-06/<file>.jsonl \
        --host 127.0.0.1 --port 4210 --tlm-port 4211 --from 75 --to 110 \
        --csv /tmp/twin_replay_beacons.csv

    # robot (Tom spotting): same, --host 192.168.2.90, default ports
    # --home-first sends RESET SERVOS + a 4 s home before the recording starts.

Safety (robot): the tilt and joint-swing guards of link/osc_probe.py run on
every beacon (ESTOP then disarm on trip); --from/--to clip the recording;
the replay always ends with a RESET SERVOS edge and a disarm, like osc_probe.
Timing: frames are sent at their recorded offsets (clock = time.monotonic),
so a 100 ms button blip stays a 100 ms blip. Records with the same second
are not thinned; the GUI sent at 20 Hz and so does this.
"""
import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "link"))
from osc_probe import OscDriver  # noqa: E402
from protocol import FLAG_ARM, FLAG_ESTOP, FLAG_HOME, FLAG_POSE, FLAG_ATT  # noqa: E402


def load(path, t_from, t_to):
    cmds = []
    for line in open(path):
        line = line.strip()
        if not line:
            continue
        try:
            r = json.loads(line)
        except json.JSONDecodeError:
            continue          # a live recording's torn last line (SMB append)
        if r.get("dir") != "cmd":
            continue
        if r["t"] < t_from or r["t"] > t_to:
            continue
        cmds.append(r)
    return cmds


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("jsonl")
    p.add_argument("--host", required=True)
    p.add_argument("--port", type=int, default=None)
    p.add_argument("--tlm-port", type=int, default=None)
    p.add_argument("--csv", required=True, help="beacon CSV (osc_probe format)")
    p.add_argument("--from", dest="t_from", type=float, default=0.0,
                   help="recording time to start at, s")
    p.add_argument("--to", dest="t_to", type=float, default=1e9,
                   help="recording time to stop at, s")
    p.add_argument("--tilt", type=float, default=0.90)
    p.add_argument("--pp-rad", type=float, default=0.5)
    p.add_argument("--home-first", action="store_true",
                   help="RESET SERVOS edge + 4 s home before the replay")
    p.add_argument("--pose", action="store_true", default=True,
                   help="request joint angles in every beacon (FLAG_POSE|FLAG_ATT); on by default")
    p.add_argument("--speed", type=float, default=1.0,
                   help="replay speed factor (1 = real time)")
    a = p.parse_args()

    cmds = load(a.jsonl, a.t_from, a.t_to)
    if not cmds:
        sys.exit(f"no cmd records in {a.jsonl} within [{a.t_from}, {a.t_to}]")
    t_base = cmds[0]["t"]
    print(f"== replay {Path(a.jsonl).name}: {len(cmds)} frames, recording "
          f"t={t_base:.2f}..{cmds[-1]['t']:.2f} s -> {a.host}", flush=True)
    changes = []
    prev = None
    for r in cmds:
        key = (r["flags"], round(r["vx"], 2), round(r["vy"], 2), round(r["wz"], 2),
               round(r["crouch"], 2), round(r.get("lift", 0.0), 2))
        if key != prev:
            changes.append((r["t"], key))
            prev = key
    for t, k in changes[:40]:
        print(f"   t={t:7.2f} flags=0x{k[0]:02x} vx={k[1]:+.2f} vy={k[2]:+.2f} "
              f"wz={k[3]:+.2f} crouch={k[4]:.2f} lift={k[5]:.2f}")
    if len(changes) > 40:
        print(f"   ... {len(changes) - 40} more changes")

    kw = {k: v for k, v in (("cmd_port", a.port), ("tlm_port", a.tlm_port))
          if v is not None}
    d = OscDriver(a.host, a.csv, a.tilt, a.pp_rad, **kw)
    print(d.host_clock_line(), flush=True)
    verdict = None
    try:
        if a.home_first:
            d.run_for(0.4, lambda t: (0.0, 0.0, FLAG_HOME), "RESET SERVOS: FLAG_HOME edge")
            d.run_for(4.0, lambda t: (0.0, 0.0, 0), "home settle")
        extra = (FLAG_POSE | FLAG_ATT) if a.pose else 0
        t0 = time.monotonic()
        for r in cmds:
            due = t0 + (r["t"] - t_base) / a.speed
            while True:
                verdict = d.pump(r["vx"], r["wz"], r["flags"] | extra)
                if verdict:
                    break
                now = time.monotonic()
                if now >= due:
                    break
                time.sleep(min(0.005, due - now))
            if verdict:
                break
            ext = dict(vy=r.get("vy", 0.0), crouch=r.get("crouch", 1.0),
                       lift=r.get("lift", 0.0), foot_dx=r.get("foot_dx", 0.0),
                       foot_dz=r.get("foot_dz", 0.0))
            d.send(r["vx"], r["wz"], r["flags"] | extra, ext=ext)
        if verdict:
            print(f"!! GUARD: {verdict} -- sending ESTOP then disarm", flush=True)
            d.run_for(1.0, lambda t: (0.0, 0.0, FLAG_ESTOP), "ESTOP")
            d.run_for(1.0, lambda t: (0.0, 0.0, 0), "disarm")
        else:
            d.run_for(0.4, lambda t: (0.0, 0.0, FLAG_ARM | FLAG_HOME),
                      "end: RESET SERVOS edge (still armed)")
            d.run_for(1.0, lambda t: (0.0, 0.0, 0), "disarm")
    finally:
        print(f"== done: {len(cmds)} frames replayed, {d.n_logged} beacons logged, "
              f"max joint p-p {d.max_pp:.3f} rad, min up_z {d.min_upz:+.2f}; "
              f"states: {d.states_seen}", flush=True)
        d.close()
    sys.exit(2 if verdict else 0)


if __name__ == "__main__":
    main()
