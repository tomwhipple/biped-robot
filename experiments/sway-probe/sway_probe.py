"""Robot-side sender for the sway-probe ladder: ONE rung per invocation, dry run by default.

    .venv/bin/python experiments/sway-probe/sway_probe.py                      # the ladder and its sim numbers
    .venv/bin/python experiments/sway-probe/sway_probe.py --rung 4             # dry run of rung 4: plan, sim prediction, stops
    .venv/bin/python experiments/sway-probe/sway_probe.py --rung 4 --go        # runs rung 4 on the robot (the owner's go)

What it sends. The firmware's streamed `pose` is the only path that moves a benched robot through
the Wi-Fi CLI, and it is a minimum-jerk move of 200..20000 ms from the servos' READ-BACK positions
to ONE target per command (cli.cpp cmdPose): it cannot play 50 Hz keyframes. The sender therefore
chains `pose <17 bus ticks> s<seg_ms>` commands, one per `seg_ms` (default 200 ms, the firmware's
minimum), each to the 50 Hz keyframe `seg_ms` ahead (sway_ref.py), so the joints follow the training
reference with a staircase of rest-to-rest moves. That is the playback sway_sim.py rehearses
(`--seg-ms`), and the predictions are for it.

Safety, in the order the sender enforces it:
  * No `--go`: nothing is opened, nothing is sent. The plan and the sim prediction are printed.
  * `--go` needs `--rung N`, exactly one rung per run, and an interactive `GO N` typed back.
  * It refuses a rung on which any plant of the sim band fell (`--accept-sim-falls` overrides, in the owner's words).
  * It refuses to start unless the flight-recorder collector is up here (tools/flight_session.sh) AND the
    robot's `rec` says it records to this host.
  * Before the rung: `bench`, torque check, `cal show` (all 12 leg joints fitted), `pose` to the zero stand,
    then the torso must read upright (IMU `up`) within `--start-tilt` degrees.
  * During the rung: after every segment it reads the IMU; tilt past the rung's stop tilt, a rate past the
    stop rate, a refused or unanswered pose, or Ctrl-C ends the rung.
  * Every ending, normal or not, is `home`: the firmware's smooth move to the calibrated zero, hip
    play take-up and read-back, ending with torque released (the way the one-leg stands end). If `home`
    does not answer, it sends `release`.
"""
from __future__ import annotations

import argparse
import datetime
import json
import math
import os
import re
import socket
import subprocess
import sys
import time
from dataclasses import dataclass, replace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "link"))
import sway_ref as SR  # noqa: E402
import bus_cal  # noqa: E402

DEFAULT_PREDICTIONS = os.path.join(HERE, "predictions.json")
SEG_MS = 200                       # firmware minimum for `pose s<ms>`
SEG_MS_MIN, SEG_MS_MAX = 200, 20000
SETTLE_MS = 2500                   # the first move to the zero stand
START_TILT_DEG = 3.0               # the robot must stand this upright before a rung starts
STOP_TILT_FLOOR, STOP_TILT_CAP = 6.0, 12.0     # deg: the host stop; 12 is the bench balance abort, 20/30 the guards
STOP_TILT_FACTOR = 1.5             # of the worst tilt the sim band reached on the rung
STOP_RATE = 1.0                    # rad/s of |gyro|: the floor of the rate stop (the flail guard's EMA limit is 1.5)
STOP_RATE_CAP = 3.0
COLLECTOR_PORT = 4220


# ------------------------------------------------------------------------------------- plan
@dataclass
class Segment:
    t_s: float                     # when it is sent, from the rung start
    ms: int                        # its s<ms>
    deg: dict                      # joint -> sim-frame degrees (the target keyframe)
    ticks: list                    # 17 bus ticks
    stance: str | None = None      # "L" | "R" when the bench roll PI rides on it, else None

    def line(self):
        tail = f" s{self.ms}" + (f" B{self.stance}" if self.stance else "")
        return "pose " + " ".join(str(t) for t in self.ticks) + tail


def illustrative_cal():
    """The compiled-in table with every leg joint marked fitted (ankle rolls at 2048): dry-run ticks only."""
    base = bus_cal.asbuilt_prototype_cal()
    legs = set(bus_cal.POLICY_ORDER)
    js = [replace(j, fitted=True) if j.name in legs or j.name.endswith("ankle_roll") else j for j in base.joints]
    return bus_cal.BusCal(js, source="illustrative")


def check_cal(cal, names):
    """The cal must carry every joint the rung drives, fitted (a refused pose is how an unfitted one shows up)."""
    miss = [n for n in names if not cal.by_name(n).fitted]
    if miss:
        raise SystemExit(f"cal: {miss} NOT FITTED on this robot -- refusing")


def plan(rung, cal, seg_ms=SEG_MS, balance="off"):
    """The chained `pose` segments that play `rung` (keyframes at 50 Hz) with segments of seg_ms."""
    if not SEG_MS_MIN <= seg_ms <= SEG_MS_MAX or seg_ms % 20:
        raise ValueError(f"seg_ms {seg_ms}: the firmware takes 200..20000 and writes on 20 ms ticks")
    step = seg_ms // 20
    n = len(rung.t)
    segs = []
    k = 0
    while k < n - 1:
        j = min(k + step, n - 1)
        deg = {nm: float(np.degrees(rung.q[j][i])) for i, nm in enumerate(rung.names)}
        stance = None
        if balance == "stance":
            mid = rung.phase[(k + j) // 2]
            stance = "R" if math.sin(mid) > 0.0 else "L"        # the foot that is not swinging
        segs.append(Segment(k / SR.KEY_HZ, seg_ms, deg, cal.pose_ticks(deg), stance))
        k += step              # every segment is a full seg_ms (the firmware's minimum is 200); the last holds the stand
    return segs


def zero_line(cal, ms=SETTLE_MS):
    return "pose " + " ".join(str(t) for t in cal.pose_ticks({})) + f" s{ms}"


# ------------------------------------------------------------------------------------- predictions
def load_predictions(path=DEFAULT_PREDICTIONS):
    try:
        with open(path) as f:
            return json.load(f)
    except OSError:
        return None


def prediction_for(pred, rung, seg_ms, balance="off"):
    """(the rung's band record, why-not) -- the prediction only counts if it was made for this plan."""
    if pred is None:
        return None, "no predictions file"
    mt = pred["meta"]
    want = dict(seg_ms=float(seg_ms), lead=rung.lead, hz=rung.hz, cycles=rung.cycles, ramp_s=rung.ramp_s)
    for k, v in want.items():
        if abs(float(mt[k]) - v) > 1e-9:
            return None, f"predictions were made for {k}={mt[k]}, this plan has {v}"
    if (mt.get("balance", "off")) != balance:
        return None, f"predictions were made for balance={mt.get('balance', 'off')}, this plan has {balance}"
    for R in pred["rungs"]:
        r = R["rung"]
        if r["index"] == rung.index and r["kind"] == rung.kind and abs(r["M_m"] - rung.M) < 1e-9:
            return R, None
    return None, "no sim rung with this index/kind/M"


def stops_for(R):
    """Per-rung stop criteria from the sim band: tilt = 1.5x the worst band tilt (6..12 deg); |gyro| = 1.5x the worst
    band roll rate (1..3 rad/s). The robot's gyro is read at one instant per segment, so the rate stop is on the
    instantaneous rate, not the guard's 0.2 s EMA."""
    worst = R["band"]["tilt_peak_deg"]["max"] if R else 0.0
    tilt = min(STOP_TILT_CAP, max(STOP_TILT_FLOOR, STOP_TILT_FACTOR * worst))
    wr = math.radians(R["band"]["roll_rate_peak_dps"]["max"]) if R else 0.0
    return dict(tilt_deg=round(tilt, 1), rate_rad_s=round(min(STOP_RATE_CAP, max(STOP_RATE, STOP_TILT_FACTOR * wr)), 2))


def fmt3(b, scale=1.0, d=1):
    return f"{b['min'] * scale:.{d}f} / {b['med'] * scale:.{d}f} / {b['max'] * scale:.{d}f}"


def describe_prediction(R):
    b = R["band"]
    out = [f"  sim band ({b['n']} plants, min / median / max): stays up {b['up']}/{b['n']}; peak torso tilt "
           f"{fmt3(b['tilt_peak_deg'])} deg; roll rate {fmt3(b['roll_rate_peak_dps'], 1, 0)} deg/s; gyro EMA {fmt3(b['gyro_ema_peak'], 1, 2)} rad/s"]
    for side, s in b["sides"].items():
        out.append(f"  {side} foot swings: clearance {fmt3(s['clear_mm'])} mm (clears {s['clears']}/{s['n']}); "
                   f"load at its lightest {fmt3(s['F_min_frac'], 100, 0)} % of weight (sim only: the robot has no force sensor); "
                   f"CoM over the stance sole by {fmt3(s['com_margin_mm'])} mm")
    return out


# ------------------------------------------------------------------------------------- the robot
class Robot:
    """The Wi-Fi CLI, one command at a time (link/wcliproto.Client). `run(line) -> reply text`."""

    def __init__(self, host, key_file=None, port=None):
        import wcliproto as W
        self.W = W
        key = W.load_key(key_file or W.DEFAULT_KEY_FILE)
        self.buf = []
        self.client = W.Client(host, key, port or W.PORT, on_text=self.buf.append)
        self.client.connect()

    def run(self, line, timeout=10.0):
        self.buf.clear()
        r = self.client.command(line, idle_timeout=timeout)
        if r.status != "done":
            raise RuntimeError(f"{line.split()[0]}: {r.status}")
        return (r.text or "".join(self.buf)).replace("\r\n", "\n")


def local_ip_toward(host):
    """The address this machine uses to reach `host` (a connected UDP socket sends nothing)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((host, 9))
        return s.getsockname()[0]
    finally:
        s.close()


def main_checkout():
    """The main checkout (hw_sessions/ lives there, as flight_session.sh puts the flight files): the parent of the git
    common dir, or this tree when git cannot say."""
    try:
        p = subprocess.run(["git", "-C", ROOT, "rev-parse", "--path-format=absolute", "--git-common-dir"], capture_output=True, text=True, timeout=10)
        if p.returncode == 0 and p.stdout.strip():
            return os.path.dirname(p.stdout.strip())
    except (OSError, subprocess.SubprocessError):
        pass
    return ROOT


def collector_local_status():
    """`tools/flight_session.sh status` -> (running, text)."""
    p = subprocess.run([os.path.join(ROOT, "tools", "flight_session.sh"), "status"], capture_output=True, text=True, timeout=10)
    txt = (p.stdout + p.stderr).strip()
    return txt.startswith("running"), txt


def check_collector(robot_run, my_ip, local_status=collector_local_status, port=COLLECTOR_PORT):
    """(ok, why): the collector runs here and the robot records to it. robot_run('rec') is the status line."""
    up, txt = local_status()
    if not up:
        return False, f"flight collector is not running here ({txt or 'no status'}): tools/flight_session.sh start sway"
    rec = robot_run("rec")
    m = re.search(r"rec\s+(ON|off)\s+->\s+(\d+\.\d+\.\d+\.\d+):(\d+)", rec)
    if not m:
        return False, f"could not read the robot's `rec` status: {rec.strip()[:100]!r}"
    if m.group(1) != "ON":
        return False, "the robot's flight recorder is OFF (`rec on`)"
    if m.group(2) != my_ip or int(m.group(3)) != port:
        return False, f"the robot records to {m.group(2)}:{m.group(3)}, not to this host {my_ip}:{port}"
    return True, f"collector up ({txt.splitlines()[-1] if txt else ''}); robot records to {my_ip}:{port}"


def parse_imu(text):
    """`imu` status -> (tilt_deg, |gyro| rad/s) from its `up` and `gyro` lines, or None."""
    u = re.search(r"^up\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)", text, re.M)
    g = re.search(r"^gyro\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)", text, re.M)
    if not (u and g):
        return None
    up = [float(x) for x in u.groups()]
    gy = [float(x) for x in g.groups()]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, up[2]))))
    return tilt, math.sqrt(sum(x * x for x in gy))


# ------------------------------------------------------------------------------------- the run
def run_rung(robot, rung, cal, segs, stops, log, my_ip, seg_ms=SEG_MS, start_tilt=START_TILT_DEG,
             collector_check=check_collector, clock=time.monotonic, sleep=time.sleep):
    """Run the rung on `robot` (an object with run(line)). Always ends with `home`. Returns (status, why)."""
    say = log

    def home(why):
        say(f"== {why}: home (smooth to the calibrated zero, torque released)")
        try:
            r = robot.run("home", 15.0)
            say(r.strip())
            return
        except Exception as e:               # noqa: BLE001 -- the end must be attempted whatever happened
            say(f"!! home failed ({e}); release")
        try:
            say(robot.run("release", 5.0).strip())
        except Exception as e:               # noqa: BLE001
            say(f"!! release failed too ({e}) -- the robot may still hold torque")

    ok, why = collector_check(robot.run, my_ip)
    say(f"collector: {why}")
    if not ok:
        return "refused", why
    try:
        say("> bench: " + robot.run("bench", 5.0).strip()[-80:])
        sleep(1.0)
        t = robot.run("torque", 5.0)
        say("> torque: " + t.strip()[-60:])
        if "busy" in t:
            return "refused", "the bus is still busy after `bench`"
        text = robot.run("cal show", 5.0)
        got = bus_cal.parse_cal_show(text)
        check_cal(got, rung.names)
        if got.pose_ticks({}) != cal.pose_ticks({}) or [j.direction for j in got.joints] != [j.direction for j in cal.joints]:
            return "refused", "the robot's calibration differs from the one this plan was encoded with"
        r = robot.run(zero_line(cal), 5.0)
        say("> zero stand: " + r.strip()[-60:])
        if "streaming" not in r:
            home("zero stand refused")
            return "refused", f"zero stand: {r.strip()[:80]}"
        sleep(SETTLE_MS / 1000 + 1.0)
        imu = parse_imu(robot.run("imu", 5.0))
        if imu is None:
            home("no IMU reading")
            return "refused", "no IMU reading before the rung"
        say(f"upright check: tilt {imu[0]:.2f} deg |gyro| {imu[1]:.3f} rad/s (start limit {start_tilt} deg)")
        if imu[0] > start_tilt:
            home("not upright")
            return "refused", f"torso tilt {imu[0]:.1f} deg before the rung"
        t0 = clock()
        say(f"== rung {rung.label}: {len(segs)} segments of {seg_ms} ms, stops: tilt {stops['tilt_deg']} deg, |gyro| {stops['rate_rad_s']} rad/s")
        for k, s in enumerate(segs):
            lag = clock() - (t0 + s.t_s)
            if lag < 0:
                sleep(-lag)
            r = robot.run(s.line(), 3.0)
            if "streaming" not in r:
                home("pose refused")
                return "stopped", f"segment {k}: {r.strip()[:80]}"
            imu = parse_imu(robot.run("imu", 3.0))
            say(f"seg {k:3d} t {s.t_s:5.2f} s late {max(lag, 0) * 1e3:4.0f} ms  " + ("imu ?" if imu is None else f"tilt {imu[0]:5.2f} deg |gyro| {imu[1]:.3f}"))
            if imu is None:
                home("IMU unreadable")
                return "stopped", f"segment {k}: no IMU reading"
            if imu[0] > stops["tilt_deg"] or imu[1] > stops["rate_rad_s"]:
                home("stop criterion")
                return "stopped", f"segment {k}: tilt {imu[0]:.1f} deg, |gyro| {imu[1]:.2f} rad/s"
        sleep(seg_ms / 1000)
    except KeyboardInterrupt:
        home("Ctrl-C")
        return "stopped", "Ctrl-C"
    except Exception as e:                    # noqa: BLE001
        home(f"error {e!r}")
        return "stopped", repr(e)
    home("rung finished")
    return "done", ""


# ------------------------------------------------------------------------------------- the CLI
def build(a):
    ref = SR.SwayReference(SR.load_config(a.config))
    rungs = ref.ladder(a.M, a.lead, a.hz, a.cycles, a.ramp)
    return ref, rungs


def print_ladder(rungs, pred, seg_ms, balance):
    print(f"ladder: {len(rungs)} rungs, clock {rungs[0].hz} Hz, lead {rungs[0].lead} rad, {rungs[0].cycles} cycles, "
          f"segments {seg_ms} ms.  Dry run: nothing was sent.")
    for r in rungs:
        R, _ = prediction_for(pred, r, seg_ms, balance)
        if R:
            b = R["band"]
            sides = "  ".join(f"{s} clr {fmt3(v['clear_mm'])} mm" for s, v in b["sides"].items())
            print(f"  rung {r.index:2d}  {r.kind:5s} M {r.M * 100:4.1f} cm   sim: up {b['up']}/{b['n']}  tilt pk {fmt3(b['tilt_peak_deg'])} deg  {sides}")
        else:
            print(f"  rung {r.index:2d}  {r.kind:5s} M {r.M * 100:4.1f} cm   sim: none for this plan")
    print("pick one: --rung N   (and --go to run it on the robot)")


def ladder_table(rungs, pred, seg_ms, balance="off"):
    """The README's per-rung table (markdown), from the predictions file: `sway_probe.py --table`."""
    L = ["| rung | kind | M cm | hip roll peak deg | sim: stays up | swing-foot clearance mm, L / R (median, band min-max) | CoM over stance sole mm (median) | peak tilt deg (median, max) | roll rate deg/s (median, max) | stop: tilt deg / gyro rad/s |",
         "|---|---|---|---|---|---|---|---|---|---|"]
    ref_roll = None
    for r in rungs:
        R, _ = prediction_for(pred, r, seg_ms, balance)
        roll = float(np.degrees(np.abs(r.q[:, [i for i, n in enumerate(r.names) if n.endswith("hip_roll")]]).max()))
        if not R:
            L.append(f"| {r.index} | {r.kind} | {r.M * 100:.1f} | {roll:.1f} | no sim | | | | | |")
            continue
        b = R["band"]
        cl = " / ".join(f"{b['sides'][x]['clear_mm']['med']:.0f} ({b['sides'][x]['clear_mm']['min']:.0f}-{b['sides'][x]['clear_mm']['max']:.0f})" for x in "LR")
        cm = " / ".join(f"{b['sides'][x]['com_margin_mm']['med']:.0f}" for x in "LR")
        L.append(f"| {r.index} | {r.kind} | {r.M * 100:.1f} | {roll:.1f} | {b['up']}/{b['n']} | {cl} | {cm} | {b['tilt_peak_deg']['med']:.1f}, {b['tilt_peak_deg']['max']:.1f} | "
                 f"{b['roll_rate_peak_dps']['med']:.0f}, {b['roll_rate_peak_dps']['max']:.0f} | {stops_for(R)['tilt_deg']:.0f} / {stops_for(R)['rate_rad_s']:.1f} |")
    return "\n".join(L)


def dry_run(rung, segs, stops, R, why, seg_ms, balance, cal_note):
    print(f"DRY RUN -- nothing is sent. rung {rung.label}: {rung.kind}, M {rung.M * 100:.1f} cm, lead {rung.lead} rad, "
          f"{rung.hz} Hz x {rung.cycles} cycles, ramp {rung.ramp_s} s in and out, {rung.t[-1]:.2f} s of keyframes")
    print(f"  playback: {len(segs)} chained `pose ... s{seg_ms}` commands" + (" with the bench roll PI (B<stance>)" if balance == "stance" else " (no balance loop)")
          + f", then `home`.  {cal_note}")
    peak = {nm: float(np.abs(np.degrees(rung.q[:, i])).max()) for i, nm in enumerate(rung.names)}
    big = ", ".join(f"{k} {v:.1f}" for k, v in sorted(peak.items(), key=lambda kv: -kv[1]) if v > 0.05)
    print(f"  joint peaks (deg, sim frame): {big}")
    print("  command sequence under --go: collector check, bench, torque, cal show, zero stand, upright check, "
          f"{len(segs)} x (pose + imu), home")
    for s in segs[:2] + segs[-1:]:
        print(f"    t {s.t_s:5.2f} s  {s.line()}")
    if R:
        print("  sim prediction for this rung:")
        for line in describe_prediction(R):
            print(line)
    else:
        print(f"  NO SIM PREDICTION for this plan ({why}): --go will refuse")
    print(f"  stop criteria: tilt > {stops['tilt_deg']} deg or |gyro| > {stops['rate_rad_s']} rad/s (the sender's, per segment); "
          "firmware: bench fall guard 30 deg / 100 ms (limp)")
    print("  to run it: --rung %d --go (flight collector up, spotter in place, robot standing at the zero pose)" % rung.index)


def main(argv=None, robot_factory=None, input_fn=input):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split("\n\n", 1)[1])
    ap.add_argument("--rung", type=int, help="the rung to show or run (1-based)")
    ap.add_argument("--go", action="store_true", help="run the rung ON THE ROBOT (needs --rung; one rung per run)")
    ap.add_argument("--host", default=os.environ.get("BIMO_HOST", "192.168.2.90"))
    ap.add_argument("--key-file", default=None)
    ap.add_argument("--config", default=None)
    ap.add_argument("--M", type=SR.parse_Ms, default=SR.LADDER_M)
    ap.add_argument("--lead", type=float, default=SR.LEAD)
    ap.add_argument("--hz", type=float, default=SR.CLOCK_HZ)
    ap.add_argument("--cycles", type=int, default=SR.CYCLES)
    ap.add_argument("--ramp", type=float, default=SR.RAMP_S)
    ap.add_argument("--seg-ms", type=int, default=SEG_MS)
    ap.add_argument("--balance", choices=("off", "stance"), default="off",
                    help="stance: ride the firmware's bench roll PI (`pose ... B<stance>`) on the foot that is not swinging")
    ap.add_argument("--predictions", default=DEFAULT_PREDICTIONS)
    ap.add_argument("--cal-file", default=None, help="a saved `cal show` text for the dry-run ticks (default: illustrative zeros)")
    ap.add_argument("--start-tilt", type=float, default=START_TILT_DEG)
    ap.add_argument("--accept-sim-falls", action="store_true",
                    help="allow --go on a rung where the sim band had plants that fell (otherwise refused)")
    ap.add_argument("--table", action="store_true", help="print the per-rung markdown table (the README's) and exit")
    ap.add_argument("--out", default=None, help="directory for the run log (default hw_sessions/<date>/sway_probe)")
    a = ap.parse_args(argv)

    ref, rungs = build(a)
    pred = load_predictions(a.predictions)
    if a.table:
        print(ladder_table(rungs, pred, a.seg_ms, a.balance))
        return 0
    if a.rung is None:
        if a.go:
            ap.error("--go needs --rung N: one rung per run")
        print_ladder(rungs, pred, a.seg_ms, a.balance)
        return 0
    if not 1 <= a.rung <= len(rungs):
        ap.error(f"--rung {a.rung}: the ladder has rungs 1..{len(rungs)}")
    rung = rungs[a.rung - 1]
    R, why = prediction_for(pred, rung, a.seg_ms, a.balance)
    stops = stops_for(R)

    if not a.go:
        if a.cal_file:
            cal, note = bus_cal.parse_cal_show(open(a.cal_file).read()), f"ticks from {a.cal_file}"
        else:
            cal, note = illustrative_cal(), "ticks use ILLUSTRATIVE zeros (the real run reads `cal show` from the robot)"
        segs = plan(rung, cal, a.seg_ms, a.balance)
        dry_run(rung, segs, stops, R, why, a.seg_ms, a.balance, note)
        return 0

    # ---- --go: everything below talks to the robot ----
    if R is None:
        print(f"refusing to run: no sim prediction for this plan ({why}) -- run sway_sim.py for it", file=sys.stderr)
        return 2
    if R["band"]["up"] < R["band"]["n"] and not a.accept_sim_falls:
        print(f"refusing to run: in the sim band only {R['band']['up']}/{R['band']['n']} plants stay up on this rung "
              f"(open loop is not safe across the band); --accept-sim-falls to run it anyway", file=sys.stderr)
        return 2
    typed = input_fn(f"About to move the robot at {a.host}: rung {rung.label}. Spotter in place, torque on, standing at zero? Type GO {rung.index}: ")
    if typed.strip() != f"GO {rung.index}":
        print("not confirmed -- nothing sent")
        return 2
    outdir = a.out or os.path.join(main_checkout(), "hw_sessions", datetime.date.today().isoformat(), "sway_probe")
    os.makedirs(outdir, exist_ok=True)
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    logf = open(os.path.join(outdir, f"{stamp}_rung{rung.index}.log"), "w")

    def log(s):
        print(s, flush=True)
        logf.write(s + "\n")
        logf.flush()
    robot = (robot_factory or (lambda: Robot(a.host, a.key_file)))()
    cal = bus_cal.parse_cal_show(robot.run("cal show", 5.0))
    check_cal(cal, rung.names)
    segs = plan(rung, cal, a.seg_ms, a.balance)
    status, why = run_rung(robot, rung, cal, segs, stops, log, local_ip_toward(a.host) if robot_factory is None else "127.0.0.1",
                           a.seg_ms, a.start_tilt)
    log(f"RESULT {status} {why}")
    return 0 if status == "done" else 3


if __name__ == "__main__":
    sys.exit(main())
