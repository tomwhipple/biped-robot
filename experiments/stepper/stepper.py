#!/usr/bin/env python3
"""A self-correcting quasi-static stepper for the v6 legs, run from the host over the Wi-Fi CLI.

The gait is not clocked. Each phase ends when the robot's own IMU says it has happened:
    crouch -> shift over the stance foot (in small monitored moves) -> raise the other foot a few mm at
    a time -> (move it) -> lower it -> next.

What the 2026-11-11 runs showed, and what this does about it. The roll servos have several degrees of
play the encoders cannot see. Shorten one leg and the pelvis sags after that foot: the body rolls
toward it exactly as fast as the foot is "raised", and the foot stays on the floor until the play is
used up. Shifting the pelvis further out does not stop the sag (81 -> 93 mm changed it by 0.4 deg); it
only puts the weight near the stance foot's outer edge, and the robot then falls outward.
So the lift here:
  * trims the STANCE hip roll against the measured tilt (the opposite hip takes up the sag), so the
    body comes back level instead of hanging toward the lifted foot;
  * estimates where the swing sole's outer edge is from the commanded lift and the measured tilt:
    P(foot clear) is a Nool, there is no contact sensor;
  * keeps the pelvis shift near the model's (weight over the stance sole) and corrects it from what
    happened: never clear -> further over; tipping outward -> back;
  * remembers per stance side the shift, lift and hip trim that worked (state file).
A real fall is detected (IMU, and the firmware's own latch) and ends the run with nothing more sent
until the robot is upright again, when it is homed.

    stepper.py lift --side L            raise the LEFT foot once (right stance), free-standing
    stepper.py march --steps 4          alternate lifts in place
    stepper.py walk --steps 4 --dx 0.02 forward steps of dx m

Run records: hw_sessions/<date>/stepper/<stamp>_<name>.jsonl (+ camera clips with --video).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
from nool import foot_clear, settled, tipping  # noqa: E402

STATE = os.path.join(HERE, "state.json")
LEARNED = ("k_shift", "h_up", "hike")
DEFAULTS = dict(
    drop=0.02,            # crouch depth, m (a ~50 deg knee bend)
    k_shift={"L": 1.1, "R": 1.1},     # x the model's shift, per STANCE side; adapted on the robot
    k_step=0.1, k_min=0.8, k_max=1.5,
    shift_step=0.02,      # the pelvis crosses in moves of this size, IMU read between, m
    shift_lean_lim=8.0,   # a lean past this (deg) during a shift ends the run
    fwd_w=0.75,           # the pelvis rides this far from the swing foot toward the stance foot, fore-aft
    dh=0.005,             # lift increment, m
    h_max=0.050,          # most the foot is commanded up, m
    need=0.004,           # clearance wanted at the outer edge, m
    g_hike=0.7,           # share of the measured tilt taken out by the stance hip at each increment
    hike_max=12.0,        # most the stance hip roll is trimmed, deg
    kb=2.0,               # in the air: mm more shift per degree it hangs toward the lifted foot, each cycle
    t_confirm=0.7,        # a foot judged up must still be up after this long, s
    ff=0.5,               # start a lift at this share of the lift and trim that side took last time
    h_up={"L": 0.0, "R": 0.0},        # learned: the lift (m) at which the foot was judged clear, per STANCE side
    hike={"L": 0.0, "R": 0.0},        # learned: the stance hip trim (deg) by then
    p_clear=0.9, p_tip=0.5,
    tilt_lim=12.0, out_lim=3.0, pitch_lim=6.0, rate_lim=30.0,
    fall_deg=35.0,
    t_crouch=2.5, t_shift=2.5, t_inc=0.35, t_swing=1.0, t_lower=0.3,
    lower_step=0.01, settle_s=0.4,
)


def load_state():
    s = json.loads(json.dumps(DEFAULTS))
    if os.path.exists(STATE):
        s.update({k: v for k, v in json.load(open(STATE)).items() if k in LEARNED})
    return s


class Abort(RuntimeError):
    """The run cannot go on, but the robot is on its feet: centre it, stand it up, let go."""


class Run:
    def __init__(self, name, robot, kin, prm, video=False, quiet=False):
        from robot_io import Fallen
        self.Fallen = Fallen
        self.r, self.k, self.p, self.quiet = robot, kin, prm, quiet
        d = os.path.join("/home/claw/code/robot", "hw_sessions", time.strftime("%Y-%m-%d"), "stepper")   # the main checkout
        os.makedirs(d, exist_ok=True)
        self.base = os.path.join(d, time.strftime("%H%M%S") + "_" + name)
        self.f = open(self.base + ".jsonl", "w")
        self.t0 = time.monotonic()
        self.off = {"L": self.k.model_offset("L"), "R": self.k.model_offset("R")}
        self.feet = {"L": [0.0, 0.0, 0.0], "R": [0.0, 0.0, 0.0]}     # each sole's offset from its stand
        self.pel = [0.0, 0.0]                                         # pelvis offset (fwd, shift)
        self.hike = {"L": 0.0, "R": 0.0}                              # hip-roll trim, deg, of the leg it stands on
        self.ref0 = None                                              # the level crouch
        self.ref = None                                               # after the last shift
        self.tilt_s = 0.0                                             # its lean toward the swing foot then
        self.fell = False
        self.video = []
        if video:
            env = dict(os.environ, INPUT_FORMAT="mjpeg", SIZE="1920x1080")
            self.video.append(subprocess.Popen(["tools/cam_record.sh", "/dev/video0", self.base + "_emeet", str(video), "emeet"],
                                               cwd="/home/claw/code/robot", env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
            self.video.append(subprocess.Popen(["tools/sony_record.sh", self.base + "_sony", str(video), "sony"],
                                               cwd="/home/claw/code/robot", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
            time.sleep(2.5)
        self.log("start", prm=prm, cal=robot.cal, off=self.off)

    # -- records ----------------------------------------------------------
    def log(self, ev, **kw):
        row = dict(t=round(time.monotonic() - self.t0, 3), ev=ev, **kw)
        self.f.write(json.dumps(row) + "\n"); self.f.flush()
        return row

    def say(self, *a):
        if not self.quiet:
            print(f"[{time.monotonic() - self.t0:6.1f}]", *a, flush=True)

    # -- motion -----------------------------------------------------------
    def go(self, ms, tag):
        try:
            q = self.k.pose(shift=self.pel[1], fwd=self.pel[0], footL=self.feet["L"], footR=self.feet["R"],
                            drop=self.p["drop"], hike={k: math.radians(v) for k, v in self.hike.items()})
        except ValueError as e:
            raise Abort(f"{tag}: {e}")
        self.r.pose(q, ms)
        self.log("pose", tag=tag, ms=ms, pel=list(self.pel), hike=dict(self.hike), feet=json.loads(json.dumps(self.feet)))

    def read(self, tag=""):
        """Two IMU reads. One wild sample between two that agree is a torn read; a large angle that is
        still there on the next read is a fall."""
        lim = self.p["fall_deg"]
        a = self.r.imu(); b = self.r.imu()
        for _ in range(2):
            if abs(b["roll"] - a["roll"]) < 12 and abs(b["pitch"] - a["pitch"]) < 12:
                break
            self.log("imu_jump", a=[a["roll"], a["pitch"]], b=[b["roll"], b["pitch"]])
            a = b; b = self.r.imu()
        if abs(b["roll"]) > lim or abs(b["pitch"]) > lim:
            c = self.r.imu()
            self.log("imu_big", b=[b["roll"], b["pitch"]], c=[c["roll"], c["pitch"]])
            if abs(c["roll"]) > lim or abs(c["pitch"]) > lim or self.r.fallen():
                raise self.Fallen(f"IMU roll {c['roll']:+.0f} pitch {c['pitch']:+.0f} at {tag!r}")
            a, b = b, c
        b["rate"] = math.degrees(math.hypot(b["gyro"][0], b["gyro"][1]))      # horizontal gyro, deg/s
        self.log("imu", tag=tag, roll=round(b["roll"], 2), pitch=round(b["pitch"], 2), rate=round(b["rate"], 1),
                 gyro=[round(x, 3) for x in b["gyro"]])
        return b

    def settle(self, tag, tmax=2.5):
        t = time.monotonic()
        while True:
            s = self.read(tag)
            if settled(s["rate"]).p > 0.7 or time.monotonic() - t > tmax:
                return s

    def lean(self, s):
        """(roll, pitch) in deg away from the level crouch."""
        return s["roll"] - self.ref0["roll"], s["pitch"] - self.ref0["pitch"]

    def tilt_to(self, swing, s):
        d = s["roll"] - self.ref0["roll"]              # roll + = leaning to the robot's RIGHT
        return d if swing == "R" else -d

    # -- the phases -------------------------------------------------------
    def stand_up(self):
        self.r.torque()
        self.r.pose(self.k.pose(straight=True), 2500)
        self.say("stand"); self.settle("stand")
        self.go(int(1e3 * self.p["t_crouch"]), "crouch")
        time.sleep(self.p["settle_s"])
        s = self.ref0 = self.settle("crouch")
        self.say(f"crouch  roll {s['roll']:+.2f} pitch {s['pitch']:+.2f}  (the level reference)")
        return s

    def stance_pel(self, stance):
        P = self.p
        swing = "R" if stance == "L" else "L"
        ox, oy = self.off[stance]
        fwd = P["fwd_w"] * self.feet[stance][0] + (1 - P["fwd_w"]) * self.feet[swing][0] + ox
        return [fwd, P["k_shift"][stance] * oy]

    def move_pel(self, to, tag):
        """Move the pelvis to `to` in monitored moves, taking any hip trim out on the way. Both feet are down."""
        P = self.p
        a, k0 = list(self.pel), dict(self.hike)
        dist = math.hypot(to[0] - a[0], to[1] - a[1])
        n = max(1, math.ceil(dist / P["shift_step"]), math.ceil(max(k0.values()) / 3.0))
        ms = int(1e3 * max(0.25, P["t_shift"] * (dist / n) / 0.08))
        s = None
        for i in range(1, n + 1):
            u = i / n
            self.pel = [a[0] + u * (to[0] - a[0]), a[1] + u * (to[1] - a[1])]
            self.hike = {k: v * (1 - u) for k, v in k0.items()}
            self.go(ms, f"{tag} {i}/{n}")
            s = self.read(tag)
            dr, dp = self.lean(s)
            if abs(dr) > P["shift_lean_lim"] or abs(dp) > P["shift_lean_lim"]:
                self.say(f"  leaning in {tag}: roll {dr:+.1f} pitch {dp:+.1f} at shift {1e3 * self.pel[1]:+.0f} mm")
                raise Abort(f"lean {dr:+.1f}/{dp:+.1f} deg during {tag}")
        return s

    def shift_over(self, stance):
        to = self.stance_pel(stance)
        self.move_pel(to, f"shift {stance}")
        time.sleep(self.p["settle_s"])
        s = self.ref = self.settle(f"over {stance}")
        dr, dp = self.lean(s)
        self.say(f"over {stance} x{self.p['k_shift'][stance]:.2f} ({1e3 * to[1]:+.0f} mm, fwd {1e3 * to[0]:+.0f})  lean {dr:+.2f} pitch {dp:+.2f}")
        return s

    def raise_foot(self, swing):
        """Raise `swing` a little at a time, trimming the stance hip against the tilt as it goes.
        Returns (how, h): how = "up" | "fell_in" | "tipping_out" | "tipping" | "max"."""
        P = self.p
        stance = "L" if swing == "R" else "R"
        base = self.feet[swing][2]
        tilt_s = self.tilt_s = self.tilt_to(swing, self.ref)    # the lean it stood at after the shift, both feet down
        pitch_s = self.lean(self.ref)[1]
        h = 0.0
        h_next = max(P["dh"], P["ff"] * P["h_up"][stance])      # the first move: half of what it took last time
        hike = P["ff"] * P["hike"][stance]
        first = True
        while h < P["h_max"] - 1e-9:
            dh = min(h_next, P["h_max"]) - h
            h += dh
            self.feet[swing][2] = base + h
            self.hike[stance] = hike
            self.go(int(1e3 * (P["t_inc"] if not first else max(P["t_inc"], P["t_inc"] * dh / P["dh"] * 0.6))), f"raise {swing} {1e3 * h:.0f}")
            first = False
            s = self.read(f"raise {swing}")
            tilt = self.tilt_to(swing, s) - tilt_s             # toward the lifted foot, since both feet were down
            clr = foot_clear(h, tilt, need=P["need"])
            tip = tipping(tilt, self.lean(s)[1] - pitch_s, s["rate"], P["tilt_lim"], P["pitch_lim"],
                          P["rate_lim"], out_lim=P["out_lim"])
            self.log("nool", swing=swing, h=h, hike=round(hike, 2), tilt=round(tilt, 2), clear=clr.row(), tip=tip.row())
            self.say(f"  raise {swing} {1e3 * h:4.0f} mm  hip trim {hike:5.2f}  tilt {tilt:+5.2f}  "
                     f"pitch {self.lean(s)[1] - pitch_s:+5.2f}  rate {s['rate']:4.1f}  clear~{clr.why['clear_mm']:+5.1f} mm "
                     f"P(clear) {clr.p:.2f}  P(tip) {tip.p:.2f}")
            if tip.p > P["p_tip"]:
                return ("tipping_out" if tilt < -0.5 * P["out_lim"] else "tipping"), h
            hike = max(0.0, min(P["hike_max"], hike + P["g_hike"] * tilt))      # the stance hip takes up the sag
            if clr.p >= P["p_clear"]:
                self.hike[stance] = hike
                self.go(int(1e3 * P["t_inc"]), f"level {swing}")
                return self.balance(swing, P["t_confirm"]), h     # up for a moment is not up: it must stay
            h_next = h + P["dh"]
        return "max", h

    def balance(self, swing, secs):
        """`swing` is in the air: watch it for `secs`, and while watching keep the body level (stance hip)
        and the weight over the stance foot (pelvis shift follows the tilt: hanging toward the lifted foot
        means the weight is inboard of the stance foot). Returns "up", or "fell_in" if the foot is back on
        the floor (2026-11-11 10:37: up at 20 mm, then a slow topple onto it at 68 mm of shift), or
        "tipping_out"."""
        P = self.p
        stance = "L" if swing == "R" else "R"
        oy = self.off[stance][1]
        h = self.feet[swing][2]
        pitch_s = self.lean(self.ref)[1]
        t = time.monotonic()
        while True:
            s = self.read(f"air {swing}")
            tilt = self.tilt_to(swing, s) - self.tilt_s
            clr = foot_clear(h, tilt, need=P["need"])
            tip = tipping(tilt, self.lean(s)[1] - pitch_s, s["rate"], P["tilt_lim"], P["pitch_lim"], P["rate_lim"],
                          out_lim=P["out_lim"])
            self.log("air", swing=swing, tilt=round(tilt, 2), shift=round(1e3 * self.pel[1], 1), hike=round(self.hike[stance], 2),
                     clear=clr.row(), tip=tip.row())
            if tip.p > P["p_tip"] and tilt < 0:
                return "tipping_out"
            if clr.p < 0.3:
                self.say(f"  fell back onto {swing}: tilt {tilt:+.1f} at shift {1e3 * self.pel[1]:+.0f} mm")
                return "fell_in"
            if time.monotonic() - t >= secs:
                return "up"
            if abs(tilt) > 0.7:
                y = abs(self.pel[1]) + 1e-3 * P["kb"] * tilt
                y = max(P["k_min"] * abs(oy), min(P["k_max"] * abs(oy), y))
                self.pel[1] = math.copysign(y, oy)
                self.hike[stance] = max(0.0, min(P["hike_max"], self.hike[stance] + P["g_hike"] * tilt))
                self.go(250, f"balance {swing}")

    def lower_foot(self, swing, fast=False):
        """Put `swing` down in monitored moves (the hip trim stays until the next shift takes it out).
        fast: it is going over outward -- foot down and pelvis back toward the middle in one move."""
        P = self.p
        h0 = self.feet[swing][2]
        n = 1 if fast else max(1, math.ceil(h0 / P["lower_step"]))
        if fast:
            self.pel[1] *= 0.7
            self.hike = {"L": 0.0, "R": 0.0}
        for i in range(1, n + 1):
            self.feet[swing][2] = h0 * (1 - i / n)
            self.go(int(1e3 * (0.25 if fast else P["t_lower"])), f"lower {swing} {1e3 * self.feet[swing][2]:.0f}")
            self.read(f"lower {swing}")
        time.sleep(P["settle_s"])
        s = self.settle(f"down {swing}")
        dr, dp = self.lean(s)
        self.say(f"  down {swing}  lean {dr:+.2f} pitch {dp:+.2f}")
        return s

    def lift(self, swing, hold=1.5, dx=0.0):
        """One self-correcting lift of `swing` (and a forward move of dx m while it is up)."""
        stance = "L" if swing == "R" else "R"
        P = self.p
        for attempt in range(4):
            self.shift_over(stance)
            how, h = self.raise_foot(swing)
            hike = self.hike[stance]
            self.log("lift", swing=swing, attempt=attempt, how=how, h=h, hike=round(hike, 2), k=P["k_shift"][stance])
            if how == "up":
                if dx:
                    self.feet[swing][0] += dx
                    self.go(int(1e3 * P["t_swing"]), f"swing {swing} {1e3 * dx:+.0f}")
                how = self.balance(swing, hold)
            if how == "up":
                hike = self.hike[stance]
                k = abs(self.pel[1] / self.off[stance][1])              # remember what this side took
                self.say(f"  up: {swing} at {1e3 * h:.0f} mm, hip trim {hike:.1f}, shift x{k:.2f}")
                a = 0.5 if P["h_up"][stance] else 1.0
                P["h_up"][stance] = round((1 - a) * P["h_up"][stance] + a * h, 4)
                P["hike"][stance] = round((1 - a) * P["hike"][stance] + a * hike, 2)
                P["k_shift"][stance] = round(max(P["k_min"], min(P["k_max"], k)), 3)
                self.lower_foot(swing)
                return how, h
            # not up: put it down and correct the shift for this stance side
            self.lower_foot(swing, fast=(how == "tipping_out"))
            dk = -P["k_step"] if how == "tipping_out" else (P["k_step"] if how in ("max", "fell_in") else 0.0)
            P["k_shift"][stance] = round(max(P["k_min"], min(P["k_max"], P["k_shift"][stance] + dk)), 3)
            self.say(f"  {how} at {1e3 * h:.0f} mm (hip trim {hike:.1f}) -> shift over {stance} now x{P['k_shift'][stance]:.2f}")
        return "gave_up", 0.0

    def centre(self):
        mid = 0.5 * (self.feet["L"][0] + self.feet["R"][0])
        return self.move_pel([mid, 0.0], "centre")

    def finish(self, save=True):
        try:
            if not self.fell:
                try:
                    self.feet["L"][2] = self.feet["R"][2] = 0.0
                    self.centre()
                    split = abs(self.feet["L"][0] - self.feet["R"][0]) > 1e-3        # straight legs cannot span a stride
                    self.r.pose(self.k.pose(straight=not split, drop=0.008 if split else None, fwd=self.pel[0],
                                            footL=(self.feet["L"][0], 0, 0), footR=(self.feet["R"][0], 0, 0)), 2500)
                    time.sleep(0.5)
                except self.Fallen as e:
                    self.fallen(e)
                except Abort as e:
                    self.log("error", err=repr(e)); print("!! ", repr(e), flush=True)
                if not self.fell:
                    self.r.release()
        finally:
            self.log("end", **{k: self.p[k] for k in LEARNED})
            if save:
                json.dump({k: self.p[k] for k in LEARNED}, open(STATE, "w"))
            for v in self.video:
                v.wait()
            self.f.close()

    def fallen(self, e):
        """It is down. Send nothing that moves; the firmware is limp and homes itself once picked up."""
        self.fell = True
        self.log("fallen", err=str(e), pel=list(self.pel), hike=self.hike, feet=self.feet)
        print(f"!! FELL: {e}\n!! nothing more is sent. Pick it up: it is homed once upright and still for 1 s.", flush=True)
        ok = self.r.wait_recovered()
        self.log("recovered", ok=ok)
        print("!! picked up and homed" if ok else "!! still down after 5 min", flush=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=("lift", "march", "walk"))
    ap.add_argument("--side", choices=("L", "R"), default="L", help="the foot that comes up (first)")
    ap.add_argument("--steps", type=int, default=2)
    ap.add_argument("--dx", type=float, default=0.02)
    ap.add_argument("--hold", type=float, default=1.5)
    ap.add_argument("--video", type=int, default=0, metavar="SECS", help="record both cameras for SECS")
    ap.add_argument("--set", nargs="*", default=[], metavar="KEY=VAL")
    ap.add_argument("--host", default="192.168.2.90")
    a = ap.parse_args(argv)
    prm = load_state()
    for kv in a.set:
        k, v = kv.split("=", 1)
        prm[k] = json.loads(v)

    from kin import Kin
    from robot_io import Fallen, Robot
    kin = Kin(prm["drop"])
    rob = Robot(a.host)
    if rob.fallen():
        print("the firmware says FALLEN: pick the robot up and let it home first"); rob.close(); return 2
    run = Run(a.what, rob, kin, prm, video=a.video)
    out = []
    try:
        run.stand_up()
        if a.what == "lift":
            out.append((a.side,) + run.lift(a.side, hold=a.hold))
        else:
            side = a.side
            for i in range(a.steps):
                out.append((side,) + run.lift(side, hold=a.hold if a.what == "march" else 0.2,
                                              dx=a.dx * (1 if i == 0 else 2) if a.what == "walk" else 0.0))
                if out[-1][1] == "gave_up":
                    break
                side = "R" if side == "L" else "L"
    except Fallen as e:
        run.fallen(e)
    except Exception as e:                                           # any other fault: say it, then stand and let go
        run.log("error", err=repr(e)); print("!! ", repr(e), flush=True)
    finally:
        run.finish()
        rob.close()
    print("RESULT", out, {k: prm[k] for k in LEARNED}, "fell" if run.fell else "", "log", run.base + ".jsonl")
    return 1 if run.fell else 0


if __name__ == "__main__":
    sys.exit(main())
