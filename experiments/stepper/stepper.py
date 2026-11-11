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
until the firmware has homed the robot (picked up: within 25 deg of upright, |gyro| < 1.0 rad/s, for 1 s).

    stepper.py lift --side L            raise the LEFT foot once (right stance), free-standing
    stepper.py march --steps 4          alternate lifts in place
    stepper.py walk --steps 4 --dx 0.02 forward steps of dx m
    stepper.py walk --dx 0 --dy 0.015   a sidestep: dy m to the robot's left per step pair (- = right)
    stepper.py walk --board             look at the front camera before every step: steer back toward the
                                        middle of the board, stop short of its edge (needs the Sony; it
                                        records, 180 s unless --video says otherwise; also on `march`)

    stepper.py walk --dx -0.02          backward steps
    stepper.py shuttle --steps 4 --cycles 3 --dx 0.02 [--dx-list 0.02,0.03] [--dy D] [--board]
                                        N steps forward, N back (undoing them, so the feet end where they
                                        began), M times in one run; --dx-list: cycle k uses the k-th dx
    stepper.py playsweep --side L       lift that foot, then in the air sweep the stance hip-roll trim, then
                                        the stance ankle-roll trim, +-sweep_deg around where it is (`sweep` rows)
    stepper.py shiftsweep               both feet down: the pelvis shift 0 -> +62 -> -62 -> 0 mm x sweep_k
    --pace X                            every phase time x X (walk --pace 0.6 = the same walk in 60 % of the time)
Every guard_every steps the eight load-bearing servos are read (`servos` rows); a servo over temp_max_c or a bus
under volt_min ends the run cleanly. The log's `start` and `lift` rows carry utc (wall clock).

    stepper.py walk --dx 0 --dyaw 7.5   turn on the spot: 7.5 deg per step pair to the robot's left (- = right); the soles
                                        are placed turned by their share (dyaw/2), the pelvis heading follows in the next
                                        shift, the IMU heading (fused yaw less drift) corrects the next swing by yaw_gain
                                        of its error; --board ends a run whose heading passes yaw_board_max (25 deg),
                                        the lateral loop being in board coordinates and the board no longer square
The link: a dropped command is retried after a reconnect (link_retries); if the stand-down cannot be sent even on
a fresh connection the run says the robot is left with torque on in its last pose.

With --board the look is the median of 3 camera frames (low-confidence ones dropped), on ONE scale for the
run: the board is board_w_mm (450) wide and a sole's position is its fraction across it. The first look fixes
each sole's lateral target on the board (the starting stance about the starting line; --board-centre steers
that line toward the board's middle, --dy moves it) and sep0, the measured separation that goes with the IK's
zero offsets. A swing foot lands where it is sent, relative to the stance foot, plus a systematic miss (both
feet land to the robot's right, 15-30 mm). So every step aims relative to the MEASURED stance foot, and
comp[side] (state file, learned from each landing's error, lat_gain per landing) is added to the command; the
`anchor` and `landing` rows show the real offset against the commanded one and each learned miss. Without
--board the saved comp is still applied, blind. A fall is waited out (the firmware homes the robot once it is
picked up); 45 s after it, notify_cmd asks Aime to ping the owner once (--no-notify turns that off).

A sidestep never narrows the stance: moving left, the LEFT foot goes out first and the right follows;
moving right, the right foot first. The board's edge distances come from board_view.py.

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
TIMES = ("t_shift", "t_inc", "t_swing", "t_lower", "settle_s", "t_confirm", "t_bal", "t_min", "t_sweep")
GUARD = ("R_hip_roll", "L_hip_roll", "R_knee", "L_knee", "R_ankle_roll", "L_ankle_roll", "R_hip_pitch", "L_hip_pitch")
LEARNED = ("k_shift", "h_up", "hike", "comp")
DEFAULTS = dict(
    drop=0.02,            # crouch depth, m (a ~50 deg knee bend)
    k_shift={"L": 1.1, "R": 1.1},     # x the model's shift, per STANCE side; adapted on the robot
    k_step=0.1, k_min=0.8, k_max=1.4, k_floor=1.0,
    shift_step=0.03,      # the pelvis crosses in moves of this size, IMU read between, m
    shift_out_lim=2.5,    # shifting over a stance foot: a lean TOWARD it this far (deg, from the start of the shift) = tipping out
    shift_back_mm=20.0,   # ... then the pelvis steps back toward the middle by this much, mm, and k_shift of that side drops a step
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
    tilt_lim=8.0, out_lim=3.0, pitch_lim=6.0, rate_lim=30.0,
    fall_deg=35.0,
    t_crouch=2.5, t_shift=2.5, t_inc=0.35, t_swing=1.0, t_lower=0.3,
    lower_step=0.01, settle_s=0.4,
    t_verdict=0.3,        # a foot-up verdict waits at least this long after the last move, s (not scaled by --pace)
    pose_margin=0.06,     # wait this long after a pose's own time before the next command, s
    link_retries=3,       # a dropped link: reconnect and resend this many times
    yaw_gain=0.5,         # share of the heading error (commanded - measured, deg) added to the next swing's yaw
    yaw_dead=0.5,         # ... below this error (deg) nothing is added
    yaw_step_max=8.0,     # most one swing foot is turned, deg
    yaw_board_max=25.0,   # --board: the run ends if the heading passes this, deg (the lateral loop assumes a square board)
    t_bal=0.25,           # the balance move in the air, s
    t_min=0.25,           # shortest monitored pelvis move / fast lower, s
    t_sweep=0.25,         # one step of a data sweep, s
    guard_every=4,        # read the load-bearing servos every this many steps
    temp_max_c=65.0,      # a servo hotter than this ends the run, deg C
    volt_min=10.5,        # a bus voltage below this ends the run, V
    sweep_deg=4.0,        # playsweep: trim swept +- this, deg, in steps of 0.5
    sweep_cycles=2,       # playsweep/shiftsweep: triangles
    sweep_k=1.0,          # shiftsweep: the pelvis shift swept +- this x 62 mm, in steps of 6 mm
    board_min_mm=25.0,    # stop the walk when a sole's outer edge is this close to the board's edge, mm
    board_ahead_px=60.0,  # ... or when the feet are this near the bottom of the picture (board runs out), px
    board_dead_mm=15.0,   # no steering while the room left and right differs by less than 2x this, mm
    board_gain=0.5,       # share of the half-difference of the margins corrected per step
    board_step_mm=15.0,   # most a single steering correction moves the lateral target, mm
    board_conf_min=0.5,   # a board_view result below this confidence is ignored
    sep_nominal_mm=121.0, # centre-to-centre separation of the soles in the crouch, mm
    board_w_mm=450.0,     # the board's width, mm: the one scale of a run (board_view)
    board_centre=False,   # steer the starting line toward the board's middle (default: hold it)
    lat_gain=0.5,         # share of a sole's landing error taken into that foot's learned miss comp, each landing
    lat_step_max=0.04,    # most a swing foot's commanded offset from the stance foot changes between two of its swings, m
    comp_lim=0.05,        # most a learned landing miss may be, m
    comp={"L": 0.0, "R": 0.0},   # learned: lateral miss of each SWING foot (m, + = it lands left of where it is sent); sent opposite
    sep_max_mm=131.0,     # the stance the targets keep: the separation the robot was put down with, clipped to this ...
    sep_min_mm=115.0,     # ... and from below to this, mm (a robot left standing wide is walked back toward nominal)
    min_sep_cmd=96.0,     # never command the soles closer than this (IK geometry), mm: nominal less 25
    notify_after_s=45.0,  # still fallen this long after a fall: run notify_cmd, once
    fall_wait_s=1200.0,   # the most it waits for the robot to be picked up and homed, s
    notify_cmd=["/home/claw/.hermes/scripts/ask-aime.sh",
                "ROBOT PING: the biped fell over at {hhmm} and has not been picked up. It is limp and safe; when you stand it "
                "upright it homes itself after 1 s. (Tom asked for this ping: please DM him this text on Mattermost and "
                "reply 'sent'.)"],     # {hhmm} = local time of the fall; [] = never notify
)


def apply_pace(prm, pace):
    """Every phase time x pace (t_crouch, the slow first stand, excluded)."""
    for k in TIMES:
        prm[k] = prm[k] * pace
    prm["pace"] = pace
    return prm


def load_state():
    s = json.loads(json.dumps(DEFAULTS))
    if os.path.exists(STATE):
        s.update({k: v for k, v in json.load(open(STATE)).items() if k in LEARNED})
    return s


class Abort(RuntimeError):
    """The run cannot go on, but the robot is on its feet: centre it, stand it up, let go."""


class Run:
    def __init__(self, name, robot, kin, prm, video=False, quiet=False, board=False, log_dir=None):
        from robot_io import Fallen, LinkDown
        self.Fallen, self.LinkDown = Fallen, LinkDown
        self.r, self.k, self.p, self.quiet = robot, kin, prm, quiet
        d = log_dir or os.path.join("/home/claw/code/robot", "hw_sessions", time.strftime("%Y-%m-%d"), "stepper")   # the main checkout
        os.makedirs(d, exist_ok=True)
        self.base = os.path.join(d, time.strftime("%H%M%S") + "_" + name)
        self.f = open(self.base + ".jsonl", "w")
        self.t0 = time.monotonic()
        self.off = {"L": self.k.model_offset("L"), "R": self.k.model_offset("R")}
        self.feet = {"L": [0.0, 0.0, 0.0], "R": [0.0, 0.0, 0.0]}     # each sole's offset from its stand
        self.last_off = {"L": 0.0, "R": 0.0}  # the offset from the stance foot each foot was last sent to (m)
        self.rel = {"L": 0.0, "R": 0.0}       # the IK offset (m) each foot was last aimed at, from the stance foot, less comp
        self.meas = {"L": 0.0, "R": 0.0}      # the soles' positions at the last good look (board mm)
        self.fresh = False                    # is `meas` from this step's look?
        self.bt = None                        # board targets, set by the first look: half, centre, sep0 (mm)
        self.t_land = {"L": None, "R": None}  # the board target a foot was aimed at, until its landing is measured
        self.pel = [0.0, 0.0]                                         # pelvis offset (fwd, shift)
        self.hike = {"L": 0.0, "R": 0.0}                              # hip-roll trim, deg, of the leg it stands on
        self.ankle = {"L": 0.0, "R": 0.0}                             # ankle-roll trim, deg (data sweeps only)
        self.ref0 = None                                              # the level crouch
        self.ref = None                                               # after the last shift
        self.tilt_s = 0.0                                             # its lean toward the swing foot then
        self.fell = False
        self.psi_ref = 0.0                    # rad: the frame the bookkeeping below is in (body-aligned, + = CCW)
        self.psi_p = 0.0                      # rad: the pelvis heading now
        self.yawf = {"L": 0.0, "R": 0.0}      # rad: the soles' world yaws (relative to that same zero)
        self.corr = {"L": 0.0, "R": 0.0}      # deg of each sole's yaw that are heading corrections, not the commanded turn
        self.corr_p = 0.0                     # ... and of the pelvis heading
        self.yaw_hist = []                    # (t, fused yaw deg) of recent IMU reads
        self.last_imu = None
        self.dyaw = 0.0                       # deg per step pair asked for by walk/shuttle
        self.look_after = False               # take the board look right after a landing (in place of a settle wait)
        self.cached_look = None
        self.has_cached = False
        self.last_tag = None
        self.video = []
        self.board = board
        self.latest = self.base + "_latest.jpg"                       # the newest front-camera frame (board mode)
        self.n_look = 0
        self.stop_why = None
        self.out = []
        self.ahead = None
        self.shift_tipped = False
        self.n_push = 0                       # how often balance has moved the pelvis
        self.k_off = None                     # the shift (x model) at which the foot was first judged clear
        self.n_steps = 0
        self.misses = 0
        self.t_fall = ""
        if video:
            env = dict(os.environ, INPUT_FORMAT="mjpeg", SIZE="1920x1080")
            self.video.append(subprocess.Popen(["tools/cam_record.sh", "/dev/video0", self.base + "_emeet", str(video), "emeet"],
                                               cwd="/home/claw/code/robot", env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
            if board:       # this checkout's recorder: it keeps LATEST as the newest frame
                self.video.append(subprocess.Popen([os.path.join(ROOT, "tools", "sony_record.sh"), self.base + "_sony", str(video), "sony"],
                                                   cwd=ROOT, env=dict(os.environ, LATEST=self.latest),
                                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
            else:
                self.video.append(subprocess.Popen(["tools/sony_record.sh", self.base + "_sony", str(video), "sony"],
                                                   cwd="/home/claw/code/robot", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
            time.sleep(2.5)
        if hasattr(robot, "on_link"):
            robot.on_link = self.link_event
        self.log("start", prm=prm, cal=robot.cal, off=self.off, utc=time.time(), pace=prm.get("pace", 1.0))

    # -- records ----------------------------------------------------------
    def log(self, ev, **kw):
        row = dict(t=round(time.monotonic() - self.t0, 3), ev=ev, **kw)
        self.f.write(json.dumps(row) + "\n"); self.f.flush()
        return row

    def say(self, *a):
        if not self.quiet:
            print(f"[{time.monotonic() - self.t0:6.1f}]", *a, flush=True)

    # -- motion -----------------------------------------------------------
    # self.feet[side][1] is where the foot is COMMANDED to stand (IK frame). Where it really lands is measured
    # on the board (look) and handled by the outer loop in board coordinates (outer / _step), not here.
    def fy(self, side):
        return self.feet[side][1]

    def cmd(self, side):
        return (self.feet[side][0], self.fy(side), self.feet[side][2])

    def go(self, ms, tag):
        try:
            q = self.k.pose(shift=self.pel[1], fwd=self.pel[0], footL=self.cmd("L"), footR=self.cmd("R"),
                            drop=self.p["drop"], hike={k: math.radians(v) for k, v in self.hike.items()},
                            ankle={k: math.radians(v) for k, v in self.ankle.items()},
                            ref=self.psi_ref, heading=self.psi_p, yawL=self.yawf["L"], yawR=self.yawf["R"])
        except ValueError as e:
            raise Abort(f"{tag}: {e}")
        self.last_tag = tag
        self.r.pose(q, ms, margin=self.p["pose_margin"])
        self.log("pose", tag=tag, ms=ms, pel=list(self.pel), hike=dict(self.hike), feet=json.loads(json.dumps(self.feet)))

    def read(self, tag=""):
        """One IMU read. A read that jumps from the last accepted one (or is large) may be torn: only then a
        second (and third) is taken, as before: one wild sample between two that agree is a torn read, a large
        angle that is still there on the next read is a fall."""
        lim = self.p["fall_deg"]
        b = self.r.imu()
        last = self.last_imu
        if (last is None or abs(b["roll"] - last["roll"]) >= 12 or abs(b["pitch"] - last["pitch"]) >= 12
                or abs(b["roll"]) > 0.5 * lim or abs(b["pitch"]) > 0.5 * lim):
            a = b
            b = self.r.imu()
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
            b = c
        self.last_imu = b
        b["rate"] = math.degrees(math.hypot(b["gyro"][0], b["gyro"][1]))      # horizontal gyro, deg/s
        if b.get("yaw") is not None:
            self.yaw_hist = (self.yaw_hist + [(b["t"], b["yaw"])])[-80:]
        self.log("imu", tag=tag, roll=round(b["roll"], 2), pitch=round(b["pitch"], 2), rate=round(b["rate"], 1),
                 gyro=[round(x, 3) for x in b["gyro"]])
        return b

    # -- heading ------------------------------------------------------------
    def heading_now(self):
        """The measured heading (deg, + = CCW, zero at the start), from the latest read; None if the robot has no
        heading (no quaternion) or it has not been started."""
        if not self.yaw_hist or not hasattr(self.r, "heading"):
            return None
        t, y = self.yaw_hist[-1]
        return self.r.heading(y, t)

    def heading_calibrate(self, n=32):
        """Standing still at the start (n reads, about 3 s): the heading is zero here and the yaw drift is the
        slope of the fused yaw over the window."""
        if not hasattr(self.r, "heading_start"):
            return
        self.yaw_hist = []
        for _ in range(n):
            self.read("heading still")
        if len(self.yaw_hist) >= 4:
            self.r.heading_start(list(self.yaw_hist))
            self.log("heading_start", rate=round(self.r.yaw_rate, 3), n=len(self.yaw_hist))
            self.say(f"  heading zeroed, yaw drift {self.r.yaw_rate:+.2f} deg/s")

    def link_event(self, d):
        self.log("link", **{("what" if k == "ev" else k): v for k, v in d.items()})
        print(f"-- link: {d}", flush=True)

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
        self.heading_calibrate()
        return s

    def stance_pel(self, stance):
        P = self.p
        swing = "R" if stance == "L" else "L"
        ox, oy = self.off[stance]
        fwd = P["fwd_w"] * self.feet[stance][0] + (1 - P["fwd_w"]) * self.feet[swing][0] + ox
        return [fwd, self.fy(stance) + P["k_shift"][stance] * oy]       # over the stance sole, wherever it stands

    def move_pel(self, to, tag, guard=None, heading_to=None):
        """Move the pelvis to `to` in monitored moves, taking any hip trim out on the way. Both feet are down.
        guard=(stance, roll0): a shift over `stance`; with both feet down the body normally leans AWAY from the
        side the pelvis goes to, so a lean toward it (roll relative to roll0) past shift_out_lim is the stance
        foot tipping over its outer edge: the pelvis steps back toward the middle, k_shift[stance] drops a step
        and the move ends there (self.shift_tipped)."""
        P = self.p
        a, k0 = list(self.pel), dict(self.hike)
        h0 = self.psi_p
        dist = math.hypot(to[0] - a[0], to[1] - a[1])
        n = max(1, math.ceil(dist / P["shift_step"]), math.ceil(max(k0.values()) / 3.0))
        ms = int(1e3 * max(P["t_min"], P["t_shift"] * (dist / n) / 0.08))
        s = None
        for i in range(1, n + 1):
            u = i / n
            self.pel = [a[0] + u * (to[0] - a[0]), a[1] + u * (to[1] - a[1])]
            self.hike = {k: v * (1 - u) for k, v in k0.items()}
            if heading_to is not None:
                self.psi_p = h0 + u * (heading_to - h0)               # the pelvis turns with the shift
            self.go(ms, f"{tag} {i}/{n}")
            s = self.read(tag)
            dr, dp = self.lean(s)
            if guard is not None:
                toward = (dr - guard[1]) if guard[0] == "R" else -(dr - guard[1])      # roll + = leaning to the robot's right
                if toward > P["shift_out_lim"]:
                    self.shift_back(guard[0], toward)
                    return s
            if abs(dr) > P["shift_lean_lim"] or abs(dp) > P["shift_lean_lim"]:
                self.say(f"  leaning in {tag}: roll {dr:+.1f} pitch {dp:+.1f} at shift {1e3 * self.pel[1]:+.0f} mm")
                raise Abort(f"lean {dr:+.1f}/{dp:+.1f} deg during {tag}")
        return s

    def rebase(self, d):
        """The body-aligned frame turns by d (rad): the feet and the pelvis keep their places in the world, so their
        coordinates in it rotate by -d. A change of coordinates: no pose changes (the world position is
        R(psi_ref) * coordinates; the pelvis heading psi_p is not touched)."""
        c, s_ = math.cos(d), math.sin(d)
        for v in (self.feet["L"], self.feet["R"], self.pel):
            v[0], v[1] = c * v[0] + s_ * v[1], -s_ * v[0] + c * v[1]
        self.psi_ref += d
        self.log("rebase", d_deg=round(math.degrees(d), 2), psi_ref=round(math.degrees(self.psi_ref), 2))

    def shift_back(self, stance, toward):
        """The shift over `stance` is tipping outward: one quick move back toward the middle, k_shift down a step."""
        P = self.p
        mid = 0.5 * (self.fy("L") + self.fy("R"))
        y = self.pel[1]
        back = max(0.0, abs(y - mid) - 1e-3 * P["shift_back_mm"])
        self.pel[1] = mid + math.copysign(back, y - mid)
        self.go(int(1e3 * P["t_min"]), f"shift back {stance}")
        old = P["k_shift"][stance]
        P["k_shift"][stance] = round(max(P["k_min"], old - P["k_step"]), 3)
        self.shift_tipped = True
        self.log("shift_tip", stance=stance, toward=round(toward, 2), shift_mm=round(1e3 * y, 1), back_to_mm=round(1e3 * self.pel[1], 1),
                 k_old=old, k=P["k_shift"][stance])
        self.say(f"  shift over {stance} tipping out ({toward:+.1f} deg toward it): back to {1e3 * self.pel[1]:+.0f} mm, k now x{P['k_shift'][stance]:.2f}")

    def shift_over(self, stance):
        d = self.yawf[stance] - self.psi_ref
        if abs(d) > 1e-9:
            self.rebase(d)                                   # the new frame: the one this stance foot's yaw defines
        to = self.stance_pel(stance)
        roll0 = self.lean(self.read(f"shift {stance} start"))[0]
        self.shift_tipped = False
        self.move_pel(to, f"shift {stance}", guard=(stance, roll0), heading_to=self.yawf[stance])
        self.psi_p = self.yawf[stance]
        self.corr_p = self.corr[stance]
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
                if tilt < -0.5 * P["out_lim"]:
                    return "tipping_out", h
                # hanging toward the lifted foot: the weight is inboard of the stance foot -- like fell_in
                return ("tipping_in" if tilt > 0.5 * P["tilt_lim"] else "tipping"), h
            hike = max(0.0, min(P["hike_max"], hike + P["g_hike"] * tilt))      # the stance hip takes up the sag
            if clr.p >= P["p_clear"]:
                self.hike[stance] = hike
                self.go(int(1e3 * P["t_inc"]), f"level {swing}")
                time.sleep(P["t_verdict"])                          # the verdict is read on a body that has stopped moving
                self.settle(f"verdict {swing}", tmax=1.0)
                self.k_off = abs((self.pel[1] - self.fy(stance)) / self.off[stance][1])    # the shift at lift-off
                self.n_push0 = self.n_push
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
        yst = self.fy(stance)                                # the shift is measured from the stance sole
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
                y = abs(self.pel[1] - yst) + 1e-3 * P["kb"] * tilt
                y = max(P["k_min"] * abs(oy), min(P["k_max"] * abs(oy), y))
                new = yst + math.copysign(y, oy)
                fl, fr = self.cmd("L"), self.cmd("R")
                while abs(new - self.pel[1]) > 1e-4 and not self.reachable(new, self.pel[0], fl, fr):
                    new = self.pel[1] + 0.5 * (new - self.pel[1])        # clamp short of the reach limit
                self.pel[1] = new
                self.n_push += 1
                self.hike[stance] = max(0.0, min(P["hike_max"], self.hike[stance] + P["g_hike"] * tilt))
                self.go(int(1e3 * P["t_bal"]), f"balance {swing}")

    def lower_foot(self, swing, fast=False):
        """Put `swing` down in monitored moves (the hip trim stays until the next shift takes it out).
        fast: it is going over outward -- foot down and pelvis back toward the middle in one move."""
        P = self.p
        h0 = self.feet[swing][2]
        n = 1 if fast else max(1, math.ceil(h0 / P["lower_step"]))
        if fast:
            stance = "L" if swing == "R" else "R"
            yst = self.fy(stance)
            self.pel[1] = yst + 0.7 * (self.pel[1] - yst)          # back toward the middle, measured from the stance sole
            self.hike = {"L": 0.0, "R": 0.0}
        for i in range(1, n + 1):
            self.feet[swing][2] = h0 * (1 - i / n)
            self.go(int(1e3 * (P["t_min"] if fast else P["t_lower"])), f"lower {swing} {1e3 * self.feet[swing][2]:.0f}")
            self.read(f"lower {swing}")
        s = self.last_imu                                    # the last lowering read: nothing needs a settle here
        dr, dp = self.lean(s)
        self.say(f"  down {swing}  lean {dr:+.2f} pitch {dp:+.2f}")
        return s

    def learn_k(self, stance, k_off, pushed):
        """The shift remembered for `stance` after a successful lift: the one at lift-off (not where balance
        wandered to later), rising by at most half a step per lift; after a lift that needed no balance push it
        decays 0.02 toward k_floor. So k cannot ratchet up to its ceiling."""
        P = self.p
        old = P["k_shift"][stance]
        new = min(k_off, old + 0.5 * P["k_step"])
        if not pushed and new > P["k_floor"]:
            new = max(P["k_floor"], new - 0.02)
        P["k_shift"][stance] = round(max(P["k_min"], min(P["k_max"], new)), 3)
        return P["k_shift"][stance]

    def lift(self, swing, hold=1.5, dx=0.0, y_to=None, air=None, yaw_to=None, corr_to=None):
        """One self-correcting lift of `swing` (and, while it is up, a forward move of dx m and, if y_to is
        not None, a move to the absolute lateral offset y_to m: +y = the robot's left). air: a callable run once
        with the foot up and judged up; it returns "up" or a tipping verdict (then the foot goes down and the
        lift ends: no retry)."""
        stance = "L" if swing == "R" else "R"
        P = self.p
        for attempt in range(4):
            self.shift_over(stance)
            how, h = self.raise_foot(swing)
            hike = self.hike[stance]
            self.log("lift", utc=time.time(), swing=swing, attempt=attempt, how=how, h=h, hike=round(hike, 2), k=P["k_shift"][stance])
            if how == "up":
                dy = 0.0 if y_to is None else y_to - self.fy(swing)
                if y_to is not None:
                    self.feet[swing][1] = y_to
                dyaw = 0.0 if yaw_to is None else yaw_to - self.yawf[swing]
                if yaw_to is not None:
                    self.yawf[swing] = yaw_to
                    self.corr[swing] = corr_to if corr_to is not None else self.corr[swing]
                if dx or abs(dy) > 1e-9 or abs(dyaw) > 1e-9:
                    self.feet[swing][0] += dx
                    self.go(int(1e3 * P["t_swing"]), f"swing {swing} {1e3 * dx:+.0f} {1e3 * dy:+.0f}")
                how = self.balance(swing, hold)
                if how == "up" and air is not None:
                    how = air(swing)
                    if how != "up":
                        self.lower_foot(swing, fast=(how == "tipping_out"))
                        return how, h
            if how == "up":
                hike = self.hike[stance]
                k = self.k_off if self.k_off is not None else abs((self.pel[1] - self.fy(stance)) / self.off[stance][1])
                self.say(f"  up: {swing} at {1e3 * h:.0f} mm, hip trim {hike:.1f}, shift x{k:.2f} at lift-off")
                a = 0.5 if P["h_up"][stance] else 1.0
                P["h_up"][stance] = round((1 - a) * P["h_up"][stance] + a * h, 4)
                P["hike"][stance] = round((1 - a) * P["hike"][stance] + a * hike, 2)
                self.learn_k(stance, k, pushed=self.n_push > getattr(self, "n_push0", 0))
                self.lower_foot(swing)
                if self.look_after:                                  # the board look in the time a settle would have taken
                    self.cached_look, self.has_cached = self.look(f"after {swing}"), True
                return how, h
            # not up: put it down and correct the shift for this stance side
            self.lower_foot(swing, fast=(how == "tipping_out"))
            dk = -P["k_step"] if how == "tipping_out" else (P["k_step"] if how in ("max", "fell_in", "tipping_in") else 0.0)
            P["k_shift"][stance] = round(max(P["k_min"], min(P["k_max"], P["k_shift"][stance] + dk)), 3)
            self.say(f"  {how} at {1e3 * h:.0f} mm (hip trim {hike:.1f}) -> shift over {stance} now x{P['k_shift'][stance]:.2f}")
        return "gave_up", 0.0

    # -- the board ----------------------------------------------------------
    def _next_frame(self, after, tmax):
        """The next front-camera frame written after `after` (file mtime) and younger than 0.6 s:
        (image, mtime), or None if none comes within tmax s."""
        import cv2
        t = time.monotonic()
        while True:
            try:
                mt = os.path.getmtime(self.latest)
                if mt > after and time.time() - mt < 0.6:
                    im = cv2.imread(self.latest)
                    if im is not None:
                        return im, mt
            except OSError:
                pass
            if time.monotonic() - t > tmax:
                return None
            time.sleep(0.03)

    def look(self, tag):
        """Measure the margins to the board's edge: the median of 3 fresh front-camera frames (board_view),
        ignoring any below board_conf_min; at least 2 must be good. Returns the dict (the median of every
        number), or None. The annotated middle frame is saved. Never raises."""
        import statistics
        self.n_look += 1
        P = self.p
        res, mid, why, after = [], None, "no frame", 0.0
        try:
            import board_view
            for i in range(3):
                got = self._next_frame(after, 1.5)
                if got is None:
                    break
                im, after = got
                m = board_view.margins(im, P["board_w_mm"])
                if i == 1 or mid is None:
                    mid = (im, m)
                if m is None:
                    why = "board not seen"
                elif m["conf"] < P["board_conf_min"]:
                    why = f"low confidence {m['conf']:.2f}"
                else:
                    res.append(m)
            if mid is not None:
                import cv2
                cv2.imwrite(f"{self.base}_board_{self.n_look:02d}.jpg", board_view.draw(*mid))
        except Exception as e:                      # looking must never stop the robot's own control
            res, why = [], f"look failed: {e!r}"
        if len(res) < 2:
            self.log("board", tag=tag, seen=False, why=why, good=len(res))
            self.say(f"  board {tag}: not seen ({why})")
            return None
        keys = ("right_mm", "left_mm", "sole_r_mm", "sole_l_mm", "sep_mm", "mm_per_px", "conf", "board_px")
        m = {k: statistics.median(r[k] for r in res) for k in keys}
        m["ahead_px"] = int(statistics.median(r["ahead_px"] for r in res))
        self.log("board", tag=tag, seen=True, n=len(res), right_mm=round(m["right_mm"], 1), left_mm=round(m["left_mm"], 1),
                 sep_mm=round(m["sep_mm"], 1), sole_r_mm=round(m["sole_r_mm"], 1), sole_l_mm=round(m["sole_l_mm"], 1),
                 ahead_px=m["ahead_px"], conf=round(m["conf"], 2))
        self.say(f"  board {tag}: right {m['right_mm']:.0f} mm  left {m['left_mm']:.0f} mm  sep {m['sep_mm']:.0f} mm  "
                 f"ahead {m['ahead_px']} px  conf {m['conf']:.2f}")
        return m

    def outer(self, m):
        """The outer loop, in board coordinates (mm; sole_l_mm / sole_r_mm of a good look `m`, both feet down).
        The first look fixes the targets: T[L], T[R] = `centre` +- `half` (the stance it was put down in) and
        sep0, the measured separation that goes with the IK's zero offsets. Every later look:
          anchor: the real offset between the feet vs the commanded one (logged; the aim below is relative to
                  the MEASURED stance foot, so the commands themselves are never jumped);
          landing: the foot that landed since (its lift ended up) has error T - measured; its learned miss
                  comp[f] takes lat_gain of it (m, clamped +-comp_lim);
          --board-centre moves the line toward the board's middle."""
        P = self.p
        sl, sr = m["sole_l_mm"], m["sole_r_mm"]
        self.meas, self.fresh = dict(L=sl, R=sr), True
        if self.bt is None:
            sep = sl - sr
            self.bt = dict(half=0.5 * max(P["sep_min_mm"], min(sep, P["sep_max_mm"])), centre=0.5 * (sl + sr), sep0=sep)
            self.log("board_target", **{k: round(v, 1) for k, v in self.bt.items()})
            return
        d_true, d_cmd = 1e-3 * (sl - sr - self.bt["sep0"]), self.feet["L"][1] - self.feet["R"][1]
        self.log("anchor", d_true=round(1e3 * d_true, 1), d_cmd=round(1e3 * d_cmd, 1), corr=round(1e3 * (d_true - d_cmd), 1))
        for f in "LR":
            t = self.t_land[f]
            if t is None:
                continue
            self.t_land[f] = None
            meas = sl if f == "L" else sr
            err = t - meas                                       # + = it should have landed further left
            P["comp"][f] = round(max(-P["comp_lim"], min(P["comp_lim"], P["comp"][f] + P["lat_gain"] * 1e-3 * err)), 5)
            self.log("landing", side=f, meas=round(meas, 1), target=round(t, 1), err=round(err, 1), comp=P["comp"][f])
            self.say(f"  {f} landed at {meas:+.0f} mm, aimed {t:+.0f}: off by {err:+.0f} -> miss compensation {1e3 * P['comp'][f]:+.1f} mm")
        if P["board_centre"]:
            e = 0.5 * (m["left_mm"] - m["right_mm"])             # + = more room on the left: move the line left
            if abs(e) > P["board_dead_mm"]:
                self.bt["centre"] += max(-P["board_step_mm"], min(P["board_step_mm"], P["board_gain"] * e))

    def aim(self, side, dx=0.0):
        """Where the swing foot is commanded (IK y, m). The stance foot g is where it was MEASURED to be: the
        wanted real offset is w = (T[f] - meas[g]); the IK offset is that less w0, the nominal one (+-sep0:
        the soles' separation that goes with zero IK offset), relative to g's commanded y, plus comp[f], the
        learned miss of this swing foot. Without a good look this step: the same relative offset as last time
        for this foot. The planned real separation is never below min_sep_cmd, one swing changes the foot's offset from the
        stance foot by at most lat_step_max, and the result is pulled back until its poses are within the legs' reach."""
        P = self.p
        g = "R" if side == "L" else "L"
        if self.bt is None:
            return self.place(side, self.feet[g][1] + P["comp"][side], dx)
        tgt = self.bt["centre"] + (self.bt["half"] if side == "L" else -self.bt["half"])
        if self.fresh:
            w0 = (1.0 if side == "L" else -1.0) * self.bt["sep0"]
            self.rel[side] = 1e-3 * (tgt - self.meas[g] - w0)
        rel = self.rel[side]
        # never plan the soles closer than min_sep_cmd: the REAL separation after the landing is sep0 + rel (the
        # miss and comp cancel); the IK's own reach limit is handled by reach()
        lim = 1e-3 * (P["min_sep_cmd"] - self.bt["sep0"])
        rel = max(rel, lim) if side == "L" else min(rel, -lim)
        y = self.feet[g][1] + rel + P["comp"][side]
        self.t_land[side] = tgt
        return self.place(side, y, dx)

    def place(self, side, y, dx):
        """Final aim for the swing foot at IK y: its offset from the stance foot may change by at most
        lat_step_max from what this foot was last sent to (a walk's absolute y drifts with comp; it is the
        offset between the feet that matters), then pulled back into the legs' reach. Remembers the offset."""
        g = "R" if side == "L" else "L"
        off = y - self.feet[g][1]
        last = self.last_off[side]
        off = last + max(-self.p["lat_step_max"], min(self.p["lat_step_max"], off - last))
        # the commanded separation (IK geometry) stays above min_sep_cmd, less what the stance was wider than the IK's
        # nominal when the offsets were zero (that width is real, it is not in the commands)
        sep0 = self.bt["sep0"] if self.bt else self.p["sep_nominal_mm"]
        lim = 1e-3 * (self.p["min_sep_cmd"] - self.p["sep_nominal_mm"] - max(0.0, sep0 - self.p["sep_nominal_mm"]))
        off = max(off, lim) if side == "L" else min(off, -lim)
        y = self.reach(side, self.feet[g][1] + off, dx)
        self.last_off[side] = y - self.feet[g][1]
        return y

    # -- reach --------------------------------------------------------------
    def reachable(self, shift, fwd, footL, footR):
        try:
            self.k.pose(shift=shift, fwd=fwd, footL=footL, footR=footR, drop=self.p["drop"])
            return True
        except ValueError:
            return False

    def reach_ok(self, swing, y_cmd, dx, headroom=True):
        """Can the poses this step needs be reached with `swing` landing at IK y `y_cmd`? (1) in the air: the pelvis
        over the stance foot as far as balance may push it (k_max), the swing foot at its lift height; (2) the
        shift over the swing foot afterwards (its k_shift), both feet down."""
        P = self.p
        g = "R" if swing == "L" else "L"
        xs, xg = self.feet[swing][0] + dx, self.feet[g][0]
        yg = self.fy(g)

        def feet(zs):
            f = {swing: (xs, y_cmd, zs), g: (xg, yg, 0.0)}
            return f["L"], f["R"]
        ox, oy = self.off[g]
        fwd = P["fwd_w"] * xg + (1 - P["fwd_w"]) * xs + ox
        zs = max(P["dh"], P["h_up"][g])
        kg = P["k_max"] if headroom else P["k_shift"][g]
        if not self.reachable(yg + kg * oy, fwd, *feet(zs)) or not self.reachable(yg + kg * oy, fwd, *feet(0.0)):   # up, and lowering
            return False
        ox, oy = self.off[swing]
        fwd = P["fwd_w"] * xs + (1 - P["fwd_w"]) * xg + ox
        return self.reachable(y_cmd + P["k_shift"][swing] * oy, fwd, *feet(0.0))

    def reach(self, swing, y_to, dx=0.0):
        """y_to, pulled toward the foot's current y in 5 mm steps until the poses it needs are reachable
        (`reach_limit` logs what was given up). If even the unchanged y is not reachable the run ends (Abort)."""
        y0 = self.feet[swing][1]
        y = y_to
        while not self.reach_ok(swing, y, dx):
            if abs(y - y0) < 1e-9:
                if self.reach_ok(swing, y, dx, headroom=False):        # only balance's headroom is out of reach: go on
                    break
                raise Abort(f"{swing} cannot be placed: out of reach even at its present y")
            y = y0 + math.copysign(max(0.0, abs(y - y0) - 0.005), y - y0)
        if abs(y - y_to) > 1e-9:
            self.log("reach_limit", side=swing, aimed=round(y_to, 4), used=round(y, 4), gave_up_mm=round(1e3 * abs(y - y_to), 1))
            self.say(f"  {swing}: out of reach at {1e3 * (y_to - y0):+.0f} mm, using {1e3 * (y - y0):+.0f}")
        return y

    # -- the servos ---------------------------------------------------------
    def servo_guard(self, tag):
        """Read position, load, voltage and temperature of the eight load-bearing joints, log them (`servos`).
        Returns why the run must end (a servo over temp_max_c, the bus under volt_min), or None. A read that
        fails is logged and does not end the run."""
        P = self.p
        rows, why = {}, None
        try:
            for n in GUARD:
                rows[n] = self.r.pos(n)
        except Exception as e:
            self.log("servos", tag=tag, err=repr(e), rows=rows)
            return None
        self.log("servos", tag=tag, rows=rows)
        hot = max(rows.items(), key=lambda kv: kv[1]["temp"])
        low = min(rows.items(), key=lambda kv: kv[1]["volt"])
        if hot[1]["temp"] > P["temp_max_c"]:
            why = f"{hot[0]} is at {hot[1]['temp']} C (limit {P['temp_max_c']:.0f})"
        elif low[1]["volt"] < P["volt_min"]:
            why = f"the bus is at {low[1]['volt']:.1f} V (limit {P['volt_min']:.1f})"
        return why

    def halt(self, why, ev="stop"):
        self.stop_why = why
        self.log(ev, why=why)
        print(f"-- stopping: {why}", flush=True)

    # -- the step loops -----------------------------------------------------
    def _step(self, side, dx, dy, hold, march, board, forward):
        """One step of a walk: servo guard, look (board), then the lift. Returns (status, result):
        "ok" (result = (how, h)), "ahead" (the board ends ahead: only for forward motion), "stop" (self.stop_why
        says why: an edge, no sight of the board, a servo) or "gave_up"."""
        P = self.p
        if self.n_steps % P["guard_every"] == 0:
            why = self.servo_guard(f"step {self.n_steps}")
            if why:
                self.halt(why, "guard_stop")
                return "stop", None
        self.n_steps += 1
        self.fresh = False
        # heading: the pelvis was commanded to psi_p; the IMU says where it is. The swing foot's yaw is this step's
        # share of the turn plus yaw_gain of the difference (a slip is taken out step by step, a drift never builds)
        share = self.dyaw / 2.0
        h = None
        if hasattr(self.r, "heading") and self.yaw_hist:
            self.read("heading")
            h = self.heading_now()
        yaw_step = share
        if h is not None:
            err = math.degrees(self.psi_p) - self.corr_p - h          # the commanded turn so far less what is measured
            if abs(err) > P["yaw_dead"]:
                yaw_step += P["yaw_gain"] * err
            self.log("heading", deg=round(h, 2), cmd=round(math.degrees(self.psi_p), 2), err=round(err, 2), yaw_step=round(yaw_step, 2))
            if board and abs(h) > P["yaw_board_max"]:
                self.halt(f"the heading is {h:+.0f} deg: the board loop assumes the robot square to the board", "board_stop")
                return "stop", None
        yaw_step = max(-P["yaw_step_max"], min(P["yaw_step_max"], yaw_step))
        corr_step = yaw_step - share
        if board:
            if self.has_cached:                                  # taken right after the last landing
                m, self.has_cached, self.cached_look = self.cached_look, False, None
            else:
                m = self.look(f"step {self.n_steps - 1}")
            self.misses = 0 if m else self.misses + 1
            if m is None:
                if self.misses >= 2:
                    self.halt("the board was not seen twice in a row", "board_stop")
                    return "stop", None
            elif min(m["left_mm"], m["right_mm"]) < P["board_min_mm"]:
                self.halt(f"a sole is {min(m['left_mm'], m['right_mm']):.0f} mm from the board's edge", "board_stop")
                return "stop", None
            elif forward and m["ahead_px"] < P["board_ahead_px"]:
                self.ahead = m["ahead_px"]
                self.log("board_ahead", ahead_px=m["ahead_px"])
                return "ahead", None
            else:
                self.outer(m)
        stance = "R" if side == "L" else "L"
        if board:
            if self.bt is not None:
                self.bt["centre"] += 0.5e3 * dy                      # a sidestep moves the line, half of dy per step
            y_to = self.aim(side, dx)
        else:
            step_out = 0.0
            for d in (dy,):                          # outward only: left foot to the left, right foot to the right
                if (d > 0 and side == "L") or (d < 0 and side == "R"):
                    step_out += d
            y_to = self.place(side, self.feet[stance][1] + step_out + self.p["comp"][side], dx)
        res = (side,) + self.lift(side, hold=hold, dx=dx, y_to=y_to, yaw_to=self.yawf[stance] + math.radians(yaw_step),
                                      corr_to=self.corr[stance] + corr_step)
        if res[1] == "gave_up" or res[1] != "up":
            self.t_land[side] = None                                 # it did not land anew
        return ("gave_up" if res[1] == "gave_up" else "ok"), res

    def walk(self, side="L", steps=2, dx=0.02, dy=0.0, hold=0.2, board=False, march=False, dyaw=0.0):
        """The step loop of `walk` and `march`. Returns the list of (side, how, h). dx < 0 walks backward.
        dy: sidestep m per pair (+ left). board: look before every step, learn where the last foot really
        landed, stop at the edge, steer toward the middle. The swing foot is aimed at the stance foot's
        believed y plus `step_out`, the sidestep and steering share that moves it OUTWARD (left foot +, right
        foot -); a move that would narrow the stance is dropped, so the feet come back to the nominal
        separation instead. self.stop_why says why a run ended early."""
        out = self.out = []                              # kept for the caller if a fall cuts the loop short
        self.stop_why, self.misses = None, 0
        self.dyaw, self.look_after, self.cached_look, self.has_cached = dyaw, board, None, False
        for i in range(steps):
            st, res = self._step(side, 0.0 if march else dx * (1 if i == 0 else 2), dy,
                                 hold if march else 0.2, march, board, forward=dx >= 0)
            if st == "ahead":
                self.halt(f"the board ends {self.ahead} px ahead", "board_stop")
            if st != "ok":
                if st == "gave_up":
                    out.append(res)
                break
            out.append(res)
            side = "R" if side == "L" else "L"
        return out

    def shuttle(self, side="L", steps=4, cycles=1, dx=0.02, dy=0.0, hold=0.2, board=False, dx_list=None, dyaw=0.0):
        """`steps` steps forward, then back the way it came, `cycles` times, in one run. A leg's first and last
        steps are half strides (dx, the others 2 dx), so the feet end level; the return leg undoes the forward
        steps in reverse order, so the feet end where they began (in belief). dx_list: cycle k uses
        dx_list[k % len]. A board's end ahead turns the forward leg round early (the return then starts from
        a split stance); an edge or a servo ends the run."""
        out = self.out = []
        self.stop_why, self.misses = None, 0
        self.dyaw, self.look_after, self.cached_look, self.has_cached = dyaw, board, None, False
        side0 = side
        for c in range(cycles):
            d = dx_list[c % len(dx_list)] if dx_list else dx
            side, hist = side0, []
            self.dyaw = dyaw
            for i in range(steps):
                x0 = self.feet[side][0]
                st, res = self._step(side, d * (1 if i in (0, steps - 1) else 2), dy, 0.2, False, board, forward=d >= 0)
                if st == "ahead":
                    self.say("  the board ends ahead: turning round early")
                    break
                if st != "ok":
                    if st == "gave_up":
                        out.append(res)
                    return out
                out.append(res)
                hist.append((side, x0))
                side = "R" if side == "L" else "L"
            self.dyaw = -dyaw                                    # back the way it came: the turn is undone too
            for sd, x0 in reversed(hist):
                st, res = self._step(sd, x0 - self.feet[sd][0], dy, 0.2, False, board, forward=d < 0)
                if st == "ahead":                        # backward legs of a negative dx walk forward
                    self.say("  the board ends ahead: ending the return leg")
                    return out
                if st != "ok":
                    if st == "gave_up":
                        out.append(res)
                    return out
                out.append(res)
        return out

    # -- data blocks --------------------------------------------------------
    def _air_state(self, swing, h, pitch_s):
        s = self.read(f"sweep {swing}")
        tilt = self.tilt_to(swing, s) - self.tilt_s
        P = self.p
        clr = foot_clear(h, tilt, need=P["need"])
        tip = tipping(tilt, self.lean(s)[1] - pitch_s, s["rate"], P["tilt_lim"], P["pitch_lim"], P["rate_lim"], out_lim=P["out_lim"])
        return s, tilt, clr, tip

    def sweeps(self, swing):
        """With `swing` up: sweep the stance hip-roll trim, then the stance ankle-roll trim, in a slow
        triangle of +-sweep_deg around where it is (0.5 deg steps, one IMU read each), sweep_cycles times.
        A leg of the sweep turns round early if P(tip) > 0.3 or P(clear) < 0.5; if that is so at the
        trim itself, the sweep ends ("tipping"). Returns "up" or a tipping verdict."""
        P = self.p
        stance = "L" if swing == "R" else "R"
        h = self.feet[swing][2]
        pitch_s = self.lean(self.ref)[1]
        for joint, trim in (("hip_roll", self.hike), ("ankle_roll", self.ankle)):
            base = trim[stance]
            off, n = 0.0, 0
            for _ in range(int(P["sweep_cycles"])):
                for target in (P["sweep_deg"], -P["sweep_deg"], 0.0):
                    d = 1.0 if target > off else -1.0
                    while (target - off) * d > 1e-9 and n < 600:
                        n += 1
                        off = round(off + d * 0.5, 6)
                        trim[stance] = base + off
                        self.go(int(1e3 * P["t_sweep"]), f"sweep {joint} {off:+.1f}")
                        s, tilt, clr, tip = self._air_state(swing, h, pitch_s)
                        self.log("sweep", joint=joint, stance=stance, offset_deg=off, roll=round(s["roll"], 2),
                                 pitch=round(s["pitch"], 2), tilt=round(tilt, 2), rate=round(s["rate"], 1),
                                 p_tip=round(tip.p, 3), p_clear=round(clr.p, 3))
                        if tip.p > 0.3 or clr.p < 0.5:
                            if abs(off) < 0.75:
                                trim[stance] = base
                                self.go(int(1e3 * P["t_sweep"]), f"sweep {joint} abort")
                                self.say(f"  sweep {joint}: not safe at the found trim (P(tip) {tip.p:.2f}, P(clear) {clr.p:.2f})")
                                return "tipping"
                            off = round(off - d * 0.5, 6)       # one step back, and this leg is over
                            trim[stance] = base + off
                            self.go(int(1e3 * P["t_sweep"]), f"sweep {joint} back {off:+.1f}")
                            self.say(f"  sweep {joint}: turning round at {off:+.1f} deg")
                            break
            trim[stance] = base
            self.go(int(1e3 * P["t_sweep"]), f"sweep {joint} done")
            self.say(f"  sweep {joint}: {n} steps")
        return "up"

    def shiftsweep(self):
        """Both feet down in the crouch: the pelvis shift 0 -> +sweep_k*62 mm -> -sweep_k*62 mm -> 0 in 6 mm
        steps, sweep_cycles times; an IMU read every step, the servos every 3rd. Ends (back at the centre) if
        the lean passes shift_lean_lim or a servo is over its limit. Returns "ok" or why it ended."""
        P = self.p
        top = P["sweep_k"] * 0.062
        n = 0
        for _ in range(int(P["sweep_cycles"])):
            for target in (top, -top, 0.0):
                y = self.pel[1]
                d = 1.0 if target > y else -1.0
                while (target - y) * d > 1e-6:
                    y = y + d * min(0.006, abs(target - y))
                    n += 1
                    self.pel[1] = y
                    self.go(int(1e3 * P["t_sweep"]), f"shiftsweep {1e3 * y:+.0f}")
                    s = self.read("shiftsweep")
                    dr, dp = self.lean(s)
                    self.log("sweep", joint="shift", shift_mm=round(1e3 * y, 1), roll=round(s["roll"], 2), pitch=round(s["pitch"], 2),
                             lean_roll=round(dr, 2), lean_pitch=round(dp, 2))
                    if abs(dr) > P["shift_lean_lim"] or abs(dp) > P["shift_lean_lim"]:
                        self.say(f"  shiftsweep: leaning {dr:+.1f}/{dp:+.1f} at {1e3 * y:+.0f} mm: back to the centre")
                        self.centre()
                        return "lean"
                    if n % 3 == 0:
                        why = self.servo_guard(f"shiftsweep {1e3 * y:+.0f}")
                        if why:
                            self.halt(why, "guard_stop")
                            self.centre()
                            return why
        return "ok"

    def centre(self):
        mid = [0.5 * (self.feet["L"][0] + self.feet["R"][0]), 0.5 * (self.fy("L") + self.fy("R"))]
        h = 0.5 * (self.yawf["L"] + self.yawf["R"])                  # square to the soles: their mean yaw
        s = self.move_pel(mid, "centre", heading_to=h)
        self.psi_p = h
        return s

    def stand_down(self):
        """Centre, stand the robot up and let go. A dropped link is retried by the robot (Robot.cmd); if it still
        fails, once more on a fresh connection; if even that fails, say plainly what state the robot is in."""
        for attempt in range(2):
            try:
                if attempt:
                    self.r.fresh()
                    self.log("link", what="fresh_connection", attempt=attempt)
                try:
                    self.feet["L"][2] = self.feet["R"][2] = 0.0
                    self.centre()
                    turned = any(abs(v) > 1e-9 for v in (self.psi_p, self.yawf["L"], self.yawf["R"]))
                    split = turned or max(abs(self.feet["L"][0] - self.feet["R"][0]), abs(self.fy("L") - self.fy("R"))) > 1e-3   # straight legs cannot span a stride, a width change or a turn
                    self.r.pose(self.k.pose(straight=not split, drop=0.008 if split else None, fwd=self.pel[0], shift=self.pel[1],
                                            footL=(self.feet["L"][0], self.fy("L"), 0),
                                            footR=(self.feet["R"][0], self.fy("R"), 0),
                                            ref=self.psi_ref, heading=self.psi_p, yawL=self.yawf["L"], yawR=self.yawf["R"]), 2500)
                    time.sleep(0.5)
                except Abort as e:
                    self.log("error", err=repr(e)); print("!! ", repr(e), flush=True)
                if not self.fell:
                    self.r.release()
                return True
            except self.Fallen as e:
                self.fallen(e)
                return False
            except self.LinkDown as e:
                self.log("link", what="stand_down_failed", attempt=attempt, err=str(e))
                print(f"!! link down while standing the robot down (attempt {attempt + 1}/2): {e}", flush=True)
        msg = (f"the robot could not be reached to stand it down. It was last commanded to {self.last_tag!r}; "
               f"torque is still ON in that pose and nothing was released: home or release it by hand")
        self.log("link", what="left_torqued", tag=self.last_tag)
        print(f"!! LINK DOWN: {msg}", flush=True)
        return False

    def finish(self, save=True):
        try:
            if not self.fell:
                self.stand_down()
        finally:
            self.log("end", feet=json.loads(json.dumps(self.feet)), **{k: self.p[k] for k in LEARNED})
            if save:
                json.dump({k: self.p[k] for k in LEARNED}, open(STATE, "w"))
            for v in self.video:
                v.wait()
            self.f.close()

    def notify(self):
        """Run notify_cmd (once, in the background); a failure to start it is logged, never raised."""
        cmd = [c.format(hhmm=self.t_fall) for c in self.p["notify_cmd"]]
        if not cmd:
            return
        try:
            subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.log("notify", cmd=cmd[0], hhmm=self.t_fall)
        except Exception as e:
            self.log("notify", err=repr(e))

    def fallen(self, e):
        """It is down. Send nothing that moves; the firmware is limp and does the homing itself once it is picked up."""
        self.fell = True
        self.t_fall = time.strftime("%H:%M")
        self.log("fallen", err=str(e), pel=list(self.pel), hike=self.hike, feet=self.feet)
        print(f"!! FELL: {e}\n!! nothing more is sent. Pick it up: the firmware homes it once it is within 25 deg of upright and still (< 1.0 rad/s) for 1 s.", flush=True)
        ok = self.r.wait_recovered(self.p["fall_wait_s"], after=(self.p["notify_after_s"], self.notify))
        self.log("recovered", ok=ok)
        print("!! picked up and homed by the firmware" if ok else "!! still down after 5 min", flush=True)


def main(argv=None, make_robot=None, log_dir=None):
    """make_robot(kin) and log_dir are for dry runs (see dry_run.py)."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=("lift", "march", "walk", "shuttle", "playsweep", "shiftsweep"))
    ap.add_argument("--side", choices=("L", "R"), default="L", help="the foot that comes up (first)")
    ap.add_argument("--steps", type=int, default=2)
    ap.add_argument("--dx", type=float, default=0.02, help="walk/shuttle: stride m (walk: negative = backward)")
    ap.add_argument("--cycles", type=int, default=1, help="shuttle: forward-and-back cycles")
    ap.add_argument("--dx-list", default="", metavar="A,B,C", help="shuttle: cycle k uses the k-th dx (wraps)")
    ap.add_argument("--pace", type=float, default=1.0, help="every phase time x this (not the first stand-up)")
    ap.add_argument("--dyaw", type=float, default=0.0, help="walk/shuttle: turn per step pair, deg (+ = to the robot's left, counter-clockwise)")
    ap.add_argument("--dy", type=float, default=0.0, help="walk: sidestep per step pair, m (+ = the robot's left)")
    ap.add_argument("--board", action="store_true", help="walk/march: watch the board's edge with the front camera")
    ap.add_argument("--hold", type=float, default=1.5)
    ap.add_argument("--video", type=int, default=0, metavar="SECS", help="record both cameras for SECS")
    ap.add_argument("--board-centre", action="store_true", help="--board: steer toward the board's centre line (default: hold the starting line)")
    ap.add_argument("--no-notify", action="store_true", help="after a fall, do not ping the owner via Aime")
    ap.add_argument("--set", nargs="*", default=[], metavar="KEY=VAL")
    ap.add_argument("--host", default="192.168.2.90")
    a = ap.parse_args(argv)
    if a.board_centre and not a.board:
        ap.error("--board-centre needs --board")
    if a.board and a.what not in ("walk", "march", "shuttle"):
        ap.error("--board is for walk, march and shuttle")
    if a.board and not a.video:
        a.video = 180
    prm = load_state()
    for kv in a.set:
        k, v = kv.split("=", 1)
        prm[k] = json.loads(v)
    if a.board_centre:
        prm["board_centre"] = True
    if a.no_notify:
        prm["notify_cmd"] = []
    if a.pace != 1.0:
        apply_pace(prm, a.pace)

    from kin import Kin
    from robot_io import Fallen, Robot
    kin = Kin(prm["drop"])
    rob = make_robot(kin) if make_robot else Robot(a.host, link_retries=int(prm["link_retries"]))
    if rob.fallen():
        print("the firmware says FALLEN: pick the robot up and let it home first"); rob.close(); return 2
    run = Run(a.what, rob, kin, prm, video=a.video, board=a.board, log_dir=log_dir)
    out = []
    try:
        run.stand_up()
        if a.what == "lift":
            out.append((a.side,) + run.lift(a.side, hold=a.hold))
        elif a.what == "playsweep":
            out.append((a.side,) + run.lift(a.side, hold=a.hold, air=run.sweeps))
        elif a.what == "shiftsweep":
            out.append(("both", run.shiftsweep()))
        elif a.what == "shuttle":
            out = run.shuttle(a.side, a.steps, a.cycles, a.dx, a.dy, hold=a.hold, board=a.board,
                              dx_list=[float(x) for x in a.dx_list.split(",")] if a.dx_list else None, dyaw=a.dyaw)
        else:
            out = run.walk(a.side, a.steps, a.dx, a.dy, hold=a.hold, board=a.board, march=a.what == "march", dyaw=a.dyaw)
    except Fallen as e:
        run.fallen(e)
    except Exception as e:                                           # any other fault: say it, then stand and let go
        run.log("error", err=repr(e)); print("!! ", repr(e), flush=True)
    finally:
        run.finish()
        rob.close()
    out = out or getattr(run, "out", [])
    print("RESULT", out, {k: prm[k] for k in LEARNED}, "fell" if run.fell else "", "log", run.base + ".jsonl")
    return 1 if run.fell else 0


if __name__ == "__main__":
    sys.exit(main())
