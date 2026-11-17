"""Position hold for the host's --cmd walk (#297): the overhead camera tracker closes the position the way the IMU
closes the heading, one bounded correction per step.

Why. The dwell gait creeps sideways a few mm per step (2026-11-16: 7-step marches in place moved the robot +11 and
+42 mm along the tracker's x with the plain 4 s gait, -29 and -24 mm with the pause variant) and walked itself to a
board edge twice. The heading hold already closes yaw. This closes lateral and fore-aft position.

Frames (tools/vision README "Frames and numbers"). Board frame: x from the board's LEFT edge as seen from the Sony
camera at the near end (the camera looks along +y, x is to its right), y from the near end, mm. Heading 0 = along +y,
counter-clockwise (a left turn) positive: forward = (-sin h, cos h), robot-left = (-cos h, -sin h). So facing the
Sony (h 180) the robot's left is +x. The eMeet looks from the far end and its IMAGE is mirrored left-right relative to
that x; the tracker works in the board frame, so nothing here ever sees image left/right.

The position sample. The torso tag swings about +-80 mm sideways with each step's pelvis shift, toward the stance
foot, and the stance foot alternates every step, so one step's average is biased by the sway and the bias flips sign
with the step. The sample is therefore the TRIMMED MEAN (15 % off each end: one mis-detected frame cannot move it)
over the last `window_steps` steps (default 2 = one full left-right gait cycle) of the tracker's per-frame position,
each frame located from the camd recorder's emeet_latest.jpg (read-only: the camera itself is never opened). A window
of one full cycle holds each phase equally, so the sample does not depend on which phase the decision falls in. (A
plain median was tried on the fake robot: the sway is two-humped, the median sits in the gap between the humps and
moves +-10 mm with the sampling.) It needs `min_samples` frames spread over at least half the
window and the newest within half a window of now; otherwise there is NO FIX.

The control. e = (sample + travel of the steps decided but not yet in the sample) - reference, in the robot frame
(lat positive = robot's left, fwd positive = forward). The reference is the start pose plus the integrated command
(the displacement each step was asked for before the correction; a zero command holds the start position). The
correction is -k e per step, clipped (lateral 6 mm, fore-aft 10 mm), added to the next unlocked step before the
envelope / slew limits are applied. `delay_steps` is how many decided steps are not yet visible in a sample (the
window half-length, the rolling upload and the camera): the loop compensates them (a Smith predictor), without it a
gain of 0.3 rings.

No fix: the correction freezes at the last trim for at most `no_fix_steps` steps, then it is zero; with a non-zero
command active, more than that many steps without a fix turns the command into `stop`. Nothing is extrapolated.

Keep on board: if the predicted position (sample + steps already decided) is within `margin` mm of a board edge, the
step is replaced by one toward the board centre (the full envelope, along the direction away from the near edges).
"""
from __future__ import annotations

import collections
import math
import os
import statistics
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
VISION = os.path.join(HERE, "..", "..", "tools", "vision")

K_DEFAULT = 0.3               # fraction of the error (beyond the dead band) fixed per step
KI_DEFAULT = 0.04             # integral gain per step: learns the standing creep and the (weak, variable) sideways response
RATE_MM = 5.0                 # the correction may change by this much per step
CLIP_LAT_MM, CLIP_FWD_MM = 12.0, 10.0
MARGIN_MM = 250.0
NO_FIX_STEPS = 4
WINDOW_STEPS = 2.0            # one full left-right cycle
DELAY_STEPS = 3
MIN_SAMPLES = 4
WARMUP_STEPS = 3              # the window must hold whole walking cycles: no correction for the first steps
DEADBAND_MM = 10.0            # the sample's step-to-step jitter on the robot is ~10 mm (2026-11-16 run): leave less than this alone
WORSE_W, WORSE_MM, WORSE_CORR_MM = 6, 60.0, 6.0   # the lateral error grew by 60 mm over 6 steps of corrections averaging >= 6 mm against it
SAT_STEPS, SAT_ERR_MM = 6, 120.0                  # corrections at the clip for 6 steps and still 120 mm off: it does not hold


# ---- frames -------------------------------------------------------------------------------------------------------

def fwd_left(h_deg):
    """Unit vectors (board x, y) of the robot's forward and left at tracker heading h (0 = +y, ccw positive)."""
    h = math.radians(h_deg)
    return (-math.sin(h), math.cos(h)), (-math.cos(h), -math.sin(h))


def to_board(h_deg, fwd, left):
    f, l = fwd_left(h_deg)
    return fwd * f[0] + left * l[0], fwd * f[1] + left * l[1]


def to_robot(h_deg, bx, by):
    """(fwd, left) components of a board-frame vector."""
    f, l = fwd_left(h_deg)
    return bx * f[0] + by * f[1], bx * l[0] + by * l[1]


def _clip(v, lim):
    return max(-lim, min(lim, v))


def trimmed_mean(v, frac=0.15):
    """Mean of the values without the lowest and highest `frac` of them (a lone mis-detected frame cannot move it; the
    symmetric swing of the torso does not bias it)."""
    v = sorted(v)
    k = int(len(v) * frac)
    v = v[k:len(v) - k] or v
    return sum(v) / len(v)


def circ_median(angles):
    a0 = angles[0]
    d = [((a - a0 + 180.0) % 360.0) - 180.0 for a in angles]
    return a0 + statistics.median(d)


# ---- the tracker, frame by frame -----------------------------------------------------------------------------------

class Locator:
    """gray frame -> dict(x, y, h) or None (`why` says). The board pose is calibrated ONCE, from the first
    `calib_frames` frames that show >= 2 board tags, and kept: the camera and the board do not move, so a frame whose
    robot hides a board tag still locates the robot. A visible board tag that sits more than `tol_px` from where the
    calibration frames had it means the camera moved: no fix until it is back (a re-aim needs a restart).
    pose_read: more pose files to try first, e.g. where.py's <camd-dir>/emeet_board_pose.json (written on every frame with
    two board tags, same JSON keys; it keeps this hold's board frame the one where.py reports margins in).
    pose_path: the calibrated pose (f, board pose, the board tags' pixel corners) is saved there, and a later start
    whose first frame shows at least one board tag where the file has it loads it instead of waiting for a frame with
    two board tags (the robot often hides one) -- and keeps the board frame the same between runs, which the
    auto-layout fit (f from the tags' coplanarity) does not."""

    def __init__(self, cfg=None, tol_px=4.0, calib_frames=3, pose_path=None, pose_read=()):
        sys.path.insert(0, VISION)
        import vgeom as G
        self.G = G
        self.cfg = cfg or G.load_config()
        self.tol, self.n_cal, self.pose_path, self.pose_read = tol_px, calib_frames, pose_path, tuple(pose_read)
        self.tr = None
        self.pending, self.ref_corners, self.cal = [], {}, None
        self.why = "not calibrated"

    def __call__(self, gray):
        from tag_track import Tracker
        from tagdefs import detect
        if self.tr is None:
            self.tr = Tracker(self.cfg, (gray.shape[1], gray.shape[0]))
        tr = self.tr
        d = detect(gray, tr.det)
        board = [i for i in d if str(i) in self.cfg["board"]["tags"]]
        if self.cal is None and board and any(self._load(d, board, pp) for pp in (*self.pose_read, self.pose_path) if pp):
            pass
        if self.cal is None:
            if len(board) >= 2:
                self.pending.append(d)
            if len(self.pending) < self.n_cal:
                self.why = f"calibrating the board pose ({len(self.pending)}/{self.n_cal} frames with >= 2 board tags)"
                return None
            self.cal = tr.calibrate(self.pending)
            if self.cal is None:
                self.pending, self.why = [], "board calibration failed"
                return None
            self.ref_corners = {i: np.asarray(self.pending[-1][i], float).reshape(4, 2) for i in self.pending[-1]
                                if str(i) in self.cfg["board"]["tags"]}
            self._save()
        for i in board:
            if i in self.ref_corners:
                sh = float(np.mean(np.linalg.norm(np.asarray(d[i], float).reshape(4, 2) - self.ref_corners[i], axis=1)))
                if sh > self.tol:
                    self.why = f"camera moved: board tag {i} is {sh:.1f} px from its calibrated place"
                    return None
        r = tr.process(gray, None, d)
        if r.get("x_mm") is None:
            self.why = "no robot torso tag visible"
            return None
        self.why = ""
        return dict(x=r["x_mm"], y=r["y_mm"], h=(None if r.get("ambiguous") else r["heading_deg"]))

    def _save(self):
        import json
        tr = self.tr
        rec = dict(size=list(tr.size), f_px=float(tr.K[0, 0]), rvec=np.ravel(tr.pose[0]).tolist(),
                   tvec=np.ravel(tr.pose[1]).tolist(), t=time.time(),
                   corners={str(i): c.round(2).tolist() for i, c in self.ref_corners.items()})
        if not self.pose_path:
            return
        try:
            tmp = self.pose_path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(rec, f)
            os.replace(tmp, self.pose_path)
        except OSError:
            pass                                    # a read-only dir only costs the shortcut

    def _load(self, d, board, path):
        import json
        try:
            with open(path) as f:
                rec = json.load(f)
        except (OSError, ValueError):
            return False
        tr = self.tr
        if tuple(rec.get("size", ())) != tuple(tr.size):
            return False
        ref = {int(i): np.array(c, float) for i, c in (rec.get("corners") or rec["tag_corners_px"]).items()}
        sh = [float(np.mean(np.linalg.norm(np.asarray(d[i], float).reshape(4, 2) - ref[i], axis=1))) for i in board if i in ref]
        if not sh or max(sh) > self.tol:
            return False
        tr.set_camera(rec["f_px"])
        tr.set_board_pose(np.array(rec["rvec"], float).reshape(3, 1), np.array(rec["tvec"], float).reshape(3, 1))
        self.cal, self.ref_corners = dict(f_px=rec["f_px"], loaded=path), ref
        self.why = ""
        return True

    def margins(self, x, y):
        return self.G.edge_margins(self.cfg, x, y)


# ---- the samples and the phase-consistent fix ------------------------------------------------------------------------

class PosSampler:
    """Time-stamped tracker samples and their windowed median. `locate(gray) -> dict(x, y, h) | None`. poll_once()
    reads DIR/emeet_latest.jpg if it is newer than the last frame used (its mtime is the frame's time) and runs the
    locator; start() polls from a thread. Tests and the fake robot call add() directly."""

    def __init__(self, locate=None, path=None, poll_s=0.2, keep_s=120.0, max_age_s=2.0, clock=time.time):
        self.locate, self.path, self.poll_s, self.keep_s, self.clock, self.max_age = locate, path, poll_s, keep_s, clock, max_age_s
        self.s = collections.deque()
        self.last_mtime = None
        self.why = "no frame yet"
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._th = None
        self.n_frames = self.n_located = 0

    def add(self, t, x, y, h=None):
        with self._lock:
            self.s.append((t, x, y, h))
            while self.s and self.s[0][0] < t - self.keep_s:
                self.s.popleft()

    def poll_once(self):
        import cv2
        try:
            mt = os.path.getmtime(self.path)
        except OSError:
            self.why = f"{self.path}: no frame"
            return False
        if mt == self.last_mtime:
            return False
        if self.clock() - mt > self.max_age:
            self.why = f"newest frame is {self.clock() - mt:.0f} s old (camd stopped?)"
            self.last_mtime = mt
            return False
        self.last_mtime = mt
        gray = cv2.imread(self.path, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            self.why = "unreadable frame"
            return False
        self.n_frames += 1
        r = self.locate(gray)
        self.why = getattr(self.locate, "why", "") if r is None else ""
        if r is None:
            return False
        self.n_located += 1
        self.add(mt, r["x"], r["y"], r["h"])
        return True

    def start(self):
        def run():
            while not self._stop.is_set():
                try:
                    self.poll_once()
                except Exception as e:                       # a bad frame must not end the thread
                    self.why = f"locator error: {e}"
                self._stop.wait(self.poll_s)
        self._th = threading.Thread(target=run, daemon=True)
        self._th.start()

    def stop(self):
        self._stop.set()

    def fix(self, t_end, window_s, min_samples=MIN_SAMPLES):
        """Trimmed-mean position over (t_end - window_s, t_end] -> dict(x, y, h, n) or None (no fix)."""
        with self._lock:
            w = [s for s in self.s if t_end - window_s < s[0] <= t_end]
        if len(w) < min_samples:
            return None
        ts = [s[0] for s in w]
        if ts[-1] - ts[0] < 0.5 * window_s or t_end - ts[-1] > 0.5 * window_s:
            return None
        hs = [s[3] for s in w if s[3] is not None]
        return dict(x=trimmed_mean([s[1] for s in w]), y=trimmed_mean([s[2] for s in w]),
                    h=(circ_median(hs) if len(hs) >= max(2, len(w) // 2) else None), n=len(w))


# ---- the hold ------------------------------------------------------------------------------------------------------

class PosHold:
    def __init__(self, sampler, step_s, margins=None, k=K_DEFAULT, ki=KI_DEFAULT, rate_mm=RATE_MM,
                 clip=(CLIP_LAT_MM, CLIP_FWD_MM), margin_mm=MARGIN_MM,
                 no_fix_steps=NO_FIX_STEPS, window_steps=WINDOW_STEPS, delay_steps=DELAY_STEPS, min_samples=MIN_SAMPLES,
                 deadband_mm=DEADBAND_MM, warmup_steps=WARMUP_STEPS, centre=None, say=print, clock=time.time):
        """margins(x, y) -> {left, right, near, far} mm (None: no edge rule); centre = (x, y) of the board centre."""
        self.S, self.T, self.margins, self.k, self.ki, self.rate = sampler, float(step_s), margins, k, ki, rate_mm
        self.clip_lat, self.clip_fwd = clip
        self.margin_mm = margin_mm
        self.no_fix_steps, self.window_steps, self.delay = int(no_fix_steps), window_steps, int(delay_steps)
        self.min_samples, self.deadband, self.warmup = min_samples, deadband_mm, int(warmup_steps)
        self.centre, self.say, self.clock = centre, say, clock
        self.p0 = None                    # reference start (board mm)
        self.h_cmd = None                 # the commanded heading now (deg)
        self.hist = []                    # per decided step: dict(cmd=(bx, by), app=(bx, by)) board-frame displacements
        self.nofix = 0
        self.trim = (0.0, 0.0)            # last (dx, dy) correction in the robot frame, mm
        self.integ = [0.0, 0.0]           # the integral part of the correction (fwd, lat), mm
        self.rebased = False
        self._pfrac = [0.0, 0.0]          # the share of the last correction that is the proportional part
        self.emeas = []                   # the measured lateral error at each sample, mm
        self.start_pose = self.end_pose = None
        self.max_err = [0.0, 0.0]
        self.sat = 0                      # consecutive steps with the lateral correction at its clip
        self.lat_off = None               # why the lateral correction is off (None = on)
        self.dec = None                   # the decision of the step being decided
        self.log = []

    # -- sampling
    def window_s(self):
        return self.window_steps * self.T

    def start_fix(self, timeout_s):
        """Wait (wall clock) for the first fix: the robot standing, before the walk. -> the fix or None."""
        end = self.clock() + timeout_s
        while True:
            f = self.S.fix(self.clock(), self.window_s(), self.min_samples)
            if f is not None and f["h"] is not None:
                return f
            if self.clock() >= end:
                return None
            time.sleep(0.2)

    def begin(self, f):
        """Take the standing pose `f` (start_fix) as the reference start."""
        self.p0, self.h_cmd, self.hist = (f["x"], f["y"]), f["h"], []
        self.start_pose = (f["x"], f["y"], f["h"])

    def _law(self, ax, e, clip, on):
        """One axis of the control law: -k x (e beyond the dead band) + an integral that learns the standing creep, held
        at half the clip and not wound up while the output is at the clip."""
        if not on:
            return 0.0
        soft = 0.0 if abs(e) <= self.deadband else e - math.copysign(self.deadband, e)
        u = -self.k * soft + self.integ[ax]
        sat = abs(u) >= clip - 1e-9 and u * soft < 0
        if not sat and soft != 0.0:
            self.integ[ax] = _clip(self.integ[ax] - self.ki * soft, clip)
        tot = _clip(-self.k * soft + self.integ[ax], clip)
        self._pfrac[ax] = (-self.k * soft) / tot if abs(tot) > 1e-9 else 0.0       # the share that answers the error
        return tot

    def _sum(self, items, key):
        return (sum(h[key][0] for h in items), sum(h[key][1] for h in items))

    # -- one decision, before CmdWalk._step
    def update(self, j, cmd_active, now=None):
        """Called once per step, as it is decided. Sets self.dec: fix / err / corr / override / notes and returns the
        reason to stop (a string) or None."""
        now = self.clock() if now is None else now
        f = self.S.fix(now, self.window_s(), self.min_samples)
        d = dict(step=j + 1, fix=None, err=None, corr=(0.0, 0.0), override=None, notes=[], frozen=False)
        self.dec = d
        if f is not None and f["h"] is None and self.h_cmd is not None:
            f = dict(f, h=self.h_cmd)                       # heading ambiguous in the window: the commanded one stands in
            d["notes"].append("heading from the command (tag pose ambiguous)")
        if f is None or (self.p0 is None and f["h"] is None):
            self.nofix += 1
            d["frozen"] = True
            why = self.S.why or "too few samples in the window"
            if self.nofix <= self.no_fix_steps and self.p0 is not None:
                d["corr"] = self.trim
                d["notes"].append(f"NO FIX ({self.nofix} steps; {why}): correction frozen at the last trim "
                                  f"dx {self.trim[0]:+.1f} dy {self.trim[1]:+.1f} mm")
            else:
                self.trim = (0.0, 0.0)
                d["notes"].append(f"NO FIX ({self.nofix} steps; {why}): correction off")
            if cmd_active and self.nofix > self.no_fix_steps:
                return f"position tracker has had no fix for {self.nofix} steps with a command active ({why})"
            return None
        self.nofix = 0
        d["fix"] = f
        if self.p0 is None:                                 # the start pose: the robot standing
            self.p0, self.h_cmd = (f["x"], f["y"]), f["h"]
            self.hist = []
            d["notes"].append(f"hold started at x {f['x']:.0f} y {f['y']:.0f} heading {f['h']:.0f}")
        n_pend = min(self.delay, len(self.hist))
        seen, pend = self.hist[:len(self.hist) - n_pend], self.hist[len(self.hist) - n_pend:]
        h = f["h"]
        ps = self._sum(pend, "app")
        # the pending corrections that will show in the position: only their proportional part (the answer to the error);
        # the integral part is the learned creep canceller, and the creep acts on those same steps too
        pc = (sum(x["pcorr"][0] for x in pend), sum(x["pcorr"][1] for x in pend))

        def ref_at(items):
            c = self._sum(items, "cmd")
            return (self.p0[0] + c[0], self.p0[1] + c[1])
        if not self.rebased and len(self.hist) >= self.warmup:
            # the mean position over a walking cycle is not the standing pose (the torso sits differently over the feet
            # in the walk): hold what the walk does, from here. The standing pose stays the start for the log.
            r = ref_at(seen)
            off = (f["x"] - r[0] + pc[0], f["y"] - r[1] + pc[1])
            self.p0 = (self.p0[0] + off[0], self.p0[1] + off[1])
            self.rebased = True
            of, ol = to_robot(h, *off)
            d["notes"].append(f"reference re-based to the walking mean (fwd {of:+.0f}, lat {ol:+.0f} mm from the standing pose)")
        ref_seen, ref_now = ref_at(seen), ref_at(self.hist)
        pred = (f["x"] + ps[0], f["y"] + ps[1])                        # the sample plus the steps decided since
        # error now = measured error at the sample + the corrections still on their way (the commanded part of a
        # pending step is in both the reference and the prediction)
        e_meas = to_robot(h, f["x"] - ref_seen[0], f["y"] - ref_seen[1])
        e_b = (f["x"] - ref_seen[0] + pc[0], f["y"] - ref_seen[1] + pc[1])
        e_fwd, e_lat = to_robot(h, *e_b)
        d["err"] = (e_lat, e_fwd)
        d["pos"] = (f["x"], f["y"], h)
        d["ref"] = ref_now
        d["pred"] = pred
        walking = len(self.hist) >= self.warmup                        # the first window still holds the standing robot
        if walking:
            self.emeas.append(e_meas[1])
            self.max_err = [max(self.max_err[0], abs(e_lat)), max(self.max_err[1], abs(e_fwd))]
        # does a sideways correction move the robot sideways? (the pause timing on 2026-11-16 moved it the wrong way)
        # Over the last WORSE_W steps the corrections pushed one way against the error, and the measured error (3-sample
        # means, then and now: the sample jitters ~10 mm) still grew. Or the corrections sit at their clip and the error is
        # far past anything a working loop settles at.
        if self.lat_off is None and walking and len(self.emeas) >= WORSE_W + 3:
            then, now_ = sum(self.emeas[-WORSE_W - 3:-WORSE_W]) / 3.0, sum(self.emeas[-3:]) / 3.0
            sg = 1.0 if now_ >= 0 else -1.0
            cl = [to_robot(x["h"], x["app"][0] - x["cmd"][0], x["app"][1] - x["cmd"][1])[1] for x in self.hist[-WORSE_W:] if "h" in x]
            mc = sum(cl) / len(cl) if cl else 0.0
            if sg * (now_ - then) >= WORSE_MM and -sg * mc >= WORSE_CORR_MM:
                self.lat_off = (f"over the last {WORSE_W} steps the corrections averaged {mc:+.1f} mm against a lateral error that "
                                f"went from {then:+.0f} to {now_:+.0f} mm: the sideways response is weak or wrong-signed")
        if self.lat_off is None and walking and self.sat >= SAT_STEPS and abs(e_lat) >= SAT_ERR_MM:
            self.lat_off = (f"the lateral error is {e_lat:+.0f} mm after {self.sat} steps of corrections at the {self.clip_lat:g} mm "
                            f"clip: they do not bring it back (response weak or wrong-signed)")
        cx, cy = self._law(0, e_fwd, self.clip_fwd, walking), self._law(1, e_lat, self.clip_lat, walking and self.lat_off is None)
        if self.lat_off is not None:
            cy = 0.0
            self.integ[1] = 0.0
            d["notes"].append("LATERAL CORRECTION OFF: " + self.lat_off)
        # the correction may not jump: at most self.rate mm from the last one
        cx = self.trim[0] + _clip(cx - self.trim[0], self.rate)
        cy = self.trim[1] + _clip(cy - self.trim[1], self.rate)
        at_clip = abs(cy) >= self.clip_lat - 1e-9
        self.sat = self.sat + 1 if at_clip and (self.sat == 0 or cy * self.trim[1] > 0) else (1 if at_clip else 0)
        if not walking:
            cx = cy = 0.0
            d["notes"].append(f"settling ({len(self.hist)}/{self.warmup} steps): no correction yet")
        d["corr"] = (cx, cy)
        d["pfrac"] = tuple(self._pfrac)
        self.trim = (cx, cy)
        # keep on board
        if self.margins is not None and self.margin_mm is not None and self.centre is not None:
            m = self.margins(*pred)
            d["margins"] = m
            near = [e for e, v in m.items() if v < self.margin_mm]
            if near:
                ux = (self.centre[0] - pred[0]) if any(e in near for e in ("left", "right")) else 0.0
                uy = (self.centre[1] - pred[1]) if any(e in near for e in ("near", "far")) else 0.0
                n = math.hypot(ux, uy) or 1.0
                fwd, left = to_robot(h, ux / n, uy / n)
                d["override"] = dict(edges=sorted(near), fwd=fwd, left=left,
                                     min_margin=min(m.values()))
                d["corr"] = (0.0, 0.0)
        return None

    # -- the step actually decided
    def commit(self, cmd, applied, dpsi):
        """cmd / applied: (dx, dy) mm in the robot frame of the step (the commanded part, and what was uploaded);
        dpsi: its heading change, deg. Integrates the reference."""
        if self.p0 is None:
            return
        hm = self.h_cmd + 0.5 * dpsi
        dec = self.dec or {}
        pf = dec.get("pfrac", (0.0, 0.0))
        pc = to_board(hm, (applied[0] - cmd[0]) * pf[0], (applied[1] - cmd[1]) * pf[1])
        self.hist.append(dict(cmd=to_board(hm, *cmd), app=to_board(hm, *applied), h=hm, pcorr=pc))
        self.h_cmd += dpsi
        d = self.dec
        d["applied"] = tuple(applied)
        d["cmd_part"] = tuple(cmd)
        d["applied_corr"] = (applied[0] - cmd[0], applied[1] - cmd[1])
        self.log.append(d)

    def record(self, d=None):
        """The JSON-able row of the summary log for one step."""
        d = d or self.dec
        r = dict(fix=d.get("fix") is not None, notes=list(d["notes"]), frozen=d["frozen"])
        if d.get("fix") is not None:
            r.update(x=round(d["pos"][0], 1), y=round(d["pos"][1], 1), h=round(d["pos"][2], 1), n=d["fix"]["n"],
                     ref=[round(v, 1) for v in d["ref"]], err_lat=round(d["err"][0], 2), err_fwd=round(d["err"][1], 2),
                     corr_dy=round(d["corr"][1], 2), corr_dx=round(d["corr"][0], 2))
        if "applied_corr" in d:
            r.update(applied_dx=round(d["applied_corr"][0], 2), applied_dy=round(d["applied_corr"][1], 2))
        if d.get("override"):
            r["keep_on_board"] = dict(edges=d["override"]["edges"], min_margin=round(d["override"]["min_margin"], 1))
        return r

    def describe(self, d=None):
        d = d or self.dec
        if d is None:
            return ""
        if d.get("fix") is None:
            return "pos NO FIX: " + "; ".join(d["notes"])
        x, y, h = d["pos"]
        el, ef = d["err"]
        cx, cy = d["applied_corr"] if "applied_corr" in d else d["corr"]
        s = (f"pos x {x:.0f} y {y:.0f} h {h:.0f} (n {d['fix']['n']}; ref {d['ref'][0]:.0f} {d['ref'][1]:.0f}, "
             f"err lat {el:+.1f} fwd {ef:+.1f} mm) -> corr dy {cy:+.1f} dx {cx:+.1f} mm")
        if d["override"]:
            o = d["override"]
            s += (f"; KEEP-ON-BOARD: {'/'.join(o['edges'])} margin {o['min_margin']:.0f} < {self.margin_mm:.0f} mm: "
                  f"step toward the centre (fwd {o['fwd']:+.2f} left {o['left']:+.2f})")
        return s + "".join(f"; {n}" for n in d["notes"])

    def finish(self, wait_s=None):
        """After the walk: the pose at rest (the trimmed mean over the last step), waiting up to wait_s of wall clock for
        the samples of a robot standing still."""
        end = self.clock() + (1.2 * self.T + 1.0 if wait_s is None else wait_s)
        while self.clock() < end:
            time.sleep(0.2)
        f = self.S.fix(self.clock(), self.T, self.min_samples)
        if f is not None and f["h"] is not None:
            self.end_pose = (f["x"], f["y"], f["h"])
        return self.end_pose

    def summary(self):
        out = dict(steps=len(self.log), no_fix_steps=sum(1 for d in self.log if d.get("fix") is None),
                   overrides=sum(1 for d in self.log if d.get("override")), lateral_off=self.lat_off,
                   start_at_rest=[round(v, 1) for v in self.start_pose] if self.start_pose else None,
                   end_at_rest=[round(v, 1) for v in self.end_pose] if self.end_pose else None,
                   max_abs_err_lat_mm=round(self.max_err[0], 1), max_abs_err_fwd_mm=round(self.max_err[1], 1))
        errs = [d["err"] for d in self.log if d.get("err")]
        if errs:
            out["err_lat_mm"] = [round(min(e[0] for e in errs), 1), round(max(e[0] for e in errs), 1)]
            out["err_fwd_mm"] = [round(min(e[1] for e in errs), 1), round(max(e[1] for e in errs), 1)]
        return out
