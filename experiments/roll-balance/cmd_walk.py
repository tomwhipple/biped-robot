"""Live velocity commands for the host's dwell gait (#297): `track_host.py --cmd SOURCE`.

The walk has no fixed length. Every step the host reads the CURRENT command -- vx mm/s forward, vy mm/s left, wz
deg/s left, or the words `stop` / `pause` -- and plans the next not-yet-uploaded step from it: the step's
displacement is the command x T, clamped, slewed and checked before one frame of it is uploaded. The rolling upload,
the heading hold, the stop rules and the lift gate are track_host's, unchanged; this module decides the steps.

Why a step can be decided late. The frames of step j up to its lift-off (the shift over the stance foot and the
settle, both feet down) depend on the steps before it only; its own displacement first shows in the frame after the
lift-off. So the plan always carries one provisional step (the last command repeated) and the host holds back the
first line that reaches past that step's lift-off until the step is decided: `CmdWalk.limit_ms()`. With the host
`--ahead 1` step in front of the play head the decision for step j is taken when the head reaches the lift-off of
step j-1 (README: the latency).

The admissible set of one step is the pre-check's (`track_host.precheck(only_step=)`), run on a two-step window
(the previous step and the candidate): a step's frames depend on the previous step only, through the swing foot's
start (-dx of the previous step from the stance foot) and the hip yaw it left, so the window reproduces them.
"""
from __future__ import annotations

import json
import math
import os
import select
import time

# ---- the envelope (README: the table) -------------------------------------------------------------------------
FWD_MM, BACK_MM, SIDE_MM, YAW_DEG = 50.0, 32.0, 15.0, 5.0       # per step; the proven strides (README: the envelope table)
SLEW = (20.0, 10.0, 3.0)         # max change of (dx mm, dy mm, dyaw deg) from one step to the next
PAUSE_STEPS = 15                 # marching in place longer than this many steps in a row ends the walk
# swing tilt from the farther of this step's |dx| and the last one's (the swing foot starts at -dx of the last step):
# 0 up to 26 mm (27 passes the pre-check at 2.1 deg), 3 deg up to 43, 5 deg beyond (proven to 50 mm)
TILT = ((26.0, 0.0), (43.0, 3.0), (1e9, 5.0))


def tilt_for(mm):
    for lim, t in TILT:
        if mm <= lim + 1e-9:
            return t
    return TILT[-1][1]


# ---- the command grammar and its sources --------------------------------------------------------------------------

def parse_command(text):
    """One line -> ("vel", (vx, vy, wz)) | ("stop",) | ("pause",) | None (empty or unparseable)."""
    line = (text or "").split("#")[0].strip().lower().replace(",", " ")
    if not line:
        return None
    if line in ("stop", "pause"):
        return (line,)
    parts = line.split()
    if len(parts) != 3:
        return None
    try:
        v = tuple(float(x) for x in parts)
    except ValueError:
        return None
    if not all(math.isfinite(x) for x in v):
        return None
    return ("vel", v)


class FileSource:
    """A small text file, re-read at every poll; its first non-empty line is the command. A missing or unparseable
    file is no news (the last command stays)."""
    closed = False

    def __init__(self, path):
        self.path, self.error = path, None

    def read(self):
        try:
            with open(self.path) as f:
                txt = f.read(4096)
        except OSError as e:
            self.error = f"{self.path}: {e.strerror}"
            return None
        for ln in txt.splitlines():
            if ln.split("#")[0].strip():
                c = parse_command(ln)
                self.error = None if c else f"unparseable {ln.strip()[:40]!r}"
                return c
        self.error = "empty"
        return None

    poll = read


class StdinSource:
    """Lines on standard input, read without blocking; the newest parseable one is the command. EOF = closed."""

    def __init__(self, fd=0):
        self.fd, self.buf, self.closed, self.error = fd, b"", False, None

    def poll(self):
        new = None
        while not self.closed:
            try:
                r, _, _ = select.select([self.fd], [], [], 0)
            except (OSError, ValueError):
                self.closed = True
                break
            if not r:
                break
            try:
                data = os.read(self.fd, 4096)
            except OSError:
                data = b""
            if not data:
                self.closed = True
                break
            self.buf += data
        *lines, self.buf = self.buf.split(b"\n")
        for ln in lines:
            c = parse_command(ln.decode("utf8", "replace"))
            if c is not None:
                new = c
            elif ln.strip():
                self.error = f"unparseable {ln.strip()[:40]!r}"
        return new


class ListSource:
    """Scripted: the i-th poll returns commands[i] (a string); past the end the last repeats as no news. For tests,
    the dry run and the SIL timeline."""
    closed = False
    error = None

    def __init__(self, commands):
        self.cmds, self.i = list(commands), 0

    def poll(self):
        if self.i >= len(self.cmds):
            return None
        c = parse_command(self.cmds[self.i])
        self.i += 1
        return c


def make_source(spec):
    if spec == "stdin":
        return StdinSource()
    if spec.startswith("file:") and len(spec) > 5:
        return FileSource(spec[5:])
    raise ValueError(f"--cmd wants file:PATH or stdin, got {spec!r}")


def _write_atomic(path, text):
    tmp = f"{path}.tmp{os.getpid()}"
    with open(tmp, "w") as f:
        f.write(text)
    os.replace(tmp, path)


# ---- the walk ---------------------------------------------------------------------------------------------------------

class CmdWalk:
    """The steps decided so far, the command in force, and the rules. `host` is the track_host module (the one
    running: its plan / precheck, so a test's spy sees every plan)."""

    def __init__(self, host, source, base, first="R", hold=None, limits=None, max_steps=120, max_time=None,
                 status=None, say=print, clock=time.monotonic, fixed_tilt=0.0, yaw_trim=None, allow_over=False,
                 ankle_side=None, first_boost=0.0, drift_side=None, step_s=None):
        """base: the plan() keyword arguments shared by every plan of the walk (T_step, seeds, lead_ms, cal, amp, model,
        ds, h, pelvis_x, ankle_ff, k_ds, drop, lift_gate; direction, n_steps and first are set here).
        ankle_side, first_boost, drift_side: the by-step feed-forwards (track_host.ankle_extra / step_drift); None / 0 =
        off. Each step's ankle extra is decided here from its absolute index (a command's "aff_x"), so the two-step
        check window sees exactly what the walk's plan will."""
        self.H, self.src, self.hold, self.say, self.clock = host, source, hold, say, clock
        self.base = dict(base)
        self.T = float(base["T_step"] if step_s is None else step_s)       # the step time: a command's displacement is v x T
        self.first, self.status_path = first, status
        lim = dict(fwd=FWD_MM, back=BACK_MM, side=SIDE_MM, yaw=YAW_DEG, slew=SLEW, pause=PAUSE_STEPS)
        lim.update(limits or {})
        self.lim = lim
        self.max_steps, self.max_time = int(max_steps), max_time
        self.fixed_tilt = float(fixed_tilt)
        self.yaw_trim = host.YAW_TRIM if yaw_trim is None else yaw_trim
        self.steps = []                                  # decided steps (command dicts, plan()'s step_cmds)
        self.log = []                                    # one record per decision
        self.cur = ("vel", (0.0, 0.0, 0.0))              # the command in force
        self.last = (0.0, 0.0, 0.0)                      # the displacement of the last decided step: mm, mm, deg
        self.fy = {"L": 0.0, "R": 0.0}                   # the feet's lateral positions (the reversal rule)
        self.finished, self.reason = False, None
        self.stop_req = None
        self.pause_n = 0
        self.t_begin = None
        self.sway = None
        self.allow_over = allow_over
        self.ankle_side, self.first_boost, self.drift_side = ankle_side, float(first_boost or 0.0), drift_side
        self.fail = None
        if self.hold is not None:
            self.hold.set_step(0, 0.0)                   # the provisional step 0

    # -- plans
    def swing_of(self, i):
        return self.first if i % 2 == 0 else ("R" if self.first == "L" else "L")

    def provisional(self):
        """The step after the decided ones, as the plan carries it until it is decided: the last one repeated (its
        frames up to its lift-off do not depend on it)."""
        if self.steps:
            return dict(self.steps[-1], close=False)
        return self._cmd(0.0, 0.0, 0.0, 0.0)

    def _cmd(self, dx, dy, dyaw, prev_dx_mm, name=None, **kw):
        tilt = self.fixed_tilt or tilt_for(max(abs(dx), abs(prev_dx_mm)))
        return dict(name=name or f"cmd({dx:g},{dy:g},{dyaw:g})", key="cmd", dx=dx / 1e3, dy=dy / 1e3,
                    yt=dyaw / self.H.YAW_GAIN, dpsi=float(dyaw), seg=0, tilt=tilt, **kw)

    def _aff(self, i, c):
        """Step i's command with its ankle extra (deg on its stance ankle)."""
        return dict(c, aff_x=self.H.ankle_extra(i, c["dy"], self.first, self.ankle_side, self.first_boost))

    def _drift(self, c):
        return self.H.step_drift(c, self.drift_side)

    def plan_kwargs(self):
        """plan()'s keywords for the whole walk so far: the decided steps and, until it is over, the provisional one."""
        cmds = list(self.steps) + ([] if self.finished else [self.provisional()])
        cmds = [self._aff(i, c) for i, c in enumerate(cmds)]
        kw = dict(self.base, direction="march", n_steps=len(cmds), step_cmds=cmds, first=self.first,
                  drift_side=self.drift_side)
        kw["yaw_trim"] = list(self.hold.asked[:len(cmds)]) if self.hold is not None else self.yaw_trim
        return kw

    def on_plan(self, pl):
        self.sway = pl["sway"]

    def limit_ms(self):
        """Frames up to this track time (ms) are the same whatever the first undecided step turns out to be."""
        if self.finished or self.sway is None:
            return float("inf")
        sw, j = self.sway, len(self.steps)
        return round(1e3 * (sw.phases[j]["t0"] - sw.t_first))

    # -- the check of one step (a two-step window)
    def _yaw_of(self, i, c):
        if self.hold is not None:
            return self.hold.asked[i] if i < len(self.hold.asked) and i < len(self.steps) else \
                (0.0 if c.get("yaw_free") else self.hold.asked_for(c["dpsi"], self._drift(c)))
        if self.drift_side is None:
            return c["yt"] + (0.0 if c.get("yaw_free") else self.yaw_trim)
        return c["yt"] + (0.0 if c.get("yaw_free") else self.yaw_trim * self._drift(c) / self.H.YAW_DRIFT)

    def check(self, cand):
        """(ok, why): the pre-check's rules on the candidate step after the last decided one."""
        j = len(self.steps)
        prev = self.steps[-1] if self.steps else None
        cmds = ([self._aff(j - 1, prev)] if prev else []) + [self._aff(j, cand)]
        kw = dict(self.base, direction="march", n_steps=len(cmds), step_cmds=cmds, cal=None, lift_gate=False,
                  first=self.swing_of(j - 1) if prev else self.first,
                  yaw_trim=([self._yaw_of(j - 1, prev)] if prev else []) + [self._yaw_of(j, cand)])
        try:
            pl = self.H.plan(**kw)
        except ValueError as e:
            return False, f"kinematics ({e})"
        pl["allow_over"] = self.allow_over
        ok, rows = self.H.precheck(pl, only_step=len(cmds) - 1)
        if ok:
            return True, ""
        bad = [r for r in rows if not r["ok"]]
        return False, ", ".join(sorted({r["at"] if r["at"] != "joint limit" else f"joint limit {r['stance']}"
                                        for r in bad}))

    # -- deciding
    def request_stop(self, why):
        if self.stop_req is None:
            self.stop_req = why

    def _refresh(self):
        new = self.src.poll()
        if new is not None:
            self.cur = new
        return self.src.error

    def decide(self):
        """Step len(steps) is about to be uploaded: read the command, fix the step. Returns the record."""
        if self.t_begin is None:
            self.t_begin = self.clock()
        j = len(self.steps)
        err = self._refresh()
        kind = self.cur[0]
        why = self.stop_req
        if why is None and getattr(self.src, "closed", False):
            why = "command source closed"
        if why is None and kind == "stop":
            why = "stop command"
        if why is None and j >= self.max_steps:
            why = f"--max-steps {self.max_steps}"
        if why is None and self.max_time and self.clock() - self.t_begin > self.max_time:
            why = f"--max-time {self.max_time:g} s"
        if why is None:
            rec = self._step(j, err)
            if rec is not None:
                return rec
            why = self.fail or "no admissible step"
        return self._final(j, why)

    def _final(self, j, why):
        """The last step: no displacement, no heading trim -- the swing foot comes alongside the stance foot, the hips
        square, the feet end together (the same motion as dwell's closing step)."""
        c = self._cmd(0.0, 0.0, 0.0, abs(self.last[0]), name="stop", yaw_free=True, close=True)
        ok, bad = self.check(c)
        notes = [f"stop: {why}"]
        if not ok:
            notes.append(f"WARNING the closing step fails the pre-check ({bad}); sent anyway: it replaces a half-uploaded step")
        self.steps.append(c)
        self.finished, self.reason = True, why
        if self.hold is not None:
            self.hold.set_step(j, 0.0, asked=0.0)
        self.last = (0.0, 0.0, 0.0)
        self._lateral(j, 0.0)
        return self._record(j, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), notes, "stopping")

    def _lateral(self, j, dy_mm):
        sw = self.swing_of(j)
        st = "R" if sw == "L" else "L"
        out = (dy_mm > 0 and sw == "L") or (dy_mm < 0 and sw == "R")
        self.fy[sw] = self.fy[st] + (dy_mm / 1e3 if out else 0.0)

    def _step(self, j, err):
        T, lim = self.T, self.lim
        notes = []
        if err:
            notes.append(f"command source: {err}: keeping the last")
        if self.cur[0] == "pause":
            req = (0.0, 0.0, 0.0)
            notes.append("pause: marching in place")
        else:
            req = self.cur[1]
        want = (req[0] * T, req[1] * T, req[2] * T)                       # mm, mm, deg per step
        # 1. the envelope: one factor for all three
        f = 1.0
        for v, cap in ((want[0], lim["fwd"] if want[0] > 0 else lim["back"]), (want[1], lim["side"]), (want[2], lim["yaw"])):
            if abs(v) > cap + 1e-9:
                f = min(f, cap / abs(v))
        d1 = tuple(f * v for v in want)
        if f < 1.0:
            notes.append(f"envelope: x{f:.2f}")
        # 2. the slew: from the last step's displacement toward d1, as far as the per-step change limits allow
        lam = 1.0
        for a, b, s in zip(self.last, d1, lim["slew"]):
            if abs(b - a) > s + 1e-9:
                lam = min(lam, s / abs(b - a))
        d2 = tuple(a + lam * (b - a) for a, b in zip(self.last, d1))
        if lam < 1.0:
            notes.append(f"slew: {lam:.2f} of the way from the last step")
        # 3. a lateral reversal with the feet apart: a closing step first (the --seq rule). The swing foot's lateral
        # travel in this step is its goal (an out-step lands dy outside the stance foot, any other step alongside it)
        # less where it stands; a step whose foot would travel AGAINST the commanded direction is the reversal --
        # `gap x dy < 0`, the rule at a --seq boundary, also fires on a sideways walk that goes on (the foot that
        # followed lands alongside, as it should) and would turn it into out, close, out, close.
        sw = self.swing_of(j)
        st = "R" if sw == "L" else "L"
        out = (d2[1] > 0 and sw == "L") or (d2[1] < 0 and sw == "R")
        travel = self.fy[st] + (d2[1] / 1e3 if out else 0.0) - self.fy[sw]
        if d2[1] * travel < -1e-9:
            c = self._cmd(0.0, 0.0, 0.0, abs(self.last[0]), name="close", close=True)
            ok, bad = self.check(c)
            notes.append("lateral reversal with the feet apart: closing step (the command waits one step)")
            if ok:
                return self._accept(j, c, (0.0, 0.0, 0.0), req, notes)
            d2 = (0.0, 0.0, 0.0)
        # 4. the pre-check, on this very step
        d3 = self._admissible(d2, notes)
        if d3 is None:
            self.fail = "even a step in place fails the pre-check"
            return None
        if self.cur[0] == "pause" or all(abs(x) < 1e-9 for x in d3):
            self.pause_n += 1
            if self.pause_n > lim["pause"] > 0:
                self.fail = f"marching in place for more than {lim['pause']} steps"
                return None
        else:
            self.pause_n = 0
        c = self._cmd(round(d3[0], 2), round(d3[1], 2), round(d3[2], 3), abs(self.last[0]))
        return self._accept(j, c, d3, req, notes)

    def _admissible(self, d, notes):
        """d (mm, mm, deg) if the step passes, else the largest fraction of the way from the last step toward d that does
        (then from the last step toward zero); None if even a step in place fails."""
        def ok_(v):
            c = self._cmd(round(v[0], 2), round(v[1], 2), round(v[2], 3), abs(self.last[0]))
            return self.check(c)
        ok, why = ok_(d)
        if ok:
            return d
        prev = self.last
        for anchor, tag in ((prev, "the last step"), ((0.0, 0.0, 0.0), "zero")):
            mix = lambda u, a=anchor: tuple(x + u * (y - x) for x, y in zip(a, d))      # noqa: E731
            if not ok_(mix(0.0))[0]:
                continue
            lo, hi = 0.0, 1.0
            for _ in range(8):
                mid = 0.5 * (lo + hi)
                if ok_(mix(mid))[0]:
                    lo = mid
                else:
                    hi = mid
            notes.append(f"pre-check ({why}): {lo:.2f} of the way from {tag}")
            return mix(lo)
        notes.append(f"pre-check ({why}): even a step in place fails")
        return None

    def _accept(self, j, c, d, req, notes):
        self.steps.append(c)
        self.last = (c["dx"] * 1e3, c["dy"] * 1e3, c["dpsi"])
        self._lateral(j, c["dy"] * 1e3)
        if self.hold is not None:
            self.hold.set_step(j, c["dpsi"], drift=self._drift(c))
            self.hold.set_step(j + 1, c["dpsi"], drift=self._drift(c))         # the provisional step
        return self._record(j, self.last, req, notes, "marching" if all(abs(x) < 1e-9 for x in self.last) else "walking")

    def _record(self, j, applied_step, req, notes, state):
        T = self.T
        rec = dict(step=j + 1, state=state, requested=[round(x, 3) for x in req],
                   applied=[round(x / T, 3) for x in applied_step], step_mm_deg=[round(x, 2) for x in applied_step],
                   tilt=self.steps[-1].get("tilt", 0.0), notes=notes, time=time.time(), T=T)
        self.log.append(rec)
        self._say(rec)
        self._status(rec)
        return rec

    def _say(self, rec):
        r, a, s = rec["requested"], rec["applied"], rec["step_mm_deg"]
        self.say(f"  cmd step {rec['step']}: asked {r[0]:+.2f} {r[1]:+.2f} {r[2]:+.2f} (mm/s, mm/s, deg/s) -> "
                 f"{rec['state']}: dx {s[0]:+.1f} mm, dy {s[1]:+.1f} mm, dyaw {s[2]:+.2f} deg per step "
                 f"({a[0]:+.2f} {a[1]:+.2f} {a[2]:+.2f}), tilt {rec['tilt']:g}"
                 + "".join(f"; {n}" for n in rec["notes"]), flush=True)

    def _status(self, rec, state=None):
        if not self.status_path:
            return
        try:
            _write_atomic(self.status_path, json.dumps(dict(rec, state=state or rec["state"],
                                                            command=list(self.cur[1]) if self.cur[0] == "vel" else self.cur[0],
                                                            limits=dict(fwd=self.lim["fwd"], back=self.lim["back"],
                                                                        side=self.lim["side"], yaw=self.lim["yaw"]))) + "\n")
        except OSError:
            pass

    def ended(self, why):
        """The walk is over (the track ended, a stop rule, an error): the status says so."""
        if self.log:
            self._status(self.log[-1], state=f"ended: {why}")

    # -- a dry run / the SIL: decide n steps against a scripted source, then stop
    def rehearse(self, n):
        while not self.finished:
            if len(self.steps) >= n:
                self.request_stop(f"dry run: {n} steps")
            self.decide()
        return self.steps
