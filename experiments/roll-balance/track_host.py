"""Host side of the on-board gait track (#297 B2): the sway gait as `track add` lines, uploaded ROLLING
while the robot plays them, with the per-step telemetry (the firmware's TS lines) and a stop rule.

DRY RUN BY DEFAULT: prints the session (the CLI lines, in order), the upload sizing and the SIL prediction
for this gait (results/sil_gait.json). `--live` sends it to the robot over the Wi-Fi CLI
(experiments/stepper/robot_io.py) -- only on the Stage C plan, a person at the robot.

The session:
    trim T L <seed>, trim T R <seed>     the learned trims (default: the robot's own, L 5.2, R 3.4 deg)
    track clear
    track add ... (the first `--ahead` steps)
    track play <lead_ms>                  min-jerk to the first frame (the crouch), then play
    ... track add as the play head moves (the host stays `--ahead` steps in front; `track` reads the head)
    the timeline ends at the last step end: the queue runs dry there, the firmware rests (= track stop)
Stop rule: a TS line with ABORT, a lean toward the swing foot past --stop-lean deg, or an OUTWARD lean (away from
the swing foot) past --stop-out deg -> `track stop`
(finish the step, rest pose); a refused line or a lost link -> `track halt` (freeze, the trim fades).
Lift gate (--lift-gate, default on for the dwell gait): the last frame before each lift-off carries kGate; the firmware
parks the clock there until the body is level and quiet (a held clock stops `play t`: the upload is paced by the play
head, so it pauses with it), and after a 3 s hold ends the track itself, both feet down -- `TS .. GATE-TIMEOUT`, which
the host treats as a stop, not a fall. Each TS line of a gated step ends with ` G <ms held>`.

    python track_host.py --T 4 --dir march --steps 6              # dry run: what would be sent
    python track_host.py --T 4 --dir march --steps 6 --live       # on the robot (Stage C only)
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import signal
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
import sil_gait as G  # noqa: E402

# The robot's own working values on the hard panels, 2026-11-12 (B1 runs fwtrim_*): static shift 1.3x over
# the left foot, 1.25x over the right (1.15x tipped in at foot-off), learned T L 5.2, R 3.4 deg. The SIL holds
# with them at T = 4 and 2 s in every direction (results/sil_gait_robotamp.json).
ROBOT_AMP = (1.3, None)          # k_static L, R; R None = dwell.k_static_R(T): 1.30 at every T
ROBOT_SEED = (5.2, 3.4)
# The robot's own heading drift: +1.9 deg/step to its LEFT with no turn asked (hw 2026-11-12, every direction),
# and 1.34 deg of heading per deg asked. -1.4 deg/step cancels it (fwd 8 at T 4: +7 deg over the run with it,
# 30-40 deg by camera over the day without). Added to every direction's own turn.
YAW_TRIM = -1.4
# The dwell gait's stance-ankle feed-forward and pelvis fore-aft (hw 2026-11-12 19:10-19:28, PR #304): 2 deg outward
# on each stance ankle turned fwd / back at T 5 and T 4 from "tips most right steps" into clean walks (T 5: 35 of
# 36 steps clean; T 4: 18 of 18); 4 deg on R at T 3.4 leaned OUTWARD 5-8 deg and fell. pelvis_x "stance" ran
# every one of those walks.
ROBOT_ANKLE_FF = (2.0, 2.0)
ROBOT_PELVIS_X = "stance"
# the dwell gait's timing per step time is dwell.SCHEDULE (from the SIL: dwell_grid.py); T below dwell.T_FLOOR is
# refused unless --below-floor
MAX_AHEAD_S = 8.0     # the robot's queue holds 128 frames (12.8 s at 10 Hz): stay <= 8 s ahead
PER_LINE = 4                  # frames per `track add` (42 hex + space each; the CLI line is <= 192 chars)
TS_RE = re.compile(r"^TS (\d+) ([LR]) in ([-\d.]+) out ([-\d.]+) e ([-+\d.]+) T ([-+\d.]+) hike ([-+\d.]+) (ok|ABORT)"
                   r"( Tback)?(?: G (\d+))?", re.M)
# the lift gate's timeout (gaitplay/lift_gate.h): the firmware ended the track itself, both feet down -- a stop, not a fall
GATE_TO_RE = re.compile(r"^TS (\d+) ([LR]) GATE-TIMEOUT held (\d+) ms dev ([-+\d.]+) rate ([-+\d.]+)", re.M)


def lines_for(frames):
    """[(t_ms of the line's last frame, 'track add ...')], 4 frames a line, in order."""
    out = []
    for i in range(0, len(frames), PER_LINE):
        chunk = frames[i:i + PER_LINE]
        out.append((chunk[-1][0], "track add " + " ".join(h for _, h, _ in chunk)))
    return out


# Per-step commands (dx mm forward, dy mm left, dyaw deg heading change): the --dir presets are the fixed ones in
# sil_gait.DIRS; --stride-mm / --side-mm scale them; --step "dx,dy,dyaw" is an arbitrary one; --seq strings them.
STRIDE_MM, SIDE_MM = 20.0, 15.0            # the presets' step lengths (sil_gait.DIRS: dx 0.02, dy 0.015)
PRESET_DPSI = {"turn": 5.0, "turnL": 5.0, "turnR": -5.0}   # the heading hold's commanded deg/step for the turn presets
YAW_GAIN, YAW_DRIFT = 1.34, 1.9            # the robot's heading per step = YAW_GAIN x asked + YAW_DRIFT (deg; + = left)
PELVIS_V_MAX = 0.252                       # m/s: the fastest pelvis move in the shift the SIL holds (161 mm in 1.2 s)


# --cmd mode's by-step feed-forwards (2026-11-16, README "By step type"): all off outside --cmd unless their flag is given.
ANKLE_SIDE_CMD = (2.0, 3.0)         # --ankle-ff-side L,R: the stance ankle's ff on a sideways step toward its own side
ANKLE_SIDE_OFF = (2.0, 2.0)         # = ROBOT_ANKLE_FF: no change
FIRST_BOOST_CMD = 1.0               # --first-ankle-boost: deg added to the first stance step of each side
DRIFT_SIDE_CMD = (0.94, YAW_DRIFT)  # --heading-drift-side L,R: the heading drift (deg/step, + = left) of a sideL / sideR step
DRIFT_SIDE_OFF = (YAW_DRIFT, YAW_DRIFT)
SIDE_BLEND_MM = 8.0                 # the heading drift is the sidestep's from this much lateral travel on, blended below (a 3 mm correction
                                    # on a march in place must not switch it: 1.9 -> 0.94 deg/step for a left one)
SIDE_REF_MM = 15.0                  # the lateral step at which the ankle ff reaches its --ankle-ff-side value


def parse_pair(txt, name, lo=-10.0, hi=10.0):
    v = [float(x) for x in str(txt).split(",")]
    if len(v) != 2 or not all(lo <= x <= hi for x in v):
        raise ValueError(f"{name} wants L,R (deg, {lo:g} to {hi:g}), got {txt!r}")
    return tuple(v)


def swing_side(i, first):
    """The swing foot of step i (0-based) of a walk whose first swing foot is `first`."""
    return first if i % 2 == 0 else ("R" if first == "L" else "L")


def ankle_extra(i, dy, first, side=None, boost=0.0):
    """Deg added to step i's stance-ankle feed-forward: toward a sideways step's own side the stance ankle there gets
    (side[stance] - ROBOT_ANKLE_FF[stance]) x min(1, |dy| / SIDE_REF_MM) (dy in m, + = left: a LEFT step loads the LEFT
    stance, a RIGHT step the RIGHT stance), and the first stance step of each side (steps 0 and 1) gets `boost`."""
    st = "R" if swing_side(i, first) == "L" else "L"
    x = boost if i < 2 else 0.0
    if side is not None and dy and ((dy > 0 and st == "L") or (dy < 0 and st == "R")):
        k = 0 if st == "L" else 1
        x += (side[k] - ROBOT_ANKLE_FF[k]) * min(1.0, abs(dy) * 1e3 / SIDE_REF_MM)
    return x


def step_drift(c, side=None):
    """The heading drift (deg / step, + = left) the hold's feed-forward cancels for step command c: YAW_DRIFT for the
    fore-aft part, side[0] (sideL) or side[1] (sideR) for the lateral part, blended by the lateral share of the step
    |dy| / (|dx| + |dy|); a step in place has the fore-aft value."""
    if side is None:
        return YAW_DRIFT
    ax, ay = abs(c["dx"]), abs(c["dy"])
    if ax + ay < 1e-9:
        return YAW_DRIFT
    f = ay / (ax + ay) * min(1.0, ay * 1e3 / SIDE_BLEND_MM)      # a few mm of sideways travel is not a sidestep: no jump
    return (1.0 - f) * YAW_DRIFT + f * side[0 if c["dy"] > 0 else 1]


def _num(x):
    return f"{float(x):g}"


def step_cmd(dx_mm, dy_mm, dyaw_deg):
    """An arbitrary per-step displacement: forward, left (mm) and heading change (deg, + = left) per step. The
    asked turn is dyaw / YAW_GAIN (the robot's measured heading gain; sil_gait's turnL is 4 asked ~ 5 deg)."""
    return dict(name=f"step({_num(dx_mm)},{_num(dy_mm)},{_num(dyaw_deg)})", key=f"step({_num(dx_mm)},{_num(dy_mm)},{_num(dyaw_deg)})",
                dx=float(dx_mm) / 1e3, dy=float(dy_mm) / 1e3, yt=float(dyaw_deg) / YAW_GAIN, dpsi=float(dyaw_deg))


def preset_cmd(name, stride_mm=None, side_mm=None):
    """A --dir preset as a command; stride_mm / side_mm replace the preset's step length (fwd / back, sideL / sideR)."""
    d = G.DIRS[name]
    if "yaw" in d:
        raise ValueError(f"{name}: the sine sway's alternating hip yaw has no per-step command")
    dx, dy = d.get("dx", 0.0), d.get("dy", 0.0)
    if stride_mm is not None and dx:
        dx = math.copysign(float(stride_mm) / 1e3, dx)
    if side_mm is not None and dy:
        dy = math.copysign(float(side_mm) / 1e3, dy)
    return dict(name=name, key=name, dx=dx, dy=dy, yt=d.get("yaw_trim", 0.0), dpsi=PRESET_DPSI.get(name, 0.0))


def parse_step(txt):
    v = [float(x) for x in txt.split(",")]
    if len(v) != 3:
        raise ValueError(f"--step wants dx_mm,dy_mm,dyaw_deg, got {txt!r}")
    return step_cmd(*v)


def parse_seq(txt, stride_mm=None, side_mm=None):
    """"fwd:4,turnL:3,step(20,10,0):2" -> [(cmd, count)]. A preset or step(dx_mm,dy_mm,dyaw_deg), colon, a step count."""
    out = []
    for tok in re.findall(r"(?:[^,(]|\([^)]*\))+", txt):
        tok = tok.strip()
        name, _, cnt = tok.rpartition(":")
        if not name or not cnt.strip().isdigit() or int(cnt) < 1:
            raise ValueError(f"--seq segment {tok!r}: want name:count (count >= 1)")
        m = re.fullmatch(r"step\(([^)]*)\)", name.strip())
        if m:
            cmd = parse_step(m.group(1))
        elif name.strip() in G.DIRS:
            cmd = preset_cmd(name.strip(), stride_mm, side_mm)
        else:
            raise ValueError(f"--seq segment {tok!r}: {name!r} is not one of {list(G.DIRS)} or step(dx,dy,dyaw)")
        out.append((cmd, int(cnt)))
    return out


def expand_seq(seq, first="R"):
    """[(cmd, count)] -> one command per step, with the segment's name. One step per boundary is added (named
    "close", dx 0, dy 0, no turn) ONLY when the feet are laterally apart at the boundary and the next segment
    reverses the lateral direction: the swing foot's first out-step there would land on the foot it started
    beside (a null step), so the closing is made explicit and the segment keeps its count. Otherwise a step is
    always placed relative to the stance foot, so the next segment's first step takes up whatever stagger or hip
    yaw the last one left: the feet stay apart, mid-stagger, no closing. (The final closing step at the end of
    the run is dwell's, as before.)"""
    out, fy = [], {"L": 0.0, "R": 0.0}

    def place(c):
        i = len(out)
        sw = first if i % 2 == 0 else ("R" if first == "L" else "L")
        st = "R" if sw == "L" else "L"
        o = (c["dy"] > 0 and sw == "L") or (c["dy"] < 0 and sw == "R")
        fy[sw] = fy[st] + (c["dy"] if o else 0.0)
        out.append(c)
    prev = None
    for k, (cmd, n) in enumerate(seq):
        gap = fy["L"] - fy["R"]
        if prev is not None and abs(gap) > 1e-4 and cmd["dy"] * gap < 0:
            place(dict(name="close", key=prev["key"], dx=0.0, dy=0.0, yt=0.0, dpsi=0.0, seg=k, close=True))
        for _ in range(n):
            place(dict(cmd, seg=k))
        prev = cmd
    return out


def plan(T_step, n_steps, direction, seeds=ROBOT_SEED, lead_ms=2500, cal=None, first="R", amp=ROBOT_AMP,
         model="dwell", ds=None, h=None, yaw_trim=YAW_TRIM, pelvis_x=ROBOT_PELVIS_X, ankle_ff=ROBOT_ANKLE_FF,
         k_ds=None, ankle_steps=None, cmd=None, seq=None, drop=None, swing_tilt=0.0, lift_gate=False,
         step_cmds=None, ankle_side=None, first_boost=0.0, drift_side=None, td_pause=0.0, settle=None,
         lower_frac=None, land_early=0.0, ss_s=None, shift_s=None, raise_s=None, move_s=None, lower_s=None, pre_hike=None,
         pre_hike_frac=1.0):
    """ankle_side (L, R) / first_boost: the by-step ankle feed-forward (ankle_extra; with ankle_steps the learner's values
    stay as given and the extra is added in the plan only); a step_cmds command's "aff_x" is its extra (cmd_walk.py
    decides it from the step's absolute index). drift_side (L, R): the heading drift of a sideL / sideR step
    (step_drift): the constant yaw_trim is scaled by each step's drift / YAW_DRIFT.
    lift_gate (dwell): kGate on the last frame before each lift-off (the firmware waits there for a level, quiet body).
    yaw_trim (deg / step, + = left) is ADDED to the direction's own turn (turnL / turnR ask +-4).
    cmd: one per-step command (step_cmd / preset_cmd) replacing the --dir preset's; seq: [(cmd, count)] segments
    in one continuous track (n_steps is then the expanded count). A yaw_trim LIST is the heading hold's asked turn
    per step, complete (it replaces the command's own turn).
    step_cmds: one command per step, used as given (cmd_walk.py, the --cmd mode: no closing step is inserted at a
    reversal -- the walk decides that itself); a command's "tilt" is that step's swing tilt (swing_tilt is then the
    default of the steps without one), "yaw_free" leaves the heading trim off that step."""
    gkw = {}
    tilt_arg = swing_tilt
    if amp[1] is None:
        import dwell as DW
        amp = (amp[0], DW.k_static_R(T_step))
    dkw = dict(G.DIRS[direction])
    meta = None
    cmds = None
    if step_cmds is not None:
        cmds = list(step_cmds)
        n_steps = len(cmds)
        dkw = dict(dx_steps=[c["dx"] for c in cmds], dy_steps=[c["dy"] for c in cmds])
        meta = [dict(name=f"{i + 1}:{c['name']}", key=c["key"]) for i, c in enumerate(cmds)]
        if any("tilt" in c for c in cmds):
            tilt_arg = [c.get("tilt", swing_tilt) for c in cmds]
    elif seq is not None:
        cmds = expand_seq(seq, first)
        n_steps = len(cmds)
        dkw = dict(dx_steps=[c["dx"] for c in cmds], dy_steps=[c["dy"] for c in cmds])
        meta = [dict(name=f"{c['seg'] + 1}:{c['name']}", key=c["key"]) for c in cmds]
    elif cmd is not None:
        dkw = dict(dx=cmd["dx"], dy=cmd["dy"], yaw_trim=cmd["yt"])
        meta = [dict(name=cmd["name"], key=cmd["key"])] * n_steps
        cmds = [cmd] * n_steps
    def _yt(c):
        return yaw_trim if drift_side is None else yaw_trim * step_drift(c, drift_side) / YAW_DRIFT
    extras = None
    if step_cmds is not None:
        extras = [c.get("aff_x", 0.0) for c in cmds]
    elif ankle_side is not None or first_boost:
        dys = [c["dy"] for c in cmds] if cmds is not None else [dkw.get("dy", 0.0)] * n_steps
        extras = [ankle_extra(i, dy, first, ankle_side, first_boost) for i, dy in enumerate(dys)]
    ankle_eff = ankle_steps
    if extras is not None and any(extras):
        base = list(ankle_steps) if ankle_steps is not None else []
        ankle_eff = []
        for i in range(n_steps):
            st_k = 0 if swing_side(i, first) == "R" else 1
            b = base[i] if i < len(base) and base[i] is not None else ankle_ff[st_k]
            ankle_eff.append(b + extras[i])
    if model == "dwell":
        gkw = dict(**({"ds": ds} if ds is not None else {}), **({"h": h} if h is not None else {}),
                   **({"pelvis_x": pelvis_x} if pelvis_x != "mid" else {}),
                   **({"ankle_ff": tuple(ankle_ff)} if any(ankle_ff) else {}),
                   **({"k_ds": tuple(k_ds)} if k_ds and any(k_ds) else {}),
                   **({"ankle_ff_steps": list(ankle_eff)} if ankle_eff is not None else {}),
                   **({"step_meta": meta} if meta is not None else {}),
                   **({"drop": drop} if drop is not None else {}),
                   **({"td_pause": td_pause} if td_pause else {}), **({"settle": settle} if settle is not None else {}),
                   **({"lower_frac": lower_frac} if lower_frac is not None else {}),
                   **({"land_early": land_early} if land_early else {}),
                   **({"pre_hike": tuple(pre_hike), "pre_hike_frac": pre_hike_frac}
                      if pre_hike is not None and any(pre_hike) else {}),
                   **{k_: v_ for k_, v_ in (("ss_s", ss_s), ("shift_s", shift_s), ("raise_s", raise_s), ("move_s", move_s),
                                           ("lower_s", lower_s)) if v_ is not None},
                   **({"swing_tilt": tilt_arg} if (any(tilt_arg) if isinstance(tilt_arg, list) else tilt_arg) else {}),
                   **({"step_lateral": [bool(c.get("lat")) for c in cmds]}
                      if step_cmds is not None and any(c.get("lat") for c in cmds) else {}))
        if isinstance(yaw_trim, (list, tuple)):
            dkw["yaw_trim"] = list(yaw_trim)          # the heading hold: the asked turn per step, complete
        elif seq is not None or step_cmds is not None:
            dkw["yaw_trim"] = [c["yt"] + (0.0 if c.get("yaw_free") else _yt(c))
                               for c in cmds]
        elif drift_side is not None and cmds is not None:
            dkw["yaw_trim"] = [c["yt"] + _yt(c) for c in cmds]
        else:
            dkw["yaw_trim"] = dkw.get("yaw_trim", 0.0) + yaw_trim
    elif seq is not None or cmd is not None or step_cmds is not None:
        raise ValueError("--step / --seq / --stride-mm need the dwell model")
    sw, fr = G.gait(T_step, n_steps, first=first, cal=cal, kL=amp[0], kR=amp[1], model=model,
                    lift_gate=lift_gate and model == "dwell", **dkw, **gkw)
    if pre_hike is not None and any(pre_hike):
        seeds = tuple(0.0 if h_ else v_ for v_, h_ in zip(seeds, pre_hike))        # the plan carries the hike: T there is 0
    pre = [f"trim T L {seeds[0]:g}", f"trim T R {seeds[1]:g}", "track clear"]
    if cmds is not None:
        dpsi = [c["dpsi"] for c in cmds]
    else:
        dpsi = [PRESET_DPSI.get(direction, 0.0)] * n_steps
    return dict(sway=sw, frames=fr, pre=pre, adds=lines_for(fr), play=f"track play {int(lead_ms)}",
                duration_s=fr[-1][0] / 1e3, cal=cal, amp=amp, model=model, cmds=cmds, dpsi=dpsi,
                args=dict(T_step=T_step, n_steps=n_steps, direction=direction, seeds=seeds, lead_ms=lead_ms, cal=cal,
                          first=first, amp=amp, model=model, ds=ds, h=h, pelvis_x=pelvis_x, ankle_ff=tuple(ankle_ff),
                          k_ds=tuple(k_ds) if k_ds else None,
                          ankle_steps=list(ankle_steps) if ankle_steps is not None else None,
                          cmd=cmd, seq=seq, drop=drop, swing_tilt=swing_tilt, lift_gate=lift_gate,
                          step_cmds=list(step_cmds) if step_cmds is not None else None,
                          ankle_side=tuple(ankle_side) if ankle_side is not None else None,
                          first_boost=first_boost, drift_side=tuple(drift_side) if drift_side is not None else None,
                          td_pause=td_pause, settle=settle, lower_frac=lower_frac, land_early=land_early,
                          ss_s=ss_s, shift_s=shift_s, raise_s=raise_s, move_s=move_s, lower_s=lower_s,
                          pre_hike=tuple(pre_hike) if pre_hike is not None else None, pre_hike_frac=pre_hike_frac))


def sil_step_kw(cmd=None, seq=None, first="R", yaw_trim=YAW_TRIM):
    """(gait() kwargs, n_steps) of a step command or a sequence, as plan() builds them: for sil_gait.run(step_kw=...)."""
    if seq is not None:
        cm = expand_seq(seq, first)
        return dict(dx_steps=[c["dx"] for c in cm], dy_steps=[c["dy"] for c in cm],
                    yaw_trim=[c["yt"] + yaw_trim for c in cm],
                    step_meta=[dict(name=f"{c['seg'] + 1}:{c['name']}", key=c["key"]) for c in cm]), len(cm)
    return dict(dx=cmd["dx"], dy=cmd["dy"], yaw_trim=cmd["yt"] + yaw_trim), None


# The heading model the hold inverts (hw 2026-11-12, T 4-5, the turn about the planted foot): heading per step =
# YAW_GAIN x asked + YAW_DRIFT (deg; + = left); the constants are above, with the step commands.


class HeadingHold:
    """Per-step heading hold on the rolling queue (#297): at every reported single support the host reads the
    fused yaw (`imu` quat; the firmware's online gyro bias keeps it from drifting at rest), compares it with the
    reference -- a commanded heading change per step, so a commanded heading RATE is the same loop -- and asks
    the steps not yet uploaded for asked = (dpsi_cmd + k x error - YAW_DRIFT) / YAW_GAIN deg each, clipped."""

    def __init__(self, n_steps, dpsi_cmd, k=0.5, clip=8.0, drift=None):
        """dpsi_cmd: the commanded heading change per step, deg: one number, or a list per step (a --seq run: the
        active segment's). drift: the heading drift the feed-forward cancels, deg / step: None = YAW_DRIFT for every
        step, or a list per step (step_drift: it depends on the step's type)."""
        self.n, self.dpsi, self.k, self.clip = n_steps, dpsi_cmd, k, clip
        self._d = list(dpsi_cmd) if isinstance(dpsi_cmd, (list, tuple)) else [dpsi_cmd] * n_steps
        self._d += [self._d[-1]] * (n_steps - len(self._d))
        self._dr = list(drift) if drift is not None else []
        self._dr += [self._dr[-1] if self._dr else YAW_DRIFT] * (len(self._d) - len(self._dr))
        self.asked = [max(-clip, min(clip, (d - dr) / YAW_GAIN)) for d, dr in zip(self._d, self._dr)]
        self.log = []
        self.err = 0.0                 # the last measured heading error, deg (the --cmd mode asks new steps with it)

    def asked_for(self, d, drift=None):
        """The asked turn (deg) of a step whose commanded heading change is d, with the last measured error."""
        return max(-self.clip, min(self.clip, (d + self.k * self.err - (YAW_DRIFT if drift is None else drift)) / YAW_GAIN))

    def set_step(self, i, d, asked=None, drift=None):
        """--cmd mode: step i (appended if it is the next one) commands a heading change d deg; asked overrides the
        asked turn (the closing step: 0); drift: that step's heading drift (default YAW_DRIFT)."""
        dr = YAW_DRIFT if drift is None else drift
        while len(self._d) <= i:
            self._d.append(d)
            self._dr.append(dr)
            self.asked.append(self.asked_for(d, dr))
        self._d[i] = d
        self._dr[i] = dr
        self.asked[i] = self.asked_for(d, dr) if asked is None else float(asked)
        self.n = len(self._d)

    def ref(self, n_done):
        """The reference heading after n_done steps, deg."""
        if not isinstance(self.dpsi, (list, tuple)):
            return n_done * self.dpsi
        return sum(self._d[:n_done])

    def update(self, n_done, psi, first_free):
        """n_done single supports reported, psi the heading (deg, from the start), first_free = the first step
        whose frames are not uploaded yet. Returns True when the asked list changed."""
        err = self.ref(n_done) - psi
        self.err = err
        new = [max(-self.clip, min(self.clip, (d + self.k * err - dr) / YAW_GAIN)) for d, dr in zip(self._d, self._dr)]
        a = new[min(first_free, self.n - 1)]
        self.log.append(dict(n=n_done, psi=round(psi, 1), ref=round(self.ref(n_done), 1), err=round(err, 1),
                             asked=round(a, 2), from_step=first_free + 1))
        if first_free >= self.n or all(abs(x - y) < 0.05 for x, y in zip(self.asked[first_free:], new[first_free:])):
            return False
        self.asked = self.asked[:first_free] + new[first_free:]
        return True


ANKLE_STATE = os.path.join(HERE, "results", "ankle_trim_state.json")
ANKLE_SEED = {"L": 2.0, "R": 2.0}
ANKLE_SEED_DIR = {"sideR": {"R": 3.0}}     # hw 2026-11-12: sideR at T 4 tipped 3/3 at R 2, ran 7/7 clean at R 3
ANKLE_MIN, ANKLE_MAX = 0.0, 4.0            # 4 on R at T 3.4: outward 5-8 deg, then the fall


class AnkleLearner:
    """The stance-ankle trim A[side] (deg, + = outward), learned at step rate on the host (#297 B2, 2026-11-12):
    at every TS line of that side

        tip (in >= 7):            A += 1.0
        out > 4:                  A -= 0.5 x (out - 4), at most 1.5
        out < 2 and in > 3:       A += 0.5
        else                      hold

    clamped to [ANKLE_MIN, ANKLE_MAX]. Keep / revert, as the firmware's T: a clean step (in < 5, out <= 4) keeps
    the A it ran with (A_keep); a step past 8 deg OUTWARD puts A back to A_keep before the rule. The steps not
    uploaded yet of that side get the new A. The state is keyed by (direction, side) in ANKLE_STATE; it is saved
    by the caller only after a run with no fall and no ABORT."""

    def __init__(self, direction, path=None, start=None):
        self.dir, self.path = direction, (path or ANKLE_STATE)
        seed = dict(ANKLE_SEED, **ANKLE_SEED_DIR.get(direction, {}))
        if start is not None:                      # an explicit start (--ankle-seed): the state file is not read
            seed = {"L": float(start[0]), "R": float(start[1])}
        else:
            seed.update(self._stored(direction))
        self.A = {k: self._clamp(v) for k, v in seed.items()}
        self.start = dict(self.A)
        self.keep = dict(self.A)
        self.log = []

    def _stored(self, direction):
        try:
            with open(self.path) as f:
                st = json.load(f).get(direction, {})
            return {k: float(v) for k, v in st.items() if k in ("L", "R")}
        except (OSError, ValueError):
            return {}

    @staticmethod
    def _clamp(x):
        return max(ANKLE_MIN, min(ANKLE_MAX, float(x)))

    def update(self, side, lean_in, lean_out, used=None):
        """One reported single support of `side` (TS in / out, deg; `used` = the A that step ran with). Returns
        the new A of that side."""
        used = self.A[side] if used is None else used
        if lean_in < 5.0 and lean_out <= 4.0:
            self.keep[side] = used
        # revert on an OUTWARD step past 8 only: tips are 9-13 deg inward, and reverting on them would hold A at
        # A_keep + 1 for good (two tips in a row could never climb past it)
        base = self.keep[side] if lean_out > 8.0 else self.A[side]
        if lean_in >= 7.0:
            d, why = 1.0, "tip"
        elif lean_out > 4.0:
            d, why = -min(1.5, 0.5 * (lean_out - 4.0)), "out"
        elif lean_out < 2.0 and lean_in > 3.0:
            d, why = 0.5, "in"
        else:
            d, why = 0.0, "hold"
        new = self._clamp(base + d)
        self.log.append(dict(side=side, used=round(used, 2), lean_in=lean_in, lean_out=lean_out, rule=why,
                             reverted=base != self.A[side], new=round(new, 2)))
        self.A[side] = new
        return new

    def save(self):
        try:
            with open(self.path) as f:
                st = json.load(f)
        except (OSError, ValueError):
            st = {}
        st[self.dir] = {k: round(v, 2) for k, v in self.A.items()}
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        with open(self.path, "w") as f:
            json.dump(st, f, indent=1, sort_keys=True)


class AnkleBank:
    """The learner of a --seq run: one AnkleLearner per segment key (direction preset, or step(dx,dy,dyaw)), the
    state file keyed as for a single direction; a step is learned by, and planned from, its active segment's."""

    def __init__(self, keys, path=None, start=None):
        self.by = {}
        for k in keys:
            if k not in self.by:
                self.by[k] = AnkleLearner(k, path=path, start=start)

    def of(self, key):
        return self.by[key]

    def save(self):
        for lr in self.by.values():
            lr.save()

    @property
    def log(self):
        return [dict(r, key=k) for k, lr in self.by.items() for r in lr.log]


def ankle_steps_for(sw, A, locked=None, first_free=0):
    """Per-step stance-ankle values: the locked list's values for steps < first_free, A[stance] after (A is one
    {L, R} dict, or an AnkleBank: then each step's own segment's)."""
    out = []
    meta = getattr(sw, "step_meta", None)
    for i, p_ in enumerate(sw.phases):
        if locked is not None and i < first_free and i < len(locked):
            out.append(locked[i])
        else:
            a_ = A.of(meta[min(i, len(meta) - 1)]["key"]).A if hasattr(A, "of") and meta else A
            out.append(a_[p_["stance"]])
    return out


def yaw_of_imu(txt):
    m = re.search(r"^quat\s+([-+\d.]+)\s+([-+\d.]+)\s+([-+\d.]+)\s+([-+\d.]+)", txt, re.M)
    if not m:
        return None
    w, x, y, z = (float(v) for v in m.groups())
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


UPPER = 0.05     # the shift must not exceed k_static by more than this (x 62 mm): 18:09 fell OUTWARD at 1.35


def precheck(pl, tol=0.01, only_step=None):
    """The robot's static shift requirement as a hard constraint (hw 2026-11-12: it fell when the foot came up
    at 1.0-1.1 x): from the FRAMES that would be sent, the pelvis must be at or beyond k_static[stance] x 62 mm
    over the stance foot at every lift-off and at every touchdown. The pelvis shift is recovered from the
    frames' stance hip roll goal (ticks -> rad through the calibration in use, interpolated at that instant)
    by inverting the IK with the plan's feet. -> (ok, rows).
    only_step (0-based): judge that one step only -- its lift-off and touchdown, the joint margin over its frames and
    the pelvis speed over its frames (the --cmd mode's incremental check of the step it is about to upload; the
    default, None, is the whole plan as before)."""
    sw, fr, cal = pl["sway"], pl["frames"], pl["cal"]
    K = G.W.S.kin(pl["args"].get("drop") or 0.02)
    oy = abs(K.model_offset("L")[1])
    need = {"L": pl["amp"][0], "R": pl["amp"][1]}
    kds = pl["args"].get("k_ds") or (0.0, 0.0)
    need_lo = {"L": kds[0] or need["L"], "R": kds[1] or need["R"]}     # the two-stage shift: k_ds at lift-off
    # the upper bound is the ROBOT's requirement + UPPER (not the plan's): an --amp past it is refused
    import dwell as DW
    top = {"L": DW.K_STATIC["L"] + UPPER, "R": DW.k_static_R(pl["args"]["T_step"]) + UPPER}
    if pl.get("allow_over"):
        top = {"L": 9.0, "R": 9.0}
    kf = []                      # (track ms, decoded {joint: rad})
    win = (-1e18, 1e18)
    if only_step is not None:
        st_ = sw._steps[only_step]
        win = (1e3 * (st_["a"] - sw.t_first) - 1.0, 1e3 * (st_["td"] - sw.t_first) + 1.0)
    for tm, hx, kind in fr:
        if kind != "kf" or not win[0] <= tm <= win[1]:
            continue
        q = {}
        for j, n in enumerate(G.LEG):
            q[n] = G.from_ticks(n, int(hx[6 + 3 * j: 9 + 3 * j], 16), cal)
        kf.append((tm, q))

    def q_at(t_s):
        tm = 1e3 * (t_s - sw.t_first)
        for (ta, qa), (tb, qb) in zip(kf, kf[1:]):
            if ta <= tm <= tb:
                u = (tm - ta) / max(tb - ta, 1e-9)
                return {n: qa[n] + u * (qb[n] - qa[n]) for n in qa}
        return None

    def seg_at(t_s):
        return min(sw.segs[1:], key=lambda sg: abs(sg["t"] - t_s))

    def shift_of(q, pa, stance):
        """the absolute pelvis shift (m) whose IK gives this stance hip roll, with the plan's feet."""
        jn = f"{stance}_hip_roll"
        feet = pa["feet"]

        def f(sh):
            return K.pose(shift=sh, fwd=pa["fwd"], footL=tuple(feet["L"]), footR=tuple(feet["R"]))[jn] - q[jn]
        # the bracket: the pelvis's reach either way (the IK raises past it)
        mid0 = 0.5 * (feet["L"][1] + feet["R"][1])
        lo, hi = mid0 - 0.12, mid0 + 0.12
        while True:
            try:
                flo = f(lo)
                f(hi)
                break
            except ValueError:
                lo, hi = mid0 + 0.9 * (lo - mid0), mid0 + 0.9 * (hi - mid0)
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            fm = f(mid)
            if (fm > 0) == (flo > 0):
                lo, flo = mid, fm
            else:
                hi = mid
        return 0.5 * (lo + hi)

    rows, ok = [], True
    for k, p in enumerate(sw.phases):
        if only_step is not None and k != only_step:
            continue
        st = p["stance"]
        sg = 1.0 if st == "L" else -1.0
        for what, t in (("lift-off", p["t0"]), ("touchdown", p["t1"])):
            q = q_at(t)
            if q is None:
                continue
            pa = seg_at(t)["pa"]
            hk = sw.hike_at(t) if hasattr(sw, "hike_at") else None
            if hk is not None and hk[0] == st:                    # the plan's pre-hike is in the stance hip roll goal
                q = dict(q, **{f"{st}_hip_roll": q[f"{st}_hip_roll"] - sg * hk[1]})
            sh = shift_of(q, pa, st)
            ratio = sg * (sh - pa["feet"][st][1]) / oy
            nd = need_lo[st] if what == "lift-off" else need[st]
            good = nd - tol <= ratio <= top[st] + tol
            warn = False
            if not good and ratio < nd - tol and pl.get("allow_under"):
                good, warn = True, True          # --allow-under: short of the plan's own requirement is a warning
            ok = ok and good
            rows.append(dict(step=k + 1, stance=st, at=what, t=round(t - sw.t_first, 2), ratio=round(ratio, 3),
                             need=nd, top=top[st], ok=good, **({"warn": True} if warn else {})))
    # every frame's joints inside the plant's joint range less JOINT_MARGIN (the knee, hip pitch / roll / yaw, ankle
    # pitch and roll; the --ankle-ff feed-forward adds to the IK's ankle roll; the firmware also clamps to its
    # calibrated range, which would bend the plan). The tightest margin over the plan is the last row.
    lim = joint_limits()
    tight = None
    bad_j = set()
    for tm, q in kf:
        for n in G.LEG:
            lo, hi = lim[n]
            m_ = min(q[n] - lo, hi - q[n])
            if tight is None or m_ < tight[0]:
                tight = (m_, n, tm / 1e3, math.degrees(q[n]))
            if m_ < JOINT_MARGIN and n not in bad_j:
                bad_j.add(n)
                ok = False
                rows.append(dict(step=0, stance=n, at="joint limit", t=round(tm / 1e3, 2), ratio=round(math.degrees(q[n]), 2),
                                 need=round(math.degrees(lo + JOINT_MARGIN), 2), top=round(math.degrees(hi - JOINT_MARGIN), 2),
                                 ok=False))
    if tight is not None:
        rows.append(dict(step=0, stance=tight[1], at="tightest joint", t=round(tight[2], 2), ratio=round(tight[0] * 180 / math.pi, 2),
                         need=round(math.degrees(JOINT_MARGIN), 2), top=round(tight[3], 2), ok=True))
    # the swing leg's joint speed over each single support (the keyframes are what the firmware interpolates between):
    # short single supports mean fast swings; the servo's no-load speed (4.04 rad/s at 12 V, measured) less a margin
    kf_all = [(tm, q) for tm, q in kf]
    sp_pk, sp_at = 0.0, None
    for k, p in enumerate(sw.phases):
        if only_step is not None and k != only_step:
            continue
        a_, b_ = 1e3 * (p["t0"] - sw.t_first), 1e3 * (p["t1"] - sw.t_first)
        seq_ = [(tm, q) for tm, q in kf_all if a_ - 1e-6 <= tm <= b_ + 1e-6]
        for (ta, qa), (tb, qb) in zip(seq_, seq_[1:]):
            for n in G.LEG:
                if n.startswith(p["swing"] + "_") and tb > ta:
                    v = abs(math.degrees(qb[n] - qa[n])) / ((tb - ta) / 1e3)
                    if v > sp_pk:
                        sp_pk, sp_at = v, (k + 1, n, round(tb / 1e3, 2))
    if sp_at is not None:
        oks = sp_pk <= SWING_SPEED_LIMIT
        ok = ok and oks
        rows.append(dict(step=sp_at[0], stance=sp_at[1], at="swing joint speed", t=sp_at[2], ratio=round(sp_pk, 1), need=0.0,
                         top=round(SWING_SPEED_LIMIT, 1), ok=oks))
    # the pelvis (x, y) speed through the shift: the double-support shift's min-jerk timing (dwell.SCHEDULE) was validated for
    # a lateral move of ~161 mm in >= 1.2 s; a longer stride adds fore-aft travel to the same time. Sampled at the keyframes.
    v = pelvis_speed(sw, None if only_step is None else (win[0] / 1e3 + sw.t_first, win[1] / 1e3 + sw.t_first))
    okv = v <= PELVIS_V_MAX * 1.01
    ok = ok and okv
    rows.append(dict(step=0, stance="pelvis", at="pelvis speed", t=0.0, ratio=round(1e3 * v, 1), need=0.0,
                     top=round(1e3 * PELVIS_V_MAX, 1), ok=okv))
    return ok, rows


def pelvis_speed(sw, window=None):
    """Peak speed (m/s) of the pelvis in the floor plane over the plan (or over window = (t0, t1) s of the plan's
    own clock), from the keyframes' pelvis (fwd, shift)."""
    peak = 0.0
    segs = [sg for sg in sw.segs[1:] if sg.get("pa") and (window is None or window[0] <= sg["t"] <= window[1])]
    for sa, sb in zip(segs, segs[1:]):
        dt = sb["t"] - sa["t"]
        if dt > 0:
            d = math.hypot(sb["pa"]["fwd"] - sa["pa"]["fwd"], sb["pa"]["pel"] - sa["pa"]["pel"])
            peak = max(peak, d / dt)
    return peak


JOINT_MARGIN = math.radians(2.0)
# the swing leg's joint speed limit: the STS3215's no-load speed, 4.04 rad/s at 12 V (measured, docs/servo-map.md §
# no-load speed, tools/measure_servo_speed.py) = 231 deg/s, less 20 % for the load and the supply
SERVO_NOLOAD_DEG_S = math.degrees(4.04)
SWING_SPEED_LIMIT = 0.8 * SERVO_NOLOAD_DEG_S
ANKLE_MARGIN = JOINT_MARGIN
_JLIM = {}


def joint_limits():
    """{joint: (lo, hi) rad} of the leg joints in the plant the IK uses (sim/gen_plant_v6 via v6_kin)."""
    if not _JLIM:
        import v6_kin as VK
        from design_gates import JN
        m, _ = VK.load(G.W.S.kin(0.02).p)
        for n in G.LEG:
            j = 1 + JN.index(n)
            _JLIM[n] = (float(m.jnt_range[j, 0]), float(m.jnt_range[j, 1]))
    return _JLIM


def ankle_limits():
    """{joint: (lo, hi) rad}: the ankle roll range of the plant the IK uses."""
    return {n: v for n, v in joint_limits().items() if n.endswith("ankle_roll")}


def predicted_dwell(T_step, direction):
    p = os.path.join(HERE, "results", "dwell_schedule.json")
    if not os.path.exists(p):
        return None
    rows = [r for r in json.load(open(p)) if "error" not in r and r["T"] == T_step and r["dir"] == direction]
    if not rows:
        return None
    med = lambda v: sorted(v)[len(v) // 2] if v else float("nan")  # noqa: E731
    li = [x for r in rows for x in r["lean_in"]]
    return dict(runs=len(rows), falls=sum(r["fell"] for r in rows), aborts=sum(r["aborts"] for r in rows),
                lifts=f"{sum(r['lifts'] for r in rows)}/{sum(r['n'] for r in rows)}", lean_in_med=med(li),
                lean_in_max=max(li), lean_out_max=max(x for r in rows for x in r["lean_out"]),
                T_mid={s: med([x for r in rows for x in r["T_mid"][s]]) for s in "LR"},
                hike_mid={s: med([x for r in rows for x in r["hike_mid"][s]]) for s in "LR"},
                clearance_mm=med([x for r in rows for x in r["clr"]]))


def predicted(T_step, direction):
    p = os.path.join(HERE, "results", "sil_gait.json")
    if not os.path.exists(p):
        return None
    rows = [r for r in json.load(open(p)) if "error" not in r and r["T"] == T_step and r["dir"] == direction]
    if not rows:
        return None
    reps = [x for r in rows for x in r["reports"]]
    med = lambda v: sorted(v)[len(v) // 2] if v else float("nan")  # noqa: E731
    return dict(runs=len(rows), falls=sum(r["fell"] for r in rows), aborts=sum(r["aborts"] for r in rows),
                lifts=f"{sum(r['lifts'] for r in rows)}/{sum(r['n_steps'] for r in rows)}",
                lean_in_med=med([x["lean_in"] for x in reps]), lean_in_max=max(x["lean_in"] for x in reps),
                lean_out_max=max(x["lean_out"] for x in reps),
                T_mid={s: med([x["T_mid"] for x in reps if x["stance"] == s]) for s in "LR"},
                hike_mid={s: med([x["hike_mid"] for x in reps if x["stance"] == s]) for s in "LR"},
                clearance_mm=med([p["clr_max_mm"] for r in rows for p in r["phases"]]))


def report_precheck(pl):
    ok, rows = precheck(pl)
    bad = [r for r in rows if not r["ok"]]
    sh = [r for r in rows if r["at"] in ("lift-off", "touchdown")]
    worst = min(sh, key=lambda r: r["ratio"] - r["need"]) if sh else None
    tj = next((r for r in rows if r["at"] == "tightest joint"), None)
    pv = next((r for r in rows if r["at"] == "pelvis speed"), None)
    print(f"static-shift pre-check ({pl['model']}): {'PASS' if ok else 'REFUSED'} -- pelvis over the stance foot at "
          f"every lift-off and touchdown, from the frames, needs L >= {pl['amp'][0]:g} x, R >= {pl['amp'][1]:g} x 62 mm"
          + (f" (lift-off: k_ds {pl['args']['k_ds']})" if pl["args"].get("k_ds") else "")
          + ", joints inside the plant range less 2 deg, pelvis speed in the shift <= "
          + f"{1e3 * PELVIS_V_MAX:.0f} mm/s"
          + (f"; tightest: step {worst['step']} {worst['stance']} {worst['at']} {worst['ratio']:.3f} x" if worst else ""))
    sj = next((r for r in rows if r["at"] == "swing joint speed"), None)
    if sj:
        print(f"   swing joint speed: peak {sj['ratio']:.0f} deg/s ({sj['stance']}, step {sj['step']}; refused above {sj['top']:.0f})")
    if tj:
        print(f"   tightest joint margin: {tj['ratio']:.1f} deg ({tj['stance']} at {tj['top']:.1f} deg, t {tj['t']} s; "
              f"refused below {tj['need']:g}); pelvis peak speed {pv['ratio']:.0f} mm/s (limit {pv['top']:.0f})")
    for r in rows:
        if r.get("warn"):
            print(f"   WARNING (--allow-under): step {r['step']} {r['stance']} {r['at']} (t {r['t']} s): {r['ratio']:.3f} x, under "
                  f"the plan's {r['need']:g} x -- the pelvis is short of the static requirement")
    import dwell as DW_
    req = {"L": DW_.K_STATIC["L"], "R": DW_.k_static_R(pl["args"]["T_step"])}
    if pl["amp"][0] < req["L"] - 1e-9 or pl["amp"][1] < req["R"] - 1e-9:
        print(f"   WARNING: the plan lifts at k L {pl['amp'][0]:g} / R {pl['amp'][1]:g}, under the robot's static requirement "
              f"(L {req['L']:g}, R {req['R']:g}) -- an under-requirement test"
              + ("" if pl.get("allow_under") else " (--allow-under says it is meant)"))
    for r in bad[:8]:
        if r["at"] == "joint limit":
            print(f"   {r['stance']} at t {r['t']} s: {r['ratio']:.1f} deg outside [{r['need']:g}, {r['top']:g}] deg "
                  f"(the step is too long / the --ankle-ff too large for the kinematics)")
            continue
        if r["at"] == "swing joint speed":
            print(f"   step {r['step']}: {r['stance']} moves {r['ratio']:.0f} deg/s in the swing, over the {r['top']:.0f} deg/s "
                  f"(80 % of the servo's no-load {SERVO_NOLOAD_DEG_S:.0f} deg/s): the single support is too short for this step")
            continue
        if r["at"] == "pelvis speed":
            print(f"   the pelvis moves {r['ratio']:.0f} mm/s in the shift, over the {r['top']:.0f} mm/s its min-jerk "
                  f"timing was validated for (step too long for this T)")
            continue
        print(f"   step {r['step']} {r['stance']} stance at {r['at']} (t {r['t']} s): {r['ratio']:.3f} x outside "
              f"[{r['need']:g}, {r['top']:g}] x (the upper bound: k_static + {UPPER:g}; --allow-over to pass it)")
    return ok


def speeds(pl):
    """(mm/s forward, mm/s left, deg/s commanded heading) over the plan's steps (the closing step excluded), and the
    per-segment list [(name, steps, fwd, left, deg/s)]."""
    cm, T = pl.get("cmds"), getattr(pl["sway"], "step_time", pl["args"]["T_step"])
    if cm is None:
        c = preset_cmd(pl["args"]["direction"]) if pl["args"]["direction"] in G.DIRS and "yaw" not in G.DIRS[pl["args"]["direction"]] \
            else dict(name=pl["args"]["direction"], dx=0.0, dy=0.0, dpsi=0.0)
        cm = [dict(c, seg=0)] * pl["args"]["n_steps"]
    segs = []
    for c in cm:
        if segs and segs[-1][0] == c.get("seg"):
            segs[-1][2].append(c)
        else:
            segs.append((c.get("seg"), c["name"], [c]))
    rows = [(nm, len(cs), 1e3 * sum(c["dx"] for c in cs) / (T * len(cs)), 1e3 * sum(c["dy"] for c in cs) / (2.0 * T * len(cs)),
             sum(c["dpsi"] for c in cs) / (T * len(cs))) for _, nm, cs in segs]
    n = len(cm)
    tot = (1e3 * sum(c["dx"] for c in cm) / (T * n), 1e3 * sum(c["dy"] for c in cm) / (2.0 * T * n),
           sum(c["dpsi"] for c in cm) / (T * n)) if n else (0.0, 0.0, 0.0)
    return tot, rows


def _label(a):
    if getattr(a, "cmd", None):
        return f"cmd {a.cmd}"
    if a.seq:
        return f"seq {a.seq}"
    if a.step:
        return f"step {a.step}"
    return a.dir + (f" (stride {a.stride_mm:g} mm)" if a.stride_mm is not None else "") \
        + (f" (side {a.side_mm:g} mm)" if a.side_mm is not None else "")


def _extra(a):
    return "".join([f", crouch {a.drop_mm:g} mm" if a.drop_mm is not None else "",
                    f", swing tilt {a.swing_tilt:g} deg" if a.swing_tilt else ""])


def timeline(sw):
    """One line: the per-step timeline of a dwell plan and its effective step time."""
    ss, F, R = sw.t_ss, sw.lower_frac, sw.raise_frac
    tl = [("pause", sw.td_pause)] if sw.td_pause else []
    tl += [("shift", sw.t_shift), ("settle", sw.t_settle), ("raise", R * ss), ("move", (1.0 - R - F) * ss),
           ("lower", F * ss)]
    return (" + ".join(f"{n} {v:.2f}" for n, v in tl) + f" = {sum(v for _, v in tl):.2f} s per step "
            + (f"(explicit phase times; --T {sw.T:g} only supplied what was not given; " if sw.explicit else f"(nominal T {sw.T:g}; ")
            + f"single support {ss:.2f} s, the next shift starts {sw.td_pause:.2f} s after the touchdown"
            + (f"; sole {1e3 * sw.land_early:g} mm below the floor at touchdown" if sw.land_early else "") + ")")


def dry(pl, a):
    adds = pl["adds"]
    nb = sum(len(l) + 1 for _, l in adds)
    Teff = getattr(a, "Teff", a.T)
    ahead_ms = min(a.ahead * Teff, MAX_AHEAD_S) * 1e3
    first = [l for t, l in adds if t <= ahead_ms]
    print(f"gait: {pl['model']} T {a.T:g} s/step, {pl['args']['n_steps']} steps, {_label(a)}{_extra(a)}; {sum(1 for f in pl['frames'] if f[2] == 'kf')} "
          f"keyframes at {G.KF_S * 1e3:.0f} ms + {sum(1 for f in pl['frames'] if f[2] == 'rest')} rest frames; "
          f"track {pl['duration_s']:.1f} s after a {a.lead} ms lead-in")
    if pl["model"] == "dwell":
        print(f"timeline: {timeline(pl['sway'])}")
    print(f"upload: {len(adds)} `track add` lines ({PER_LINE} frames each), {nb} bytes; longest line "
          f"{max(len(l) for _, l in adds)} chars (CLI limit 192)")
    print(f"before play: {len(first)} lines (the first {ahead_ms / 1e3:.1f} s); at {a.ms_per_line:.0f} ms a line "
          f"(assumed CLI round trip, to be measured) that is {len(first) * a.ms_per_line / 1e3:.1f} s; while "
          f"playing the host needs {1e3 / (G.KF_S * 1e3) / PER_LINE:.1f} lines/s to keep pace, "
          f"{(1e3 / (G.KF_S * 1e3) / PER_LINE) * a.ms_per_line / 10:.0f} % of the link at that rate")
    print(f"queue: at most ~{int(ahead_ms / (G.KF_S * 1e3)) + 2 * PER_LINE} of 128 frames held "
          f"({ahead_ms / 1e3:.1f} s ahead)")
    n_gate = sum(1 for f in pl["frames"] if f[2] == "kf" and int(f[1][5], 16) & 8)
    print(f"lift gate: {n_gate} gate frames (the last frame before each lift-off)" if n_gate
          else "lift gate: no gate frames (--lift-gate off, or the sway model)")
    if pl["model"] == "dwell":
        tot, rows_ = speeds(pl)
        print(f"speed (commanded, one step per {Teff:g} s): {tot[0]:+.1f} mm/s forward, {tot[1]:+.1f} mm/s sideways (+ = left), "
              f"{tot[2]:+.2f} deg/s heading (+ = left)")
        if pl.get("cmds") is not None and pl["args"].get("seq") is not None:
            for nm, n_, vx, vy, vw in rows_:
                print(f"   segment {nm} x{n_}: {vx:+.1f} mm/s fwd, {vy:+.1f} mm/s side, {vw:+.2f} deg/s")
    pr = None if (getattr(a, "cmd", None) or a.seq or a.step or a.stride_mm is not None or a.side_mm is not None or a.drop_mm is not None
                  or a.swing_tilt or a.td_pause or a.settle is not None or a.lower_frac is not None
                  or a.lower_s is not None or a.land_early_mm) else \
        (predicted_dwell(a.T, a.dir) if pl["model"] == "dwell" else predicted(a.T, a.dir))
    if pr:
        print(f"SIL prediction ({pl['model']}, {pr['runs']} runs over plants x delay): falls {pr['falls']}, aborts "
              f"{pr['aborts']}, lifts {pr['lifts']}; TS lean toward swing median {pr['lean_in_med']:.1f} max "
              f"{pr['lean_in_max']:.1f} deg, outward max {pr['lean_out_max']:.1f}; T at mid support L "
              f"{pr['T_mid']['L']:.1f} R {pr['T_mid']['R']:.1f}, hike L {pr['hike_mid']['L']:.1f} R "
              f"{pr['hike_mid']['R']:.1f} deg; swing sole clearance median {pr['clearance_mm']:.0f} mm")
    report_precheck(pl)
    print("\nsession (dry run -- nothing sent):")
    for l in pl["pre"]:
        print("  " + l)
    for _, l in adds[:len(first)][:3]:
        print("  " + l)
    if len(first) > 3:
        print(f"  ... {len(first) - 3} more `track add` lines")
    print("  " + pl["play"])
    print(f"  ... {len(adds) - len(first)} `track add` lines while it plays, then the firmware rests by itself")


def live(pl, a):
    sys.path.insert(0, os.path.join(ROOT, "experiments", "stepper"))
    import robot_io
    log_path = a.log or os.path.join(HERE, "results", time.strftime("track_%Y%m%dT%H%M%S.jsonl"))
    logf = open(log_path, "a")

    def log(d):
        logf.write(json.dumps(d) + "\n")
        logf.flush()

    if not report_precheck(pl):
        print("REFUSED: the plan puts the pelvis under the static requirement -- nothing sent")
        return
    r = robot_io.Robot(host=a.host, log=log)
    steps, sent, t_line = [], 0, []
    adds = pl["adds"]
    hold = pl.get("hold")
    learn = pl.get("learn")
    cw = pl.get("cmdwalk")          # --cmd: the steps are decided as the walk goes (cmd_walk.CmdWalk)
    if cw is not None:
        cw.on_plan(pl)
    fell = {"v": False, "abort": False, "gate": None}
    stop = False
    yaw_state = {"prev": None, "psi": 0.0}         # unwrapped heading since the start, deg
    yaw_state["prev"] = yaw_state["start"] = yaw_of_imu(r.cmd("imu"))
    if a.note:
        print(f"  note: {a.note}")
    if hold is not None and yaw_state["prev"] is None:
        print("  heading hold: no `quat` in the imu reply -- holding off")
        hold = None

    def first_free_step():
        if cw is not None:
            return len(cw.steps)           # a decided step may have frames uploaded; the provisional one is free
        last_t = adds[sent - 1][0] if sent else 0
        sw = pl["sway"]
        return sum(1 for p_ in sw.phases if 1e3 * (p_["t0"] - sw.t_first) <= last_t)

    def replan():
        """Re-plan the steps not uploaded yet with the heading hold's asked turns and the learned ankle trim."""
        nonlocal adds
        kw = dict(pl["args"])
        if hold is not None:
            kw["yaw_trim"] = hold.asked
        if cw is not None:
            kw.update(cw.plan_kwargs())
            if learn is not None:              # the steps decided since get the learned trim of their stance side
                lst = list(kw.get("ankle_steps") or [])
                for i_ in range(len(lst), len(kw["step_cmds"])):
                    lst.append(learn.A["R" if cw.swing_of(i_) == "L" else "L"])
                kw["ankle_steps"] = pl["args"]["ankle_steps"] = lst
        newp = plan(**kw)
        adds = newp["adds"]
        pl["sway"], pl["frames"] = newp["sway"], newp["frames"]
        if cw is not None:
            cw.on_plan(newp)

    def decide_if_due():
        """--cmd: the next line reaches past the lift-off of the first undecided step: fix that step (read the command,
        check it) and re-plan. True when it did, the caller looks at the new line list."""
        if cw is None or cw.finished or sent >= len(adds) or adds[sent][0] <= cw.limit_ms():
            return False
        cw.decide()
        replan()
        return True

    def seg_of(n):
        """(name, key) of the segment step n (1-based) belongs to; None when the plan has none (a plain --dir run)."""
        meta = getattr(pl["sway"], "step_meta", None)
        return meta[min(max(n, 1), len(meta)) - 1] if meta else None

    def ankle_step(new_steps):
        """After new single supports: the learned ankle trim's update, the not-uploaded steps re-planned."""
        changed = False
        for s_ in new_steps:
            used = (pl["args"].get("ankle_steps") or [None] * 999)[s_["n"] - 1] if s_["n"] >= 1 else None
            sg = seg_of(s_["n"])
            lr = learn.of(sg["key"]) if hasattr(learn, "of") else learn      # the active segment's learner
            old = lr.A[s_["stance"]]
            new = lr.update(s_["stance"], s_["lean_in"], s_["lean_out"], used=used)
            s_["A_used"], s_["A_next"] = used, round(new, 2)
            print(f"  ankle trim {s_['stance']}{' [' + sg['name'] + ']' if sg else ''}: ran "
                  f"{used if used is not None else old:.2f} -> {new:.2f} "
                  f"({lr.log[-1]['rule']}{', reverted' if lr.log[-1]['reverted'] else ''})", flush=True)
            changed = changed or abs(new - old) > 1e-6
        if changed:
            ff = first_free_step()
            pl["args"]["ankle_steps"] = ankle_steps_for(pl["sway"], learn if hasattr(learn, "of") else learn.A, pl["args"].get("ankle_steps"), ff)
            replan()

    def heading_step():
        """After a new single support: read the heading, maybe re-plan the steps not uploaded yet."""
        y = yaw_of_imu(r.cmd("imu"))
        if y is None:
            return
        yaw_state["psi"] += (y - yaw_state["prev"] + 180.0) % 360.0 - 180.0
        yaw_state["prev"] = y
        psi = yaw_state["psi"]
        first_free = first_free_step()
        if hold.update(len(steps), psi, first_free):
            replan()
        h = hold.log[-1]
        sg = seg_of(h["from_step"])
        print(f"  heading {h['psi']:+.1f} (ref {h['ref']:+.1f}, err {h['err']:+.1f}) -> asked {h['asked']:+.2f} "
              f"deg/step from step {h['from_step']}{' [' + sg['name'] + ']' if sg else ''}", flush=True)
    ahead_ms = min(a.ahead * getattr(a, "Teff", a.T), MAX_AHEAD_S) * 1e3

    def scan(txt):
        """Every new TS line in txt: record, print, and say whether one of them trips the stop rule. (2026-11-12:
        it returned at the first tripping line, so the lines after it were never printed.)"""
        trip = False
        for m in TS_RE.finditer(txt):
            s = dict(n=int(m.group(1)), stance=m.group(2), lean_in=float(m.group(3)), lean_out=float(m.group(4)),
                     e=float(m.group(5)), T=float(m.group(6)), hike=float(m.group(7)), ok=m.group(8) == "ok",
                     tback="Tback" in m.group(0), gate_ms=int(m.group(10)) if m.group(10) else None)
            if all(x["n"] != s["n"] for x in steps):
                sg = seg_of(s["n"])
                if sg:
                    s["seg"] = sg["name"]
                steps.append(s)
                print(f"  step {s['n']} {s['stance']}{' [' + sg['name'] + ']' if sg else ''}: in {s['lean_in']:.1f} out {s['lean_out']:.1f} e {s['e']:+.1f} "
                      f"T {s['T']:+.1f} hike {s['hike']:+.1f} {'ok' if s['ok'] else 'ABORT'}"
                      f"{' Tback' if s['tback'] else ''}"
                      f"{'' if s['gate_ms'] is None else ' (gate held ' + str(s['gate_ms']) + ' ms)'}", flush=True)
                if not s["ok"]:
                    fell["abort"] = True
                if not s["ok"] or s["lean_in"] > a.stop_lean:
                    trip = True
                if s["lean_out"] > a.stop_out:
                    # (19:27: R out 7.0 on one walk, 7.2 and 8.2 on the next, then the outward fall)
                    print(f"  OUTWARD lean {s['lean_out']:.1f} > --stop-out {a.stop_out:g} deg: stopping", flush=True)
                    trip = True
        for m in GATE_TO_RE.finditer(txt):
            # the lift gate waited for a level, quiet body and gave up: the firmware has ended the track itself with both
            # feet down (rest pose of the last step end). A stop, not a fall: no more frames, nothing marked as a fall.
            if not fell["gate"]:
                fell["gate"] = dict(n=int(m.group(1)), stance=m.group(2), held_ms=int(m.group(3)), dev=float(m.group(4)),
                                    rate=float(m.group(5)))
                print(f"  LIFT GATE TIMEOUT before step {m.group(1)} ({m.group(2)} stance): held {m.group(3)} ms, roll "
                      f"{m.group(4)} deg off its settled double support, rate {m.group(5)} deg/s -- track ended, feet down",
                      flush=True)
            trip = True
        return trip

    def add(line):
        t = time.monotonic()
        txt = r.cmd(line)
        t_line.append(time.monotonic() - t)
        if not txt.lstrip().startswith("track +"):
            raise RuntimeError(f"refused: {txt.strip()[-160:]}")
        return txt

    saved_sig = {}
    stop = False
    if cw is not None:
        def on_signal(sig, _frame):
            name = signal.Signals(sig).name
            if cw.stop_req is not None:
                raise RuntimeError(f"second {name}")
            cw.request_stop(name)
            print(f"  {name}: the walk stops after the current step (once more: halt)", flush=True)
        for sg_ in (signal.SIGINT, signal.SIGTERM):
            try:
                saved_sig[sg_] = signal.signal(sg_, on_signal)
            except ValueError:               # not the main thread
                pass
    try:
        for l in pl["pre"]:
            r.cmd(l)
        while sent < len(adds) and adds[sent][0] <= ahead_ms:
            if decide_if_due():
                continue
            add(adds[sent][1])
            sent += 1
        r.cmd(pl["play"])
        stop = False
        holding = False
        while True:
            txt = r.cmd("track")
            m = re.search(r"play t (\d+) ms", txt)
            head = int(m.group(1)) if m else 0
            # A held clock (the lift gate parks it at a lift-off until the body is level and quiet) stops `play t`:
            # nothing is wrong -- the upload below is paced by the head, so it just pauses (the queue cannot overflow),
            # and the link is judged by the replies, not by the head moving.
            if "HOLDING" in txt and not holding:
                holding = True
                print(f"  lift gate holding at play t {head} ms (the clock is parked; timeout 3 s)", flush=True)
            elif "HOLDING" not in txt and holding:
                holding = False
                print("  lift gate released", flush=True)
            if "GATE-TIMEOUT" in txt:
                stop = scan(txt) or stop
            # the firmware's TS lines are async output: over Wi-Fi they never reach a command's reply (2026-11-12:
            # 0 of 34 seen live, so the stop rule never ran). `track steps` repeats the last 16: poll it.
            m2 = re.search(r"(\d+) single supports reported", txt)
            if m2 and int(m2.group(1)) > len(steps):
                n_before = len(steps)
                stop = scan(r.cmd("track steps")) or stop
                if learn is not None and len(steps) > n_before and not stop:
                    ankle_step(steps[n_before:])
                if hold is not None and len(steps) > n_before and not stop:
                    heading_step()
            if stop:
                r.cmd("track stop")
                print("  lift gate timeout -> the firmware ended the track" if fell["gate"] else "  stop rule hit -> track stop")
                break
            while sent < len(adds) and adds[sent][0] <= head + ahead_ms:
                if decide_if_due():
                    continue
                stop = scan(add(adds[sent][1])) or stop
                sent += 1
            if re.search(r"^track (done|idle)", txt, re.M) or (sent >= len(adds) and "resting" in txt):
                break
            time.sleep(a.poll)
    except Exception as e:  # noqa: BLE001 -- any failure: freeze, the trim fades
        print(f"  {e!r} -> track halt")
        fell["abort"] = True
        try:
            r.cmd("track halt")
        except Exception:  # noqa: BLE001
            pass
    finally:
        for sg_, h_ in saved_sig.items():
            signal.signal(sg_, h_)
        if cw is not None:
            cw.ended("stop rule / fall / link" if (fell["abort"] or stop) else (cw.reason or "track ended"))
        try:
            r.c.idle(3.0)
            scan(r.cmd("track steps"))
            print(r.cmd("track").strip())
            print(r.cmd("trim").strip())
        except Exception:  # noqa: BLE001
            pass
        if t_line:
            ts = sorted(t_line)
            print(f"upload: {len(t_line)} lines, round trip median {1e3 * ts[len(ts) // 2]:.0f} ms, max "
                  f"{1e3 * ts[-1]:.0f} ms -> {PER_LINE / ts[len(ts) // 2]:.0f} frames/s")
        y_end = None
        try:
            y_end = yaw_of_imu(r.cmd("imu"))
        except Exception:  # noqa: BLE001
            pass
        d_head = None
        if y_end is not None and yaw_state.get("start") is not None:
            d_head = round((y_end - yaw_state["start"] + 180.0) % 360.0 - 180.0, 1)
            print(f"heading over the run (fused yaw, start -> end): {d_head:+.1f} deg (+ = left)")
        ankle = None
        if learn is not None:
            try:
                fell["v"] = r.fallen()
            except Exception:  # noqa: BLE001 -- unknown: do not save
                fell["v"] = True
            saved = not fell["v"] and not fell["abort"]
            if saved:
                learn.save()
            if hasattr(learn, "of"):
                ankle = dict(keys={k: dict(start=lr.start, end=lr.A, keep=lr.keep) for k, lr in learn.by.items()},
                             log=learn.log, saved=saved)
                for k, lr in learn.by.items():
                    print(f"ankle trim ({k}): L {lr.start['L']:.2f} -> {lr.A['L']:.2f}, R {lr.start['R']:.2f} -> "
                          f"{lr.A['R']:.2f}; {'saved' if saved else 'NOT saved (fall / abort / link)'}")
            else:
                ankle = dict(dir=learn.dir, start=learn.start, end=learn.A, keep=learn.keep, log=learn.log, saved=saved)
                print(f"ankle trim ({learn.dir}): L {learn.start['L']:.2f} -> {learn.A['L']:.2f}, R {learn.start['R']:.2f} -> "
                      f"{learn.A['R']:.2f}; {'saved to ' + learn.path if saved else 'NOT saved (fall / abort / link)'}")
        if cw is not None and cw.ph is not None:
            cw.ph.finish()
            sm = cw.ph.summary()
            print(f"pos-hold: start at rest {sm['start_at_rest']}, end at rest {sm['end_at_rest']} (x, y, heading); max |err| lat "
                  f"{sm['max_abs_err_lat_mm']:.0f} fwd {sm['max_abs_err_fwd_mm']:.0f} mm; {sm['no_fix_steps']} steps without a fix, "
                  f"{sm['overrides']} keep-on-board steps" + (f"; LATERAL OFF: {sm['lateral_off']}" if sm['lateral_off'] else ""))
        log(dict(summary=True, note=a.note, args=dict(T=a.T, dir=_label(a), steps=(len(cw.steps) if cw is not None else pl["args"]["n_steps"]), amp=list(pl["amp"]),
                                                      model=pl["model"], first=a.first, pelvis_x=a.pelvis_x,
                                                      ankle_ff=list(pl["args"]["ankle_ff"]), k_ds=pl["args"]["k_ds"],
                                                      lift_gate=pl["args"].get("lift_gate"),
                                                      ankle_side=pl["args"].get("ankle_side"),
                                                      first_boost=pl["args"].get("first_boost"),
                                                      drift_side=pl["args"].get("drift_side")),
                 gate_timeout=fell["gate"],
                 # how the run ended, for tools/vision/run_report.py: a firmware ABORT line, the stop rule or a link error
                 # (abort / stop_rule), the lift-gate timeout, and whether a fall was read (only known with ankle learning)
                 outcome=dict(abort=bool(fell["abort"]), stop_rule=bool(stop), gate_timeout=bool(fell["gate"]),
                              fell=(fell["v"] if learn is not None else None)),
                 steps=steps, line_s=t_line, heading_change_deg=d_head, ankle_learn=ankle,
                 heading=(hold.log if hold is not None else None),
                 cmd_walk=(dict(reason=cw.reason, steps=cw.log, **({"pos_hold": cw.ph.summary()} if cw.ph is not None else {}))
                          if cw is not None else None)))
        r.close()
        print(f"log: {log_path}")


def timing_kw(a):
    """plan()'s keywords for the step-timing flags (empty = today's frames)."""
    return dict(td_pause=a.td_pause, settle=a.settle, lower_frac=a.lower_frac, land_early=a.land_early_mm,
                pre_hike=a.pre_hike, pre_hike_frac=a.pre_hike_frac,
                ss_s=a.ss_s, shift_s=a.shift_s, raise_s=a.raise_s, move_s=a.move_s, lower_s=a.lower_s)


def main_cmd(a, seeds, amp, aff, by_step):
    """--cmd: the walk with live velocity commands (cmd_walk.py). Dry run: the first --dry-steps steps from the command
    now in the source, then a stop. Live: the rolling upload of live() with every step decided as it is reached."""
    import cmd_walk as CW
    me = sys.modules[__name__]
    bad = [n for n, v in (("--seq", a.seq), ("--step", a.step), ("--stride-mm", a.stride_mm), ("--side-mm", a.side_mm),
                          ("--turn", a.turn), ("--heading-rate", a.heading_rate)) if v is not None]
    if bad:
        print(f"REFUSED: {', '.join(bad)} do not combine with --cmd (the commands are the walk; wz is the heading rate)")
        return 2
    if a.model != "dwell":
        print("REFUSED: --cmd needs --model dwell")
        return 2
    if a.ahead < 1.0:
        print("REFUSED: --cmd needs --ahead >= 1 (the next step is decided when the play head is one step from its lift-off)")
        return 2
    try:
        src = CW.make_source(a.cmd)
        lim = {}
        for key, val in (("fwd", a.max_fwd_mm), ("back", a.max_back_mm), ("side", a.max_side_mm), ("yaw", a.max_turn_deg),
                         ("pause", a.pause_steps)):
            if val is not None:
                lim[key] = val
        if lim.get("side", 0.0) > 30.0:
            raise ValueError("--max-side-mm: the pre-check passes fixed sidesteps up to 30 mm at T 4 (pelvis speed at its limit "
                             "at 30, 25 has margin); more is refused")
        if a.slew:
            lim["slew"] = tuple(float(x) for x in a.slew.split(","))
            if len(lim["slew"]) != 3:
                raise ValueError("--slew wants DX,DY,DYAW")
    except ValueError as e:
        print(f"REFUSED: {e}")
        return 2
    if isinstance(src, CW.FileSource):
        c0 = src.read()
        if c0 is None:
            print(f"REFUSED: no readable command in {src.path} ({src.error}); write `0 0 0` there first")
            return 2
        if c0[0] == "stop":
            print(f"{src.path} says stop: nothing to do")
            return 0
    status = a.status or (src.path + ".status" if isinstance(src, CW.FileSource) else None)
    learn = None
    if a.ankle_learn == "on":
        learn = AnkleLearner("cmd", start=(tuple(float(x) for x in a.ankle_seed.split(",")) if a.ankle_seed else None))
        aff = (learn.A["L"], learn.A["R"])
        print(f"ankle trim learning (cmd): starts L {aff[0]:.2f}, R {aff[1]:.2f} deg (--ankle-ff ignored)")
    base = dict(T_step=a.T, seeds=seeds, lead_ms=a.lead, cal=None, amp=amp, model=a.model, ds=a.ds, h=a.h,
                pelvis_x=a.pelvis_x, ankle_ff=aff, k_ds=(tuple(float(x) for x in a.k_ds.split(",")) if a.k_ds else None),
                drop=(a.drop_mm / 1e3 if a.drop_mm is not None else None), lift_gate=a.lift_gate == "on",
                **timing_kw(a))
    hold = HeadingHold(1, [0.0], k=a.heading_k) if (a.heading_hold == "on" and a.live) else None
    side = by_step["ankle_side"] if by_step["ankle_side"] is not None else ANKLE_SIDE_CMD
    boost = by_step["first_boost"] if by_step["first_boost"] is not None else FIRST_BOOST_CMD
    drift = by_step["drift_side"] if by_step["drift_side"] is not None else DRIFT_SIDE_CMD
    print(f"by step: stance ankle ff on a sideways step L,R = {side[0]:g},{side[1]:g} (scaled by |dy| / {SIDE_REF_MM:g} mm), "
          f"first stance step of each side +{boost:g} deg; heading drift sideL,sideR {drift[0]:g},{drift[1]:g} deg/step "
          f"(forward {YAW_DRIFT:g})")

    side_timing = a.side_timing or ("plain" if a.td_pause else "same")
    ph = None
    if a.pos_hold == "on":
        if not a.live:
            print("pos-hold on: a dry run reads no tracker and applies no correction (the correction needs the robot's "
                  "position over time); the rehearsal below is the plain command")
        else:
            ph = make_poshold(a)
            if ph is None:
                return 2

    def walk(cal):
        b = dict(base, cal=cal)
        w = CW.CmdWalk(me, src, b, first=a.first, hold=hold, limits=lim, max_steps=a.max_steps,
                       max_time=a.max_time or None, status=status, fixed_tilt=a.swing_tilt, yaw_trim=a.yaw_trim,
                       allow_over=a.allow_over, ankle_side=side, first_boost=boost, drift_side=drift,
                       step_s=a.Teff, poshold=ph, side_timing=side_timing)
        return w
    if side_timing == "plain" and a.td_pause:
        print("side timing plain: a step with more than 3 mm of sideways travel takes the plain timing (no --td-pause, the "
              "schedule's shift / settle / single support) for its single support, the next step throughout "
              "(a step's own pause and shift come before it is decided)")
    print(f"cmd walk: T {a.T:g} s/step, commands from {a.cmd}, ahead {a.ahead:g}; envelope forward {lim.get('fwd', CW.FWD_MM):g} / "
          f"back {lim.get('back', CW.BACK_MM):g} / side {lim.get('side', CW.SIDE_MM):g} mm, turn {lim.get('yaw', CW.YAW_DEG):g} deg "
          f"per step, slew {lim.get('slew', CW.SLEW)}; max {a.max_steps} steps"
          + (f", {a.max_time:g} s" if a.max_time else "") + (f"; status {status}" if status else ""))
    if not a.live:
        cw = walk(None)
        try:
            cw.rehearse(a.dry_steps)
            pd = plan(**cw.plan_kwargs())
        except ValueError as e:
            print(f"REFUSED: the kinematics cannot do this plan ({e})")
            return 2
        pd["allow_over"] = a.allow_over
        pd["allow_under"] = a.allow_under
        print(f"\ndry run: the first {a.dry_steps} steps from the command read now ({cw.log[0]['requested']}), each step "
              f"re-reads SOURCE live; then a stop (the stop step is the last one below)")
        dry(pd, a)
        return 0
    sys.path.insert(0, os.path.join(ROOT, "experiments", "stepper"))
    import robot_io
    probe = robot_io.Robot(host=a.host)
    cal = {n: (z, d) for n, (_, z, d) in probe.cal.items()}
    probe.close()
    cw = walk(cal)
    try:
        pl = plan(**dict(cw.plan_kwargs(), **({"ankle_steps": [learn.A[cw.provisional() and ("R" if cw.swing_of(0) == "L" else "L")]]}
                                              if learn is not None else {})))
    except ValueError as e:
        print(f"REFUSED: the kinematics cannot do this plan ({e}) -- nothing sent")
        return 2
    pl["allow_over"] = a.allow_over
    pl["allow_under"] = a.allow_under
    pl["hold"], pl["learn"], pl["cmdwalk"] = hold, learn, cw
    if hold is not None:
        print(f"heading hold: the commanded heading rate is wz x T per step, feed-forward + feedback k {a.heading_k:g}")
    if a.gate:
        pl["pre"] = [f"liftgate set {' '.join(x.strip() for x in a.gate.split(','))}"] + pl["pre"]
    try:
        live(pl, a)
    finally:
        if ph is not None:
            ph.S.stop()
    return 0


def make_poshold(a):
    """--pos-hold on, live: the tracker sampler (a thread reading camd's emeet_latest.jpg), the start fix, the PosHold.
    None (after printing why) when the start pose cannot be read: a hold that cannot see the robot is refused."""
    import pos_hold as PH
    sys.path.insert(0, os.path.join(ROOT, "tools", "vision"))
    import vgeom as G
    from camd import DEFAULT_DIR
    cfg = G.load_config(a.pos_config)
    d = a.pos_camd_dir or DEFAULT_DIR
    path = os.path.join(d, "emeet_latest.jpg")
    loc = PH.Locator(cfg, pose_path=os.path.join(d, "emeet_poshold_pose.json"),
                    pose_read=[os.path.join(d, "emeet_board_pose.json")])
    sam = PH.PosSampler(loc, path)
    margin = a.keep_on_board if a.keep_on_board is not None else PH.MARGIN_MM
    W, L = cfg["board"]["width_mm"], cfg["board"]["length_mm"]
    ph = PH.PosHold(sam, a.Teff, margins=lambda x, y: G.edge_margins(cfg, x, y), k=a.pos_k, ki=a.pos_ki, rate_mm=a.pos_rate_mm, min_step_mm=a.pos_min_step_mm, clip=a.pos_clip,
                    margin_mm=(margin if margin > 0 else None), no_fix_steps=a.pos_no_fix_steps,
                    window_steps=a.pos_window_steps, delay_steps=a.pos_delay_steps, deadband_mm=a.pos_deadband_mm,
                    centre=(W / 2.0, L / 2.0))
    sam.start()
    print(f"pos-hold: tracker frames from {path}; step {a.Teff:g} s, sample = fit over {a.pos_window_steps:g} steps "
          f"({ph.window_s():g} s), k {a.pos_k:g} ki {a.pos_ki:g} rate {a.pos_rate_mm:g} mm/step, clip lat {a.pos_clip[0]:g} / fwd {a.pos_clip[1]:g} mm per step, "
          f"delay {a.pos_delay_steps} steps, deadband {a.pos_deadband_mm:g} mm, "
          + (f"keep-on-board {margin:g} mm" if margin > 0 else "keep-on-board off")
          + f", no-fix limit {a.pos_no_fix_steps} steps; waiting for the start pose ...", flush=True)
    f = ph.start_fix(ph.window_s() + 6.0)
    if f is None:
        print(f"REFUSED: --pos-hold on but the tracker gives no position of the robot standing ({sam.why or 'too few frames'}); "
              f"is camd --source emeet running and the robot's tags in view? Nothing sent")
        sam.stop()
        return None
    ph.begin(f)
    m = G.edge_margins(cfg, f["x"], f["y"])
    print(f"pos-hold: start x {f['x']:.0f} y {f['y']:.0f} mm heading {f['h']:.0f} deg ({f['n']} frames); margins "
          f"L {m['left']:.0f} R {m['right']:.0f} near {m['near']:.0f} far {m['far']:.0f} mm"
          + ("" if cfg["board"].get("measured") else "  (board layout not measured: absolute +-100 mm, relative changes ~10 mm)"),
          flush=True)
    return ph


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--T", type=float, default=6.0,
                    help="s per step: the dwell schedule has 6, 5, 4, 3.4, 3.2 (the SIL floor), 3; start at 6")
    ap.add_argument("--below-floor", action="store_true", help="allow T under dwell.T_FLOOR (the SIL says no)")
    ap.add_argument("--dir", default="march", choices=list(G.DIRS))
    ap.add_argument("--steps", type=int, default=6)
    ap.add_argument("--stride-mm", type=float, default=None,
                    help="dwell: fwd / back step length, mm (default 20: the presets'); the pre-check refuses what the "
                         "kinematics or the shift's timing cannot do (README: the stride table)")
    ap.add_argument("--side-mm", type=float, default=None, help="dwell: sideL / sideR step length, mm (default 15)")
    ap.add_argument("--drop-mm", type=float, default=None,
                    help="dwell: the crouch depth, mm below the straight stand (default 20: the robot's). A shallower crouch "
                         "leaves the swing leg more reach (README: the stride table by drop) but cannot lift the foot "
                         "at all below ~15 mm (IK out of reach); not run on the robot")
    ap.add_argument("--swing-tilt", type=float, default=0.0,
                    help="dwell: deg, toe-down tilt of the swing sole (ankle pitch +), in over the raise, flat at lift-off "
                         "and touchdown; relieves the swing ankle pitch that caps the stride. Not run on the robot")
    ap.add_argument("--step", default=None, metavar="DX,DY,DYAW",
                    help="dwell: one arbitrary per-step displacement instead of --dir: forward mm, left mm, heading "
                         "change deg per step (+ = left); diagonal and curved walking")
    ap.add_argument("--seq", default=None, metavar="SEG:N,...",
                    help="dwell: several segments in one continuous track, e.g. fwd:4,turnL:3,fwd:4,sideR:4,back:4 "
                         "(a --dir preset or step(dx_mm,dy_mm,dyaw_deg), colon, step count; --steps is ignored; "
                         "--stride-mm / --side-mm scale the presets)")
    ap.add_argument("--first", default="R", choices=["L", "R"],
                    help="the first swing foot (R: the first stance is the left; the first right stance tipped in "
                         "fwd / back / turn runs out of the rest crouch)")
    ap.add_argument("--seed", default=f"{ROBOT_SEED[0]:g},{ROBOT_SEED[1]:g}", help="trim T L,R (hike deg)")
    ap.add_argument("--amp", default=None,
                    help="k_static L,R: the pelvis over the stance foot from before lift-off to touchdown (x 62 mm)")
    ap.add_argument("--model", default="dwell", choices=["dwell", "sway"],
                    help="dwell (default): the static requirement as a hard constraint; sway: A2's sine (refused "
                         "by the pre-check)")
    ap.add_argument("--ds", type=float, default=None, help="dwell: double-support fraction of T (default per T)")
    ap.add_argument("--h", type=float, default=None, help="dwell: swing sole height, m (default 0.022)")
    ap.add_argument("--yaw-trim", type=float, default=YAW_TRIM,
                    help="dwell: heading change per step asked, deg, + = to the robot's left, ADDED to the direction's "
                         "own turn (turnL / turnR +-4). The robot turns 1.34 x asked + 1.9 deg/step left; the default "
                         "-1.4 cancels the drift. 0 to see the bare drift")
    ap.add_argument("--lead", type=int, default=2500, help="ms from where it stands to the first frame")
    ap.add_argument("--heading-hold", choices=["on", "off"], default="on",
                    help="dwell, live: read the fused yaw after every step and re-plan the asked turn of the steps not "
                         "uploaded yet (replaces --yaw-trim with the heading model's feed-forward + feedback)")
    ap.add_argument("--turn", type=float, default=None,
                    help="with the heading hold: the total heading change to reach over the walk, deg, + = left "
                         "(default: 0, or +-5 deg/step for turnL / turnR)")
    ap.add_argument("--heading-rate", type=float, default=None,
                    help="with the heading hold: a commanded heading rate instead, deg/s (+ = left)")
    ap.add_argument("--heading-k", type=float, default=0.5, help="heading hold: fraction of the error fixed per step")
    ap.add_argument("--pelvis-x", choices=["mid", "stance"], default=ROBOT_PELVIS_X,
                    help="dwell: the pelvis fore-aft through single support -- over the stance foot, moving forward only "
                         "in the shift (the default; fwd T 3.4 with mid: right-stance pitch +7 deg), or over the feet's "
                         "mid point")
    ap.add_argument("--ahead", type=float, default=None,
                    help="steps the host stays ahead of the play head (default 2; 1 with --cmd, which needs >= 1)")
    ap.add_argument("--cmd", default=None, metavar="SOURCE",
                    help="live velocity commands (cmd_walk.py): file:PATH (a text file re-read every step: `vx vy wz` in "
                         "mm/s, mm/s, deg/s, or `stop` / `pause`) or stdin. The walk has no fixed length; every step is "
                         "clamped, slewed and pre-checked before it is uploaded. --dir / --steps / --step / --seq / "
                         "--stride-mm / --side-mm / --turn are not used")
    ap.add_argument("--side-timing", choices=["plain", "same"], default=None,
                    help="--cmd: with `plain` a step with more than 3 mm of sideways travel takes the plain timing (no --td-pause, "
                         "the schedule's shift / settle / single support for T) for its single support, and the next step "
                         "takes it throughout; `same` keeps one timing. Default plain when --td-pause is given (README: Position hold)")
    ap.add_argument("--pos-hold", choices=["on", "off"], default="off",
                    help="--cmd, live: close the robot's position from the overhead camera tracker (tools/vision, camd's "
                         "emeet_latest.jpg; read-only, no camera access). Per step: the median position over the last "
                         "--pos-window-steps steps against the start pose plus the integrated command, a bounded correction "
                         "-k x error on the next step's dy / dx (README: Position hold). Default off: frames unchanged")
    ap.add_argument("--pos-k", type=float, default=0.3, help="--pos-hold: fraction of the error (beyond the dead band) corrected per step")
    ap.add_argument("--pos-ki", type=float, default=0.06, help="--pos-hold: integral gain per step (learns the standing creep and the sideways response; clipped to the clip)")
    ap.add_argument("--pos-min-step-mm", type=float, default=4.0, help="--pos-hold: corrections smaller than this are added up and applied as fewer, larger steps")
    ap.add_argument("--pos-rate-mm", type=float, default=5.0, help="--pos-hold: the correction may change by this much per step")
    ap.add_argument("--pos-clip", default="6,10", metavar="LAT,FWD", help="--pos-hold: largest correction per step, mm (lateral, fore-aft)")
    ap.add_argument("--pos-window-steps", type=float, default=3.0, help="--pos-hold: steps in the fit window (3 = one and a half left-right cycles)")
    ap.add_argument("--pos-delay-steps", type=int, default=4, help="--pos-hold: decided steps not yet visible in a sample (compensated)")
    ap.add_argument("--pos-deadband-mm", type=float, default=5.0, help="--pos-hold: error under this is left alone (soft: only the excess is corrected)")
    ap.add_argument("--pos-no-fix-steps", type=int, default=4,
                    help="--pos-hold: steps without a tracker fix the last trim is held for; more than this with a non-zero command = stop")
    ap.add_argument("--keep-on-board", type=float, default=None, metavar="MM",
                    help="--pos-hold: a step toward the board centre when the predicted position is within MM of an edge "
                         "(default 250; 0 = off)")
    ap.add_argument("--pos-camd-dir", default=None, help="--pos-hold: camd's directory (default camd.DEFAULT_DIR)")
    ap.add_argument("--pos-config", default=None, help="--pos-hold: tools/vision config JSON (board, robot tags)")
    ap.add_argument("--max-steps", type=int, default=120, help="--cmd: stop after this many steps (default 120)")
    ap.add_argument("--max-time", type=float, default=600.0, help="--cmd: stop after this many seconds (default 600; 0 = none)")
    ap.add_argument("--max-fwd-mm", type=float, default=None, help="--cmd: forward step cap (default 50)")
    ap.add_argument("--max-back-mm", type=float, default=None, help="--cmd: backward step cap (default 32; 40 is proven only as a fixed walk, README envelope table)")
    ap.add_argument("--max-side-mm", type=float, default=None, help="--cmd: sideways step cap, the lead step's dy = the body's travel per PAIR of steps (default 15, up to 30; "
                                                                  "the delivered sideways velocity is at most cap / (2 T))")
    ap.add_argument("--max-turn-deg", type=float, default=None, help="--cmd: heading change cap per step (default 5)")
    ap.add_argument("--slew", default=None, metavar="DX,DY,DYAW",
                    help="--cmd: largest change of the step's dx mm, dy mm, dyaw deg from one step to the next (default 20,10,3)")
    ap.add_argument("--pause-steps", type=int, default=None,
                    help="--cmd: a march in place (`pause` / `0 0 0`) longer than this many steps in a row ends the walk "
                         "(default 15 = 60 s at T 4; 0 = no limit)")
    ap.add_argument("--status", default=None, help="--cmd: the host's status file, one JSON line, rewritten every step "
                    "(default PATH.status for file:PATH)")
    ap.add_argument("--dry-steps", type=int, default=8, help="--cmd dry run: how many steps to rehearse")
    ap.add_argument("--stop-lean", type=float, default=14.0,
                    help="deg toward the swing foot -> track stop (inward tips to ~10 deg recover and the walk goes on, hw 2026-11-12 18:2x; the firmware aborts at 15)")
    ap.add_argument("--stop-out", type=float, default=6.0,
                    help="deg AWAY from the swing foot (outward over the stance foot) -> track stop. hw 2026-11-12: "
                         "clean walks stay <= 5.0; the 19:27 fall came after R out 7.0, 7.2, 8.2")
    ap.add_argument("--ms-per-line", type=float, default=29.0, help="dry run: CLI round trip (measured 29 ms median, 2026-11-12)")
    ap.add_argument("--poll", type=float, default=0.25)
    ap.add_argument("--live", action="store_true", help="SEND to the robot (Stage C, a person at it)")
    ap.add_argument("--note", default="", help="free text into the log's summary row, e.g. where the robot stands")
    ap.add_argument("--ankle-ff", default=",".join(f"{x:g}" for x in ROBOT_ANKLE_FF),
                    help="dwell: L,R deg, a fixed feed-forward on the STANCE ankle roll, + = outward (the CoM toward the "
                         "stance foot's outside, ~3.7 mm/deg), ramped in over the raise, held, out over the lower "
                         "(default 2,2: hw 2026-11-12 19:10-19:28; 0,0 = off)")
    ap.add_argument("--ankle-ff-side", default=None, metavar="L,R",
                    help="dwell: the stance ankle's ff (deg) on a sideways step toward its own side: the RIGHT stance on "
                         "a step right (dy < 0) gets R instead of 2, the LEFT stance on a step left L; added as "
                         "(value - 2) x min(1, |dy| / 15 mm) to the stance ankle (--ankle-ff or the learner's value). "
                         "Default 2,3 with --cmd, 2,2 (no change) otherwise")
    ap.add_argument("--first-ankle-boost", type=float, default=None, metavar="DEG",
                    help="dwell: deg added to the stance ankle ff of the first stance step on each side of a walk only "
                         "(default 1 with --cmd, 0 otherwise)")
    ap.add_argument("--heading-drift-side", default=None, metavar="L,R",
                    help="dwell: the heading drift (deg / step, + = left; 1.9 = the forward-walking calibration) the "
                         "feed-forward cancels on a sideL / sideR step, blended by the step's lateral share (a --seq / "
                         "--cmd step; the heading hold's and --yaw-trim's feed-forward both follow). Default 0.94,1.9 "
                         "with --cmd, 1.9,1.9 (no change) otherwise")
    ap.add_argument("--td-pause", type=float, default=0.0, metavar="S",
                    help="dwell: the next step's lateral shift (and the fore-aft pelvis move with it) starts S seconds AFTER "
                         "the planned touchdown of the swing foot: the lowering, the ankle ff's and the swing tilt's ramp-out "
                         "are over first. Every step but the first, and the final shift back, is S longer (the plan "
                         "summary prints the timeline and the step time) unless --settle shortens the pre-lift settle. "
                         "Default 0: today's frames")
    ap.add_argument("--settle", type=float, default=None, metavar="S2",
                    help="dwell: the pre-lift settle, both feet down, s (default per T: 0.4 at T 3.4-6); the shift and the "
                         "single support keep their lengths, so the step is (S2 - default) longer or shorter")
    ap.add_argument("--lower-frac", type=float, default=None, metavar="F",
                    help="dwell: the swing foot's lowering as a share of the single support (default 0.35; 0.20 to 0.45); "
                         "the move takes what the raise (0.35) and the lowering leave. The step time does not change")
    ap.add_argument("--lower-s", type=float, default=None, metavar="S3",
                    help="dwell: the same, as the lowering's duration in s")
    ap.add_argument("--ss-s", type=float, default=None, metavar="S",
                    help="dwell: the single support, s (raise + move + lower, split 0.35 : 0.30 : 0.35 unless --raise-s / "
                         "--move-s / --lower-s are given). With --shift-s the step is shift + settle + this (+ --td-pause): "
                         "--T then only supplies what is not given, and the T floor / shift >= 1.4 s guards print a warning "
                         "instead of refusing")
    ap.add_argument("--shift-s", type=float, default=None, metavar="S",
                    help="dwell: the lateral shift's duration, s (default per T: 1.4 at T 3.4-6); see --ss-s")
    ap.add_argument("--raise-s", type=float, default=None, metavar="S", help="dwell: the swing foot's raise, s; see --ss-s")
    ap.add_argument("--move-s", type=float, default=None, metavar="S", help="dwell: the swing foot's move, s; see --ss-s")
    ap.add_argument("--pre-hike", default=None, metavar="L,R",
                    help="dwell: deg per stance side, a stance hip-roll hike IN THE PLAN (the firmware trim's convention: + "
                         "raises the swing-side hip): min-jerk in over the settle (both feet down), held through the single "
                         "support, out over the start of the next step (over --td-pause if given, else half the shift). The "
                         "swing leg is solved for the rolled pelvis so its sole stays where the plan has it. The firmware's "
                         "learned trim T of a side with a pre-hike is seeded 0 (printed) so the two do not add. Seeds to try: "
                         "the trims the robot learns, 5.2,3.4. Default off")
    ap.add_argument("--pre-hike-frac", type=float, default=1.0, metavar="F",
                    help="dwell: the pre-hike's ramp-in takes this share of the settle (default 1: all of it; 0.05 to 1)")
    ap.add_argument("--allow-under", action="store_true",
                    help="dwell: the static-shift pre-check's lower bound (the plan's own k_static) is a WARNING, not a refusal "
                         "(an --amp / timing test that lifts with the pelvis short of the requirement); the joint-range, "
                         "IK, pelvis-speed and swing-joint-speed checks stay refusals")
    ap.add_argument("--land-early-mm", type=float, default=0.0, metavar="D",
                    help="dwell: 0 to 4 mm: the swing sole is commanded D mm BELOW the floor at touchdown (a slight press: "
                         "the foot reaches the floor early in height terms), held through the pause and let up over the "
                         "next shift. The pre-check's joint margins cover it")
    ap.add_argument("--ankle-learn", choices=["on", "off"], default="off",
                    help="dwell: learn the stance-ankle trim per side at step rate (AnkleLearner): starts from "
                         "results/ankle_trim_state.json for this --dir (seeds 2/2, sideR R 3), replaces --ankle-ff, "
                         "re-plans the steps not uploaded yet (use --ahead 1), saved after a run with no fall / abort")
    ap.add_argument("--ankle-seed", default=None,
                    help="with --ankle-learn on: start from these L,R deg instead of the state file")
    ap.add_argument("--k-ds", default=None,
                    help="dwell: L,R, the two-stage shift -- double support only to k_ds x 62 mm, the rest to --amp over "
                         "the raise; 0 = single-stage for that side (default: single-stage both)")
    ap.add_argument("--lift-gate", choices=["on", "off"], default="on",
                    help="dwell: kGate on the last frame before each lift-off; the firmware then waits there while the "
                         "body is off level (|roll - its settled double support| > 3 deg) or moving (> 15 deg/s) and ends "
                         "the track, feet down, after a 3 s hold (`liftgate`; sim/lift_gate.py). off = frames as before")
    ap.add_argument("--gate", default=None, metavar="LVL,RATE,DWELL,TIMEOUT",
                    help="send `liftgate set` first: deg, deg/s, ms, ms (firmware defaults 3,15,150,3000)")
    ap.add_argument("--allow-over", action="store_true",
                    help="let the pre-check pass a shift past k_static + 0.05 (an --amp A/B on the outward side)")
    ap.add_argument("--host", default="192.168.2.90")
    ap.add_argument("--log")
    a = ap.parse_args(argv)
    try:
        a.pos_clip = tuple(float(x) for x in a.pos_clip.split(","))
        if len(a.pos_clip) != 2 or min(a.pos_clip) < 0:
            raise ValueError
    except ValueError:
        print("REFUSED: --pos-clip wants LAT,FWD (mm, >= 0)")
        return 2
    if a.pos_hold == "on" and not a.cmd:
        print("REFUSED: --pos-hold on needs --cmd (a fixed walk's steps are all planned before it starts)")
        return 2
    if a.ahead is None:
        a.ahead = 1.0 if a.cmd else 2.0
    seeds = tuple(float(x) for x in a.seed.split(","))
    try:
        if a.pre_hike is not None:
            a.pre_hike = parse_pair(a.pre_hike, "--pre-hike", 0.0, 12.0)
            if a.model != "dwell":
                raise ValueError("--pre-hike needs --model dwell")
            if not 0.05 <= a.pre_hike_frac <= 1.0:
                raise ValueError("--pre-hike-frac wants 0.05 to 1")
    except ValueError as e:
        print(f"REFUSED: {e}")
        return 2
    if a.pre_hike is not None and any(a.pre_hike):
        print(f"pre-hike: L {a.pre_hike[0]:g} / R {a.pre_hike[1]:g} deg on the stance hip roll in the plan (in over "
              f"{a.pre_hike_frac:g} of the settle, held through the single support, out over the start of the next step); "
              f"the firmware's learned trim T is seeded "
              + ", ".join(f"{n} {0.0 if h_ else v_:g}" for n, v_, h_ in zip("LR", seeds, a.pre_hike)) + f" (was {seeds[0]:g},{seeds[1]:g}); "
              "its closed loop stays on")
    import dwell as DW
    amp = tuple(float(x) for x in a.amp.split(",")) if a.amp else ROBOT_AMP
    aff = tuple(float(x) for x in a.ankle_ff.split(","))
    try:
        by_step = dict(
            ankle_side=(parse_pair(a.ankle_ff_side, "--ankle-ff-side", 0.0, 5.0) if a.ankle_ff_side is not None else None),
            first_boost=a.first_ankle_boost,
            drift_side=(parse_pair(a.heading_drift_side, "--heading-drift-side", -5.0, 5.0)
                        if a.heading_drift_side is not None else None))
        if by_step["first_boost"] is not None and not 0.0 <= by_step["first_boost"] <= 3.0:
            raise ValueError("--first-ankle-boost wants 0 to 3 deg")
    except ValueError as e:
        print(f"REFUSED: {e}")
        return 2
    explicit = any(v is not None for v in (a.ss_s, a.shift_s, a.raise_s, a.move_s))
    try:
        if a.lower_s is not None and a.lower_frac is not None:
            raise ValueError("--lower-frac and --lower-s are alternatives")
        if a.model != "dwell" and (a.td_pause or a.settle is not None or a.lower_frac is not None or a.land_early_mm
                                   or a.lower_s is not None or explicit):
            raise ValueError("--td-pause / --settle / --lower-frac / --lower-s / --ss-s / --shift-s / --raise-s / --move-s / "
                             "--land-early-mm need --model dwell")
        for nm, v, lo, hi in (("--td-pause", a.td_pause, 0.0, 2.0), ("--settle", a.settle, 0.0, 2.0),
                              ("--lower-frac", a.lower_frac, DW.LOWER_FRAC_RANGE[0], DW.LOWER_FRAC_RANGE[1]),
                              ("--land-early-mm", a.land_early_mm, 0.0, DW.LAND_EARLY_MAX_MM)):
            if v is not None and not lo - 1e-9 <= v <= hi + 1e-9:
                raise ValueError(f"{nm} {v:g}: want {lo:g} to {hi:g}")
        tk = timing_kw(a)
        DW.resolve_times(a.T, **({} if a.ds is None else {"ds": a.ds}), settle=a.settle, ss_s=a.ss_s, shift_s=a.shift_s,
                         raise_s=a.raise_s, move_s=a.move_s, lower_s=a.lower_s, lower_frac=a.lower_frac)
    except ValueError as e:
        print(f"REFUSED: {e}")
        return 2
    if explicit and a.model == "dwell":
        rt = DW.resolve_times(a.T, **({} if a.ds is None else {"ds": a.ds}), settle=a.settle, ss_s=a.ss_s, shift_s=a.shift_s,
                              raise_s=a.raise_s, move_s=a.move_s, lower_s=a.lower_s, lower_frac=a.lower_frac)
        print(f"note: explicit phase times: the step is shift {rt['t_sh']:.2f} + settle {rt['t_set']:.2f} + single support "
              f"{rt['t_ss']:.2f}" + (f" + pause {a.td_pause:g}" if a.td_pause else "") + f" = {rt['period'] + a.td_pause:.2f} s; "
              f"--T {a.T:g} is not the step time (it only supplies what was not given)")
        for nm, v, lim, why in (("single support", rt["t_ss"], 1.8, "the SIL-validated schedule needs >= 1.8 s"),
                                ("shift", rt["t_sh"], 1.4, "the schedule's shift is >= 1.4 s: a shorter one rings through lift-off"),
                                ("step", rt["period"], DW.T_FLOOR, f"the dwell gait's SIL floor T is {DW.T_FLOOR:g} s")):
            if v < lim - 1e-9:
                print(f"WARNING: {nm} {v:.2f} s is under {lim:g} s ({why}); planned anyway because it was given")
    elif a.model == "dwell" and a.T < DW.T_FLOOR and not a.below_floor:
        print(f"REFUSED: T {a.T:g} s is under the dwell gait's SIL floor ({DW.T_FLOOR:g} s): it falls or leans 11-14 deg "
              f"there (experiments/roll-balance/README.md); --below-floor to plan it anyway")
        return 2
    a.Teff = DW.effective_step_s(a.T, a.td_pause, a.settle, **({} if a.ds is None else {"ds": a.ds}),
                                 ss_s=a.ss_s, shift_s=a.shift_s, raise_s=a.raise_s, move_s=a.move_s, lower_s=a.lower_s,
                                 lower_frac=a.lower_frac) if a.model == "dwell" else a.T
    if a.cmd:
        return main_cmd(a, seeds, amp, aff, by_step)
    learn = None
    if a.ankle_learn == "on" and a.model == "dwell":
        learn = AnkleLearner(a.dir, start=(tuple(float(x) for x in a.ankle_seed.split(","))
                                           if a.ankle_seed else None))
        aff = (learn.A["L"], learn.A["R"])
        print(f"ankle trim learning ({_label(a)}): starts L {aff[0]:.2f}, R {aff[1]:.2f} deg (--ankle-ff ignored)"
              + ("; --ahead > 1: an update reaches the side's next step only if it is not uploaded yet" if a.ahead > 1.0 else ""))
    kds = tuple(float(x) for x in a.k_ds.split(",")) if a.k_ds else None
    cmd = seq = None
    try:
        if sum(x is not None for x in (a.seq, a.step)) > 1:
            raise ValueError("--seq and --step are alternatives")
        if (a.seq or a.step or a.stride_mm is not None or a.side_mm is not None or a.drop_mm is not None
                or a.swing_tilt) and a.model != "dwell":
            raise ValueError("--stride-mm / --side-mm / --step / --seq / --drop-mm / --swing-tilt need --model dwell")
        if a.seq:
            seq = parse_seq(a.seq, a.stride_mm, a.side_mm)
            a.steps = len(expand_seq(seq, a.first))
        elif a.step:
            cmd = parse_step(a.step)
        elif a.stride_mm is not None or a.side_mm is not None:
            cmd = preset_cmd(a.dir, a.stride_mm, a.side_mm)
    except ValueError as e:
        print(f"REFUSED: {e}")
        return 2
    if learn is not None and (seq or cmd):
        keys = [c["key"] for c, _ in seq] if seq else [cmd["key"]]
        st0 = tuple(float(x) for x in a.ankle_seed.split(",")) if a.ankle_seed else None
        learn = AnkleBank(keys, start=st0) if seq else AnkleLearner(keys[0], start=st0)
        first_lr = learn.of(keys[0]) if seq else learn
        aff = (first_lr.A["L"], first_lr.A["R"])
        if seq and len(learn.by) > 1:
            print("ankle trim learning per segment key: " + ", ".join(f"{k} L {lr.A['L']:.2f} R {lr.A['R']:.2f}"
                                                                      for k, lr in learn.by.items()))
    kw_cmd = dict(cmd=cmd, seq=seq, drop=(a.drop_mm / 1e3 if a.drop_mm is not None else None), swing_tilt=a.swing_tilt,
                  lift_gate=a.lift_gate == "on", **{k: v for k, v in by_step.items() if v is not None},
                  **timing_kw(a))
    if not a.live:
        try:
            pd = plan(a.T, a.steps, a.dir, seeds, a.lead, first=a.first, amp=amp, model=a.model, ds=a.ds, h=a.h,
                      yaw_trim=a.yaw_trim, pelvis_x=a.pelvis_x, ankle_ff=aff, k_ds=kds, **kw_cmd)
        except ValueError as e:     # the IK cannot reach the plan's feet / pelvis
            print(f"REFUSED: the kinematics cannot do this plan ({e})")
            return 2
        pd["allow_over"] = a.allow_over
        pd["allow_under"] = a.allow_under
        dry(pd, a)
        return 0
    sys.path.insert(0, os.path.join(ROOT, "experiments", "stepper"))
    import robot_io
    probe = robot_io.Robot(host=a.host)
    cal = {n: (z, d) for n, (_, z, d) in probe.cal.items()}
    probe.close()
    try:
        pl = plan(a.T, a.steps, a.dir, seeds, a.lead, cal=cal, first=a.first, amp=amp, model=a.model, ds=a.ds, h=a.h,
                  yaw_trim=a.yaw_trim, pelvis_x=a.pelvis_x, ankle_ff=aff, k_ds=kds, **kw_cmd)
    except ValueError as e:
        print(f"REFUSED: the kinematics cannot do this plan ({e}) -- nothing sent")
        return 2
    if a.model == "dwell" and a.heading_hold == "on":
        if a.heading_rate is not None:
            dpsi = a.heading_rate * a.Teff
        elif a.turn is not None:
            dpsi = a.turn / pl["args"]["n_steps"]
        elif seq or cmd:
            dpsi = list(pl["dpsi"])           # the active segment's commanded heading change, per step
        else:
            dpsi = {"turnL": 5.0, "turn": 5.0, "turnR": -5.0}.get(a.dir, 0.0)
        hold = HeadingHold(pl["args"]["n_steps"], dpsi, k=a.heading_k,
                           drift=([step_drift(c, by_step["drift_side"]) for c in pl["cmds"]]
                                  if by_step["drift_side"] is not None and pl["cmds"] is not None else None))
        pl = plan(**{**pl["args"], "yaw_trim": hold.asked})
        pl["hold"] = hold
        print(f"heading hold: {dpsi if not isinstance(dpsi, list) else 'per segment (' + ', '.join(f'{x:+g}' for x in sorted(set(dpsi))) + ')'}"
              f"{' deg/step commanded' if not isinstance(dpsi, list) else ' deg/step'}, feed-forward {hold.asked[0]:+.2f} "
              f"deg/step asked at the start, k {a.heading_k:g}")
    if learn is not None:
        hold_ = pl.get("hold")
        pl = plan(**{**pl["args"], "ankle_steps": ankle_steps_for(pl["sway"], learn if hasattr(learn, "of") else learn.A),
                     **({"yaw_trim": hold_.asked} if hold_ is not None else {})})
        pl["hold"], pl["learn"] = hold_, learn
    pl["allow_over"] = a.allow_over
    pl["allow_under"] = a.allow_under
    if a.gate:
        pl["pre"] = [f"liftgate set {' '.join(x.strip() for x in a.gate.split(','))}"] + pl["pre"]
    live(pl, a)
    return 0


if __name__ == "__main__":
    sys.exit(main())
