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

    python track_host.py --T 4 --dir march --steps 6              # dry run: what would be sent
    python track_host.py --T 4 --dir march --steps 6 --live       # on the robot (Stage C only)
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
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
                   r"( Tback)?", re.M)


def lines_for(frames):
    """[(t_ms of the line's last frame, 'track add ...')], 4 frames a line, in order."""
    out = []
    for i in range(0, len(frames), PER_LINE):
        chunk = frames[i:i + PER_LINE]
        out.append((chunk[-1][0], "track add " + " ".join(h for _, h, _ in chunk)))
    return out


def plan(T_step, n_steps, direction, seeds=ROBOT_SEED, lead_ms=2500, cal=None, first="R", amp=ROBOT_AMP,
         model="dwell", ds=None, h=None, yaw_trim=YAW_TRIM, pelvis_x=ROBOT_PELVIS_X, ankle_ff=ROBOT_ANKLE_FF,
         k_ds=None, ankle_steps=None):
    """yaw_trim (deg / step, + = left) is ADDED to the direction's own turn (turnL / turnR ask +-4)."""
    gkw = {}
    if amp[1] is None:
        import dwell as DW
        amp = (amp[0], DW.k_static_R(T_step))
    dkw = dict(G.DIRS[direction])
    if model == "dwell":
        gkw = dict(**({"ds": ds} if ds is not None else {}), **({"h": h} if h is not None else {}),
                   **({"pelvis_x": pelvis_x} if pelvis_x != "mid" else {}),
                   **({"ankle_ff": tuple(ankle_ff)} if any(ankle_ff) else {}),
                   **({"k_ds": tuple(k_ds)} if k_ds and any(k_ds) else {}),
                   **({"ankle_ff_steps": list(ankle_steps)} if ankle_steps is not None else {}))
        if isinstance(yaw_trim, (list, tuple)):
            dkw["yaw_trim"] = list(yaw_trim)          # the heading hold: the asked turn per step, complete
        else:
            dkw["yaw_trim"] = dkw.get("yaw_trim", 0.0) + yaw_trim
    sw, fr = G.gait(T_step, n_steps, first=first, cal=cal, kL=amp[0], kR=amp[1], model=model, **dkw, **gkw)
    pre = [f"trim T L {seeds[0]:g}", f"trim T R {seeds[1]:g}", "track clear"]
    return dict(sway=sw, frames=fr, pre=pre, adds=lines_for(fr), play=f"track play {int(lead_ms)}",
                duration_s=fr[-1][0] / 1e3, cal=cal, amp=amp, model=model,
                args=dict(T_step=T_step, n_steps=n_steps, direction=direction, seeds=seeds, lead_ms=lead_ms, cal=cal,
                          first=first, amp=amp, model=model, ds=ds, h=h, pelvis_x=pelvis_x, ankle_ff=tuple(ankle_ff),
                          k_ds=tuple(k_ds) if k_ds else None,
                          ankle_steps=list(ankle_steps) if ankle_steps is not None else None))


# The heading model the hold inverts (hw 2026-11-12, T 4-5, the turn about the planted foot): heading per step =
# YAW_GAIN x asked + YAW_DRIFT (deg; + = left).
YAW_GAIN, YAW_DRIFT = 1.34, 1.9


class HeadingHold:
    """Per-step heading hold on the rolling queue (#297): at every reported single support the host reads the
    fused yaw (`imu` quat; the firmware's online gyro bias keeps it from drifting at rest), compares it with the
    reference -- a commanded heading change per step, so a commanded heading RATE is the same loop -- and asks
    the steps not yet uploaded for asked = (dpsi_cmd + k x error - YAW_DRIFT) / YAW_GAIN deg each, clipped."""

    def __init__(self, n_steps, dpsi_cmd, k=0.5, clip=8.0):
        self.n, self.dpsi, self.k, self.clip = n_steps, dpsi_cmd, k, clip
        ff = (dpsi_cmd - YAW_DRIFT) / YAW_GAIN
        self.asked = [max(-clip, min(clip, ff))] * n_steps
        self.log = []

    def update(self, n_done, psi, first_free):
        """n_done single supports reported, psi the heading (deg, from the start), first_free = the first step
        whose frames are not uploaded yet. Returns True when the asked list changed."""
        err = n_done * self.dpsi - psi
        a = (self.dpsi + self.k * err - YAW_DRIFT) / YAW_GAIN
        a = max(-self.clip, min(self.clip, a))
        self.log.append(dict(n=n_done, psi=round(psi, 1), ref=round(n_done * self.dpsi, 1), err=round(err, 1),
                             asked=round(a, 2), from_step=first_free + 1))
        if first_free >= self.n or all(abs(x - a) < 0.05 for x in self.asked[first_free:]):
            return False
        self.asked = self.asked[:first_free] + [a] * (self.n - first_free)
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


def ankle_steps_for(sw, A, locked=None, first_free=0):
    """Per-step stance-ankle values: the locked list's values for steps < first_free, A[stance] after."""
    out = []
    for i, p_ in enumerate(sw.phases):
        if locked is not None and i < first_free and i < len(locked):
            out.append(locked[i])
        else:
            out.append(A[p_["stance"]])
    return out


def yaw_of_imu(txt):
    m = re.search(r"^quat\s+([-+\d.]+)\s+([-+\d.]+)\s+([-+\d.]+)\s+([-+\d.]+)", txt, re.M)
    if not m:
        return None
    w, x, y, z = (float(v) for v in m.groups())
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


UPPER = 0.05     # the shift must not exceed k_static by more than this (x 62 mm): 18:09 fell OUTWARD at 1.35


def precheck(pl, tol=0.01):
    """The robot's static shift requirement as a hard constraint (hw 2026-11-12: it fell when the foot came up
    at 1.0-1.1 x): from the FRAMES that would be sent, the pelvis must be at or beyond k_static[stance] x 62 mm
    over the stance foot at every lift-off and at every touchdown. The pelvis shift is recovered from the
    frames' stance hip roll goal (ticks -> rad through the calibration in use, interpolated at that instant)
    by inverting the IK with the plan's feet. -> (ok, rows)."""
    sw, fr, cal = pl["sway"], pl["frames"], pl["cal"]
    K = G.W.S.kin(0.02)
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
    for tm, hx, kind in fr:
        if kind != "kf":
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
        st = p["stance"]
        sg = 1.0 if st == "L" else -1.0
        for what, t in (("lift-off", p["t0"]), ("touchdown", p["t1"])):
            q = q_at(t)
            if q is None:
                continue
            pa = seg_at(t)["pa"]
            sh = shift_of(q, pa, st)
            ratio = sg * (sh - pa["feet"][st][1]) / oy
            nd = need_lo[st] if what == "lift-off" else need[st]
            good = nd - tol <= ratio <= top[st] + tol
            ok = ok and good
            rows.append(dict(step=k + 1, stance=st, at=what, t=round(t - sw.t_first, 2), ratio=round(ratio, 3),
                             need=nd, top=top[st], ok=good))
    # every frame's ankle rolls inside the plant's joint range less ANKLE_MARGIN (the --ankle-ff feed-forward
    # adds to the IK's ankle roll; the firmware also clamps to its calibrated range, which would bend the plan)
    lim = ankle_limits()
    for tm, q in kf:
        for n in ("L_ankle_roll", "R_ankle_roll"):
            lo, hi = lim[n]
            if not lo + ANKLE_MARGIN <= q[n] <= hi - ANKLE_MARGIN:
                ok = False
                rows.append(dict(step=0, stance=n, at="ankle limit", t=round(tm / 1e3, 2), ratio=round(math.degrees(q[n]), 2),
                                 need=round(math.degrees(lo + ANKLE_MARGIN), 2), top=round(math.degrees(hi - ANKLE_MARGIN), 2),
                                 ok=False))
                break
    return ok, rows


ANKLE_MARGIN = math.radians(2.0)
_ANKLE_LIM = {}


def ankle_limits():
    """{joint: (lo, hi) rad}: the ankle roll range of the plant the IK uses (sim/gen_plant_v6 via v6_kin)."""
    if not _ANKLE_LIM:
        import v6_kin as VK
        from design_gates import JN
        m, _ = VK.load(G.W.S.kin(0.02).p)
        for n in ("L_ankle_roll", "R_ankle_roll"):
            j = 1 + JN.index(n)
            _ANKLE_LIM[n] = (float(m.jnt_range[j, 0]), float(m.jnt_range[j, 1]))
    return _ANKLE_LIM


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
    sh = [r for r in rows if r["at"] != "ankle limit"]
    worst = min(sh, key=lambda r: r["ratio"] - r["need"]) if sh else None
    print(f"static-shift pre-check ({pl['model']}): {'PASS' if ok else 'REFUSED'} -- pelvis over the stance foot at "
          f"every lift-off and touchdown, from the frames, needs L >= {pl['amp'][0]:g} x, R >= {pl['amp'][1]:g} x 62 mm"
          + (f" (lift-off: k_ds {pl['args']['k_ds']})" if pl["args"].get("k_ds") else "")
          + ", ankle rolls inside the plant range less 2 deg"
          + (f"; tightest: step {worst['step']} {worst['stance']} {worst['at']} {worst['ratio']:.3f} x" if worst else ""))
    for r in bad[:8]:
        if r["at"] == "ankle limit":
            print(f"   {r['stance']} at t {r['t']} s: {r['ratio']:.1f} deg outside [{r['need']:g}, {r['top']:g}] deg "
                  f"(--ankle-ff too large for this pose)")
            continue
        print(f"   step {r['step']} {r['stance']} stance at {r['at']} (t {r['t']} s): {r['ratio']:.3f} x outside "
              f"[{r['need']:g}, {r['top']:g}] x (the upper bound: k_static + {UPPER:g}; --allow-over to pass it)")
    return ok


def dry(pl, a):
    adds = pl["adds"]
    nb = sum(len(l) + 1 for _, l in adds)
    ahead_ms = min(a.ahead * a.T, MAX_AHEAD_S) * 1e3
    first = [l for t, l in adds if t <= ahead_ms]
    print(f"gait: {pl['model']} T {a.T:g} s/step, {a.steps} steps, {a.dir}; {sum(1 for f in pl['frames'] if f[2] == 'kf')} "
          f"keyframes at {G.KF_S * 1e3:.0f} ms + {sum(1 for f in pl['frames'] if f[2] == 'rest')} rest frames; "
          f"track {pl['duration_s']:.1f} s after a {a.lead} ms lead-in")
    print(f"upload: {len(adds)} `track add` lines ({PER_LINE} frames each), {nb} bytes; longest line "
          f"{max(len(l) for _, l in adds)} chars (CLI limit 192)")
    print(f"before play: {len(first)} lines (the first {ahead_ms / 1e3:.1f} s); at {a.ms_per_line:.0f} ms a line "
          f"(assumed CLI round trip, to be measured) that is {len(first) * a.ms_per_line / 1e3:.1f} s; while "
          f"playing the host needs {1e3 / (G.KF_S * 1e3) / PER_LINE:.1f} lines/s to keep pace, "
          f"{(1e3 / (G.KF_S * 1e3) / PER_LINE) * a.ms_per_line / 10:.0f} % of the link at that rate")
    print(f"queue: at most ~{int(ahead_ms / (G.KF_S * 1e3)) + 2 * PER_LINE} of 128 frames held "
          f"({ahead_ms / 1e3:.1f} s ahead)")
    pr = predicted_dwell(a.T, a.dir) if pl["model"] == "dwell" else predicted(a.T, a.dir)
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
    fell = {"v": False, "abort": False}
    yaw_state = {"prev": None, "psi": 0.0}         # unwrapped heading since the start, deg
    yaw_state["prev"] = yaw_state["start"] = yaw_of_imu(r.cmd("imu"))
    if a.note:
        print(f"  note: {a.note}")
    if hold is not None and yaw_state["prev"] is None:
        print("  heading hold: no `quat` in the imu reply -- holding off")
        hold = None

    def first_free_step():
        last_t = adds[sent - 1][0] if sent else 0
        sw = pl["sway"]
        return sum(1 for p_ in sw.phases if 1e3 * (p_["t0"] - sw.t_first) <= last_t)

    def replan():
        """Re-plan the steps not uploaded yet with the heading hold's asked turns and the learned ankle trim."""
        nonlocal adds
        kw = dict(pl["args"])
        if hold is not None:
            kw["yaw_trim"] = hold.asked
        newp = plan(**kw)
        adds = newp["adds"]
        pl["sway"], pl["frames"] = newp["sway"], newp["frames"]

    def ankle_step(new_steps):
        """After new single supports: the learned ankle trim's update, the not-uploaded steps re-planned."""
        changed = False
        for s_ in new_steps:
            used = (pl["args"].get("ankle_steps") or [None] * 999)[s_["n"] - 1] if s_["n"] >= 1 else None
            old = learn.A[s_["stance"]]
            new = learn.update(s_["stance"], s_["lean_in"], s_["lean_out"], used=used)
            s_["A_used"], s_["A_next"] = used, round(new, 2)
            print(f"  ankle trim {s_['stance']}: ran {used if used is not None else old:.2f} -> {new:.2f} "
                  f"({learn.log[-1]['rule']}{', reverted' if learn.log[-1]['reverted'] else ''})", flush=True)
            changed = changed or abs(new - old) > 1e-6
        if changed:
            ff = first_free_step()
            pl["args"]["ankle_steps"] = ankle_steps_for(pl["sway"], learn.A, pl["args"].get("ankle_steps"), ff)
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
        print(f"  heading {h['psi']:+.1f} (ref {h['ref']:+.1f}, err {h['err']:+.1f}) -> asked {h['asked']:+.2f} "
              f"deg/step from step {h['from_step']}", flush=True)
    ahead_ms = min(a.ahead * a.T, MAX_AHEAD_S) * 1e3

    def scan(txt):
        """Every new TS line in txt: record, print, and say whether one of them trips the stop rule. (2026-11-12:
        it returned at the first tripping line, so the lines after it were never printed.)"""
        trip = False
        for m in TS_RE.finditer(txt):
            s = dict(n=int(m.group(1)), stance=m.group(2), lean_in=float(m.group(3)), lean_out=float(m.group(4)),
                     e=float(m.group(5)), T=float(m.group(6)), hike=float(m.group(7)), ok=m.group(8) == "ok",
                     tback="Tback" in m.group(0))
            if all(x["n"] != s["n"] for x in steps):
                steps.append(s)
                print(f"  step {s['n']} {s['stance']}: in {s['lean_in']:.1f} out {s['lean_out']:.1f} e {s['e']:+.1f} "
                      f"T {s['T']:+.1f} hike {s['hike']:+.1f} {'ok' if s['ok'] else 'ABORT'}"
                      f"{' Tback' if s['tback'] else ''}", flush=True)
                if not s["ok"]:
                    fell["abort"] = True
                if not s["ok"] or s["lean_in"] > a.stop_lean:
                    trip = True
                if s["lean_out"] > a.stop_out:
                    # (19:27: R out 7.0 on one walk, 7.2 and 8.2 on the next, then the outward fall)
                    print(f"  OUTWARD lean {s['lean_out']:.1f} > --stop-out {a.stop_out:g} deg: stopping", flush=True)
                    trip = True
        return trip

    def add(line):
        t = time.monotonic()
        txt = r.cmd(line)
        t_line.append(time.monotonic() - t)
        if not txt.lstrip().startswith("track +"):
            raise RuntimeError(f"refused: {txt.strip()[-160:]}")
        return txt

    try:
        for l in pl["pre"]:
            r.cmd(l)
        while sent < len(adds) and adds[sent][0] <= ahead_ms:
            add(adds[sent][1])
            sent += 1
        r.cmd(pl["play"])
        stop = False
        while True:
            txt = r.cmd("track")
            m = re.search(r"play t (\d+) ms", txt)
            head = int(m.group(1)) if m else 0
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
                print("  stop rule hit -> track stop")
                break
            while sent < len(adds) and adds[sent][0] <= head + ahead_ms:
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
            ankle = dict(dir=learn.dir, start=learn.start, end=learn.A, keep=learn.keep, log=learn.log, saved=saved)
            print(f"ankle trim ({learn.dir}): L {learn.start['L']:.2f} -> {learn.A['L']:.2f}, R {learn.start['R']:.2f} -> "
                  f"{learn.A['R']:.2f}; {'saved to ' + learn.path if saved else 'NOT saved (fall / abort / link)'}")
        log(dict(summary=True, note=a.note, args=dict(T=a.T, dir=a.dir, steps=a.steps, amp=list(pl["amp"]),
                                                      model=pl["model"], first=a.first, pelvis_x=a.pelvis_x,
                                                      ankle_ff=list(pl["args"]["ankle_ff"]), k_ds=pl["args"]["k_ds"]),
                 steps=steps, line_s=t_line, heading_change_deg=d_head, ankle_learn=ankle,
                 heading=(hold.log if hold is not None else None)))
        r.close()
        print(f"log: {log_path}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--T", type=float, default=6.0,
                    help="s per step: the dwell schedule has 6, 5, 4, 3.4, 3.2 (the SIL floor), 3; start at 6")
    ap.add_argument("--below-floor", action="store_true", help="allow T under dwell.T_FLOOR (the SIL says no)")
    ap.add_argument("--dir", default="march", choices=list(G.DIRS))
    ap.add_argument("--steps", type=int, default=6)
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
    ap.add_argument("--ahead", type=float, default=2.0, help="steps the host stays ahead of the play head")
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
    ap.add_argument("--ankle-learn", choices=["on", "off"], default="off",
                    help="dwell: learn the stance-ankle trim per side at step rate (AnkleLearner): starts from "
                         "results/ankle_trim_state.json for this --dir (seeds 2/2, sideR R 3), replaces --ankle-ff, "
                         "re-plans the steps not uploaded yet (use --ahead 1), saved after a run with no fall / abort")
    ap.add_argument("--ankle-seed", default=None,
                    help="with --ankle-learn on: start from these L,R deg instead of the state file")
    ap.add_argument("--k-ds", default=None,
                    help="dwell: L,R, the two-stage shift -- double support only to k_ds x 62 mm, the rest to --amp over "
                         "the raise; 0 = single-stage for that side (default: single-stage both)")
    ap.add_argument("--allow-over", action="store_true",
                    help="let the pre-check pass a shift past k_static + 0.05 (an --amp A/B on the outward side)")
    ap.add_argument("--host", default="192.168.2.90")
    ap.add_argument("--log")
    a = ap.parse_args(argv)
    seeds = tuple(float(x) for x in a.seed.split(","))
    import dwell as DW
    if a.model == "dwell" and a.T < DW.T_FLOOR and not a.below_floor:
        print(f"REFUSED: T {a.T:g} s is under the dwell gait's SIL floor ({DW.T_FLOOR:g} s): it falls or leans 11-14 deg "
              f"there (experiments/roll-balance/README.md); --below-floor to plan it anyway")
        return 2
    amp = tuple(float(x) for x in a.amp.split(",")) if a.amp else ROBOT_AMP
    aff = tuple(float(x) for x in a.ankle_ff.split(","))
    learn = None
    if a.ankle_learn == "on" and a.model == "dwell":
        learn = AnkleLearner(a.dir, start=(tuple(float(x) for x in a.ankle_seed.split(","))
                                           if a.ankle_seed else None))
        aff = (learn.A["L"], learn.A["R"])
        print(f"ankle trim learning ({a.dir}): starts L {aff[0]:.2f}, R {aff[1]:.2f} deg (--ankle-ff ignored)"
              + ("; --ahead > 1: an update reaches the side's next step only if it is not uploaded yet" if a.ahead > 1.0 else ""))
    kds = tuple(float(x) for x in a.k_ds.split(",")) if a.k_ds else None
    if not a.live:
        pd = plan(a.T, a.steps, a.dir, seeds, a.lead, first=a.first, amp=amp, model=a.model, ds=a.ds, h=a.h,
                  yaw_trim=a.yaw_trim, pelvis_x=a.pelvis_x, ankle_ff=aff, k_ds=kds)
        pd["allow_over"] = a.allow_over
        dry(pd, a)
        return 0
    sys.path.insert(0, os.path.join(ROOT, "experiments", "stepper"))
    import robot_io
    probe = robot_io.Robot(host=a.host)
    cal = {n: (z, d) for n, (_, z, d) in probe.cal.items()}
    probe.close()
    pl = plan(a.T, a.steps, a.dir, seeds, a.lead, cal=cal, first=a.first, amp=amp, model=a.model, ds=a.ds, h=a.h,
              yaw_trim=a.yaw_trim, pelvis_x=a.pelvis_x, ankle_ff=aff, k_ds=kds)
    if a.model == "dwell" and a.heading_hold == "on":
        if a.heading_rate is not None:
            dpsi = a.heading_rate * a.T
        elif a.turn is not None:
            dpsi = a.turn / a.steps
        else:
            dpsi = {"turnL": 5.0, "turn": 5.0, "turnR": -5.0}.get(a.dir, 0.0)
        hold = HeadingHold(a.steps, dpsi, k=a.heading_k)
        pl = plan(**{**pl["args"], "yaw_trim": hold.asked})
        pl["hold"] = hold
        print(f"heading hold: {dpsi:+.1f} deg/step commanded, feed-forward {hold.asked[0]:+.2f} deg/step asked, "
              f"k {a.heading_k:g}")
    if learn is not None:
        hold_ = pl.get("hold")
        pl = plan(**{**pl["args"], "ankle_steps": ankle_steps_for(pl["sway"], learn.A),
                     **({"yaw_trim": hold_.asked} if hold_ is not None else {})})
        pl["hold"], pl["learn"] = hold_, learn
    pl["allow_over"] = a.allow_over
    live(pl, a)
    return 0


if __name__ == "__main__":
    sys.exit(main())
