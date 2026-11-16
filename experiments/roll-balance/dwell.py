"""#297 B2, after the first track run on hardware (2026-11-12 16:25, T = 4 s march): a flattened sway with the
robot's STATIC shift requirement as a hard constraint.

The sine sway (sway.py) peaks over the stance foot only at the middle of single support, so its foot
leaves the floor with the pelvis at ~1.0-1.1 x 62 mm. The robot needs 1.25 x over its right foot and 1.3 x
over its left before the foot comes up (B1 lifts: 1.15 x tips in at foot-off, 1.25 x holds), and at 4 s a
step there are no dynamics to carry it: it fell toward the lifted foot on the first step. The sim did not
(it is optimistic on right stance: fw_replay.py), so the constraint is imposed, not learned from the sim.

One step of T s, per stance side:
    shift   the pelvis moves (min-jerk) from over the last stance foot to `k_static[side]` x 62 mm over the
            new one (+ that foot's own offset from where it stands), both feet down        t_shift
    settle  it holds there, both feet down                                                 t_settle
    swing   single support, the pelvis still holding: the swing sole rises (min-jerk) to h over the first
            35 %, moves (x, y, yaw) over the middle, comes down over the last 35 %          t_ss
The pelvis is at or beyond k_static from before lift-off until after touchdown. The default split is a
fraction of T (shift 0.3, settle 0.1, single support 0.6; `ds` sets shift + settle) unless T is in SCHEDULE or
t_shift / t_settle are given (absolute, s).

The firmware maps the frames' phase to the support state (gaitplay::supportFromPhase: single support =
u in [0.25, 0.75] of each half cycle, raise = its first 35 %, lowering after 65 %). The phase here is
warped piecewise-linearly in time so those points land on this gait's lift-off, raise end, lower start and
touchdown: no firmware change.

The object has the interface sil_gait.gait / testbed expect: segs (t, q, pa), phases, q_start, t_end,
phase_at(t), plus phi(t), step_ends and liftoff (per step: t, stance, the commanded shift ratio).
"""
from __future__ import annotations

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import synth as S  # noqa: E402

# x 62 mm, the robot's static requirement (hw 2026-11-12). R 1.30: at 1.25 the right stance tipped toward the
# lifted foot in mid single support at T <= 4 (in 9.2-10.6 deg on 7 of 12 right steps); at 1.30, T 4 / 3.4 / 3.2 /
# fwd 8 at T 4: 0 of 16 right steps past 4 deg. (The servos track their SENT goals within 1-2 deg; the "lag" once
# quoted here was nominal - measured, i.e. the trim's hike. The tips are the stance sole rolling onto its inner
# edge -- video 18:2x.)
# R 1.35 (hw 17:47-17:51): at 1.30 the first right stance of back 12 at T 5 still tipped (in 10.1-10.4, P 32 and
# P 64 alike); at 1.35 back 12 and back 8 ran clean (right in 1.3-1.7, out <= 1.5) -- but 1.35 fell outward
# at T 4 (18:09, k_static_R below), so R is 1.30 at every T.
K_STATIC = {"L": 1.3, "R": 1.30}


def k_static_R(T):
    """The right-stance requirement, 1.30 at every T (kept as a function of T for the callers). hw 2026-11-12 18:09:
    at T 4 with 1.35 the robot fell OUTWARD over the right foot in the settle, both feet still down (roll +1 -> +22
    deg in 0.5 s after the pelvis arrived). An inward tip costs a step and the walk goes on (18:2x: tips to 9-10 deg
    recover); an outward topple is a fall nothing catches. track_host's pre-check refuses a plan above k_static +
    0.05 (track_host.UPPER)."""
    return K_STATIC["R"]
# (shift, settle) s per step time; single support = the rest. From the SIL (dwell_grid.py timing): the
# shift (min-jerk, ~158 mm from 1.25 x over one foot to 1.3 x over the other) needs >= 1.2 s, single
# support >= 1.8 s (raise 0.63 s); below T = 3.2 s it falls or leans 11-14 deg. 6 s is the gentle start
# (raise 1.4 s, the stepper's pace).
# With R 1.30 the SIL (fit plant, 6 directions) also holds 3.0 and 2.8 with the 1.2 s shift kept and the single
# support shortened (2.8: 1.4 s, raise 0.49 s, lean <= 3.3 deg); a shorter shift leans 6.6-10 deg. The shift is
# the limiter (on the robot at 3.2 s the roll rings at 22-47 deg/s through it and is quiet only 0.1-0.4 s before
# lift-off); the single support has slack.
SCHEDULE = {6.0: (1.5, 0.5), 5.0: (1.4, 0.4), 4.0: (1.4, 0.4), 3.4: (1.4, 0.4), 3.2: (1.4, 0.2), 3.0: (1.4, 0.2)}
# k R 1.35 (hw 17:56-17:58): the shift onto the right foot grew to ~165 mm, and with the 1.2 s shift at T 4 the
# roll was still ringing at lift-off (pre-lift roll rate 22-52 deg/s on the two steps that tipped, 5-10 on the clean
# ones). Every schedule entry now shifts for >= 1.4 s; the single support gives way (SIL: >= 1.4 s holds).
T_FLOOR = 3.0
# the hip yaw sign that turns the body to its LEFT about the stance foot (calibrated in the sim: see yaw_trim)
YAW_SIGN = -1.0          # sim (sil_gait, T 5, fit): yaw_trim +4 deg/step with +1 turned the body -35 deg over 6 steps


LOWER_FRAC = 0.35            # the lowering's share of the single support (the raise has the same, the move the rest)
LAND_EARLY_MAX_MM = 4.0
LOWER_FRAC_RANGE = (0.20, 0.50)


def split_times(T, ds=0.4, settle_frac=0.25, t_shift=None, t_settle=None):
    """(shift, settle, double support, single support) s of a step of nominal time T, as the constructor lays them
    out: the SCHEDULE entry for T, else fractions of T."""
    t_ds = ds * T
    t_set = settle_frac * t_ds
    t_sh = t_ds - t_set
    if t_shift is None and float(T) in SCHEDULE:
        t_shift, t_settle = SCHEDULE[float(T)]
    if t_shift is not None:               # absolute durations override the fractions
        t_sh = float(t_shift)
        t_set = float(t_settle if t_settle is not None else 0.25)
        t_ds = t_sh + t_set
    return t_sh, t_set, t_ds, T - t_ds


def effective_step_s(T, td_pause=0.0, settle=None, **kw):
    """The step time with --td-pause and --settle: T + pause (every step but the first) + (settle - the schedule's
    settle) when a settle is given; the single support and the shift keep the schedule's lengths."""
    t_sh, t_set, t_ds, t_ss = split_times(T, **kw)
    return T + (td_pause or 0.0) + ((settle - t_set) if settle is not None else 0.0)


def _mj(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * u * (10 - 15 * u + 6 * u * u)


class Dwell:
    def __init__(self, T=4.0, n_steps=6, kL=K_STATIC["L"], kR=None, ds=0.4, settle_frac=0.25, h=0.022,
                 dx=0.0, dy=0.0, yaw=0.0, first="L", drop=0.02, t_crouch=2.0, t_settle0=1.0, dt=0.04,
                 t_shift=None, t_settle=None, yaw_trim=0.0, pelvis_x="mid", close=True, ankle_ff=None, k_ds=None,
                 ankle_ff_steps=None, dx_steps=None, dy_steps=None, step_meta=None, swing_tilt=0.0,
                 td_pause=0.0, settle=None, lower_frac=None, land_early=0.0):
        """td_pause: s, the next step's lateral shift (all pelvis / hip roll / ankle roll motion of the shift, and the
        fore-aft pelvis move with it) starts this long AFTER the planned touchdown of the swing foot: the lowering, the
        ankle feed-forward's and the swing tilt's ramp-out are complete first (they follow the foot's height, 0 at
        touchdown). Every step but the first has it, and so does the final shift back to the centre; each step is
        that much longer. settle: s, the pre-lift settle (default per schedule); a shorter one shortens the step
        (the single support and the shift keep their lengths). lower_frac: the lowering's share of the single
        support (default 0.35; the move takes what the raise and the lowering leave). land_early: mm, the swing sole is
        commanded this far BELOW the floor at touchdown (min-jerk with the lowering; a slight press), held through the
        pause and released over the next shift, 0..4.
        ankle_ff: {"L": deg, "R": deg} (or an (L, R) pair), a fixed feed-forward on the STANCE ankle roll,
        + = outward (the body rolls toward the stance foot's outside; 1 deg moves the CoM ~3.7 mm in the plant FK),
        ramped in over the raise, held, taken out over the lower (the swing foot's height profile).
        ankle_ff_steps: a list, deg per step (step i's stance ankle), overriding ankle_ff where given (the host's
        learned ankle trim re-plans the steps not uploaded yet; steps past the list's end use ankle_ff).
        k_ds: {"L": k, "R": k} (or a pair; 0 / None = off for that side), the two-stage shift: the double support
        goes to k_ds x 62 mm over the stance foot, and the rest of the way to k_static is travelled over the raise
        (lift-off -> raise end, min-jerk), so the pelvis reaches k_static only once the swing foot is off.
        (hw 2026-11-12: the outward topple came in double support at 1.35, the inward tips in single support.)
        dx_steps / dy_steps: per-step forward / left displacement, m (a list per step; steps past its end repeat
        the last value), overriding dx / dy: a sequence of segments in one track (track_host --seq). yaw_trim may
        be a list too. step_meta: a list of dicts (name, key) per step, kept as self.step_meta for the host's
        prints and the ankle learner.
        swing_tilt: deg, toe-down tilt of the SWING sole (its ankle pitch + tilt, + = plantarflexion), ramped in over the
        raise, held, out over the lower like the ankle feed-forward: flat at lift-off and at touchdown. The swing leg's
        ankle pitch is what bounds the stride (track_host's pre-check); the tilt gives it room. A list is one tilt
        per step (track_host --cmd picks it from the step length; steps past its end repeat the last).
        """
        K = S.kin(drop)
        self.oy = abs(K.model_offset("L")[1])
        if kR is None:
            kR = k_static_R(T)
        self.k = {"L": float(kL), "R": float(kR)}

        def per_side(v):
            if v is None:
                return {"L": 0.0, "R": 0.0}
            if isinstance(v, dict):
                return {"L": float(v.get("L") or 0.0), "R": float(v.get("R") or 0.0)}
            return {"L": float(v[0] or 0.0), "R": float(v[1] or 0.0)}
        self.ankle_ff = per_side(ankle_ff)
        self.ankle_ff_steps = list(ankle_ff_steps) if ankle_ff_steps is not None else None
        self.k_ds = {s_: (v if v > 0 else None) for s_, v in per_side(k_ds).items()}
        self.step_meta = list(step_meta) if step_meta is not None else None

        def _at(lst, i, default):
            return default if lst is None else (lst[i] if i < len(lst) else lst[-1])
        t_sh, t_set, t_ds, t_ss = split_times(T, ds, settle_frac, t_shift, t_settle)
        t_pause = float(td_pause or 0.0)
        if t_pause < 0.0 or t_pause > 2.0:
            raise ValueError(f"td_pause {t_pause:g} s: want 0 to 2")
        period = T                            # the step time: T, longer by a settle change (the pause is per step, below)
        if settle is not None:
            if not 0.0 <= settle <= 2.0:
                raise ValueError(f"settle {settle:g} s: want 0 to 2")
            period = T + (float(settle) - t_set)
            t_set = float(settle)
            t_ds = t_sh + t_set
        land = float(land_early or 0.0)
        if not 0.0 <= land <= LAND_EARLY_MAX_MM + 1e-9:
            raise ValueError(f"land_early {land:g} mm: want 0 to {LAND_EARLY_MAX_MM:g}")
        land /= 1e3
        if lower_frac is not None and not LOWER_FRAC_RANGE[0] <= lower_frac <= LOWER_FRAC_RANGE[1]:
            raise ValueError(f"lower_frac {lower_frac:g}: want {LOWER_FRAC_RANGE[0]:g} to {LOWER_FRAC_RANGE[1]:g}")
        if lower_frac == LOWER_FRAC:
            lower_frac = None                  # the default written out is the default
        F_ = LOWER_FRAC if lower_frac is None else float(lower_frac)       # the lowering's share of the single support
        u_l = 0.65 if lower_frac is None else 1.0 - F_                     # where it starts
        self.T, self.t_shift, self.t_settle, self.t_ss, self.h = T, t_sh, t_set, t_ss, h
        self.td_pause, self.lower_frac, self.land_early = t_pause, F_, land
        self.step_time = period + t_pause
        self.stamp = (f"dwell T{T:g} ds{ds:g} (shift {t_sh:.2f} settle {t_set:.2f} ss {t_ss:.2f} s) "
                      f"kL{kL:g} kR{kR:g} h{1e3 * h:.0f}mm dx{dx:+g} dy{dy:+g} yaw{yaw:+g}"
                      + "".join(f" kds{s_}{v:g}" for s_, v in self.k_ds.items() if v)
                      + "".join(f" aff{s_}{v:+g}" for s_, v in self.ankle_ff.items() if v)
                      + (f" pause{t_pause:g}" if t_pause else "") + (f" settle{t_set:g}" if settle is not None else "")
                      + (f" lower{F_:g}" if lower_frac is not None else "")
                      + (f" land{1e3 * land:g}mm" if land else ""))
        self.r = None
        self.q_start = K.pose(straight=True)
        swing_of = (lambda i: first if i % 2 == 0 else ("R" if first == "L" else "L"))
        t0 = t_crouch + t_settle0
        feet = {"L": [0.0, 0.0], "R": [0.0, 0.0]}
        yawq = {"L": 0.0, "R": 0.0}
        # the schedule: step i = [shift, settle, single support]; then a final shift back to the centre
        steps = []
        t = t0
        for i in range(n_steps):
            sw = swing_of(i)
            st = "R" if sw == "L" else "L"
            pz = t_pause if i > 0 else 0.0
            steps.append(dict(i=i, swing=sw, stance=st, a=t, s0=t + pz, b=t + pz + t_sh, lo=t + pz + t_ds,
                              td=t + period + pz))
            t += period + pz
        # the feet's goals per step (the stepper's convention: the leading foot steps out, the other follows)
        goals = []
        f = {s: list(v) for s, v in feet.items()}
        yq = dict(yawq)
        for p in steps:
            sw, st = p["swing"], p["stance"]
            dx_i, dy_i = _at(dx_steps, p["i"], dx), _at(dy_steps, p["i"], dy)
            out = (dy_i > 0 and sw == "L") or (dy_i < 0 and sw == "R")
            start = list(f[sw])
            goal = [f[st][0] + dx_i, f[st][1] + (dy_i if out else 0.0)]
            y_from = dict(yq)
            y_to = dict(yq)
            y_to[sw] = (yaw if p["i"] % 2 == 0 else -yaw) if yaw else 0.0
            per_step = isinstance(yaw_trim, (list, tuple))
            yt = (yaw_trim[p["i"]] if p["i"] < len(yaw_trim) else yaw_trim[-1]) if per_step else yaw_trim
            if per_step or yt:
                # a heading change of yaw_trim deg per step (+ = to the robot's left; a list = one per step, the
                # host's heading hold): over the middle of single support the stance hip yaw goes from -c to +c
                # (the body turns about the planted foot) and the swing hip from +c to -c (the foot lands square to
                # the new heading); c = half the step's turn
                c = YAW_SIGN * math.radians(yt) / 2.0
                y_to[st] = c                      # was -c, as the last step's swing (0 on the first step)
                y_to[sw] = y_to[sw] - c
            goals.append((start, goal, y_from, y_to))
            f[sw] = goal
            yq = y_to
        # the closing step (hw 2026-11-12): a run that ends with the feet staggered or the hips yawed leaves them
        # so, and the next run's lead-in drags them back together under load (4-6 deg of twist before its first
        # step). One more step brings the trailing foot alongside the last stance foot and both hip yaws to 0.
        last_sw = steps[-1]["swing"] if steps else None
        if close and steps:
            st_c = last_sw
            sw_c = "R" if st_c == "L" else "L"
            staggered = abs(f[sw_c][0] - f[st_c][0]) > 1e-4 or abs(f[sw_c][1] - f[st_c][1]) > 1e-4
            if staggered or abs(yq["L"]) > 1e-4 or abs(yq["R"]) > 1e-4:
                i = len(steps)
                steps.append(dict(i=i, swing=sw_c, stance=st_c, a=t, s0=t + t_pause, b=t + t_pause + t_sh,
                                  lo=t + t_pause + t_ds, td=t + period + t_pause, closing=True))
                t += period + t_pause
                goals.append((list(f[sw_c]), [f[st_c][0], f[st_c][1]], dict(yq), {"L": 0.0, "R": 0.0}))
                f[sw_c] = [f[st_c][0], f[st_c][1]]
        t_end_steps = t
        t_back = t_end_steps + t_pause + t_sh  # back to the centre, both feet down
        self.t_end = t_back + 1.0
        self._steps, self._goals = steps, goals

        def stance_target(i, fe, ds=False):
            """The pelvis (absolute shift, m) over step i's stance foot: k_static x 62 mm + that foot's offset
            (ds=True: the double-support target, k_ds when the two-stage shift is on for that side)."""
            st = steps[i]["stance"]
            sg = 1.0 if st == "L" else -1.0
            k = (self.k_ds[st] or self.k[st]) if ds else self.k[st]
            return fe[st][1] + sg * k * self.oy

        segs = [dict(t=0.0, ms=1e3 * t_crouch, q=K.pose(), row=dict(tag="crouch"))]
        phases, self.liftoff, self.step_ends = [], [], []
        fe = {s: list(v) for s, v in feet.items()}
        yq = {"L": 0.0, "R": 0.0}
        pel_from = 0.0
        px_from = 0.0
        tt = t0
        cur = 0
        while tt <= self.t_end + 1e-9:
            while cur < len(steps) and tt > steps[cur]["td"] + 1e-9:
                # step cur is over: its swing foot is at its goal
                start, goal, y_from, y_to = goals[cur]
                fe[steps[cur]["swing"]] = list(goal)
                yq = dict(y_to)
                pel_from = stance_target(cur, fe)
                px_from = fe[steps[cur]["stance"]][0]
                cur += 1
            press = None
            if cur < len(steps):
                p = steps[cur]
                tgt = stance_target(cur, fe, ds=True)
                tgt_ss = stance_target(cur, fe)
                px_tgt = fe[p["stance"]][0]
                if tt <= p["b"]:
                    m_sh = _mj((tt - p["s0"]) / max(t_sh, 1e-6))
                    pel = pel_from + m_sh * (tgt - pel_from)
                    px = px_from + m_sh * (px_tgt - px_from)
                    if land and cur > 0:                  # the last step's pressed sole, released over this shift
                        press = (steps[cur - 1]["swing"], -land * (1.0 - m_sh))
                else:
                    pel = tgt
                    px = px_tgt
                fz = {"L": 0.0, "R": 0.0}
                fxy = {s: list(v) for s, v in fe.items()}
                yy = dict(yq)
                if press is not None:
                    fz[press[0]] = press[1]
                if tt > p["lo"]:
                    u = (tt - p["lo"]) / t_ss
                    pel = tgt + _mj(u / 0.35) * (tgt_ss - tgt)     # the two-stage shift's second stage (else 0)
                    sw = p["swing"]
                    start, goal, y_from, y_to = goals[cur]
                    if u < 0.35:
                        fz[sw] = h * _mj(u / 0.35)
                    elif u < u_l:
                        fz[sw] = h
                    else:
                        v_ = _mj((u - u_l) / F_)
                        fz[sw] = h * (1.0 - v_) - land * v_ if land else h * (1.0 - v_)
                    m = _mj((u - 0.35) / (0.3 if lower_frac is None else u_l - 0.35))
                    fxy[sw] = [start[0] + m * (goal[0] - start[0]), start[1] + m * (goal[1] - start[1])]
                    yy[sw] = y_from[sw] + m * (y_to[sw] - y_from[sw])
                    st_ = p["stance"]
                    yy[st_] = y_from[st_] + m * (y_to[st_] - y_from[st_])
            else:
                # after the last step: back to the centre between the feet
                mid = 0.5 * (fe["L"][1] + fe["R"][1])
                m_sh = _mj((tt - t_end_steps - t_pause) / max(t_sh, 1e-6))
                pel = pel_from + m_sh * (mid - pel_from)
                px = px_from + m_sh * (0.5 * (fe["L"][0] + fe["R"][0]) - px_from)
                fz = {"L": 0.0, "R": 0.0}
                if land and steps:
                    fz[steps[-1]["swing"]] = -land * (1.0 - m_sh)
                fxy = {s: list(v) for s, v in fe.items()}
                yy = dict(yq)
            # pelvis_x "mid": over the feet's mid point (moves forward with the swing foot); "stance": over the stance
            # foot through single support, moving fore-aft only in the shift (fwd at T 3.4 on 11-12: the right stance
            # pitched to +7 deg as the pelvis went ahead of its foot, then tipped toward the swing foot)
            x_pel = px if pelvis_x == "stance" else 0.5 * (fxy["L"][0] + fxy["R"][0])
            # the ankle feed-forward on the stance side, scaled by the swing foot's height profile (0 -> 1 over the
            # raise, 1 -> 0 over the lower)
            ank = None
            if cur < len(steps) and h > 0:
                st_a = steps[cur]["stance"]
                r_a = fz[steps[cur]["swing"]] / h
                ffv = self.ankle_ff[st_a]
                if self.ankle_ff_steps is not None and cur < len(self.ankle_ff_steps) and \
                        self.ankle_ff_steps[cur] is not None:
                    ffv = float(self.ankle_ff_steps[cur])
                if ffv and r_a > 0:
                    ank = {st_a: math.radians(ffv) * r_a}
            qd = K.pose(shift=pel, fwd=x_pel, footL=(fxy["L"][0], fxy["L"][1], fz["L"]),
                        footR=(fxy["R"][0], fxy["R"][1], fz["R"]), ankle=ank)
            if swing_tilt and cur < len(steps) and h > 0:
                sw_t = steps[cur]["swing"]
                tilt_i = (swing_tilt[cur] if cur < len(swing_tilt) else swing_tilt[-1]) \
                    if isinstance(swing_tilt, (list, tuple)) else swing_tilt
                qd[f"{sw_t}_ankle"] += math.radians(tilt_i) * fz[sw_t] / h
            qd["L_hip_yaw"] += yy["L"]
            qd["R_hip_yaw"] += yy["R"]
            segs.append(dict(t=tt, ms=1e3 * dt, q=qd, row=None,
                             pa=dict(fwd=x_pel, shift=0.5 * (fxy["L"][1] + fxy["R"][1]), pel=pel,
                                     feet={s: [fxy[s][0], fxy[s][1], fz[s]] for s in fxy}, yaw=dict(yy))))
            tt += dt
        for i, p in enumerate(steps):
            lo, td = p["lo"], p["td"]
            phases.append(dict(t0=lo, raise_end=lo + 0.35 * t_ss, hold_from=lo + 0.35 * t_ss,
                               lower_start=lo + u_l * t_ss, t1=td, swing=p["swing"], stance=p["stance"],
                               ss_a=lo, ss_b=td))
            self.step_ends.append(td + min(0.1, 0.5 * t_sh))     # both feet down: a stop may end here
        # the commanded shift ratio at each lift-off, over the stance foot (the constraint the host checks)
        for i, p in enumerate(steps):
            seg = min(segs[1:], key=lambda sg: abs(sg["t"] - p["lo"]))
            st = p["stance"]
            sg = 1.0 if st == "L" else -1.0
            fst = seg["pa"]["feet"][st][1]
            self.liftoff.append(dict(i=i, t=p["lo"], stance=st, ratio=sg * (seg["pa"]["pel"] - fst) / self.oy,
                                     need=self.k_ds[st] or self.k[st]))
        self.segs, self.phases, self.first = segs, phases, first

    def phase_at(self, t):
        for p in self.phases:
            if p["t0"] <= t <= p["t1"]:
                return p
        return None

    def phi(self, t):
        """The gait phase for the firmware: u = 0.25 at lift-off, 0.75 at touchdown (linear between), 0.75 ->
        1.25 across the double support up to the next lift-off; half cycle k = step k, the left foot swings
        in the half cycles where sin(phi) > 0."""
        ps = self.phases
        off = 0.0 if self.first == "L" else math.pi
        if t <= ps[0]["t0"]:
            # before the first lift-off: back from u = 0.25 over one double support
            d = ps[0]["t0"] - (ps[0]["t0"] - (self.t_shift + self.t_settle))
            u = 0.25 - 0.5 * min(1.0, (ps[0]["t0"] - t) / max(d, 1e-6))
            return math.pi * u + off
        for k, p in enumerate(ps):
            if p["t0"] <= t <= p["t1"]:
                w = (t - p["t0"]) / (p["t1"] - p["t0"])
                if self.lower_frac != LOWER_FRAC:
                    # a lowering longer or shorter than 0.35: the firmware's raise end (0.35) and lowering start (0.65)
                    # still land on this gait's, the phase is warped piecewise-linearly between them
                    u_l = 1.0 - self.lower_frac
                    w = w if w < 0.35 else (0.35 + 0.3 * (w - 0.35) / (u_l - 0.35) if w < u_l
                                            else 0.65 + 0.35 * (w - u_l) / (1.0 - u_l))
                    return math.pi * (k + 0.25 + 0.5 * w) + off
                u = 0.25 + 0.5 * (t - p["t0"]) / (p["t1"] - p["t0"])
                return math.pi * (k + u) + off
            nxt = ps[k + 1]["t0"] if k + 1 < len(ps) else p["t1"] + (self.t_shift + self.t_settle)
            if p["t1"] < t < nxt:
                u = 0.75 + 0.5 * min(1.0, (t - p["t1"]) / max(nxt - p["t1"], 1e-6))
                return math.pi * (k + u) + off
        return math.pi * (len(ps) - 1 + 1.25) + off
