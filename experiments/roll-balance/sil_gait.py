"""The sway gait through the FIRMWARE's own track player and stance trim (#297 B2, software in the loop).

Host side (`gait()`): the sway gait (sway.py with the stepper's IK) as the keyframes the robot is sent,
in the CLI's hex form (`track add`), with the gait phase, kLift while stepping, a kStepEnd frame at every
mid double support and, right after it, that step end's kRest frame (the centred stand on those feet).
The timeline ends at the last step end: the queue runs dry there and the firmware rests (as `track stop`).
The host appends ROLLING, `lead_steps` steps ahead of the play head (the robot's 256-frame queue never
holds a whole walk).

Robot side (`run()`): firmware/host/build/libctrl_sil.so (sil_gait.cpp -> gaitplay/track +
balance/stance_trim, the code the robot runs) samples the track every 20 ms, derives the support state from
the phase and drives the trim; the goals (ticks -> joint rad through the calibration) plus the trim on the
stance hip roll go into the sim plant (testbed's: grade.py's servo chain + the per-side roll play), the
torso roll comes back one tick old (+ --delays). Per step the firmware's own report (peak lean each way,
e / T / hike at mid single support, abort) is collected: the numbers the robot should print.

    python sil_gait.py --T 4,3,2.5,2      # all six directions x plants fit/lo/hi/hard x delay 0/60 ms
"""
from __future__ import annotations

import argparse
import ctypes
import json
import math
import os
import sys
from collections import deque
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
import sway as W  # noqa: E402
import testbed as TB  # noqa: E402

LIB = os.path.join(ROOT, "firmware", "host", "build", "libctrl_sil.so")
LEG = ["L_hip_yaw", "L_hip_roll", "L_hip_pitch", "L_knee", "L_ankle", "L_ankle_roll",
       "R_hip_yaw", "R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle", "R_ankle_roll"]
# joint -> (zero ticks, dir): the robot's `cal show` in the 2026-11-11 stepper start rows. Only the round trip
# matters here (rad -> ticks -> rad); the host generator for the robot reads the live calibration.
CAL = {"R_hip_roll": (2082, -1), "R_hip_pitch": (1997, 1), "R_knee": (2125, -1), "R_ankle": (2045, 1),
       "L_hip_roll": (1991, -1), "L_hip_pitch": (2058, 1), "L_knee": (1977, -1), "L_ankle": (2091, 1),
       "R_hip_yaw": (2184, 1), "L_hip_yaw": (2153, 1), "R_ankle_roll": (2108, 1), "L_ankle_roll": (1997, 1)}
LIFT, STEP_END, REST = 1, 2, 4
PLANTS = {"fit": TB.BOARD,
          "lo": "grade+play_L_hip=4,play_R_hip=6,play_L_ankle=2,play_R_ankle=0",
          "hi": "grade+play_L_hip=10,play_R_hip=10,play_L_ankle=3,play_R_ankle=2",
          "hard": TB.BOARD + ",floor_tc=0.01"}
DIRS = {"march": {}, "fwd": dict(dx=0.02), "back": dict(dx=-0.02), "sideL": dict(dy=0.015),
        "sideR": dict(dy=-0.015), "turn": dict(yaw=0.15)}
SEED = (8.0, 3.0)          # `trim T L 8`, `trim T R 3` (hike deg; A2's seeds)
KF_S = 0.1                 # keyframe spacing, s


def to_ticks(name, rad, cal=None):
    zero, d = (cal or CAL)[name]
    return int(round(zero + d * rad * 4096 / (2 * math.pi)))


def from_ticks(name, ticks):
    zero, d = CAL[name]
    return (ticks - zero) * d * 2 * math.pi / 4096


def hex_frame(dt_ms, phi, flags, q, cal=None):
    """One `track add` frame (gaitplay/track.h): ddd pp f + 12 x qqq, joint goals q {name: rad}."""
    ph = int(round((phi % (2 * math.pi)) * 256 / (2 * math.pi))) & 0xFF
    s = f"{max(0, min(0xFFF, dt_ms)):03x}{ph:02x}{flags & 0xF:01x}"
    return s + "".join(f"{max(0, min(4095, to_ticks(n, q[n], cal))):03x}" for n in LEG)


def gait(T_step=2.0, n_steps=8, kf_s=KF_S, first="L", cal=None, **kw):
    """The sway gait (A2's settings + the width-aware amplitude) and its keyframes: (sway, [(t_ms, hex, kind)]), kind 'kf' | 'rest'.
    Track time 0 = the first keyframe (the crouched, centred stand after the crouch and settle)."""
    kw.setdefault("kL", 1.35)
    kw.setdefault("kR", 1.1)
    sw = W.Sway(T=T_step, n_steps=n_steps, dt=kf_s, k=1.2, ss=0.5, h=0.04, wide=True, first=first, **kw)
    K = W.S.kin(0.02)
    c0 = sw.phases[0]["t0"] + 0.25 * T_step              # the first single support's centre
    ends = [p["t1"] + 0.25 * T_step for p in sw.phases]   # mid double support after each step
    t_lift0, t_last = c0 - 0.5 * T_step, ends[-1]
    t_first = sw.segs[1]["t"]
    out, prev, end_i = [], 0, 0
    for seg in sw.segs[1:]:
        t = seg["t"]
        if t > t_last + 0.5 * kf_s:
            break
        tm = int(round(1e3 * (t - t_first)))
        # the policy convention: L swings while sin(phi) > 0; single support = the middle half of each half cycle
        phi = math.pi * (t - c0) / T_step + math.pi / 2 + (0.0 if first == "L" else math.pi)
        flags = LIFT if t_lift0 - 0.5 * kf_s <= t <= t_last + 0.5 * kf_s else 0
        is_end = end_i < len(ends) and t >= ends[end_i] - 0.5 * kf_s
        if is_end:
            flags |= STEP_END
        out.append((tm, hex_frame(tm - prev, phi, flags, seg["q"], cal), "kf"))
        prev = tm
        if is_end:
            pa = seg["pa"]
            rq = K.pose(shift=pa["shift"], fwd=pa["fwd"], footL=(pa["feet"]["L"][0], pa["feet"]["L"][1], 0.0),
                        footR=(pa["feet"]["R"][0], pa["feet"]["R"][1], 0.0))
            rq["L_hip_yaw"] += pa["yaw"]["L"]
            rq["R_hip_yaw"] += pa["yaw"]["R"]
            out.append((tm, hex_frame(0, 0.0, REST, rq, cal), "rest"))
            end_i += 1
    sw.t_first = t_first
    return sw, out


def _lib(path=LIB):
    L = ctypes.CDLL(path)
    L.sil_gait_new.restype = ctypes.c_void_p
    L.sil_gait_new.argtypes = [ctypes.c_float] * 6
    for f in ("sil_gait_free", "sil_gait_start", "sil_gait_stop", "sil_gait_queued"):
        getattr(L, f).argtypes = [ctypes.c_void_p]
    L.sil_gait_set_learned.argtypes = [ctypes.c_void_p, ctypes.c_char, ctypes.c_float]
    L.sil_gait_learned.argtypes = [ctypes.c_void_p, ctypes.c_char]
    L.sil_gait_learned.restype = ctypes.c_float
    L.sil_gait_add.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    L.sil_gait_tick.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_float, ctypes.c_float,
                                ctypes.POINTER(ctypes.c_int32), ctypes.POINTER(ctypes.c_float),
                                ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_int),
                                ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_float)]
    return L


def run(T_step=2.0, direction="march", plant="fit", extra_ms=0.0, n_steps=8, seed=SEED, gains=(0.25, 1.0),
        lead_steps=2.0, first="L", amp=None):
    """One sway gait through the firmware's track + trim in the sim plant: metrics + the per-step reports."""
    import gate_common as GC
    import rec
    import simh
    L = _lib()
    sw, fr = gait(T_step, n_steps, first=first, **DIRS[direction], **(dict(kL=amp[0], kR=amp[1]) if amp else {}))
    g = L.sil_gait_new(gains[0], gains[1], 15.0, 14.0, 0.6, 15.0)
    L.sil_gait_set_learned(g, b"L", seed[0])
    L.sil_gait_set_learned(g, b"R", seed[1])
    lead_ms = 1e3 * lead_steps * T_step
    pend = deque(fr)
    max_q = 0

    def feed(t_ms):
        nonlocal max_q
        while pend and pend[0][0] <= t_ms + lead_ms:
            tm, hx, _ = pend.popleft()
            rc = L.sil_gait_add(g, hx.encode())
            if rc != 0:
                raise RuntimeError(f"track add refused ({rc}) at {tm} ms")
        max_q = max(max_q, L.sil_gait_queued(g))

    feed(0)
    assert L.sil_gait_start(g) == 0
    t_track_end = fr[-1][0] / 1e3 + 3.0
    _, env = TB.SR.build(PLANTS[plant], t_track_end + 15)
    pol = list(rec.POL)
    q0 = np.array([sw.q_start[n] for n in pol])
    simh.place_and_hold(env, np.clip(q0, env._lo, env._hi), 1.0)
    q12 = (ctypes.c_int32 * 12)()
    phi, hipj = ctypes.c_float(), ctypes.c_float()
    stance, tstate = ctypes.c_int(), ctypes.c_int()
    rep = (ctypes.c_float * 7)()
    cur = {"t": q0.copy()}
    env._action_to_ctrl = lambda a_: cur["t"]
    a0 = np.zeros(env.action_space.shape[0], np.float32)
    d = env.data
    pads = GC.pad_gids(env)
    # the crouch (a 2 s pose move to the first keyframe) and 1 s settle: the stepper's start, before `track play`
    L.sil_gait_tick(g, 0, float("nan"), 0.02, q12, ctypes.byref(phi), ctypes.byref(hipj), ctypes.byref(stance),
                    ctypes.byref(tstate), rep)
    q_first = {n: from_ticks(n, q12[i]) for i, n in enumerate(LEG)}
    for k in range(150):
        u = TB._minjerk(min(1.0, (k + 1) / 100))
        cur["t"] = np.clip(np.array([q0[i] + u * (q_first[n] - q0[i]) for i, n in enumerate(pol)]), env._lo, env._hi)
        env.step(a0)
    lag = 1 + int(round(extra_ms / 20.0))
    buf = deque(maxlen=lag + 1)
    reports, fell, fall_dir, t_fall, peak_tilt = [], False, None, None, 0.0
    ph_up = {i: [0, 0, -1.0] for i in range(len(sw.phases))}      # n_hold, n_up, max clearance
    done_at = None
    for k in range(int(t_track_end * 50)):
        t_ms = 20 * k
        feed(t_ms)
        up = GC.up_of(d.qpos[3:7])
        roll = -math.asin(max(-1.0, min(1.0, up[1])))
        buf.append(roll)
        r = L.sil_gait_tick(g, t_ms, buf[0], 0.02, q12, ctypes.byref(phi), ctypes.byref(hipj), ctypes.byref(stance),
                            ctypes.byref(tstate), rep)
        if r & 2:
            reports.append(dict(stance=chr(int(rep[0])), t=t_ms / 1e3, lean_in=math.degrees(rep[1]),
                                lean_out=math.degrees(rep[2]), e_mid=math.degrees(rep[3]), T_mid=math.degrees(rep[4]),
                                hike_mid=math.degrees(rep[5]), aborted=bool(rep[6])))
        tgt = {n: from_ticks(n, q12[i]) for i, n in enumerate(LEG)}
        if stance.value:
            tgt[f"{chr(stance.value)}_hip_roll"] += hipj.value
        cur["t"] = np.clip(np.array([tgt[n] for n in pol]), env._lo, env._hi)
        env.step(a0)
        # the lift, as testbed/A2: the swing sole's lowest pad > 5 mm over the hold (raise end -> lower start)
        ts = sw.t_first + t_ms / 1e3
        for i, p in enumerate(sw.phases):
            if p["hold_from"] <= ts <= p["lower_start"]:
                c = GC.clearance(env, pads)[p["swing"]]
                s = ph_up[i]
                s[0] += 1
                s[1] += c > TB.UP_M
                s[2] = max(s[2], c)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, up[2]))))
        peak_tilt = max(peak_tilt, tilt)
        if tilt > TB.FALL_DEG:
            fell, t_fall = True, t_ms / 1e3
            p = sw.phase_at(ts) or sw.phases[min(len(sw.phases) - 1, sum(ts > q["t1"] for q in sw.phases))]
            fall_dir = "OUT" if roll * (-1.0 if p["stance"] == "L" else 1.0) > 0 else "IN"
            break
        if r & 1:
            done_at = done_at if done_at is not None else t_ms
            if t_ms - done_at >= 1000:          # 1 s standing at rest after the track is done
                break
    learned = (L.sil_gait_learned(g, b"L"), L.sil_gait_learned(g, b"R"))
    L.sil_gait_free(g)
    phs = [dict(stance=p["stance"], up_frac=ph_up[i][1] / max(ph_up[i][0], 1), clr_max_mm=1e3 * ph_up[i][2])
           for i, p in enumerate(sw.phases) if ph_up[i][0]]
    return dict(T=T_step, dir=direction, plant=plant, delay=extra_ms, fell=fell, fall_dir=fall_dir, t_fall=t_fall,
                done=done_at is not None, n_steps=n_steps, lifts=sum(p["up_frac"] >= 0.5 for p in phs),
                phases=phs, peak_tilt=round(peak_tilt, 1), reports=reports, aborts=sum(x["aborted"] for x in reports),
                learned=learned, max_queued=max_q, frames=len([f for f in fr if f[2] == "kf"]))


def _job(a):
    try:
        return run(*a)
    except Exception as e:  # noqa: BLE001
        return dict(args=list(a), error=repr(e)[:300])


def summary(rows):
    ok = [r for r in rows if "error" not in r]
    lines = []
    for Ts in sorted({r["T"] for r in ok}, reverse=True):
        rs = [r for r in ok if r["T"] == Ts]
        reps = [x for r in rs for x in r["reports"]]
        li = [x["lean_in"] for x in reps] or [0.0]
        lo = [x["lean_out"] for x in reps] or [0.0]
        n_ph = sum(r["n_steps"] for r in rs)
        tm = {s: [x["T_mid"] for x in reps if x["stance"] == s] or [float("nan")] for s in "LR"}
        hk = {s: [x["hike_mid"] for x in reps if x["stance"] == s] or [float("nan")] for s in "LR"}
        clr = [p["clr_max_mm"] for r in rs for p in r["phases"]] or [0.0]
        lines.append(
            f"T {Ts:g} s: {len(rs)} runs, falls {sum(r['fell'] for r in rs)}, aborts {sum(r['aborts'] for r in rs)}, "
            f"lifts {sum(r['lifts'] for r in rs)}/{n_ph} ({100 * sum(r['lifts'] for r in rs) / max(n_ph, 1):.0f}%), "
            f"swing sole peak clearance median {np.median(clr):.0f} mm (10th pct {np.percentile(clr, 10):.0f}); "
            f"lean toward swing median {np.median(li):.1f} max {max(li):.1f} deg, outward median {np.median(lo):.1f} "
            f"max {max(lo):.1f}; at mid support T L {np.median(tm['L']):.1f} R {np.median(tm['R']):.1f} deg, "
            f"hike L {np.median(hk['L']):.1f} R {np.median(hk['R']):.1f} deg; queue max {max(r['max_queued'] for r in rs)}")
        for dn in sorted({r["dir"] for r in rs}):
            rd = [r for r in rs if r["dir"] == dn]
            nd = sum(r["n_steps"] for r in rd)
            lines.append(f"    {dn:6s} falls {sum(r['fell'] for r in rd)} lifts {sum(r['lifts'] for r in rd)}/{nd} "
                         f"peak tilt max {max(r['peak_tilt'] for r in rd):.1f} deg")
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--T", default="4,3,2.5,2")
    ap.add_argument("--dirs", default=",".join(DIRS))
    ap.add_argument("--plants", default=",".join(PLANTS))
    ap.add_argument("--delays", default="0,60")
    ap.add_argument("--steps", type=int, default=8)
    ap.add_argument("--amp", default="1.3,1.25", help="sway amplitude over L,R (x 62 mm); A2 used 1.35,1.1")
    ap.add_argument("--seed", default="5.2,3.4", help="trim T L,R (hike deg); A2 used 8,3")
    ap.add_argument("--workers", type=int, default=14)
    ap.add_argument("--out", default=os.path.join(HERE, "results", "sil_gait.json"))
    a = ap.parse_args(argv)
    amp = tuple(float(x) for x in a.amp.split(","))
    seed = tuple(float(x) for x in a.seed.split(","))
    jobs = [(float(t), dn, p, float(dl), a.steps, seed, (0.25, 1.0), 2.0, "L", amp) for t in a.T.split(",")
            for dn in a.dirs.split(",") for p in a.plants.split(",") for dl in a.delays.split(",")]
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(_job, jobs))
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w") as f:
        json.dump(rows, f, indent=0)
    errs = [r for r in rows if "error" in r]
    print(f"{len(rows) - len(errs)} runs, {len(errs)} errors")
    print(summary(rows))
    if errs:
        print("e.g.", errs[0])
    return 0


if __name__ == "__main__":
    sys.exit(main())
