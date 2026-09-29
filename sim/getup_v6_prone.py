"""Get-up from PRONE with the arms (#85, 2026-09-29).

Forward falls end prone. The legs-only roll prone -> side -> supine of the
2026-09-14 study (design record section 12.1) fails 0/6 on the as-drawn robot
with the arms folded up (no3250_getup.txt): the arms block the roll. This
study searches, by the method of getup-decision-2026-09-17.md section 4 --
continuous search over full joint ranges, self_collide=True, the deploy servo
model, a winner verified 3x slower and across play / mu / servo strength --
for a path from prone (or from the forward fall itself) back to standing.

Plant: getup_v6_shoulder's r5_asdrawn_rom120 (lumped as-drawn robot with the
girdle and arms, 2.10 kg, hip ROM -120), with the elbow's as-drawn stops
(DESIGN.md section 3: -100 ... +10; the plant draws +-150). Servos: Plan B
(STS3215 everywhere, P x4 on the rolls and knees). Search bounds = the
as-drawn joint ranges. The start is the realistic one: the robot lies prone
with its arms at the walk's idle pose (shoulder +15, elbow 0), legs straight.

Candidate families (the recorded dead ends -- prone push-up, one-arm
side-seat push, a third shoulder DOF, legs only -- are not re-searched):
  stow     the arms go to ONE searched pose and stay there; the legs roll the
           body (every leg joint free per keyframe): "an arm stow that clears
           the roll"
  onearm   the left arm is free per keyframe (it can push the floor to lift
           the left side), the right arm holds one searched pose, legs free:
           "a one-arm roll to supine"
  free     every arm and leg joint free per keyframe (the union)
  catch    from STANDING, a forward kick; when the torso pitches past a
           searched angle the arms swing forward to catch the fall on the
           hands, then searched keyframes push back over the feet into the
           seat push's crouch hold, and the seat push's own rise stands it:
           "catching a forward fall on the hands and pushing back"

A roll succeeds when the torso ends on its back (front_z > 0.7, the section
12.1 criterion) with the arms folded up to 180 and the legs straight -- the
seat push's start -- and the whole chain (roll + the recommended seat push)
then has to stand.

    .venv/bin/python sim/getup_v6_prone.py probe                       # stills + the section 12.1 roll per arm pose
    .venv/bin/python sim/getup_v6_prone.py rollsearch stow 6 40         # family, restarts, iterations
    .venv/bin/python sim/getup_v6_prone.py catchsearch 6 40
    .venv/bin/python sim/getup_v6_prone.py verify <family> '<json x>'   # chain, 6 conditions, x1 and x3
    .venv/bin/python sim/getup_v6_prone.py render <family> '<json x>' out.mp4
"""
from __future__ import annotations

import dataclasses
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402

from gen_plant_v6 import build_xml  # noqa: E402
import design_gates as DG  # noqa: E402
import getup_v6 as G  # noqa: E402
import getup_v6_shoulder as S  # noqa: E402
import gate_no3250 as GN  # noqa: E402

PLANT = "r5_asdrawn_rom120"
SET = "3215_p4rk"
TMP = os.environ.get("TMPDIR", "/tmp")
NOMINAL = dict(play_deg=3.0, mu=0.7, servo_scale=1.0)
IDLE = dict(shoulder=15, elbow=0)              # the walk's idle arm pose (DESIGN.md 5.4)
FOLD = dict(shoulder=180, elbow=0)             # the seat push's start
LEG = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle")
# as-drawn ranges (DESIGN.md section 3), degrees, per joint name
RANGE = {}
for _s in "LR":
    RANGE[f"{_s}_hip_yaw"] = (-45, 45)
    RANGE[f"{_s}_hip_roll"] = (-30, 45) if _s == "L" else (-45, 30)   # + = L abduction
    RANGE[f"{_s}_hip_pitch"] = (-120, 90)
    RANGE[f"{_s}_knee"] = (-130, 0)
    RANGE[f"{_s}_ankle"] = (-40, 40)
    RANGE[f"{_s}_shoulder"] = (-90, 200)
    RANGE[f"{_s}_elbow"] = (-100, 10)
KEY_JOINTS = [f"{s}_{j}" for s in "LR" for j in LEG] + ["L_shoulder", "L_elbow", "R_shoulder", "R_elbow"]
T_MOVE, T_HOLD = (0.4, 2.0), (0.2, 1.5)
NK = 5


# ------------------------------------------------------------------ plant / env
_PLANT_CACHE = {}


def plant():
    if "p" not in _PLANT_CACHE:
        p = dataclasses.replace(S.BASE, **S.CONFIGS[PLANT])
        src = build_xml(p)
        n1 = src.count('range="-150 150"')
        src = src.replace('range="-150 150"', 'range="-100 10"')
        old = f'ctrlrange="{math.radians(-150):.10f} {math.radians(150):.10f}"'
        n2 = src.count(old)
        src = src.replace(old, f'ctrlrange="{math.radians(-100):.10f} {math.radians(10):.10f}"')
        assert n1 == 2 and n2 == 2, (n1, n2)          # both elbows, joint and actuator
        xml = os.path.join(TMP, f"prone_{PLANT}_{os.getpid()}.xml")
        with open(xml, "w") as fh:
            fh.write(src)
        _PLANT_CACHE["p"] = (p, xml)
    return _PLANT_CACHE["p"]


def make_env(p, xml, cond):
    env = G.make_env(p, xml, mu=cond["mu"], play_deg=cond["play_deg"])
    env.reset(seed=0)
    G.EXTRA[:] = [env.model.actuator(i).name for i in range(12, env.model.nu)]
    kp, kd, stall, w0 = env._servo
    na = env._nq_act
    sc = cond["servo_scale"]
    stall_a = np.full(na, stall) * sc
    w0_a = np.full(na, w0) * sc
    kpa = np.full(na, float(kp)); kda = np.full(na, float(kd))
    pj = GN.SETS[SET]
    for i, n in enumerate(DG.JN):
        if n in pj:
            s = DG.SERVOS[pj[n]]
            stall_a[i] = s["stall"] * sc; w0_a[i] = s["w0"] * sc
            kpa[i] *= s["kp_scale"]; kda[i] *= s["kp_scale"]
    env._servo = np.array([kpa, kda, stall_a, w0_a], dtype=object)
    env.fall_up_z = -2.0
    env.fall_height = -1.0
    return env


def names_of(env):
    return list(DG.JN) + list(G.EXTRA)


def q_of(off, na):
    q = G.q_from_offsets(off)
    return q[:na] if len(q) >= na else np.concatenate([q, np.zeros(na - len(q))])


def _inv(env):
    d0, hi, lo = env._default, env._hi, env._lo
    return lambda q: np.clip(np.where(q >= d0, (q - d0) / np.maximum(hi - d0, 1e-6),
                                      (q - d0) / np.maximum(d0 - lo, 1e-6)), -1, 1)


def torso_axes(env):
    R = env.data.xmat[env._torso_bid].reshape(3, 3)
    return float(R[2, 2]), float(R[2, 0]), float(R[2, 1])     # up, front, side


class Recorder:
    def __init__(self, render=None, cam=(1.4, -18, 60), size=(480, 640), label=""):
        self.front_max = -2.0
        self.frames = [] if render else None
        self.render, self.cam0, self.size, self.label = render, cam, size, label
        self.rnd = None
        self.k = 0

    def __call__(self, env, step_label, t):
        up, front, side = torso_axes(env)
        self.front_max = max(self.front_max, front)
        if self.frames is not None and self.k % 2 == 0:
            if self.rnd is None:
                self.rnd = mujoco.Renderer(env.model, self.size[0], self.size[1])
                self.cam = mujoco.MjvCamera()
                self.cam.distance, self.cam.elevation, self.cam.azimuth = self.cam0
            d = env.data
            self.cam.lookat[:] = [d.qpos[0], d.qpos[1], 0.15]
            self.rnd.update_scene(d, self.cam)
            self.frames.append(G._caption(self.rnd.render().copy(), f"{self.label}  {step_label}  t={t:4.1f}s"))
        self.k += 1

    def write(self):
        if self.frames:
            import imageio
            from static_gait import _write_video
            _write_video(self.frames, self.render, fps=25)
            # a contact sheet over the WHOLE run (the helper's strip covers 22-50 % only)
            idx = np.linspace(0, len(self.frames) - 1, 12).astype(int)
            rows = [np.concatenate([self.frames[i] for i in idx[r * 6:(r + 1) * 6]], axis=1) for r in range(2)]
            imageio.imwrite(os.path.splitext(self.render)[0] + "_sheet.png", np.concatenate(rows, axis=0))


def run_keys(env, keys, q_prev, rec=None, time_scale=1.0, t0=0.0, prot=None):
    """min-jerk through (label, q_target (na,), move_s, hold_s) keyframes"""
    inv = _inv(env)
    dt = env.control_dt
    t = t0
    peak = np.zeros(env._nq_act)
    for label, q_tgt, move_s, hold_s in keys:
        n = int(move_s * time_scale / dt)
        for i in range(n + int(hold_s * time_scale / dt)):
            s = min(1.0, (i + 1) / max(n, 1))
            s = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
            env.step(inv(q_prev + (q_tgt - q_prev) * s))
            t += dt
            np.maximum(peak, np.abs(np.asarray(env._servo_tau, dtype=float)), out=peak)
            if rec is not None:
                rec(env, label, t)
            if prot is not None:
                prot.tick(env, dt)
        q_prev = q_tgt
    return q_prev, t, peak


def standing_state(env, p):
    return bool(torso_axes(env)[0] > 0.9 and env.data.qpos[2] > 0.8 * p.z_yaw_above_sole)


# ------------------------------------------------------------------ the recommended seat push, as keys
def seat_push_keys(na):
    return [(lab, q_of(off, na), m, h) for lab, off, m, h in
            S.seat_push(False, 90, 0, 2.0, -25, elbow=(-90, 0))]


# ------------------------------------------------------------------ the seat push's ENTRY from a real fall
# The recommended seat push starts with the arms already folded up to 180.
# A real backward fall lands supine with the arms at the walk's idle pose
# (+15), and every arm path from there to 180 passes "straight back" (+90),
# which on the back points into the floor. So the entry -- the arm path and
# the hip flexion through lie / prop / sit-up / fold, before the brace -- is
# searched, from supine with the arms idle, and every candidate is also run
# 3x slower so that the winner is quasi-static (the method's check). From the
# brace on, the sequence is the recommended one.
ENTRY_NAMES = ["l_sh", "l_el", "l_t", "p_hip", "p_sh", "p_el", "p_t", "su_hip", "su_sh", "su_el", "su_t",
               "f_hip", "f_sh", "f_el", "f_t", "b_t"]
_T = (0.5, 3.0)
ENTRY_BOUNDS = np.array([RANGE["L_shoulder"], RANGE["L_elbow"], _T,
                         (-120, 0), RANGE["L_shoulder"], RANGE["L_elbow"], _T,
                         (-120, -30), RANGE["L_shoulder"], RANGE["L_elbow"], _T,
                         (-120, -60), RANGE["L_shoulder"], RANGE["L_elbow"], _T, _T], float)
ENTRY_SEEDS = [
    # arms forward (up to the sky), then sit up
    dict(l_sh=-90, l_el=0, l_t=1.0, p_hip=0, p_sh=-90, p_el=0, p_t=1.0, su_hip=-90, su_sh=-90, su_el=0, su_t=1.5,
         f_hip=-110, f_sh=-45, f_el=-100, f_t=1.0, b_t=1.5),
    # prop up on the forearms at the sides, then sit up
    dict(l_sh=15, l_el=-100, l_t=1.0, p_hip=0, p_sh=60, p_el=-100, p_t=1.5, su_hip=-90, su_sh=60, su_el=-100, su_t=1.5,
         f_hip=-110, f_sh=60, f_el=-90, f_t=1.0, b_t=1.5),
    # the round-1 winner of the 9-parameter entry (6/6 at x1, 0/6 at x3)
    dict(l_sh=-21.86, l_el=-2.07, l_t=0.68, p_hip=0, p_sh=-21.86, p_el=-2.07, p_t=0.5, su_hip=-90, su_sh=-29.12,
         su_el=0.67, su_t=1.44, f_hip=-110, f_sh=33.63, f_el=-97.89, f_t=1.0, b_t=0.86),
    # the recommended fold to 180, slowly, elbow bent
    dict(l_sh=180, l_el=-100, l_t=3.0, p_hip=0, p_sh=180, p_el=0, p_t=1.0, su_hip=-90, su_sh=180, su_el=0, su_t=1.5,
         f_hip=-110, f_sh=180, f_el=0, f_t=1.0, b_t=1.0)]


def entry_keys(x, na):
    """supine (arms idle) -> lie (arms move) -> prop -> sit up -> fold ->
    brace (the recommended 90 / -90) -> the recommended tuck, push, crouch
    hold and rise"""
    sp = S.seat_push(False, 90, 0, 2.0, -25, elbow=(-90, 0))
    arms = lambda s, e: dict(shoulder=s, elbow=e)
    g = lambda k, d: x.get(k, d)
    keys = [("lie (arms move)", q_of(arms(x["l_sh"], x["l_el"]), na), x["l_t"], 0.8),
            ("prop", q_of(dict(hip_pitch=g("p_hip", 0.0), **arms(g("p_sh", x["l_sh"]), g("p_el", x["l_el"]))), na),
             g("p_t", 0.5), 0.6),
            ("sit up", q_of(dict(hip_pitch=g("su_hip", -90.0), **arms(x["su_sh"], x["su_el"])), na), x["su_t"], 0.8),
            ("fold", q_of(dict(hip_pitch=g("f_hip", -110.0), **arms(x["f_sh"], x["f_el"])), na), g("f_t", 1.0), 0.6),
            ("brace", q_of(dict(hip_pitch=-110, **arms(90, -90)), na), x["b_t"], 0.8)]
    return keys + [(lab, q_of(off, na), m, h) for lab, off, m, h in sp[4:]]


def run_entry(x, cond=NOMINAL, time_scale=1.0, render=None, protection=None, start_arms=IDLE):
    _set_hk()
    p, xml = plant()
    env = make_env(p, xml, cond)
    na = env._nq_act
    q0 = q_of(dict(**start_arms), na)
    G.settle_fallen(env, p, "supine", q0)
    rec = Recorder(render=render, label="supine (arms idle), seat push")
    prot = None
    if protection is not None:
        from sts_servo_model import STSProtection
        prot = STSProtection(env, names_of(env), **protection)
    q, t, pk = run_keys(env, entry_keys(x, na), q0, rec, time_scale, prot=prot)
    up, front, side = torso_axes(env)
    nm = names_of(env)
    out = dict(stand=standing_state(env, p), up=up, z=float(env.data.qpos[2]), peak=float(pk.max()),
               peak_joint=nm[int(pk.argmax())])
    if prot is not None:
        out["prot"] = prot.report()
    rec.write()
    return out


def _entry_run_score(r):
    return 3.0 * (1.0 if r["stand"] else 0.0) + max(0.0, r["up"]) + 4.0 * r["z"]


def _eval_entry(args):
    """conds: (condition dict, time scale) pairs"""
    x, conds = args
    rs = []
    for c, ts in conds:
        try:
            rs.append(run_entry(x, cond=c, time_scale=ts))
        except Exception as e:  # noqa: BLE001
            rs.append(dict(stand=False, up=-1.0, z=0.0, err=str(e)))
    sc = [_entry_run_score(r) for r in rs]
    return dict(score=min(sc) + 0.2 * float(np.mean(sc)), n_stand=sum(r["stand"] for r in rs), per=rs)


def _set_hk():
    S.H, S.K = -120.0, -130.0


# ------------------------------------------------------------------ roll families
def _base(family):
    """'stow_fold' -> 'stow': the _fold suffix only changes where the roll
    ends (arms folded up to 180, the recommended seat push's start)"""
    return family[:-5] if family.endswith("_fold") else family


def roll_space(family):
    """parameter names and bounds for a roll family"""
    family = _base(family)
    names, bounds = [], []
    if family in ("stow", "onearm"):
        arms = ("L_shoulder", "L_elbow", "R_shoulder", "R_elbow") if family == "stow" else ("R_shoulder", "R_elbow")
        for j in arms:
            names.append(f"stow_{j}"); bounds.append(RANGE[j])
    for k in range(NK):
        for j in KEY_JOINTS:
            if family == "stow" and j in ("L_shoulder", "L_elbow", "R_shoulder", "R_elbow"):
                continue
            if family == "onearm" and j in ("R_shoulder", "R_elbow"):
                continue
            names.append(f"k{k}_{j}"); bounds.append(RANGE[j])
        names += [f"k{k}_t", f"k{k}_h"]; bounds += [T_MOVE, T_HOLD]
    return names, np.array(bounds, float)


def roll_keys(family, x, na):
    """x: {name: value} -> keyframe list. The roll ends with a fixed keyframe
    that straightens the legs and either puts the arms back at the idle pose
    -- the state a backward fall lands in, where the seat push's searched
    entry (entry_keys) starts -- or, for a '_fold' family, folds them up to
    180, where the recommended seat push starts."""
    keys = []
    for k in range(NK):
        off = {}
        for j in KEY_JOINTS:
            if f"stow_{j}" in x:
                off[j] = x[f"stow_{j}"]
            else:
                off[j] = x[f"k{k}_{j}"]
        keys.append((f"k{k}", q_of(off, na), x[f"k{k}_t"], x[f"k{k}_h"]))
    if family.endswith("_fold"):
        keys.append(("arms folded up, legs straight", q_of(dict(**FOLD), na), 1.5, 1.5))
    else:
        keys.append(("arms idle, legs straight", q_of(dict(**IDLE), na), 1.5, 1.5))
    return keys


# the seat-push entry the chain uses: the entry search's winner, passed as JSON
# in PRONE_ENTRY (an environment variable, so spawned pool workers see it too)
ENTRY_BEST = json.loads(os.environ.get("PRONE_ENTRY", "{}"))


def run_roll(family, x, cond=NOMINAL, time_scale=1.0, chain=False, render=None, protection=None):
    """prone (arms idle) -> roll keys [-> the seat push with the searched
    entry, ENTRY_BEST]. Returns front/up at the end of the roll, max front
    reached, and whether the chain stands."""
    _set_hk()
    p, xml = plant()
    env = make_env(p, xml, cond)
    na = env._nq_act
    q0 = q_of(dict(**IDLE), na)
    G.settle_fallen(env, p, "prone", q0)
    rec = Recorder(render=render, label=f"prone {family}")
    prot = None
    if protection is not None:
        from sts_servo_model import STSProtection
        prot = STSProtection(env, names_of(env), **protection)
    q, t, pk = run_keys(env, [("lie prone", q0, 0.5, 0.8)] + roll_keys(family, x, na), q0, rec, time_scale, prot=prot)
    up, front, side = torso_axes(env)
    qa = env.data.qpos[7:7 + na]
    nm = names_of(env)
    arm_end = FOLD["shoulder"] if family.endswith("_fold") else IDLE["shoulder"]
    arm_err = max(abs(math.degrees(qa[nm.index(j)]) - arm_end) for j in ("L_shoulder", "R_shoulder"))
    out = dict(front=front, up=up, side=side, front_max=rec.front_max, arm_err=arm_err, z=float(env.data.qpos[2]),
               roll_ok=bool(front > 0.7), peak_roll=float(pk.max()), peak_roll_joint=nm[int(pk.argmax())])
    if chain:
        tail = seat_push_keys(na) if family.endswith("_fold") else entry_keys(ENTRY_BEST, na)
        q, t, pk2 = run_keys(env, tail, q, rec, time_scale, t0=t, prot=prot)
        up, front, side = torso_axes(env)
        out.update(stand=standing_state(env, p), up_end=up, z_end=float(env.data.qpos[2]),
                   peak_chain=float(np.maximum(pk, pk2).max()), peak_chain_joint=nm[int(np.maximum(pk, pk2).argmax())])
    if prot is not None:
        out["prot"] = prot.report()
    rec.write()
    return out


def roll_score(r):
    return r["front"] + 0.3 * r["front_max"] + (1.0 if r["roll_ok"] else 0.0) - 0.004 * r["arm_err"]


# ------------------------------------------------------------------ catch family
CATCH_NAMES = ["trig_deg", "c_sh", "c_el", "c_hip", "c_knee", "c_ankle", "c_t",
               "p1_sh", "p1_el", "p1_hip", "p1_knee", "p1_ankle", "p1_t", "p1_h",
               "p2_sh", "p2_el", "p2_hip", "p2_knee", "p2_ankle", "p2_t", "p2_h",
               "p3_sh", "p3_el", "p3_ankle", "p3_t", "p3_h"]
CATCH_BOUNDS = np.array([(3, 30), RANGE["L_shoulder"], RANGE["L_elbow"], RANGE["L_hip_pitch"], RANGE["L_knee"], RANGE["L_ankle"], (0.2, 1.0),
                         RANGE["L_shoulder"], RANGE["L_elbow"], RANGE["L_hip_pitch"], RANGE["L_knee"], RANGE["L_ankle"], T_MOVE, T_HOLD,
                         RANGE["L_shoulder"], RANGE["L_elbow"], RANGE["L_hip_pitch"], RANGE["L_knee"], RANGE["L_ankle"], T_MOVE, T_HOLD,
                         RANGE["L_shoulder"], RANGE["L_elbow"], RANGE["L_ankle"], T_MOVE, T_HOLD], float)
_TIP = {}


def _stand_reset(env, p, arms=IDLE):
    m, d = env.model, env.data
    mujoco.mj_resetData(m, d)
    na = env._nq_act
    q = q_of(dict(**arms), na)
    d.qpos[2] = p.z_yaw_above_sole - p.hip_z
    d.qpos[7:7 + na] = q
    mujoco.mj_forward(m, d)
    # re-seed the actuation chain (dead time, lag, play shaft) at the held pose
    env._act_hist = None; env._lag_y = None; env._shaft = None
    return q


def tip_speed(cond=NOMINAL):
    """smallest forward CoM kick (m/s) that tips the standing robot (arms idle)
    out of standing within 3 s -- getup_v6_side.find_tip_speed on this plant"""
    key = (cond["play_deg"], cond["mu"], cond["servo_scale"])
    if key in _TIP:
        return _TIP[key]
    p, xml = plant()
    env = make_env(p, xml, cond)
    inv = _inv(env)

    def falls(v):
        q = _stand_reset(env, p)
        env.data.qvel[0] = v
        for _ in range(int(3.0 / env.control_dt)):
            env.step(inv(q))
        return torso_axes(env)[0] < 0.9
    lo, hi = 0.05, 1.5
    for _ in range(8):
        mid = 0.5 * (lo + hi)
        lo, hi = (lo, mid) if falls(mid) else (mid, hi)
    _TIP[key] = hi
    return hi


def catch_keys(x, na):
    sym = lambda sh, el, hip=None, knee=None, ankle=None: q_of(dict(
        shoulder=sh, elbow=el, **({} if hip is None else dict(hip_pitch=hip)),
        **({} if knee is None else dict(knee=knee)), **({} if ankle is None else dict(ankle=ankle))), na)
    return [("catch", sym(x["c_sh"], x["c_el"], x["c_hip"], x["c_knee"], x["c_ankle"]), x["c_t"], 0.3),
            ("push back 1", sym(x["p1_sh"], x["p1_el"], x["p1_hip"], x["p1_knee"], x["p1_ankle"]), x["p1_t"], x["p1_h"]),
            ("push back 2", sym(x["p2_sh"], x["p2_el"], x["p2_hip"], x["p2_knee"], x["p2_ankle"]), x["p2_t"], x["p2_h"]),
            ("to the crouch", sym(x["p3_sh"], x["p3_el"], -120, -130, x["p3_ankle"]), x["p3_t"], x["p3_h"])]


def rise_keys(na):
    """the seat push's own crouch hold and rise (getup_v6_shoulder.seat_push)"""
    sp = S.seat_push(False, 90, 0, 2.0, -25, elbow=(-90, 0))
    return [(lab, q_of(off, na), m, h) for lab, off, m, h in sp[6:]]


def run_catch(x, kick=1.5, cond=NOMINAL, time_scale=1.0, render=None, heading_deg=0.0, protection=None):
    """stand (arms idle) -> forward kick (kick x the tip speed) -> hold until
    the torso pitches past trig_deg (or 2 s) -> catch keys -> rise."""
    _set_hk()
    p, xml = plant()
    v = kick * tip_speed(NOMINAL)
    env = make_env(p, xml, cond)
    na = env._nq_act
    inv = _inv(env)
    q = _stand_reset(env, p)
    h = math.radians(heading_deg)
    env.data.qvel[0] = v * math.cos(h); env.data.qvel[1] = v * math.sin(h)
    rec = Recorder(render=render, label="forward fall, catch")
    t, dt = 0.0, env.control_dt
    trig = math.cos(math.radians(x["trig_deg"]))
    t_trig = None
    while t < 2.0:
        env.step(inv(q))
        t += dt
        rec(env, "falling", t)
        up, front, side = torso_axes(env)
        if up < trig:
            t_trig = t
            break
    prot = None
    if protection is not None:
        from sts_servo_model import STSProtection
        prot = STSProtection(env, names_of(env), **protection)
    ck = catch_keys(x, na)
    q, t, pk = run_keys(env, ck, q, rec, time_scale, t0=t, prot=prot)
    up_c, front_c, _ = torso_axes(env)
    z_c = float(env.data.qpos[2])
    feet_only = set(G.contacts_summary(env.model, env.data)) <= {"L_foot", "R_foot"}
    q, t, pk2 = run_keys(env, rise_keys(na), q, rec, time_scale, t0=t, prot=prot)
    up, front, side = torso_axes(env)
    nm = names_of(env)
    pkm = np.maximum(pk, pk2)
    out = dict(stand=standing_state(env, p), up=up, z=float(env.data.qpos[2]), t_trig=t_trig, up_crouch=up_c,
               z_crouch=z_c, feet_only=feet_only, front_crouch=front_c, kick=v, peak=float(pkm.max()),
               peak_joint=nm[int(pkm.argmax())])
    if prot is not None:
        out["prot"] = prot.report()
    rec.write()
    return out


def catch_score(r):
    s = 3.0 * (1.0 if r["stand"] else 0.0) + max(0.0, r["up"]) + 4.0 * r["z"]
    s += 0.5 * max(0.0, r["up_crouch"]) + (0.5 if r["feet_only"] else 0.0)
    return s


# ------------------------------------------------------------------ search (CEM, pooled)
def _eval_roll(args):
    family, x = args
    try:
        r = run_roll(family, x)
    except Exception as e:  # noqa: BLE001 -- a diverged sim scores as a failure
        return dict(score=-9.0, err=str(e))
    r["score"] = roll_score(r)
    return r


def _eval_catch(args):
    x, kicks = args
    rs = []
    for k in kicks:
        try:
            rs.append(run_catch(x, kick=k))
        except Exception as e:  # noqa: BLE001
            rs.append(dict(stand=False, up=-1.0, z=0.0, up_crouch=-1.0, feet_only=False, err=str(e)))
    return dict(score=min(catch_score(r) for r in rs) + 0.2 * np.mean([catch_score(r) for r in rs]),
                n_stand=sum(r["stand"] for r in rs), per=rs)


def cem(names, bounds, evalf, restarts, iters, pop, n_elite, seeds=(), tag="", seed0=0):
    lo, hi = bounds[:, 0], bounds[:, 1]
    best_all = None
    for r in range(restarts):
        rng = np.random.default_rng(seed0 + 1000 * r + 7)
        mu = np.array([seeds[r][n] for n in names], float) if r < len(seeds) else rng.uniform(lo, hi)
        sig = 0.3 * (hi - lo) if r >= len(seeds) else 0.12 * (hi - lo)
        best = None
        for it in range(iters):
            X = np.clip(mu + sig * rng.standard_normal((pop, len(names))), lo, hi)
            if best is not None:
                X[0] = best[1]
            xs = [dict(zip(names, map(float, row))) for row in X]
            ev = evalf(xs)
            sc = np.array([e["score"] for e in ev])
            order = np.argsort(-sc)
            if best is None or sc[order[0]] > best[0]:
                best = (float(sc[order[0]]), X[order[0]].copy(), ev[order[0]])
            el = X[order[:n_elite]]
            mu = el.mean(0)
            sig = np.maximum(0.7 * sig + 0.3 * el.std(0), 0.01 * (hi - lo))
            print(f"   {tag} restart {r} iter {it:2d}: best {best[0]:+.3f}  {_brief(best[2])}", flush=True)
        xb = dict(zip(names, map(float, best[1])))
        print(f"-- {tag} restart {r} BEST {best[0]:+.3f}  {_brief(best[2])}\n   x {json.dumps({k: round(v, 2) for k, v in xb.items()})}", flush=True)
        if best_all is None or best[0] > best_all[0]:
            best_all = (best[0], xb, best[2])
    return best_all


def _brief(r):
    if "front" in r:
        return (f"front {r['front']:+.2f} (max {r['front_max']:+.2f}) up {r['up']:+.2f} arm err {r['arm_err']:.0f} deg "
                f"{'ON BACK' if r['roll_ok'] else ''}")
    if "per" in r:
        return f"stands {r['n_stand']}/{len(r['per'])}  " + " | ".join(
            f"up {q.get('up', -1):+.2f} z {q.get('z', 0):.2f} crouch up {q.get('up_crouch', -1):+.2f}{' feet' if q.get('feet_only') else ''}"
            for q in r["per"])
    return str(r)


# ------------------------------------------------------------------ seeds
def seed_12_1(family):
    """the section 12.1 legs-only roll (gate_no3250._prone_roll), mapped into
    the keyframe vector; arms at the idle pose (the searched stow starts there)"""
    ks = [dict(R_hip_yaw=45, R_hip_roll=-45, R_hip_pitch=-100, R_knee=-130, R_ankle=0),
          dict(R_hip_yaw=45, R_hip_roll=-45, R_hip_pitch=-20, R_knee=-20, L_hip_roll=-45, L_hip_pitch=-60, L_knee=-90),
          dict(),
          dict(R_hip_pitch=60, R_hip_roll=-30, L_hip_roll=30),
          dict()]
    tt = [(1.5, 0.8), (1.5, 1.2), (1.5, 1.0), (1.0, 1.5), (1.5, 1.5)]
    names, bounds = roll_space(family)
    x = {}
    for n, (lo, hi) in zip(names, bounds):
        if n.startswith("stow_"):
            j = n[5:]
            v = IDLE["shoulder"] if j.endswith("shoulder") else IDLE["elbow"]
        else:
            k = int(n[1]); j = n[3:]
            if j == "t":
                v = tt[k][0]
            elif j == "h":
                v = tt[k][1]
            elif j.endswith(("shoulder", "elbow")):
                v = IDLE["shoulder"] if j.endswith("shoulder") else IDLE["elbow"]
            else:
                v = ks[k].get(j, 0.0)
        x[n] = float(np.clip(v, lo, hi))
    return x


# ------------------------------------------------------------------ verification
CONDS = GN.GETUP_CONDS


def _verify_roll_job(args):
    family, x, c, ts = args
    return run_roll(family, x, cond=c, time_scale=ts, chain=True, protection=dict(enforce=True, load="torque"))


def _verify_catch_job(args):
    x, c, ts, kick, hd = args
    return run_catch(x, kick=kick, cond=c, time_scale=ts, heading_deg=hd, protection=dict(enforce=True, load="torque"))


def _prot_line(rep):
    worst = max(rep.items(), key=lambda kv: kv[1]["peak_torque"])
    hot = max(rep.items(), key=lambda kv: kv[1]["cont_max_duty"])
    trips = [j for j, v in rep.items() if v["trip_t"] is not None]
    return (f"peak load {worst[0]} {100 * worst[1]['peak_torque']:.0f} %, longest duty>80% {hot[0]} "
            f"{hot[1]['cont_max_duty']:.2f} s, trips {trips or 'none'}")


def _verify_entry_job(args):
    x, c, ts = args
    return run_entry(x, cond=c, time_scale=ts, protection=dict(enforce=True, load="torque"))


def verify(family, x):
    if family in ("entry", "catch"):
        chain = ""
    elif family.endswith("_fold"):
        chain = "; chain = roll (ends with the arms folded up) + the recommended seat push"
    else:
        chain = "; chain = roll (ends with the arms idle) + the seat push with entry " + json.dumps(ENTRY_BEST)
    print(f"== VERIFY {family}, six conditions, pace x1 and x3 (3x slower), STS overload cutoff enforced "
          f"(torque reading){chain}", flush=True)
    if family == "entry":
        jobs = [(x, c, ts) for ts in (1.0, 3.0) for c in CONDS]
        res = GN.pmap(_verify_entry_job, jobs)
        for (xx, c, ts), r in zip(jobs, res):
            print(f"   x{ts:.0f} {GN._cond_label(c)}: {'STANDING' if r['stand'] else 'no      '} up {r['up']:+.2f} "
                  f"z {r['z']:.3f}  peak {r['peak']:.2f} N*m ({r['peak_joint']})  {_prot_line(r['prot'])}", flush=True)
        for ts in (1.0, 3.0):
            n = sum(r["stand"] for (xx, c, t_), r in zip(jobs, res) if t_ == ts)
            print(f"-- entry, pace x{ts:.0f}: stands {n}/6", flush=True)
        return
    if family == "catch":
        jobs = [(x, c, ts, kick, hd) for ts in (1.0, 3.0) for c in CONDS for kick, hd in ((1.0, 0), (1.5, 0), (2.0, 0), (1.5, -20), (1.5, 20))]
        res = GN.pmap(_verify_catch_job, jobs)
        for (xx, c, ts, kick, hd), r in zip(jobs, res):
            print(f"   x{ts:.0f} {GN._cond_label(c)} kick {kick:.1f}x tip heading {hd:+.0f}: "
                  f"{'STANDING' if r['stand'] else 'no      '} up {r['up']:+.2f} z {r['z']:.3f}  crouch up {r['up_crouch']:+.2f} "
                  f"z {r['z_crouch']:.3f}{' feet only' if r['feet_only'] else ''}  trig t {r['t_trig']}  peak {r['peak']:.2f} N*m "
                  f"({r['peak_joint']})  {_prot_line(r['prot'])}", flush=True)
        for ts in (1.0, 3.0):
            n = sum(r["stand"] for (xx, c, t_, k, h), r in zip(jobs, res) if t_ == ts)
            print(f"-- catch, pace x{ts:.0f}: stands {n}/{len(CONDS) * 5}", flush=True)
        return
    jobs = [(family, x, c, ts) for ts in (1.0, 3.0) for c in CONDS]
    res = GN.pmap(_verify_roll_job, jobs)
    for (f, xx, c, ts), r in zip(jobs, res):
        print(f"   x{ts:.0f} {GN._cond_label(c)}: roll {'ON BACK' if r['roll_ok'] else 'no     '} front {r['front']:+.2f} "
              f"arm err {r['arm_err']:.0f} deg, peak {r['peak_roll']:.2f} N*m ({r['peak_roll_joint']});  chain "
              f"{'STANDING' if r['stand'] else 'no      '} up {r['up_end']:+.2f} z {r['z_end']:.3f} peak {r['peak_chain']:.2f} N*m "
              f"({r['peak_chain_joint']});  {_prot_line(r['prot'])}", flush=True)
    for ts in (1.0, 3.0):
        rr = [r for (f, xx, c, t_), r in zip(jobs, res) if t_ == ts]
        print(f"-- {family}, pace x{ts:.0f}: on the back {sum(r['roll_ok'] for r in rr)}/6, standing {sum(r['stand'] for r in rr)}/6",
              flush=True)


# ------------------------------------------------------------------ probe
def probe():
    """the section 12.1 roll with the arms held at a grid of poses (a probe to
    shape the search, not a verdict), plus the prone start's cross-section"""
    _set_hk()
    p, xml = plant()
    print(f"== PROBE: r5_asdrawn_rom120 + elbow stops, Plan B, nominal (play 3, mu 0.7); prone start with the arms at "
          f"(shoulder, elbow) = idle {IDLE}", flush=True)
    env = make_env(p, xml, NOMINAL)
    na = env._nq_act
    G.settle_fallen(env, p, "prone", q_of(dict(**IDLE), na))
    up, front, side = torso_axes(env)
    print(f"   settled prone: up {up:+.2f} front {front:+.2f} side {side:+.2f}, pelvis z {env.data.qpos[2]:.3f}, "
          f"contacts {G.contacts_summary(env.model, env.data)}", flush=True)
    base = seed_12_1("stow")
    jobs = []
    for sh, el in ((15, 0), (0, 0), (0, -100), (90, 0), (90, -100), (180, 0), (200, 0), (-90, 0), (-45, -100), (135, -100)):
        x = dict(base)
        x.update(stow_L_shoulder=sh, stow_R_shoulder=sh, stow_L_elbow=el, stow_R_elbow=el)
        jobs.append(("stow", x))
    res = GN.pmap(_eval_roll, jobs)
    for (f, x), r in zip(jobs, res):
        print(f"   12.1 roll, arms held at shoulder {x['stow_L_shoulder']:+4.0f} elbow {x['stow_L_elbow']:+4.0f}: "
              f"{_brief(r)}  peak {r['peak_roll']:.2f} N*m ({r['peak_roll_joint']})", flush=True)


def main():
    mode = sys.argv[1]
    if mode == "probe":
        probe()
    elif mode == "rollsearch":
        family = sys.argv[2]
        restarts = int(sys.argv[3]) if len(sys.argv) > 3 else 6
        iters = int(sys.argv[4]) if len(sys.argv) > 4 else 40
        pop = int(os.environ.get("POP", "48"))
        names, bounds = roll_space(family)
        print(f"== ROLL SEARCH, family {family}: prone (arms idle) -> {NK} free keyframes -> arms up + legs straight; "
              f"{len(names)} parameters, CEM {restarts} restarts x {iters} iterations x {pop}, restart 0 seeded from the "
              f"section 12.1 roll; score = front_end + 0.3 front_max + 1[on back] - 0.004 arm error (deg); "
              f"r5_asdrawn_rom120 + elbow stops, Plan B, nominal; self_collide on; workers {GN.n_workers()}", flush=True)
        evalf = lambda xs: GN.pmap(_eval_roll, [(family, x) for x in xs])
        best = cem(names, bounds, evalf, restarts, iters, pop, max(4, pop // 6), seeds=[seed_12_1(family)], tag=family)
        print(f"\n== {family}: OVERALL BEST {best[0]:+.3f}  {_brief(best[2])}\n   x {json.dumps({k: round(v, 2) for k, v in best[1].items()})}",
              flush=True)
        if best[2].get("roll_ok") and (ENTRY_BEST or family.endswith("_fold")):
            verify(family, best[1])
        elif best[2].get("roll_ok"):
            print("   (no PRONE_ENTRY given: the chain is verified separately with `verify`)", flush=True)
    elif mode == "catchsearch":
        restarts = int(sys.argv[2]) if len(sys.argv) > 2 else 6
        iters = int(sys.argv[3]) if len(sys.argv) > 3 else 40
        pop = int(os.environ.get("POP", "48"))
        print(f"== CATCH SEARCH: stand (arms idle) -> forward kick 1.0 / 1.5 / 2.0 x the tip speed "
              f"({tip_speed():.2f} m/s) -> trigger at a searched pitch -> catch -> push back -> the seat push's crouch "
              f"hold and rise; {len(CATCH_NAMES)} parameters, CEM {restarts} x {iters} x {pop}; score = worst-kick "
              f"standing / up / height (+ crouch terms) + 0.2 mean; workers {GN.n_workers()}", flush=True)
        evalf = lambda xs: GN.pmap(_eval_catch, [(x, (1.0, 1.5, 2.0)) for x in xs])
        best = cem(CATCH_NAMES, CATCH_BOUNDS, evalf, restarts, iters, pop, max(4, pop // 6), tag="catch")
        print(f"\n== catch: OVERALL BEST {best[0]:+.3f}  {_brief(best[2])}\n   x {json.dumps({k: round(v, 2) for k, v in best[1].items()})}",
              flush=True)
        if best[2]["n_stand"] == 3:
            verify("catch", best[1])
    elif mode == "entrysearch":
        restarts = int(sys.argv[2]) if len(sys.argv) > 2 else 6
        iters = int(sys.argv[3]) if len(sys.argv) > 3 else 30
        pop = int(os.environ.get("POP", "36"))
        conds3 = [(GN.GETUP_CONDS[0], 1.0), (GN.GETUP_CONDS[2], 1.0), (GN.GETUP_CONDS[5], 1.0), (GN.GETUP_CONDS[0], 3.0)]
        print(f"== SEAT-PUSH ENTRY SEARCH: supine with the arms at the idle pose (a real backward fall, or a roll "
              f"that ends on the back) -> searched arm path through lie / sit up / fold -> the recommended brace, "
              f"tuck, push and rise; {len(ENTRY_NAMES)} parameters, CEM {restarts} x {iters} x {pop}, restarts 0-3 "
              f"seeded (arms forward / propped on the forearms / the 9-parameter round's winner / the fold to 180, slowly); each candidate run at "
              f"{', '.join(GN._cond_label(c) + f' x{ts:.0f}' for c, ts in conds3)}; score = worst run + 0.2 mean, run = 3 x standing + "
              f"up + 4 x pelvis z; workers {GN.n_workers()}", flush=True)
        base = GN.pmap(_eval_entry, [(s, conds3) for s in ENTRY_SEEDS])
        for s, b in zip(ENTRY_SEEDS, base):
            print(f"   seed {json.dumps(s)}: {_brief(b)}", flush=True)
        evalf = lambda xs: GN.pmap(_eval_entry, [(x, conds3) for x in xs])
        best = cem(ENTRY_NAMES, ENTRY_BOUNDS, evalf, restarts, iters, pop, max(4, pop // 6), seeds=ENTRY_SEEDS, tag="entry")
        print(f"\n== entry: OVERALL BEST {best[0]:+.3f}  {_brief(best[2])}\n   x {json.dumps({k: round(v, 2) for k, v in best[1].items()})}",
              flush=True)
        if best[2]["n_stand"] == len(conds3):
            verify("entry", best[1])
    elif mode == "verify":
        if len(sys.argv) > 4:
            ENTRY_BEST.update(json.loads(sys.argv[4])); os.environ["PRONE_ENTRY"] = sys.argv[4]
        verify(sys.argv[2], json.loads(sys.argv[3]))
    elif mode == "render":
        family, x, out = sys.argv[2], json.loads(sys.argv[3]), sys.argv[4]
        if len(sys.argv) > 5:
            ENTRY_BEST.update(json.loads(sys.argv[5]))
        if family == "catch":
            r = run_catch(x, kick=1.5, render=out)
        elif family == "entry":
            r = run_entry(x, render=out)
        else:
            r = run_roll(family, x, chain=bool(ENTRY_BEST), render=out)
        print(r)


if __name__ == "__main__":
    main()
