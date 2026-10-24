"""Analytic get-up references for phase-indexed tracking (getup campaign, 2026-10-21).

The RL get-up tracks a recorded open-loop rollout (train_mjx --rise-ref, the
DeepMimic-style objective of docs/training.md section 11). A reference is only
as good as the open-loop sequence behind it: tracking an infeasible one cannot
work. Two findings set this script's scope (2026-10-21, on the current plant):

- the recorded prone reference (prone_roll + seat push) stands at joint
  frictionloss 0 and 0.05 but FAILS at 0.35 -- the roll never reaches the
  side -- so robot_getup_s1k/s1kb trained at 0.35 against an impossible
  target; the branch roll of getup-prone-2026-09-29 (stow_fold, 12/12 on the
  09-29 plant) scores 0 % on today's plant;
- the supine seat push stands at every friction tried, but its rise
  keyframes are hand-tuned and torque-marginal (4/8 alone under the claim).

So it builds references from kinematics and scores them by their WORST case:

  rise        the crouch -> stand rise, solved analytically: a dense grid of
              sagittal leg poses (hip, knee, ankle) with both soles levelled
              on the floor; keep the poses whose whole-body CoM sits over the
              middle of the soles; in each pelvis-height band pick the one
              with the lowest worst-joint gravity torque / stall, and chain
              the bands by dynamic programming (torque ratio + joint travel).
  search-roll prone -> back with the arms stowed (the 09-29 method: CEM over
              five free keyframes, scored on the worst robustness condition),
              re-searched on the current plant with joint friction 0.05..0.5
              among the conditions.
  verify      a sequence over the robustness grid (joint friction, sole mu,
              play, servo strength, mass, start tilt).
  record      a sequence's open-loop rollout as a --rise-ref npz (the format
              of record_getup_ref.py).

    .venv/bin/python sim/getup_ref_analytic.py rise
    .venv/bin/python sim/getup_ref_analytic.py search-roll --restarts 4 --workers 12
    .venv/bin/python sim/getup_ref_analytic.py verify --seq supine_ik
    .venv/bin/python sim/getup_ref_analytic.py record --seq prone_ik --out sim/getup_ref_v6ar_prone_ik.npz
    .venv/bin/python sim/getup_ref_analytic.py record --seq prone_ikc --out sim/getup_ref_v6ar_prone_ikc.npz

prone_ikc / supine_ikc are prone_ik / supine_ik with the centred crouch hold
(seat_push_ik centred=True). Grade a recorded reference through the CPU
referee with no RL: eval_precision.py --ref-replay <npz>.
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("MUJOCO_GL", "cgl" if sys.platform == "darwin" else "egl")
import mujoco  # noqa: E402
import getup_v6 as G  # noqa: E402
import getup_v6_shoulder as S  # noqa: E402

XML = os.path.join(HERE, "bimo_biped_v6ar.xml")
ROLL_JSON = os.path.join(HERE, "getup_ref_analytic_roll.json")   # search-roll's winner
H_STOP, K_STOP = -120.0, -130.0         # the hip's real stop; the knee's (dimensions_v6.ROM)
REC_FRIC = 0.35                         # references record at the training plant's --joint-frictionloss
IDLE = dict(shoulder=15, elbow=0)       # the walk's arm pose: where a real fall leaves the arms
FOLD = dict(shoulder=180, elbow=0)      # the seat push's start
REST = dict(shoulder=60, elbow=0)       # the seat push's arms from the crouch on
NOMINAL = dict(fric=REC_FRIC, mu=0.7, play=3.0, stall_k=1.0, mass_k=1.0, tilt=0.0)
# verify: every combination (3*3*2*2*2*3 = 216 rollouts)
GRID = dict(fric=[0.05, 0.35, 0.5], mu=[0.5, 0.7, 1.0], play=[1.0, 4.0], stall_k=[0.85, 1.0],
            mass_k=[1.0, 1.1], tilt=[-10.0, 0.0, 10.0])
# search-roll scores a candidate on the worst of these
SEARCH_CONDS = [dict(NOMINAL, fric=0.05), dict(NOMINAL), dict(NOMINAL, fric=0.5, mu=0.5),
                dict(NOMINAL, mu=1.0, play=1.0), dict(NOMINAL, stall_k=0.85, mass_k=1.1),
                dict(NOMINAL, tilt=10.0), dict(NOMINAL, tilt=-10.0)]

_PLANT = []


def _plant():
    if not _PLANT:
        S.H, S.K = H_STOP, K_STOP
        _PLANT.append(S.plant("r6_asdrawn_rom120")[0])
    return _PLANT[0]


# ------------------------------------------------------------------ rollouts
def make_env(cond):
    """the record_getup_ref.py env (deploy servo model, STS3250 rolls+knees),
    under one robustness condition"""
    p = _plant()
    env = G.make_env(p, XML, mu=cond["mu"], play_deg=cond["play"])
    env.reset(seed=0)
    G.EXTRA[:] = [env.model.actuator(i).name for i in range(12, env.model.nu)]
    kp, kd, stall, w0 = env._servo
    na = env._nq_act
    stall = np.full(na, stall); w0 = np.full(na, w0)
    kpa = np.full(na, float(kp)); kda = np.full(na, float(kd))
    for i, n in enumerate(G.JN):
        s = G.SERVOS[G.PJ_DEFAULT[n]] if n in G.PJ_DEFAULT else None
        if s:
            stall[i] = s["stall"]; w0[i] = s["w0"]; kpa[i] *= s["kp_scale"]; kda[i] *= s["kp_scale"]
    env._servo = np.array([kpa, kda, stall * cond["stall_k"], w0], dtype=object)
    env.fall_up_z = -2.0; env.fall_height = -1.0
    if cond["mass_k"] != 1.0:
        env.model.body_mass[:] *= cond["mass_k"]
    env.model.dof_frictionloss[env._sqvel] = cond["fric"]
    return env, p


def _inv(env):
    na, d0, hi, lo = env._nq_act, env._default, env._hi, env._lo

    def inv(q):
        q = q[:na]
        return np.clip(np.where(q >= d0, (q - d0) / np.maximum(hi - d0, 1e-6),
                                (q - d0) / np.maximum(d0 - lo, 1e-6)), -1, 1)
    return inv


def torso_axes(env):
    """(up, front): world-z components of the torso's z and x axes; lying on
    the back is front > 0.7, standing is up > 0.9"""
    R = env.data.xmat[env._torso_bid].reshape(3, 3)
    return float(R[2, 2]), float(R[2, 0])


def run(seq, start, cond=NOMINAL, log=None):
    """seq: [(label, offsets deg, move_s, hold_s)], min-jerk between keys, from
    a settled start ('supine' / 'prone'). log(env, q_target, label) is called
    every control step (record uses it). Returns the end state."""
    env, p = make_env(cond)
    d, dt, inv = env.data, env.control_dt, _inv(env)
    q_prev = G.q_from_offsets(seq[0][1])
    G.settle_fallen(env, p, start, q_prev)
    if cond["tilt"]:       # roll the settled body about its long axis and re-settle
        a = math.radians(cond["tilt"])
        out = np.zeros(4)
        mujoco.mju_mulQuat(out, np.array([math.cos(a / 2), math.sin(a / 2), 0, 0]), d.qpos[3:7].copy())
        d.qpos[3:7] = out; d.qpos[2] += 0.01
        for _ in range(int(0.8 / dt)):
            env.step(inv(q_prev))
    trace = []
    for label, off, move_s, hold_s in seq:
        q_tgt = G.q_from_offsets(off)
        n = int(move_s / dt)
        for i in range(n + int(hold_s / dt)):
            s = min(1.0, (i + 1) / max(n, 1))
            s = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
            q = q_prev + (q_tgt - q_prev) * s
            if log is not None:
                log(env, q, label)
            env.step(inv(q))
        trace.append((label, round(float(d.qpos[2]), 3), *(round(v, 2) for v in torso_axes(env))))
        q_prev = q_tgt
    up, front = torso_axes(env)
    z = float(d.qpos[2])
    na = env._nq_act
    names = [env.model.actuator(i).name for i in range(na)]
    qa = d.qpos[7:7 + na]
    arm_err = max(abs(math.degrees(qa[names.index(j)]) - FOLD["shoulder"]) for j in ("L_shoulder", "R_shoulder"))
    return dict(stood=bool(up > 0.9 and z > 0.8 * p.z_yaw_above_sole), z=z, up=up, front=front,
                arm_err=arm_err, trace=trace)


def _one(a):
    seq, start, cond = a
    try:
        return run(seq, start, cond)
    except Exception as e:          # an unstable rollout scores as a failure, never kills a sweep
        return dict(stood=False, z=0.0, up=-1.0, front=-1.0, arm_err=180.0, trace=[("error", repr(e))])


def grid_conds():
    keys = list(GRID)
    return [dict(zip(keys, v)) for v in itertools.product(*GRID.values())]


# ------------------------------------------------------------------ the analytic rise
def _levelled_pose(m, d, hip, knee, ankle, arms=REST):
    """symmetric sagittal pose (deg) with the soles rotated level about y:
    returns the pelvis height over the sole centre, the CoM's x offset from it,
    the per-leg gravity torque / stall at hip pitch, knee and ankle, and the
    lowest body-centre height (floor clearance)"""
    d.qpos[:] = 0.0
    d.qpos[3] = 1.0
    for s in "LR":
        d.qpos[m.joint(f"{s}_hip_pitch").qposadr[0]] = math.radians(hip)
        d.qpos[m.joint(f"{s}_knee").qposadr[0]] = math.radians(knee)
        d.qpos[m.joint(f"{s}_ankle").qposadr[0]] = math.radians(ankle)
        d.qpos[m.joint(f"{s}_shoulder").qposadr[0]] = math.radians(arms["shoulder"])
        d.qpos[m.joint(f"{s}_elbow").qposadr[0]] = math.radians(arms["elbow"])
    mujoco.mj_kinematics(m, d)
    mujoco.mj_comPos(m, d)
    gl, gr = m.geom("L_sole").id, m.geom("R_sole").id
    sx = d.geom_xmat[gl].reshape(3, 3)[:, 0]
    th = math.atan2(sx[2], sx[0])                       # rotate by th about y: sole x axis level
    c, s_ = math.cos(th), math.sin(th)
    c0 = 0.5 * (d.geom_xpos[gl] + d.geom_xpos[gr])

    def lev(p):                                          # (x, z) in the levelled frame, sole centre at 0
        v = p - c0
        return v[0] * c + v[2] * s_, -v[0] * s_ + v[2] * c
    sz = d.geom_xmat[gl].reshape(3, 3)[:, 2]
    if -sz[0] * s_ + sz[2] * c < 0.95:                   # sole facing down after levelling: not a stance
        return None
    xs, zs = zip(*(lev(d.xipos[b]) for b in range(1, m.nbody)))
    xs, zs, ms = np.array(xs), np.array(zs), m.body_mass[1:]
    com_x = float((ms * xs).sum() / ms.sum())
    pelvis_z = lev(d.xpos[1])[1]
    g = 9.81
    out = {}
    for jn, stall in (("hip_pitch", G.SERVOS["sts3215"]["stall"]), ("knee", G.SERVOS["sts3250"]["stall"]),
                      ("ankle", G.SERVOS["sts3215"]["stall"])):
        jx = lev(d.xanchor[m.joint(f"L_{jn}").id])[0]
        # foot-rooted, symmetric: each leg carries half of the moment of every
        # body proximal to the joint (the distal links of BOTH legs excluded)
        distal = set()
        for side in "LR":
            b0 = m.jnt_bodyid[m.joint(f"{side}_{jn}").id]
            distal |= {b for b in range(1, m.nbody) if _is_desc(m, b, b0)}
        keep = np.array([b not in distal for b in range(1, m.nbody)])
        out[jn] = abs(0.5 * g * float((ms[keep] * (xs[keep] - jx)).sum())) / stall
    floor = min(z for b, z in zip(range(1, m.nbody), zs)
                if m.body(b).name.split("_")[-1] not in ("foot", "blk"))
    return dict(h=pelvis_z, dx=com_x, r=max(out.values()), tau=out, floor=floor)


def _is_desc(m, b, root):
    while b > 0:
        if b == root:
            return True
        b = m.body_parentid[b]
    return False


def rise_path(n_bands=8, step=3.0, margin=0.012, lam=0.02, verbose=False):
    """the crouch -> stand rise: grid the sagittal poses, keep the balanced
    ones (|CoM x - sole centre| < margin, every body centre clear of the floor),
    band them by pelvis height and DP through the bands from the seat push's
    crouch hold to standing. Returns [(hip, knee, ankle)] in degrees."""
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    crouch = (H_STOP, K_STOP, -25.0)
    p0, p1 = _levelled_pose(m, d, *crouch), _levelled_pose(m, d, 0.0, 0.0, 0.0)
    cands = []
    for hip in np.arange(-120.0, 0.1, step):
        for knee in np.arange(-130.0, 0.1, step):
            for ankle in np.arange(-40.0, 40.1, step):
                r = _levelled_pose(m, d, hip, knee, ankle)
                if r and abs(r["dx"]) < margin and r["floor"] > 0.01:
                    cands.append(((hip, knee, ankle), r))
    edges = np.linspace(p0["h"], p1["h"], n_bands + 2)[1:-1]
    bands = [[c for c in cands if abs(c[1]["h"] - e) < 0.5 * (edges[1] - edges[0])] for e in edges]
    bands = [b for b in bands if b]
    # DP: cost = torque ratio + lam * joint travel (deg) from the previous waypoint
    prev = [(crouch, 0.0, [])]
    for band in bands:
        nxt = []
        for q, r in band:
            best = min(prev, key=lambda t: t[1] + lam * np.abs(np.subtract(q, t[0])).max())
            nxt.append((q, best[1] + r["r"] + lam * np.abs(np.subtract(q, best[0])).max(), best[2] + [(q, r)]))
        prev = nxt
    path = min(prev, key=lambda t: t[1] + lam * np.abs(np.subtract((0, 0, 0), t[0])).max())[2]
    if verbose:
        print(f"crouch h {p0['h']:.3f} dx {p0['dx']*1e3:+.0f} mm r {p0['r']:.2f}; stand h {p1['h']:.3f}; "
              f"{len(cands)} balanced poses")
        for q, r in path:
            print(f"  hip {q[0]:6.1f} knee {q[1]:6.1f} ankle {q[2]:5.1f}  h {r['h']:.3f}  dx {r['dx']*1e3:+4.0f} mm  "
                  + "  ".join(f"{k} {v:.2f}" for k, v in r["tau"].items()))
    return [q for q, _ in path]


def rise_keys(path, deg_per_s=20.0, hold_s=0.4):
    """waypoints -> (label, offsets, move_s, hold_s): quasi-static pace. 40 deg/s
    tipped it over backwards on reaching straight at joint frictionloss 0.05
    (0/72 of the grid), and 25 deg/s still did at 0.05 with mu 1.0 and +10 % mass
    (arriving at straight pitched back); 20 deg/s with 0.4 s holds stands there"""
    keys, last = [], (H_STOP, K_STOP, -25.0)
    for i, q in enumerate(list(path) + [(0.0, 0.0, 0.0)]):
        move = max(0.6, np.abs(np.subtract(q, last)).max() / deg_per_s)
        keys.append((f"ik rise {i}", dict(hip_pitch=q[0], knee=q[1], ankle=q[2], **REST), move, hold_s))
        last = q
    return keys


# ------------------------------------------------------------------ sequences
def seat_push():
    """the recommended seat push (record_getup_ref.py's arguments)"""
    _plant()
    return S.seat_push(False, 60, 0, 2.0, -25.0, elbow=(-60, 0))


CROUCH_C = dict(ankle=-35.0, shoulder=0.0, elbow=0.0)   # the centred crouch hold (seat_push_ik centred=True)


def seat_push_ik(path=None, centred=False):
    """the seat push up to its crouch hold, then the analytic rise.

    centred: the crouch hold keeps the hands on the floor (the push pose,
    shoulder 0) and dorsiflexes the ankle to -35 deg, so the CoM sits 12 mm
    behind the sole centre instead of 48 mm (arms at REST, ankle -25); the
    arms go to REST during the first rise waypoint, once the pelvis is off the
    floor. Lifting the hands at the crouch hold is where the replayed reference
    and every hold-trained policy tipped back onto the seat under the
    referee's DR (2026-10-24: seeds 7/607/707). Same keys, same timing."""
    sp = seat_push()
    i = [k[0] for k in sp].index("crouch hold")
    hold = sp[i]
    if centred:
        hold = (hold[0], dict(hold[1], **CROUCH_C), hold[2], hold[3])
    return sp[:i] + [hold] + rise_keys(path if path is not None else rise_path()) + [("straight", dict(**REST), 1.0, 0.8)]


NK = 5
KJ = [f"{s}_{j}" for s in "LR" for j in ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle")]
ARMS = ("L_shoulder", "L_elbow", "R_shoulder", "R_elbow")
RANGE = {}
for _s in "LR":
    RANGE.update({f"{_s}_hip_yaw": (-45, 45), f"{_s}_hip_roll": (-30, 45) if _s == "L" else (-45, 30),
                  f"{_s}_hip_pitch": (-120, 90), f"{_s}_knee": (-130, 0), f"{_s}_ankle": (-40, 40),
                  f"{_s}_shoulder": (-90, 200), f"{_s}_elbow": (-100, 10)})
# the 09-29 search's recommended stow_fold roll (branch sims-overload-prone-current,
# getup_v6_prone.py ROLL_STOW_FOLD): the CEM seed
ROLL_SEED = {"stow_L_shoulder": 194.57, "stow_L_elbow": -22.98, "stow_R_shoulder": 140.4,
             "stow_R_elbow": -68.65, "k0_L_hip_yaw": -32.42, "k0_L_hip_roll": -12.28, "k0_L_hip_pitch": -10.43,
             "k0_L_knee": -31.41, "k0_L_ankle": 31.13, "k0_R_hip_yaw": -34.05, "k0_R_hip_roll": -36.22,
             "k0_R_hip_pitch": 17.78, "k0_R_knee": -34.04, "k0_R_ankle": 6.44, "k0_t": 1.09, "k0_h": 0.65,
             "k1_L_hip_yaw": 39.87, "k1_L_hip_roll": 15.23, "k1_L_hip_pitch": 76.29, "k1_L_knee": -31.8,
             "k1_L_ankle": -33.6, "k1_R_hip_yaw": -28.6, "k1_R_hip_roll": 22.35, "k1_R_hip_pitch": -30.56,
             "k1_R_knee": -46.48, "k1_R_ankle": -31.38, "k1_t": 0.88, "k1_h": 0.75, "k2_L_hip_yaw": -0.08,
             "k2_L_hip_roll": 14.11, "k2_L_hip_pitch": 38.42, "k2_L_knee": -97.48, "k2_L_ankle": 32.88,
             "k2_R_hip_yaw": 38.96, "k2_R_hip_roll": 3.54, "k2_R_hip_pitch": -91.32, "k2_R_knee": -112.17,
             "k2_R_ankle": -15.34, "k2_t": 0.96, "k2_h": 0.72, "k3_L_hip_yaw": -6.9, "k3_L_hip_roll": -16.9,
             "k3_L_hip_pitch": -28.45, "k3_L_knee": -119.22, "k3_L_ankle": -35.15, "k3_R_hip_yaw": 42.29,
             "k3_R_hip_roll": -27.88, "k3_R_hip_pitch": -70.59, "k3_R_knee": -48.42, "k3_R_ankle": -29.5,
             "k3_t": 0.47, "k3_h": 0.41, "k4_L_hip_yaw": 2.1, "k4_L_hip_roll": 33.31, "k4_L_hip_pitch": 84.78,
             "k4_L_knee": -51.29, "k4_L_ankle": 32.12, "k4_R_hip_yaw": 39.12, "k4_R_hip_roll": 25.49,
             "k4_R_hip_pitch": -92.81, "k4_R_knee": -16.53, "k4_R_ankle": -24.32, "k4_t": 2.0, "k4_h": 0.96}


def roll_space():
    names = [f"stow_{j}" for j in ARMS]
    bounds = [RANGE[j] for j in ARMS]
    for k in range(NK):
        names += [f"k{k}_{j}" for j in KJ] + [f"k{k}_t", f"k{k}_h"]
        bounds += [RANGE[j] for j in KJ] + [(0.4, 2.0), (0.2, 1.5)]
    return names, np.array(bounds, float)


def roll_keys(x):
    """prone (arms idle, as a forward fall leaves them) -> arms stowed, five
    leg keyframes roll the body onto its back -> legs straight, arms folded to
    180 (the seat push's start)"""
    keys = [("lie prone", dict(**IDLE), 0.5, 0.8)]
    for k in range(NK):
        off = {j: x[f"k{k}_{j}"] for j in KJ}
        off.update({j: x[f"stow_{j}"] for j in ARMS})
        keys.append((f"roll k{k}", off, x[f"k{k}_t"], x[f"k{k}_h"]))
    return keys + [("arms folded up, legs straight", dict(**FOLD), 1.5, 1.5)]


def roll_score(r):
    """on the back (front -> +1) with both arms folded to 180"""
    return r["front"] + (1.0 if (r["front"] > 0.7 and r["arm_err"] < 15.0) else 0.0) - 0.01 * r["arm_err"]


def _eval_roll(x):
    return min(roll_score(_one((roll_keys(x), "prone", c))) for c in SEARCH_CONDS)


def search_roll(restarts=4, iters=25, pop=64, n_elite=10, workers=12, seed=0):
    names, bounds = roll_space()
    lo, hi = bounds[:, 0], bounds[:, 1]
    best = (-np.inf, None)
    with Pool(workers) as pool:
        for r in range(restarts):
            rng = np.random.default_rng(seed + r)
            seeded = r % 2 == 0          # even restarts start at the 09-29 winner, odd ones from scratch
            mu = np.array([ROLL_SEED[n] for n in names]) if seeded else rng.uniform(lo, hi)
            sd = (hi - lo) * (0.15 if seeded else 0.35)
            for it in range(iters):
                X = np.clip(mu + sd * rng.standard_normal((pop, len(names))), lo, hi)
                X[0] = mu
                sc = np.array(pool.map(_eval_roll, [dict(zip(names, x)) for x in X]))
                el = X[np.argsort(sc)[-n_elite:]]
                mu, sd = el.mean(0), 0.7 * sd + 0.3 * el.std(0)
                if sc.max() > best[0]:
                    best = (float(sc.max()), dict(zip(names, (float(v) for v in X[sc.argmax()]))))
                print(f"restart {r} iter {it:2d}: best {sc.max():+.3f} mean {sc.mean():+.3f} | overall {best[0]:+.3f}",
                      flush=True)
            json.dump(dict(score=best[0], x=best[1]), open(ROLL_JSON, "w"), indent=1)
    return best


def sequences():
    path = rise_path()
    ikc = seat_push_ik(path, centred=True)
    seqs = {"supine": ("supine", seat_push()), "supine_ik": ("supine", seat_push_ik(path)),
            "supine_ikc": ("supine", ikc)}
    if os.path.exists(ROLL_JSON):
        roll = roll_keys(json.load(open(ROLL_JSON))["x"])
        seqs["prone"] = ("prone", roll + seat_push()[1:])
        seqs["prone_ik"] = ("prone", roll + seat_push_ik(path)[1:])
        seqs["prone_ikc"] = ("prone", roll + ikc[1:])
    return seqs


def verify(name, workers=12):
    start, seq = sequences()[name]
    conds = grid_conds()
    with Pool(workers) as pool:
        res = pool.map(_one, [(seq, start, c) for c in conds])
    ok = [r["stood"] for r in res]
    print(f"{name}: stands {sum(ok)}/{len(ok)} ({sum(ok)/len(ok):.0%}) of the robustness grid")
    for k, vals in GRID.items():
        print(f"  {k:8s} " + "  ".join(f"{v}: {sum(o for o, c in zip(ok, conds) if c[k] == v)}/"
                                        f"{sum(1 for c in conds if c[k] == v)}" for v in vals))
    print("  nominal trace:", run(seq, start)["trace"])
    return sum(ok) / len(ok)


def record(name, out):
    """the open-loop rollout under NOMINAL, in record_getup_ref.py's format"""
    start, seq = sequences()[name]
    T, Q, QP, QV, LAB = [], [], [], [], []
    st = {}

    def log(env, q, label):
        st["env"] = env
        T.append(len(T) * env.control_dt); Q.append(q[:env._nq_act].copy())
        QP.append(env.data.qpos.copy()); QV.append(env.data.qvel.copy()); LAB.append(label)
    r = run(seq, start, NOMINAL, log=log)
    env = st["env"]
    names = [env.model.actuator(i).name for i in range(env._nq_act)]
    np.savez(out, t=np.array(T), q=np.array(Q), qpos=np.array(QP), qvel=np.array(QV), label=np.array(LAB),
             joint_names=np.array(names), control_dt=env.control_dt, final_up=r["up"], final_z=r["z"],
             source=f"getup_ref_analytic.py {name} on bimo_biped_v6ar.xml, joint frictionloss {REC_FRIC}, "
                    f"mu 0.7, play 3 deg, PJ_DEFAULT servos")
    print(f"{out}: {len(T)} steps ({T[-1]:.1f} s), final up {r['up']:+.2f} pelvis {r['z']:.3f} "
          f"{'STANDS' if r['stood'] else 'DOES NOT STAND'}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=("rise", "search-roll", "verify", "record"))
    ap.add_argument("--seq", default="supine_ik")
    ap.add_argument("--out", default=None)
    ap.add_argument("--workers", type=int, default=12)   # mira by day: 12 cores (24 at night)
    ap.add_argument("--restarts", type=int, default=4)
    ap.add_argument("--iters", type=int, default=25)
    a = ap.parse_args()
    if a.cmd == "rise":
        rise_path(verbose=True)
    elif a.cmd == "search-roll":
        print(search_roll(restarts=a.restarts, iters=a.iters, workers=a.workers))
    elif a.cmd == "verify":
        verify(a.seq, a.workers)
    else:
        record(a.seq, a.out)
