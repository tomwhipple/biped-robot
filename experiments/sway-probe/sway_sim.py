"""Sim rehearsal of the sway-probe ladder: every rung played OPEN LOOP (no balance loop) on the
measured-sole plant under the actuator chain of a prec training recipe, over a band of the
training DR ranges, with what the robot's firmware guards would have seen.

Plant: sim/bimo_biped_v6legs_meas_sole.xml in `walker_env.BimoWalkerEnv`, built from the recipe's
config.json the way the referee builds it (`eval_precision.make_env`): the per-servo stiffness map,
the sts2026 per-type lag and dead time, the firmware shaper (`fw_shaper_hz`), integer goal ticks.
The DR terms the open-loop question depends on are drawn over the recipe's own training ranges
(`Band` below: floor friction, joint friction, roll play, gear backlash, torso mass, torso CoM,
action-chain lag, dead time, bus latency, servo stiffness) with a Latin hypercube, plus the range
midpoint; each rung is one episode per sample.

What the robot's `pose` path can play. The firmware streamer is a minimum-jerk move of 200..20000 ms
from the servos' READ-BACK positions to one target per command (cli.cpp cmdPose, startSmooth): it has no 50 Hz
keyframe playback. `--seg-ms S` plays the keyframes the way the sender does, one `pose ... s<S>` per S ms to the
keyframe S ms ahead, each starting from where the joints are; `--seg-ms 0` feeds the 50 Hz keyframes
straight to the chain (what a policy's actions do). `--chain pose` drops the policy-loop shaper
(`fw_shaper_hz`), which the bench `pose` path does not have; `--chain prec` keeps the recipe's chain.

Per rung and per side (the "swing" foot is the one the reference lifts: L while sin(phase) > 0):
  F_min      lowest normal force under the swing foot, N and as a fraction of the robot's weight
  unloaded   F_min < 5 % of the weight
  clear      highest the swing foot's lowest pad rises above its standing height, mm (>= 5 mm = clears)
  roll/rate  peak torso roll (deg) and roll rate (deg/s) in the plateau
  guards     peak tilt against the bench fall guard (30 deg / 100 ms), the armed flail guard's tilt (20 deg / 200 ms)
             and its |gyro| EMA (tau 0.2 s, 1.5 rad/s), as margins; `up` = never tilted past 30 deg and back
             within 10 deg of upright at the end

    .venv/bin/python experiments/sway-probe/sway_sim.py --n 16 --json out.json --report out.md

`--refit` runs the ladder on the refit plant instead: gate_common's mass and symmetry terms (REFIT_TERMS), the torso
mass fixed at the weighed value, and a series roll spring (walker_env `roll_series_k`) drawn per plant over hip 10:25 and
ankle 16:40 N m/rad, two more Latin-hypercube dimensions beside the DR ranges. The default is the old plant.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import asdict, dataclass

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sway_ref as SR  # noqa: E402  (sets sys.path for sim/, sim/mjx/)
sys.path.insert(0, os.path.join(SR.ROOT, "experiments", "one-leg-stand"))
sys.path.insert(0, os.path.join(SR.ROOT, "experiments", "policy", "weight-transfer-gate"))
import mujoco  # noqa: E402

DT = 1.0 / SR.KEY_HZ
G = 9.81
UNLOAD_FRAC = 0.05             # a foot carrying < 5 % of the weight (100 ms moving average) is unloaded
UNLOAD_TICKS = 5               # 100 ms
CLEAR_MM = 5.0                 # a swing foot 5 mm up is clear (the 10-25 bench's own threshold: static_gait run_walk)
# firmware guards (firmware/components/{fallguard,flailguard}/include/...)
FALL_BENCH_DEG = 30.0          # fallguard::kBenchLimits: 30 deg for 100 ms -> limp
FLAIL_TILT_DEG = 20.0          # flailguard::kLimits.tilt: 20 deg for 200 ms (armed policy loop)
FLAIL_RATE = 1.5               # flailguard::kLimits.rate_rad_s: EMA |gyro|, tau 0.2 s
FLAIL_TAU = 0.2
SETTLE_S = 1.0
HOLD_END_S = 1.0


# --------------------------------------------------------------------------------- the band
@dataclass
class Plant:
    mu: float = 0.5                # effective floor friction (config contact_mu_range 0.3..0.7)
    jfric: float = 0.25            # joint Coulomb friction N m (joint_frictionloss_range 0.1..0.4)
    roll_lash_deg: float = 1.25    # extra roll-joint play, deg p-p (roll_lash_deg 0.5..2.0)
    backlash_deg: float = 0.125    # gear backlash (backlash_deg_max 0.25)
    torso_scale: float = 1.0       # torso mass and inertia (torso_mass_range 0.4)
    torso_com_y: float = 0.0       # m (torso_com_y 0.015)
    lag_hz: float = 5.325          # action-chain pole (act_lag_hz 2.65 .. act_lag_hz_max 8.0)
    delay_3215: int = 3            # control ticks of dead time, STS3215 (act_delay_types 2..4)
    delay_3250: int = 0            # hip rolls, STS3250 (0..1)
    latency_ms: float = 4.0        # latency_ms_max 8
    kp_scale: float = 0.9          # servo stiffness (servo_kp_range 0.6..1.2); kd follows
    ser_hip: float = 0.0           # series roll spring beyond the encoder, N m/rad (0 = rigid: the old plant)
    ser_ankle: float = 0.0


# The refit plant (weight-transfer-gate Round 3, #259; the robot measured 2026-11-10): the mass and
# symmetry terms gate_common applies (torso 0.4855 kg, the 83 g pack, no L ankle-roll zero error) and a
# spring in series with each roll servo, walker_env's `roll_series_k`. The fit (hip 11.0, ankle 17.75
# N m/rad) is too soft with both feet down; the stiffer end of the bracket is ~2.3x it.
REFIT_TERMS = "torso_kg=0.4855,pack=0.083,zoff_La=0"        # gate_common.MASS_SYM: applied by gate_common.apply_terms
REFIT_SERIES = dict(ser_hip=(10.0, 25.0), ser_ankle=(16.0, 40.0))     # N m/rad, drawn as two more LHS dimensions


class Band:
    """The draw ranges: the recipe's own DR ranges (config.json). refit=True: the refit plant (above): the
    torso mass is the weighed one (no draw) and the two series springs are drawn over REFIT_SERIES."""

    def __init__(self, cfg, refit=False):
        self.refit = refit
        g = cfg.get
        self.r = dict(
            mu=tuple(g("contact_mu_range", [0.3, 0.7])),
            jfric=tuple(g("joint_frictionloss_range", [0.1, 0.4])),
            roll_lash_deg=(g("roll_lash_deg", 0.5), g("roll_lash_deg_max", 2.0)),
            backlash_deg=(0.0, g("backlash_deg_max", 0.25)),
            torso_scale=(1 - g("torso_mass_range", 0.4), 1 + g("torso_mass_range", 0.4)),
            torso_com_y=(-g("torso_com_y", 0.015), g("torso_com_y", 0.015)),
            lag_hz=(g("act_lag_hz", 2.65), g("act_lag_hz_max", 8.0)),
            delay_3215=tuple(g("act_delay_types", {}).get("sts3215", [2, 4])),
            delay_3250=tuple(g("act_delay_types", {}).get("sts3250", [0, 1])),
            latency_ms=(0.0, g("latency_ms_max", 8.0)),
            kp_scale=tuple(g("servo_kp_range", [0.6, 1.2])),
        )
        if refit:
            self.r["torso_scale"] = (1.0, 1.0)
            self.r.update(REFIT_SERIES)

    def mid(self):
        d = {k: 0.5 * (a + b) for k, (a, b) in self.r.items()}
        d["delay_3215"] = int(round(d["delay_3215"]))
        d["delay_3250"] = int(round(d["delay_3250"]))
        return Plant(**d)

    def lhs(self, n, seed=0):
        """n plants, a Latin hypercube over every range (each range cut into n slices, one draw per slice)."""
        rng = np.random.default_rng(seed)
        keys = list(self.r)
        u = np.stack([(rng.permutation(n) + rng.uniform(size=n)) / n for _ in keys], axis=1)
        out = []
        for row in u:
            d = {}
            for k, x in zip(keys, row):
                a, b = self.r[k]
                d[k] = a + x * (b - a)
            d["delay_3215"] = int(round(d["delay_3215"]))
            d["delay_3250"] = int(round(d["delay_3250"]))
            out.append(Plant(**d))
        return out


# --------------------------------------------------------------------------------- the env
def build_env(cfg, plant, chain="prec", xml=None, seconds=40.0, refit=False):
    import eval_precision as EP
    import walker_env as W
    extra = dict(episode_seconds=seconds, act_lag_hz=plant.lag_hz, act_lag_dr_max=None,
                 latency_ms=plant.latency_ms, latency_ms_max=None, latency_jitter_ms=0.0,
                 backlash_deg=plant.backlash_deg, backlash_deg_max=None,
                 roll_lash_deg=plant.roll_lash_deg, roll_lash_deg_max=None,
                 mimic_sway=0.0, push_prob=0.0, push_force=0.0)
    if refit:       # walker_env's own series spring (fixed k per plant: the band draws it here)
        extra["roll_series_k"] = f"hip={plant.ser_hip}:{plant.ser_hip},ankle={plant.ser_ankle}:{plant.ser_ankle}"
    if chain == "pose":
        extra["fw_shaper_hz"] = 0.0
        extra["fw_shaper_hz_max"] = None
    elif chain != "prec":
        raise ValueError("chain: prec | pose")
    env = EP.make_env(cfg, seconds, True, xml or SR.DEFAULT_XML, extra=extra)
    env.reset(seed=0)
    m = env.model
    tb = env._torso_bid
    m.body_mass[tb] = env._nom_mass[tb] * plant.torso_scale
    m.body_inertia[tb] = env._nom_inertia[tb] * plant.torso_scale
    m.body_ipos[tb] = env._nom_ipos[tb]
    m.body_ipos[tb, 1] += plant.torso_com_y
    mujoco.mj_setConst(m, env.data)
    EP.set_contact_friction(env, plant.mu)
    m.dof_frictionloss[env._sqvel] = plant.jfric
    kinds = [W.joint_role(n) for n in env._servo_names]
    env._act_delay_vec = np.array([plant.delay_3250 if k == "hip_roll" else plant.delay_3215 for k in kinds], dtype=int)
    env._kp_prof = np.asarray(env._kp_prof) * plant.kp_scale
    env._kd_prof = np.asarray(env._kd_prof) * plant.kp_scale
    if refit:
        import gate_common as GC
        GC.apply_terms(env, GC.parse_terms(REFIT_TERMS))
    return env


def inv_action(env, q):
    """joint targets (rad) -> the env's [-1, 1] action (action_map full): the inverse of _action_to_ctrl."""
    d0, hi, lo = env._default, env._hi, env._lo
    return np.clip(np.where(q >= d0, (q - d0) / np.maximum(hi - d0, 1e-9), (q - d0) / np.maximum(d0 - lo, 1e-9)), -1, 1)


PI = dict(kp=0.15, ki=1.0, kd=0.0, clamp=math.radians(10.0), i_lim=math.radians(3.0))   # roll_pi.h Config, balance.DEFAULT
PI_ABORT = math.radians(12.0)                                                             # balance abort: |err| past this


class Stream:
    """The goal vector for every control tick, as the firmware's streamer produces it.

    seg_ms == 0: the 50 Hz keyframes themselves, settle at keyframe 0 first, hold at the end. seg_ms > 0:
    `pose ... s<seg_ms>` chained: every seg_ms a minimum-jerk move from the joints' present (read-back,
    whole-tick quantised) positions to the keyframe seg_ms ahead (+ adv_ms). balance="stance": each segment
    carries `B<stance>` (the foot that is not swinging at its middle): the bench roll PI
    (experiments/one-leg-stand/balance.RollPI, roll_pi.h) adds ka*u to the stance ankle roll and kh*u to its
    hip roll on top of the nominal goal; a segment on the SAME stance continues the PI and starts from the
    previous nominal target (cmdPose bal_cont), a change of stance starts a new PI (u = 0, ref = the roll now)
    from the read-back positions."""

    def __init__(self, env, rung, seg_ms, settle_ticks, hold_ticks, adv_ms=0.0, balance="off", ka=1.0, kh=-1.0):
        self.env, self.rung, self.seg_ms, self.settle, self.hold = env, rung, seg_ms, settle_ticks, hold_ticks
        self.n = len(rung.q)
        self.total = settle_ticks + self.n + hold_ticks
        self.seg = max(int(round(seg_ms / 20.0)), 1)
        self.adv = int(round(adv_ms / 20.0))
        self.balance, self.ka, self.kh = balance, ka, kh
        self.q_start = self.q_goal = None
        self.stance = None
        self.pi = None
        self.ref = 0.0
        self.u = 0.0
        self.names = list(rung.names)
        self.aborted = False
        self.u_peak = 0.0

    def kq(self, i):
        return self.rung.q[min(max(i - self.settle, 0), self.n - 1)]

    def goal(self, i):
        env = self.env
        j = i - self.settle
        if self.seg_ms <= 0 or i < self.settle:
            return self.kq(i)
        if j % self.seg == 0:
            mid = self.rung.phase[min(j + self.seg // 2, self.n - 1)]
            stance = None
            if self.balance == "stance":
                stance = "R" if math.sin(mid) > 0.0 else "L"
            if self.balance == "stance" and stance == self.stance and self.q_goal is not None:
                self.q_start = self.q_goal                              # bal_cont: from the previous nominal target
            else:
                self.q_start = env._quant_angle(np.asarray(env.data.qpos[env._sqpos], dtype=float))
                if stance is not None:
                    import balance as BAL
                    self.pi = BAL.RollPI(PI["kp"], PI["ki"], PI["clamp"], PI["kd"], PI["i_lim"])
                    self.ref = self._roll()
                    self.u = 0.0
                    self.aborted = False
            self.stance = stance
            self.q_goal = self.kq(i + self.seg + self.adv)
        s = (j % self.seg + 1) / self.seg                     # the tick's goal is the move's value at the END of its 20 ms
        q = self.q_start + SR.minjerk(s) * (self.q_goal - self.q_start)
        if self.stance is not None:
            err = self._roll() - self.ref
            if abs(err) > PI_ABORT:
                self.aborted = True                           # abort: the correction is frozen (the firmware lands the pose)
            elif not self.aborted:
                self.u = self.pi.step(err, 0.02, float(env.data.qvel[3]))
            q = q.copy()
            q[self.names.index(f"{self.stance}_ankle_roll")] += self.ka * self.u
            q[self.names.index(f"{self.stance}_hip_roll")] += self.kh * self.u
            self.u_peak = max(self.u_peak, abs(self.u))
        return q

    def _roll(self):
        R = self.env.data.xmat[self.env._torso_bid].reshape(3, 3)
        return math.atan2(R[2, 1], R[2, 2])


def foot_state(env):
    """(normal force N, clearance m) per foot [L, R]: floor-pad contact forces and the lowest pad's height."""
    m, d = env.model, env.data
    f6 = np.zeros(6)
    force = [0.0, 0.0]
    pads = [set(p) for p in env._pad_gids]
    for c in range(d.ncon):
        con = d.contact[c]
        for side in (0, 1):
            if (con.geom1 == env._floor_gid and con.geom2 in pads[side]) or \
               (con.geom2 == env._floor_gid and con.geom1 in pads[side]):
                mujoco.mj_contactForce(m, d, c, f6)
                force[side] += max(f6[0], 0.0)
    clear = [min(d.geom_xpos[g][2] - m.geom_size[g][0] for g in p) for p in env._pad_gids]
    return force, clear


def _edges(env):
    d, m_ = env.data, env.model
    out = []
    for pads in env._pad_gids:
        ys = [d.geom_xpos[g][1] for g in pads]
        r = m_.geom_size[pads[0]][0]
        out += [min(ys) - r, max(ys) + r]
    return out


def play(env, rung, seg_ms=0.0, adv_ms=0.0, balance="off"):
    """One open-loop episode. Returns the per-tick trace of the rung (settle and the final hold included)."""
    env.reset(seed=0)           # re-stand; the plant edits made in build_env persist (reset leaves DR off)
    settle = int(SETTLE_S * SR.KEY_HZ)
    hold = int(HOLD_END_S * SR.KEY_HZ)
    cols = {k: [] for k in ("F", "h", "roll", "tilt", "rate", "gyro", "y", "z", "pad", "com", "edge")}
    stance0 = None
    fell = False
    stream = Stream(env, rung, seg_ms, settle, hold, adv_ms, balance)
    for i in range(stream.total):
        env.step(inv_action(env, stream.goal(i)))
        d = env.data
        R = d.xmat[env._torso_bid].reshape(3, 3)
        F, H = foot_state(env)
        cols["F"].append(F)
        cols["h"].append(H)
        cols["roll"].append(math.atan2(R[2, 1], R[2, 2]))
        cols["tilt"].append(math.acos(max(-1.0, min(1.0, R[2, 2]))))
        w = np.array(d.qvel[3:6])           # free joint: angular velocity in the body frame
        cols["rate"].append(w[0])
        cols["gyro"].append(float(np.linalg.norm(w)))
        cols["y"].append(float(d.xpos[env._torso_bid][1]))
        cols["z"].append(float(d.xpos[env._torso_bid][2]))
        cols["pad"].append([d.xpos[env._foot_bids[0]][:2].copy(), d.xpos[env._foot_bids[1]][:2].copy()])
        cols["com"].append(float(d.subtree_com[0][1]))
        # each foot's support extent across the body (world y of its pads' outer faces): [Lmin, Lmax, Rmin, Rmax]
        cols["edge"].append(_edges(env))
        if cols["tilt"][-1] > math.radians(60):
            fell = True
            break
    tr = {k: np.array(v) for k, v in cols.items()}
    tr["settle"] = settle
    tr["fell"] = fell
    tr["u_peak_deg"] = math.degrees(stream.u_peak)
    tr["pi_aborted"] = stream.aborted
    return tr


def weight_n(env):
    return float(np.sum(env.model.body_mass)) * G


def ema(x, tau=FLAIL_TAU, dt=DT):
    a = 1.0 - math.exp(-dt / tau)
    y = np.zeros(len(x))
    s = 0.0
    for i, v in enumerate(x):
        s += a * (v - s)
        y[i] = s
    return y


def metrics(env, rung, tr):
    """Per-side and whole-run numbers for one episode (see the module docstring). The per-side
    numbers use the plateau cycles up to the tick the torso first passes the bench fall line."""
    W_ = weight_n(env)
    n_run = len(tr["F"])
    off = tr["settle"]
    n_key = len(rung.t)
    sl = slice(off, min(off + n_key, n_run))
    ph = rung.phase[: sl.stop - sl.start]
    plateau = rung.plateau()[: len(ph)]
    tilt_all = np.degrees(tr["tilt"])
    over = np.nonzero(tilt_all[sl] > FALL_BENCH_DEG)[0]
    alive = np.ones(len(ph), bool)
    t_fall = None
    if len(over):
        alive[over[0]:] = False
        t_fall = float(rung.t[over[0]])
    # a foot's load is read as its 100 ms moving average: one-tick dropouts of a position-held foot are contact
    # chatter, not an unloading that a camera or a person would see
    Fa = np.asarray(tr["F"], dtype=float)
    k = np.ones(UNLOAD_TICKS) / UNLOAD_TICKS
    Fma = np.stack([np.convolve(np.pad(Fa[:, i], (UNLOAD_TICKS // 2, UNLOAD_TICKS // 2), mode="edge"), k, mode="valid")[: len(Fa)]
                    for i in (0, 1)], axis=1)
    F, Fraw, H, EDGE, COM = Fma[sl], Fa[sl], tr["h"][sl], np.array(tr["edge"])[sl], tr["com"][sl] if isinstance(tr["com"], np.ndarray) else np.array(tr["com"])[sl]
    tilt = tilt_all
    out = dict(weight_N=W_, completed=bool(len(ph) == n_key and not tr["fell"]), t_fall_s=t_fall)
    standing_h = np.median(tr["h"][:off], axis=0) if off else np.zeros(2)
    sides = {}
    for si, side in enumerate("LR"):
        swing = (np.sin(ph) > 0.0) if side == "L" else (np.sin(ph) < 0.0)
        m = plateau & swing & alive
        if not m.any():
            continue
        stance = 1 - si
        Fs, hs = F[m, si], H[m, si] - standing_h[si]
        # quasi-static margin: the whole-body CoM's y inside the STANCE foot's support extent (m > 0: over the
        # stance sole, the foot that swings can be unloaded without a dynamic assist), best of the window
        lo, hi = EDGE[m, 2 * stance], EDGE[m, 2 * stance + 1]
        com_margin = np.minimum(COM[m] - lo, hi - COM[m])
        sides[side] = dict(
            F_min_N=float(Fs.min()), F_min_frac=float(Fs.min() / W_),
            F_med_frac=float(np.median(Fs) / W_),
            F_raw_min_N=float(Fraw[m, si].min()),
            clear_mm=float(1e3 * hs.max()),
            unloaded=bool(Fs.min() < UNLOAD_FRAC * W_), clears=bool(1e3 * hs.max() >= CLEAR_MM),
            stance_F_max_frac=float(F[m, stance].max() / W_),
            com_margin_mm=float(1e3 * com_margin.max()),
        )
    out["sides"] = sides
    roll = np.degrees(tr["roll"][sl])
    rate = np.degrees(tr["rate"][sl])
    pl = plateau & alive
    out["roll_peak_deg"] = float(np.abs(roll[pl]).max()) if pl.any() else float("nan")
    out["roll_rate_peak_dps"] = float(np.abs(rate[pl]).max()) if pl.any() else float("nan")
    out["tilt_peak_deg"] = float(tilt.max())
    e = ema(tr["gyro"])
    out["gyro_ema_peak"] = float(e.max())

    def held(limit, ms):                    # the debounced guard: past the limit for `ms` straight
        k = int(round(ms / 20.0))
        run = 0
        for v in tilt > limit:
            run = run + 1 if v else 0
            if run >= k:
                return True
        return False
    out["fall_guard_trip"] = held(FALL_BENCH_DEG, 100)
    out["flail_tilt_trip"] = held(FLAIL_TILT_DEG, 200)
    out["flail_rate_trip"] = bool(e.max() > FLAIL_RATE)
    out["margin_fall_deg"] = FALL_BENCH_DEG - out["tilt_peak_deg"]
    out["margin_flail_tilt_deg"] = FLAIL_TILT_DEG - out["tilt_peak_deg"]
    out["margin_flail_rate"] = FLAIL_RATE - out["gyro_ema_peak"]
    out["end_tilt_deg"] = float(tilt[-1])
    out["up"] = bool((not tr["fell"]) and tilt.max() < FALL_BENCH_DEG and tilt[-1] < 10.0)
    out["torso_y_peak_mm"] = float(1e3 * np.abs(np.array(tr["y"])[sl][pl] - np.median(tr["y"][:off])).max()) if pl.any() else float("nan")
    pad = np.array(tr["pad"])
    a_ = np.nonzero(alive)[0]
    out["slip_mm"] = [float(1e3 * np.linalg.norm(pad[sl][a_][:, i] - pad[off - 1, i], axis=1).max()) for i in (0, 1)]
    return out


# --------------------------------------------------------------------------------- the band
def band_stats(vals):
    v = np.asarray(vals, dtype=float)
    return dict(min=float(np.nanmin(v)), p10=float(np.nanpercentile(v, 10)), med=float(np.nanmedian(v)),
                p90=float(np.nanpercentile(v, 90)), max=float(np.nanmax(v)))


def run_rung(cfg, rung, plants, chain, seg_ms, balance="off", refit=False):
    eps = []
    for p in plants:
        env = build_env(cfg, p, chain, refit=refit)
        tr = play(env, rung, seg_ms, 0.0, balance)
        eps.append(dict(plant=asdict(p), u_peak_deg=tr["u_peak_deg"], pi_aborted=bool(tr["pi_aborted"]), **metrics(env, rung, tr)))
    return eps


def summarise(eps):
    n = len(eps)
    s = dict(n=n, up=sum(e["up"] for e in eps), completed=sum(e["completed"] for e in eps),
             t_fall_s=band_stats([e["t_fall_s"] for e in eps if e["t_fall_s"] is not None]) if any(e["t_fall_s"] is not None for e in eps) else None,
             u_peak_deg=band_stats([e.get("u_peak_deg", 0.0) for e in eps]),
             pi_aborts=sum(e.get("pi_aborted", False) for e in eps),
             fall_guard_trips=sum(e["fall_guard_trip"] for e in eps),
             flail_tilt_trips=sum(e["flail_tilt_trip"] for e in eps),
             flail_rate_trips=sum(e["flail_rate_trip"] for e in eps),
             tilt_peak_deg=band_stats([e["tilt_peak_deg"] for e in eps]),
             roll_peak_deg=band_stats([e["roll_peak_deg"] for e in eps]),
             roll_rate_peak_dps=band_stats([e["roll_rate_peak_dps"] for e in eps]),
             gyro_ema_peak=band_stats([e["gyro_ema_peak"] for e in eps]),
             slip_mm=band_stats([max(e["slip_mm"]) for e in eps]), sides={})
    for side in "LR":
        ss = [e["sides"][side] for e in eps if side in e["sides"]]
        if not ss:
            continue
        s["sides"][side] = dict(
            F_min_frac=band_stats([x["F_min_frac"] for x in ss]),
            F_min_N=band_stats([x["F_min_N"] for x in ss]),
            clear_mm=band_stats([x["clear_mm"] for x in ss]),
            unloaded=sum(x["unloaded"] for x in ss), clears=sum(x["clears"] for x in ss), n=len(ss),
            stance_F_max_frac=band_stats([x["stance_F_max_frac"] for x in ss]),
            com_margin_mm=band_stats([x["com_margin_mm"] for x in ss]))
    return s


def rehearse(cfg=None, Ms=SR.LADDER_M, lead=SR.LEAD, hz=SR.CLOCK_HZ, cycles=SR.CYCLES, ramp_s=SR.RAMP_S,
             n=16, chain="prec", seg_ms=0.0, seed=0, only=None, progress=None, balance="off", refit=False):
    cfg = cfg or SR.load_config()
    ref = SR.SwayReference(cfg)
    band = Band(cfg, refit)
    plants = [band.mid()] + band.lhs(n, seed)
    rungs = ref.ladder(Ms, lead, hz, cycles, ramp_s)
    res = dict(meta=dict(chain=chain, seg_ms=seg_ms, balance=balance, plant="refit" if refit else "old", n_band=n, seed=seed, lead=lead, hz=hz, cycles=cycles, ramp_s=ramp_s,
                         Ms=list(Ms), config=os.path.basename(cfg.get("_path", SR.DEFAULT_CONFIG)),
                         xml=os.path.basename(SR.DEFAULT_XML), band=band.r), rungs=[])
    for r in rungs:
        if only and r.index not in only:
            continue
        t0 = time.time()
        eps = run_rung(cfg, r, plants, chain, seg_ms, balance, refit)
        res["rungs"].append(dict(rung=r.meta(), nominal=eps[0], band=summarise(eps[1:]), episodes=eps[1:]))
        if progress:
            progress(r, res["rungs"][-1], time.time() - t0)
    return res


def fmt_band(b, scale=1.0, d=1):
    return f"{b['min'] * scale:.{d}f} / {b['med'] * scale:.{d}f} / {b['max'] * scale:.{d}f}"


def report(res):
    mt = res["meta"]
    L = [f"chain `{mt['chain']}`, playback `{'direct 50 Hz' if not mt['seg_ms'] else 'pose s%d chained' % mt['seg_ms']}`"
         f"{', bench roll PI on the stance foot' if mt.get('balance', 'off') == 'stance' else ', no balance loop'}, "
         f"clock {mt['hz']} Hz, lead {mt['lead']} rad, {mt['cycles']} cycles, band of {mt['n_band']} Latin-hypercube plants "
         f"(cells are band min / median / max over the plants; per-side cells use the plants that were still up)", "",
         "| rung | up | foot | F_min % weight | unloaded | clear mm | clears | CoM margin mm | roll pk deg | roll rate pk deg/s | tilt pk deg | gyro EMA pk rad/s | flail trips (tilt/rate) |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for R in res["rungs"]:
        b, r = R["band"], R["rung"]
        for side in "LR":
            s = b["sides"].get(side)
            if not s:
                continue
            tail = (f"{fmt_band(b['roll_peak_deg'])} | {fmt_band(b['roll_rate_peak_dps'], 1, 0)} | {fmt_band(b['tilt_peak_deg'])} | "
                    f"{fmt_band(b['gyro_ema_peak'], 1, 2)} | {b['flail_tilt_trips']}/{b['flail_rate_trips']} |" if side == "L" else " | | | | |")
            L.append(f"| {r['index']} {r['kind']} {r['M_m'] * 100:.1f} cm | {b['up']}/{b['n']} | {side} swing | "
                     f"{fmt_band(s['F_min_frac'], 100, 0)} | {s['unloaded']}/{s['n']} | {fmt_band(s['clear_mm'])} | {s['clears']}/{s['n']} | "
                     f"{fmt_band(s['com_margin_mm'])} | " + tail)
    return "\n".join(L)


def skill_report(rows, chain):
    n = len(rows)
    q = lambda v, d=1: f"{np.min(v):.{d}f} / {np.median(v):.{d}f} / {np.max(v):.{d}f}"
    L = [f"trained `sway` skill ({os.path.basename(SKILL_RUN)}), chain `{chain}`, referee `hip_sway` command (vy = 0.12 sin(2 pi (t-1)/1.6), 6 s), "
         f"{n} plants (the band midpoint + {n - 1} Latin-hypercube draws; cells min / median / max)", "",
         f"- stays up {sum(r['up'] for r in rows)}/{n}; peak torso tilt {q([r['tilt_peak_deg'] for r in rows])} deg",
         f"- torso lateral sway amplitude (1.6 s moving mean removed) {q([r['amp_mm'] for r in rows], 0)} mm; net drift over the 6 s {q([r['drift_mm'] for r in rows], 0)} mm "
         "(the skill side-steps: its feet do not stay planted)", ""]
    for side in "LR":
        ss = [r[side] for r in rows if side in r]
        if ss:
            L.append(f"- {side} foot is the unloaded one (torso at 70 % of its excursion toward the other): {len(ss)} plants; load at its lightest "
                     f"{q([100 * x['F_min_frac'] for x in ss], 0)} % of weight; clearance {q([x['clear_mm'] for x in ss])} mm; CoM margin over the other sole {q([x['com_margin_mm'] for x in ss])} mm")
    return "\n".join(L)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--config", default=None)
    ap.add_argument("--M", type=SR.parse_Ms, default=SR.LADDER_M)
    ap.add_argument("--lead", type=float, default=SR.LEAD)
    ap.add_argument("--hz", type=float, default=SR.CLOCK_HZ)
    ap.add_argument("--cycles", type=int, default=SR.CYCLES)
    ap.add_argument("--n", type=int, default=16, help="plants in the band (plus the midpoint)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--chain", choices=("prec", "pose"), default="prec")
    ap.add_argument("--seg-ms", type=float, default=0.0, help="0 = direct 50 Hz; else chained pose s<ms> segments")
    ap.add_argument("--balance", choices=("off", "stance"), default="off", help="stance: the bench roll PI on the non-swinging foot (needs --seg-ms > 0)")
    ap.add_argument("--refit", action="store_true",
                    help="the refit plant (REFIT_TERMS and the series roll springs over REFIT_SERIES) instead of the old one")
    ap.add_argument("--rung", type=int, action="append", help="only this rung (repeatable)")
    ap.add_argument("--json", default=None)
    ap.add_argument("--keep-episodes", action="store_true", help="keep every plant's episode record in the json (large)")
    ap.add_argument("--report", default=None)
    ap.add_argument("--skill", action="store_true", help="rehearse the trained `sway` skill (SKILL_RUN) instead of the ladder")
    a = ap.parse_args(argv)
    cfg = SR.load_config(a.config)
    cfg["_path"] = a.config or SR.DEFAULT_CONFIG

    if a.skill:
        rows = skill_sway(cfg, n=a.n, chain=a.chain, seed=a.seed)
        txt = skill_report(rows, a.chain)
        if a.json:
            json.dump(dict(chain=a.chain, run=os.path.basename(SKILL_RUN), n=len(rows), rows=rows), open(a.json, "w"), indent=1)
        if a.report:
            open(a.report, "w").write(txt + "\n")
        print(txt)
        return 0

    def prog(r, R, dt):
        b = R["band"]
        print(f"rung {r.index:2d} {r.kind:5s} M {r.M * 100:4.1f}: up {b['up']}/{b['n']}  tilt pk "
              f"{fmt_band(b['tilt_peak_deg'])}  " + "  ".join(
                  f"{s} F_min% {fmt_band(v['F_min_frac'], 100, 0)} clear {fmt_band(v['clear_mm'])}" for s, v in b["sides"].items())
              + f"  ({dt:.0f}s)", flush=True)
    res = rehearse(cfg, a.M, a.lead, a.hz, a.cycles, SR.RAMP_S, a.n, a.chain, a.seg_ms, a.seed, set(a.rung or []), prog, a.balance, a.refit)
    if a.json:
        if not a.keep_episodes:
            for R in res["rungs"]:
                R.pop("episodes", None)
        with open(a.json, "w") as f:
            json.dump(res, f, indent=1)
    txt = report(res)
    if a.report:
        with open(a.report, "w") as f:
            f.write(txt + "\n")
    print(txt)
    return 0



# --------------------------------------------------------------------------------- the trained `sway` skill
SKILL_RUN = "/home/claw/code/robot/sim/runs/robot_legs_prec45_ws1_only"      # gitignored: the main checkout only


def skill_sway(cfg, run_dir=SKILL_RUN, n=8, chain="prec", seed=0):
    """The trained policy's `sway` skill (docs/training.md section 4: c1 oscillating, feet planted; the referee's
    `hip_sway` scenario: vy = 0.12 sin(2 pi (t - 1) / 1.6) from 1 s to 7 s), on the same plant band, with the same
    foot and CoM measures: per side, the foot that is weight-bearing when the torso is at 70 % of its excursion toward
    it, and the OTHER foot's load and the CoM margin over the loaded one then."""
    import eval_precision as EP
    band = Band(cfg)
    plants = [band.mid()] + band.lhs(n, seed)
    rows = []
    act = None
    for p in plants:
        env = build_env(cfg, p, chain, seconds=12.0)
        obs, _ = env.reset(seed=0)
        if act is None:
            act = EP.load_policy(run_dir, int(np.asarray(obs).shape[0]), int(env.action_space.shape[0]))
        tr = {k: [] for k in ("F", "h", "y", "com", "edge", "tilt")}
        for k in range(int(8.0 / DT)):
            t = k * DT
            vy = 0.12 * math.sin(2 * math.pi * (t - 1.0) / 1.6) if 1.0 <= t < 7.0 else 0.0
            env.set_command(0, vy, 0, 1, 0)
            obs, *_ = env.step(act(obs))
            d = env.data
            F, H = foot_state(env)
            tr["F"].append(F)
            tr["h"].append(H)
            tr["y"].append(float(d.xpos[env._torso_bid][1]))
            tr["com"].append(float(d.subtree_com[0][1]))
            tr["edge"].append(_edges(env))
            R = d.xmat[env._torso_bid].reshape(3, 3)
            tr["tilt"].append(math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2])))))
        F = np.asarray(tr["F"])
        k_ = np.ones(UNLOAD_TICKS) / UNLOAD_TICKS
        Fma = np.stack([np.convolve(np.pad(F[:, i], (2, 2), mode="edge"), k_, mode="valid")[: len(F)] for i in (0, 1)], axis=1)
        y = np.asarray(tr["y"])
        win = slice(int(1.0 / DT), int(7.0 / DT))
        y0 = np.median(y[: int(1.0 / DT)])
        yw = y[win] - y0
        # the skill drifts (it side-steps): the sway is what is left after the 1.6 s moving mean is taken out
        k_ma = int(1.6 / DT)
        ypad = np.pad(y, (k_ma // 2, k_ma // 2), mode="edge")
        ydt = (y - np.convolve(ypad, np.ones(k_ma) / k_ma, mode="valid")[: len(y)])[win]
        amp = 0.5 * (ydt.max() - ydt.min())
        W_ = weight_n(env)
        row = dict(plant=asdict(p), amp_mm=1e3 * amp, drift_mm=1e3 * abs(y[win][-1] - y[win][0]), tilt_peak_deg=max(tr["tilt"]), up=bool(max(tr["tilt"]) < FALL_BENCH_DEG))
        edge = np.asarray(tr["edge"])
        com = np.asarray(tr["com"])
        h0 = np.median(np.asarray(tr["h"])[: int(1.0 / DT)], axis=0)
        for side, si in (("L", 0), ("R", 1)):               # `side` = the foot that SWINGS (the other one carries)
            stance = 1 - si
            shifted = (ydt < -0.7 * amp) if stance == 1 else (ydt > 0.7 * amp)   # torso -y = toward the R foot
            m = np.zeros(len(y), bool)
            m[win] = shifted
            if m.any():
                lo, hi = edge[m, 2 * stance], edge[m, 2 * stance + 1]
                row[side] = dict(F_min_frac=float(Fma[m, si].min() / W_),
                                 clear_mm=float(1e3 * (np.asarray(tr["h"])[m, si] - h0[si]).max()),
                                 com_margin_mm=float(1e3 * np.minimum(com[m] - lo, hi - com[m]).max()))
        rows.append(row)
    return rows


if __name__ == "__main__":
    sys.exit(main())
