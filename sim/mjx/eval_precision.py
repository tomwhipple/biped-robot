"""CPU-referee evaluation of PRECISION locomotion skills (ext_cmd policies).

Sibling of eval_ref.py. Where eval_ref referees plain command-tracking (walk /
stand / pivot), this suite grades the 7-channel precision skills the ext_cmd
policy family is trained on: one-leg balance, air-drawn foot circles, straight
1 m walks that stop on a line, backward walks, sidesteps, crouch holds, and
closed-loop waypoint navigation.

Same rule as eval_ref: MJX training numbers are never trusted; every policy is
re-run here in the CPU BimoWalkerEnv (the engine every prior result was
validated in) under conditions that MATCH the hardware claim -- GoPro payload
(0.154 kg), latency (4 ms), gear backlash (0.7 deg), full domain randomization,
IMU-realizable observations. `--nominal` strips DR/latency/backlash for a
clean-plant reference.

Each scenario is a per-step command SCRIPT driven by GROUND-TRUTH state and
graded against explicit success criteria (see SCENARIOS below). Every scenario
opens with 1.0 s of stand command (0,0,0,1,0) to settle; a fall in that first
second (or ever) is a failure.

CAVEAT -- sim-only odometry: the closed-loop scenarios (square_return,
circle_return) steer on the simulator's ground-truth torso pose. The real robot
has no such pose estimate; closing that odometry gap on hardware is a known open
item, so these two scenarios grade the GAIT/turning controller, not a shippable
navigation stack.

Run:
  JAX_PLATFORMS=cpu .venv/bin/python sim/mjx/eval_precision.py \
      --run-name mjx_prec_v1 [--episodes 8] [--nominal] [--render] \
      [--scenarios balance_L,line_1m,...]
"""
import argparse
import inspect
import json
import math
import os
import pickle
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
RUNS = os.path.join(HERE, "..", "runs")
DEFAULT_XML = os.path.join(HERE, "..", "bimo_biped_v2_asbuilt.xml")

import jax
from brax.training.acme import running_statistics
from brax.training.agents.ppo import networks as ppo_networks

from walker_env import BimoWalkerEnv

G = 9.81
_ENV_PARAMS = set(inspect.signature(BimoWalkerEnv.__init__).parameters)


# =====================================================================
#  policy + env loading (mirrors eval_ref.py)
# =====================================================================
def load_policy(run_dir, obs_size, act_size=8):
    with open(os.path.join(run_dir, "params.pkl"), "rb") as f:
        params = pickle.load(f)
    net = ppo_networks.make_ppo_networks(
        observation_size=obs_size, action_size=act_size,
        preprocess_observations_fn=running_statistics.normalize,
        policy_hidden_layer_sizes=(128, 128),
        value_hidden_layer_sizes=(256, 256))
    make_policy = ppo_networks.make_inference_fn(net)
    policy = jax.jit(make_policy((params[0], params[1]), deterministic=True))
    rng = jax.random.PRNGKey(0)

    def act(obs):
        a, _ = policy(obs[None].astype(np.float32), rng)
        return np.asarray(a[0])
    return act


def make_env(cfg, episode_seconds, nominal, xml):
    """Eval env: ext_cmd precision plant mirroring config.json construction,
    but pinned to the hardware-claim conditions (or --nominal clean plant)."""
    kw = {k: v for k, v in cfg.items() if k in _ENV_PARAMS}
    kw.update(
        xml_path=xml, command_mode=True, ext_cmd=True,
        actuator_model="sts3215", imu_obs=True, getup=False, cmd_fixed=None,
        payload_mass=0.154, payload_max=None,
        episode_seconds=episode_seconds, render_mode="rgb_array",
    )
    if nominal:
        kw.update(domain_rand=False, latency_ms=0.0, latency_ms_max=None,
                  latency_jitter_ms=0.0, backlash_deg=0.0, backlash_deg_max=None)
    else:
        kw.update(domain_rand=True, latency_ms=4.0, latency_ms_max=None,
                  latency_jitter_ms=0.0, backlash_deg=0.7, backlash_deg_max=None)
    return BimoWalkerEnv(**kw)


# =====================================================================
#  rollout driver + shared metrics
# =====================================================================
def _yaw(qpos):
    qw, qx, qy, qz = qpos[3:7]
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Driver:
    """Runs one episode, applying a per-step command from a controller and
    recording ground-truth state + smoothness metrics."""

    def __init__(self, env, act, seed, record):
        self.env = env
        self.act = act
        self.record = record
        self.dt = env.control_dt
        obs, _ = env.reset(seed=seed)
        self.obs = obs
        self.fell = False
        self.frames = []
        self.rows = []
        # shared-metric accumulators
        self.slip = []                 # per-contact sole planar speed samples
        self.air_l = self.air_r = 0
        self._prev_sole = [env.data.geom_xpos[g][:2].copy()
                           for g in env._sole_gids]

    def initial_gt(self):
        d = self.env.data
        cl, cr = self.env._foot_contacts()
        return dict(x=float(d.qpos[0]), y=float(d.qpos[1]), yaw=_yaw(d.qpos),
                    planar=float(np.hypot(d.qvel[0], d.qvel[1])),
                    con_l=cl, con_r=cr, height=float(d.qpos[2]), up_z=1.0,
                    info=None)

    def step(self, cmd):
        env = self.env
        env.set_command(*cmd)
        self.obs = env._obs()
        a = self.act(self.obs)
        self.obs, _, term, _, info = env.step(a)
        d = env.data
        cl, cr = env._foot_contacts()
        # sole slip: planar geom speed while that foot is in stance
        for i, con in enumerate((cl, cr)):
            p = d.geom_xpos[env._sole_gids[i]][:2].copy()
            v = float(np.linalg.norm(p - self._prev_sole[i]) / self.dt)
            if con:
                self.slip.append(v)
            self._prev_sole[i] = p
        if not cl:
            self.air_l += 1
        if not cr:
            self.air_r += 1
        t_post = len(self.rows) * self.dt + self.dt
        self.rows.append(dict(
            t=t_post, x=float(d.qpos[0]), y=float(d.qpos[1]), yaw=_yaw(d.qpos),
            planar=float(np.hypot(d.qvel[0], d.qvel[1])),
            wob2=float(d.qvel[3] ** 2 + d.qvel[4] ** 2),
            watts=float(info["power_w"]), height=float(info["height"]),
            up_z=float(info["up_z"]), con_l=cl, con_r=cr,
            foot_err=float(info["foot_err"])))
        if self.record and len(self.rows) % 3 == 0:
            self.frames.append(env.render())
        if term:
            self.fell = True
        gt = dict(x=float(d.qpos[0]), y=float(d.qpos[1]), yaw=_yaw(d.qpos),
                  planar=float(np.hypot(d.qvel[0], d.qvel[1])),
                  con_l=cl, con_r=cr, height=float(info["height"]),
                  up_z=float(info["up_z"]), info=info)
        return gt, term

    def shared_metrics(self, mass):
        r = self.rows
        n = max(len(r), 1)
        wob = float(np.sqrt(np.mean([x["wob2"] for x in r]))) if r else float("nan")
        watts = float(np.mean([x["watts"] for x in r])) if r else float("nan")
        vbar = float(np.mean([x["planar"] for x in r])) if r else 0.0
        cot = (watts / (mass * G * vbar)) if vbar > 0.05 else None
        slip = float(np.mean(self.slip)) if self.slip else 0.0
        al, ar = self.air_l / n, self.air_r / n
        mean_air = 0.5 * (al + ar)
        sym = abs(al - ar) / max(mean_air, 1e-6)
        return dict(wobble_rms=wob, mean_watts=watts, mean_speed=vbar,
                    cot=cot, foot_slip=slip, air_l=al, air_r=ar, symmetry=sym)


# --- window helpers ---------------------------------------------------
def _win(rows, t0, t1):
    return [r for r in rows if t0 - 1e-9 <= r["t"] <= t1 + 1e-9]


def _frac(rows, key):
    return sum(1 for r in rows if r[key]) / len(rows) if rows else float("nan")


def _mean(rows, key):
    return float(np.mean([r[key] for r in rows])) if rows else float("nan")


def _drift_rms(rows):
    if not rows:
        return float("nan")
    xs = np.array([r["x"] for r in rows])
    ys = np.array([r["y"] for r in rows])
    return float(np.sqrt(np.mean((xs - xs.mean()) ** 2 + (ys - ys.mean()) ** 2)))


def _wob(rows):
    return float(np.sqrt(np.mean([r["wob2"] for r in rows]))) if rows else float("nan")


def _settles(rows, t0, t1, hold=1.0, thresh=0.15):
    """True if planar speed stays < thresh continuously for `hold` s, with the
    hold COMPLETING within [t0, t1]."""
    run = None
    for r in rows:
        if r["t"] < t0:
            continue
        if r["t"] > t1:
            break
        if r["planar"] < thresh:
            if run is None:
                run = r["t"]
            if r["t"] - run >= hold - 1e-9:
                return True
        else:
            run = None
    return False


# =====================================================================
#  scenarios: each returns (build, evaluate, meta)
#    build()   -> (ctrl(t, gt, ev) -> cmd_tuple, ev_dict)
#    evaluate(rows, ev, fell, N, shared) -> dict(success, metrics, headline)
# =====================================================================
def _lift_of(side):   # -1 = left foot lifted, +1 = right foot lifted
    return -1 if side == "L" else 1


def _swing_key(side):
    return "con_l" if side == "L" else "con_r"


def scen_balance(side):
    lift = _lift_of(side)

    def build():
        ev = {}

        def ctrl(t, gt, ev):
            if t < 1.0:
                return (0, 0, 0, 1, 0)
            return (0, 0, 0, 1, lift)
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        lift_win = _win(rows, 1.0, 11.0)
        judge_win = _win(rows, 2.0, 11.0)           # skip first 1 s of lift
        contact = _frac(judge_win, _swing_key(side))
        drift = _drift_rms(lift_win)
        wob = _wob(lift_win)
        watts = _mean(lift_win, "watts")
        success = (not fell) and contact < 0.05
        return dict(success=success,
                    metrics=dict(lifted_contact=contact, drift_rms=drift,
                                 wobble=wob, watts=watts),
                    headline=f"contact {contact*100:.1f}%")
    return build, evaluate


def scen_circle_air(side):
    lift = _lift_of(side)

    def build():
        ev = {}

        def ctrl(t, gt, ev):
            if t < 1.0:
                return (0, 0, 0, 1, 0)
            if t < 2.5:                              # lift 1.5 s
                return (0, 0, 0, 1, lift)
            if t < 7.5:                              # 2 periods of 2.5 s
                tc = t - 2.5
                fx = 0.04 * math.cos(2 * math.pi * tc / 2.5)
                fz = 0.04 * math.sin(2 * math.pi * tc / 2.5)
                return (0, 0, 0, 1, lift, fx, fz)
            return (0, 0, 0, 1, lift)                # hold 1 s
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        circ = _win(rows, 2.5, 7.5)
        touchdown = _frac(circ, _swing_key(side))
        foot_err = _mean(circ, "foot_err")
        success = (not fell) and touchdown < 0.10
        return dict(success=success,
                    metrics=dict(foot_err=foot_err, touchdown_frac=touchdown,
                                 lifted_contact=touchdown),
                    headline=f"trkErr {foot_err*1000:.0f}mm")
    return build, evaluate


def scen_line_1m():
    def build():
        ev = {}

        def ctrl(t, gt, ev):
            if t < 1.0:
                return (0, 0, 0, 1, 0)
            if ev.get("cross_t") is None:
                if gt["x"] >= 1.0:
                    ev.update(cross_t=t, cross_x=gt["x"], cross_y=gt["y"],
                              cross_yaw=gt["yaw"])
                    return (0, 0, 0, 1, 0)
                return (0.4, 0, 0, 1, 0)
            return (0, 0, 0, 1, 0)
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        crossed = "cross_t" in ev
        t2m = (ev["cross_t"] - 1.0) if crossed else float("nan")
        y_at = abs(ev["cross_y"]) if crossed else float("nan")
        head = abs(ev["cross_yaw"]) if crossed else float("nan")
        if crossed:
            after = [r for r in rows if r["t"] >= ev["cross_t"]]
            overshoot = (max(r["x"] for r in after) - ev["cross_x"]) if after else 0.0
            settled = _settles(rows, ev["cross_t"], ev["cross_t"] + 2.5)
        else:
            overshoot = float("nan")
            settled = False
        success = (not fell) and crossed and t2m <= 8.0 and settled \
            and y_at < 0.15 and overshoot < 0.30
        return dict(success=success,
                    metrics=dict(time_to_1m=t2m, y_at_cross=y_at,
                                 overshoot=overshoot, heading_dev=head),
                    headline=(f"t={t2m:.1f}s" if crossed else "no-cross"))
    return build, evaluate


def scen_backward_1m():
    def build():
        ev = {}

        def ctrl(t, gt, ev):
            if t < 1.0:
                return (0, 0, 0, 1, 0)
            if ev.get("cross_t") is None and t < 11.0:
                if gt["x"] <= -1.0:
                    ev.update(cross_t=t, cross_y=gt["y"])
                    return (0, 0, 0, 1, 0)
                return (-0.3, 0, 0, 1, 0)
            return (0, 0, 0, 1, 0)
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        crossed = "cross_t" in ev
        t = (ev["cross_t"] - 1.0) if crossed else float("nan")
        y_at = abs(ev["cross_y"]) if crossed else float("nan")
        settled = _settles(rows, ev["cross_t"], ev["cross_t"] + 2.5) if crossed else False
        success = (not fell) and crossed and y_at < 0.20 and settled
        return dict(success=success,
                    metrics=dict(time=t, y_drift=y_at),
                    headline=(f"t={t:.1f}s" if crossed else "no-cross"))
    return build, evaluate


def scen_sidestep(side):
    vy = 0.2 if side == "L" else -0.2

    def build():
        ev = {}

        def ctrl(t, gt, ev):
            if t < 1.0:
                return (0, 0, 0, 1, 0)
            if ev.get("cross_t") is None and t < 7.0:
                if abs(gt["y"]) >= 0.5:
                    ev.update(cross_t=t, cross_x=gt["x"])
                    return (0, 0, 0, 1, 0)
                return (0, vy, 0, 1, 0)
            return (0, 0, 0, 1, 0)
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        crossed = "cross_t" in ev
        t = (ev["cross_t"] - 1.0) if crossed else float("nan")
        reached = crossed and t <= 6.0
        x_drift = abs(ev["cross_x"]) if crossed else float("nan")
        settled = _settles(rows, ev["cross_t"], ev["cross_t"] + 2.5) if crossed else False
        success = (not fell) and reached and x_drift < 0.20 and settled
        return dict(success=success,
                    metrics=dict(time=t, x_drift=x_drift),
                    headline=(f"t={t:.1f}s" if crossed else "no-reach"))
    return build, evaluate


def scen_square_return():
    wps = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]

    def build():
        ev = {"wp": 0, "reached": []}

        def ctrl(t, gt, ev):
            if t < 1.0:
                return (0, 0, 0, 1, 0)
            i = ev["wp"]
            if i >= len(wps):
                return (0, 0, 0, 1, 0)
            tx, ty = wps[i]
            dx, dy = tx - gt["x"], ty - gt["y"]
            if math.hypot(dx, dy) < 0.15:
                ev["reached"].append(t)
                ev["wp"] = i + 1
                if ev["wp"] >= len(wps):
                    ev["done_t"] = t
                return (0, 0, 0, 1, 0)
            herr = _wrap(math.atan2(dy, dx) - gt["yaw"])
            if abs(herr) > 0.5:
                return (0, 0, float(np.clip(1.2 * herr, -0.8, 0.8)), 1, 0)
            return (0.35, 0, float(np.clip(1.5 * herr, -0.7, 0.7)), 1, 0)
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        reached = len(ev["reached"])
        final = rows[-1] if rows else None
        ret_err = math.hypot(final["x"], final["y"]) if final else float("nan")
        total_t = (ev["done_t"] - 1.0) if "done_t" in ev else float("nan")
        success = (not fell) and reached == len(wps) and ret_err < 0.25
        return dict(success=success,
                    metrics=dict(return_err=ret_err, total_time=total_t,
                                 waypoints=float(reached)),
                    headline=f"ret {ret_err*100:.0f}cm {reached}/4wp")
    return build, evaluate


def scen_circle_return():
    period = 2 * math.pi / 0.7                       # ~8.98 s

    def build():
        ev = {}

        def ctrl(t, gt, ev):
            if t < 1.0:
                return (0, 0, 0, 1, 0)
            if "start" not in ev:
                ev["start"] = (gt["x"], gt["y"])
            if t - 1.0 < period:
                return (0.35, 0, 0.7, 1, 0)
            return (0, 0, 0, 1, 0)
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        final = rows[-1] if rows else None
        sx, sy = ev.get("start", (0.0, 0.0))
        ret_err = math.hypot(final["x"] - sx, final["y"] - sy) if final else float("nan")
        circ = _win(rows, 1.0, 1.0 + period)
        if circ:
            cx = np.mean([r["x"] for r in circ])
            cy = np.mean([r["y"] for r in circ])
            radius = float(np.mean([math.hypot(r["x"] - cx, r["y"] - cy)
                                    for r in circ]))
        else:
            radius = float("nan")
        # radius gate: standing still trivially "returns to start" -- the
        # precision_v1 scorecard's only locomotion "pass" was exactly that.
        # Require the trajectory to actually sweep a circle (commanded
        # radius = v/w = 0.5 m; accept >= 0.3 m mean excursion).
        success = ((not fell) and ret_err < 0.35
                   and not math.isnan(radius) and radius >= 0.3)
        return dict(success=success,
                    metrics=dict(return_err=ret_err, radius=radius),
                    headline=f"ret {ret_err*100:.0f}cm r={radius:.2f}m")
    return build, evaluate


def scen_crouch_hold():
    def build():
        ev = {}

        def ctrl(t, gt, ev):
            if t < 1.0:
                return (0, 0, 0, 1, 0)
            if t < 4.0:
                return (0, 0, 0, 0.7, 0)
            return (0, 0, 0, 1, 0)
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        crouch_h = _mean(_win(rows, 2.0, 4.0), "height")   # last 2 s of crouch
        target = 0.7 * N
        herr = abs(crouch_h - target)
        recov = _mean(_win(rows, 6.0, 7.0), "height")      # last 1 s recovery
        success = (not fell) and herr <= 0.035 and recov >= 0.9 * N
        return dict(success=success,
                    metrics=dict(height_err=herr, recovery=recov,
                                 crouch_height=crouch_h),
                    headline=f"hErr {herr*1000:.0f}mm")
    return build, evaluate


def scen_crouch_leg(side):
    lift = _lift_of(side)

    def build():
        ev = {}

        def ctrl(t, gt, ev):
            if t < 1.0:
                return (0, 0, 0, 1, 0)
            if t < 4.0:
                return (0, 0, 0, 0.85, lift)
            return (0, 0, 0, 1, 0)
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        hold = _win(rows, 1.0, 4.0)
        contact = _frac(hold, _swing_key(side))
        herr = abs(_mean(hold, "height") - 0.85 * N)
        success = (not fell) and contact < 0.15
        return dict(success=success,
                    metrics=dict(lifted_contact=contact, height_err=herr),
                    headline=f"contact {contact*100:.1f}%")
    return build, evaluate


def scen_stand_10s():
    def build():
        ev = {}

        def ctrl(t, gt, ev):
            return (0, 0, 0, 1, 0)
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        if rows:
            x0, y0 = rows[0]["x"], rows[0]["y"]
            drift = max(math.hypot(r["x"] - x0, r["y"] - y0) for r in rows)
        else:
            drift = float("nan")
        success = (not fell) and drift < 0.15
        return dict(success=success,
                    metrics=dict(drift=drift, watts=shared["mean_watts"],
                                 wobble=shared["wobble_rms"]),
                    headline=f"drift {drift*100:.1f}cm")
    return build, evaluate


# name -> (episode_seconds, factory, is_locomotion)
def _registry():
    reg = {}
    for s in ("L", "R"):
        reg[f"balance_{s}"] = (12.0, scen_balance(s), False)
        reg[f"circle_air_{s}"] = (12.0, scen_circle_air(s), False)
        reg[f"sidestep_{s}"] = (12.0, scen_sidestep(s), True)
        reg[f"crouch_leg_{s}"] = (10.0, scen_crouch_leg(s), False)
    reg["line_1m"] = (14.0, scen_line_1m(), True)
    reg["backward_1m"] = (14.0, scen_backward_1m(), True)
    reg["square_return"] = (45.0, scen_square_return(), True)
    reg["circle_return"] = (16.0, scen_circle_return(), True)
    reg["crouch_hold"] = (10.0, scen_crouch_hold(), False)
    reg["stand_10s"] = (11.0, scen_stand_10s(), False)
    return reg


ORDER = ["balance_L", "balance_R", "circle_air_L", "circle_air_R",
         "line_1m", "backward_1m", "sidestep_L", "sidestep_R",
         "square_return", "circle_return", "crouch_hold",
         "crouch_leg_L", "crouch_leg_R", "stand_10s"]

# base-name -> (metric key, formatter) for the md headline column, formatted
# from the across-seed MEAN of that metric
HEADLINE = {
    "balance": ("lifted_contact", lambda v: f"contact {v*100:.1f}%"),
    "circle_air": ("foot_err", lambda v: f"trkErr {v*1000:.0f}mm"),
    "line_1m": ("time_to_1m", lambda v: f"t={v:.1f}s"),
    "backward_1m": ("time", lambda v: f"t={v:.1f}s"),
    "sidestep": ("time", lambda v: f"t={v:.1f}s"),
    "square_return": ("return_err", lambda v: f"ret {v*100:.0f}cm"),
    "circle_return": ("return_err", lambda v: f"ret {v*100:.0f}cm"),
    "crouch_hold": ("height_err", lambda v: f"hErr {v*1000:.0f}mm"),
    "crouch_leg": ("lifted_contact", lambda v: f"contact {v*100:.1f}%"),
    "stand_10s": ("drift", lambda v: f"drift {v*100:.1f}cm"),
}


def _headline(name, metrics):
    base = name.rsplit("_", 1)[0] if name.rsplit("_", 1)[-1] in ("L", "R") else name
    key, fmt = HEADLINE[base]
    v = metrics.get(key, {}).get("mean")
    return fmt(v) if v is not None else "n/a"


# =====================================================================
#  aggregation + output
# =====================================================================
def _agg(vals):
    a = np.array([v for v in vals if v is not None and not (isinstance(v, float)
                  and math.isnan(v))], dtype=float)
    if a.size == 0:
        return dict(mean=None, worst=None)
    return dict(mean=float(a.mean()), worst=float(a.max()))


def run_one(env, act, factory, seed, record, N, mass):
    build, evaluate = factory
    ctrl, ev = build()
    drv = Driver(env, act, seed, record)
    gt = drv.initial_gt()
    for step in range(env.max_steps):
        t = step * drv.dt
        gt, term = drv.step(ctrl(t, gt, ev))
        if term:
            break
    shared = drv.shared_metrics(mass)
    res = evaluate(drv.rows, ev, drv.fell, N, shared)
    res.update(fell=drv.fell, shared=shared, frames=drv.frames)
    return res


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run-name", required=True)
    p.add_argument("--xml", default=None)
    p.add_argument("--episodes", type=int, default=8)
    p.add_argument("--nominal", action="store_true")
    p.add_argument("--render", action="store_true")
    p.add_argument("--scenarios", default=None,
                   help="comma list filter (e.g. balance_L,line_1m)")
    args = p.parse_args()

    run_dir = os.path.join(RUNS, args.run_name)
    with open(os.path.join(run_dir, "config.json")) as f:
        cfg = json.load(f)
    xml = args.xml or cfg.get("xml_path") or DEFAULT_XML
    if not os.path.isabs(xml) and not os.path.exists(xml):
        # train_mjx.py stores the basename (the run may have trained on a
        # remote host) -- resolve it against the local sim/ directory
        xml = os.path.join(HERE, "..", os.path.basename(xml))

    reg = _registry()
    names = ORDER
    if args.scenarios:
        want = [s.strip() for s in args.scenarios.split(",") if s.strip()]
        names = [n for n in ORDER if n in want]
        unknown = [w for w in want if w not in reg]
        if unknown:
            print(f"warning: unknown scenarios ignored: {unknown}", file=sys.stderr)

    # one env per distinct episode length (payload recompiles the model)
    env_cache = {}

    def get_env(secs):
        if secs not in env_cache:
            env_cache[secs] = make_env(cfg, secs, args.nominal, xml)
        return env_cache[secs]

    obs_size = get_env(12.0).observation_space.shape[0]
    mass = float(get_env(12.0).model.body_mass.sum())
    N = float(get_env(12.0)._nominal_h)
    act = load_policy(run_dir, obs_size)

    cond = "nominal (no DR)" if args.nominal else \
        "hardware-claim (GoPro 154 g, 4 ms latency, 0.7 deg backlash, DR, IMU obs)"
    print(f"CPU precision referee: {args.run_name}  [{cond}]  "
          f"{args.episodes} seeds/scenario, plant {os.path.basename(xml)}\n")

    scorecard = {}
    all_shared = {k: [] for k in
                  ("wobble_rms", "mean_watts", "foot_slip")}
    loco_shared = {k: [] for k in ("cot", "symmetry")}
    tot_succ = tot_runs = tot_fall = 0
    scen_all_pass = 0
    md_rows = []

    for name in names:
        secs, factory, is_loco = reg[name]
        env = get_env(secs)
        results = [run_one(env, act, factory, seed=100 * i + 7,
                           record=(args.render and i == 0), N=N, mass=mass)
                   for i in range(args.episodes)]
        succ = sum(r["success"] for r in results)
        falls = sum(r["fell"] for r in results)
        tot_succ += succ
        tot_runs += args.episodes
        tot_fall += falls
        if succ == args.episodes:
            scen_all_pass += 1

        # per-metric mean/worst across seeds
        mkeys = results[0]["metrics"].keys()
        metrics = {k: _agg([r["metrics"][k] for r in results]) for k in mkeys}
        shared_keys = ("wobble_rms", "mean_watts", "mean_speed", "foot_slip",
                       "symmetry")
        sh = {k: _agg([r["shared"][k] for r in results]) for k in shared_keys}
        cot = _agg([r["shared"]["cot"] for r in results])
        sh["cot"] = cot

        for k in all_shared:
            all_shared[k].extend(r["shared"][k] for r in results)
        if is_loco:
            loco_shared["symmetry"].extend(r["shared"]["symmetry"] for r in results)
            loco_shared["cot"].extend(r["shared"]["cot"] for r in results
                                      if r["shared"]["cot"] is not None)

        scorecard[name] = dict(
            n=args.episodes, successes=int(succ), fall_count=int(falls),
            episode_seconds=secs, locomotion=is_loco,
            metrics=metrics, shared=sh)

        head = _headline(name, metrics)          # across-seed mean
        wob = sh["wobble_rms"]["mean"]
        wts = sh["mean_watts"]["mean"]
        md_rows.append((name, f"{succ}/{args.episodes}", head,
                        f"{wob:.2f}" if wob is not None else "-",
                        f"{wts:.1f}" if wts is not None else "-"))

        if args.render:
            frames = results[0]["frames"]
            if frames:
                import imageio
                gif = os.path.join(run_dir, f"prec_{name}.gif")
                imageio.mimsave(gif, frames, fps=20)
                print(f"  render -> {gif}")

    # summary shared aggregates
    def _m(vals):
        a = np.array([v for v in vals if v is not None
                      and not (isinstance(v, float) and math.isnan(v))], float)
        return float(a.mean()) if a.size else None

    summary = dict(
        conditions=cond, episodes=args.episodes, plant=os.path.basename(xml),
        scenarios_run=len(names),
        scenarios_all_pass=scen_all_pass,
        seed_pass=f"{tot_succ}/{tot_runs}",
        seed_pass_rate=round(tot_succ / max(tot_runs, 1), 3),
        fall_rate=round(tot_fall / max(tot_runs, 1), 3),
        wobble_rms_mean=_m(all_shared["wobble_rms"]),
        mean_watts=_m(all_shared["mean_watts"]),
        foot_slip_mean=_m(all_shared["foot_slip"]),
        cot_locomotion=_m(loco_shared["cot"]),
        symmetry_locomotion=_m(loco_shared["symmetry"]),
    )
    scorecard["summary"] = summary

    with open(os.path.join(run_dir, "scorecard.json"), "w") as f:
        json.dump(scorecard, f, indent=2)

    # --- human table ---
    lines = []
    lines.append(f"# Precision scorecard - {args.run_name}")
    lines.append(f"_{cond}; {args.episodes} seeds/scenario; "
                 f"plant {os.path.basename(xml)}_")
    lines.append("")
    lines.append(f"| {'scenario':14s} | pass | {'headline':18s} | wobble | watts |")
    lines.append(f"|{'-'*16}|------|{'-'*20}|--------|-------|")
    for nm, pv, hd, wb, wt in md_rows:
        lines.append(f"| {nm:14s} | {pv:4s} | {hd:18s} | {wb:>6s} | {wt:>5s} |")
    lines.append("")
    cot = summary["cot_locomotion"]
    overall = (f"**overall {tot_succ}/{tot_runs} seed-pass** "
               f"({scen_all_pass}/{len(names)} scenarios clean); "
               f"falls {summary['fall_rate']*100:.0f}%; "
               f"wobble {summary['wobble_rms_mean']:.2f} rad/s; "
               f"watts {summary['mean_watts']:.1f} W; "
               f"slip {summary['foot_slip_mean']*100:.1f} cm/s")
    if cot is not None:
        overall += f"; CoT {cot:.1f}"
    lines.append(overall)
    md = "\n".join(lines) + "\n"
    with open(os.path.join(run_dir, "scorecard.md"), "w") as f:
        f.write(md)
    print(md)


if __name__ == "__main__":
    main()
