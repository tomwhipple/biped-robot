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

`--sil` swaps the python policy for the REAL firmware control stack
(sim/sil/harness.py -> libctrl_sil): the same scenarios, seeds and plant, but
every tick goes sim -> calibrated servo ticks -> firmware obs assembler / gait
clock / MLP -> goal ticks -> sim.  Results land in scorecard_sil.{md,json} and
NEVER overwrite the python-path scorecard (see docs/sil-harness.md).

Run:
  JAX_PLATFORMS=cpu .venv/bin/python sim/mjx/eval_precision.py \
      --run-name mjx_prec_v1 [--episodes 8] [--nominal] [--render] \
      [--sil] [--scenarios balance_L,line_1m,...]
"""
import argparse
import inspect
import json
import math
import os
import pickle
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
RUNS = os.path.join(HERE, "..", "runs")
DEFAULT_XML = os.path.join(HERE, "..", "bimo_biped_v2_asbuilt.xml")
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
SIL_DIR = os.path.join(ROOT, "sim", "sil")

import jax
from brax.training.acme import running_statistics
from video_annot import caption
from brax.training.agents.ppo import networks as ppo_networks

from walker_env import BimoWalkerEnv

G = 9.81
MOV_FPS = 20          # referee reel playback rate (see reel_index.json)
_ENV_PARAMS = set(inspect.signature(BimoWalkerEnv.__init__).parameters)


# =====================================================================
#  policy + env loading (mirrors eval_ref.py)
# =====================================================================
def _hidden_sizes(net_params, fallback):
    """Derive MLP hidden sizes from checkpoint param shapes (brax names all
    layers hidden_i; the last one is the output layer). Robust across runs
    with different --precision network configurations."""
    try:
        layers = net_params["params"]
        ks = sorted((k for k in layers if k.startswith("hidden_")),
                    key=lambda k: int(k.split("_")[1]))
        return tuple(int(layers[k]["kernel"].shape[1]) for k in ks[:-1])
    except Exception:
        return fallback


def load_policy(run_dir, obs_size, act_size=8):
    with open(os.path.join(run_dir, "params.pkl"), "rb") as f:
        params = pickle.load(f)
    net = ppo_networks.make_ppo_networks(
        observation_size=obs_size, action_size=act_size,
        preprocess_observations_fn=running_statistics.normalize,
        policy_hidden_layer_sizes=_hidden_sizes(params[1], (128, 128)),
        value_hidden_layer_sizes=(_hidden_sizes(params[2], (256, 256))
                                  if len(params) > 2 else (256, 256)))
    make_policy = ppo_networks.make_inference_fn(net)
    policy = jax.jit(make_policy((params[0], params[1]), deterministic=True))
    rng = jax.random.PRNGKey(0)

    def act(obs):
        a, _ = policy(obs[None].astype(np.float32), rng)
        return np.asarray(a[0])
    return act


def make_env(cfg, episode_seconds, nominal, xml, extra=None, act_lag_hz=0.0):
    """Eval env: ext_cmd precision plant mirroring config.json construction,
    but pinned to the hardware-claim conditions (or --nominal clean plant).
    extra: per-scenario env overrides (e.g. recover_mix=1.0).
    act_lag_hz: PINNED referee-side servo lag pole (never inherited from the
    run's own training DR -- the referee models the measured servo, not the
    training distribution)."""
    kw = {k: v for k, v in cfg.items() if k in _ENV_PARAMS}
    kw.update(
        xml_path=xml, command_mode=True, ext_cmd=True,
        actuator_model="sts3215", imu_obs=True, getup=False, cmd_fixed=None,
        payload_mass=0.154, payload_max=None,
        episode_seconds=episode_seconds, render_mode="rgb_array",
        act_lag_hz=act_lag_hz,
    )
    # eval-time default: recovery machinery off unless a scenario asks;
    # training-time recover_mix in the config must not leak into e.g.
    # the balance scenarios (fallen resets would break every script)
    kw["recover_mix"] = 0.0
    if extra:
        kw.update(extra)
    if nominal:
        kw.update(domain_rand=False, latency_ms=0.0, latency_ms_max=None,
                  latency_jitter_ms=0.0, backlash_deg=0.0, backlash_deg_max=None)
    else:
        kw.update(domain_rand=True, latency_ms=4.0, latency_ms_max=None,
                  latency_jitter_ms=0.0, backlash_deg=0.7, backlash_deg_max=None)
    # hardware-claim pushes are pinned to the HISTORICAL standard (gentle 5 N
    # force shoves @1%) regardless of what the run trained with -- v7/v7b
    # trained with strong velocity kicks and inheriting those into the eval
    # made their scorecards incomparably harsher than v1-v6 (falls 88% under
    # inherited 1 m/s kicks vs 0% nominal, found 2026-07-22)
    kw.update(push_kick=False, push_force=5.0 if not nominal else 0.0,
              push_prob=0.01 if not nominal else 0.0)
    return BimoWalkerEnv(**kw)


# =====================================================================
#  SIL referee column (--sil): the real firmware stack in the act() slot
# =====================================================================
def _run(cmd, what, cwd=None, env=None):
    """Run a prerequisite build/export step; die loudly rather than half-run."""
    print(f"[sil] {what}: {' '.join(cmd)}")
    e = dict(os.environ)
    e.setdefault("JAX_PLATFORMS", "cpu")
    if env:
        e.update(env)
    p = subprocess.run(cmd, cwd=cwd, env=e, text=True,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if p.returncode != 0:
        tail = "\n".join((p.stdout or "").strip().splitlines()[-25:])
        raise SystemExit(
            f"[sil] {what} FAILED (exit {p.returncode}).  The SIL referee "
            f"cannot run; no scorecard_sil written.\n--- last output ---\n"
            f"{tail}")
    return p.stdout or ""


def sil_setup(run_name, run_dir, verbose=True):
    """Import the SIL harness with its prerequisites satisfied.

    (a) libctrl_sil missing        -> `make -C firmware/host sil`
    (b) <run>.silw missing/stale   -> tools/export_policy_weights.py --run <run>
    (c) cal/cal_nominal.json missing -> harness.write_cal_files()

    Returns (harness_module, SilLib, info_dict).  Any failure is a SystemExit
    with the build/export output attached -- never a partially-run referee."""
    if SIL_DIR not in sys.path:
        sys.path.insert(0, SIL_DIR)
    try:
        import harness as H                                   # noqa: E402
    except Exception as e:                                    # pragma: no cover
        raise SystemExit(f"[sil] cannot import sim/sil/harness.py: {e}")

    # (a) the shared library -------------------------------------------
    lib_path = H.find_lib()
    if lib_path is None:
        _run(["make", "-C", os.path.join(ROOT, "firmware", "host"), "sil"],
             "building libctrl_sil")
        lib_path = H.find_lib()
        if lib_path is None:
            raise SystemExit(
                "[sil] `make -C firmware/host sil` reported success but no "
                f"libctrl_sil landed in {', '.join(H.LIB_DIRS)} "
                "(set $SIL_LIB to point at it)")

    # (b) the exported weights ------------------------------------------
    weights = H.default_weights(run_name)
    params = os.path.join(run_dir, "params.pkl")
    if not os.path.exists(params):
        raise SystemExit(f"[sil] {params} missing -- nothing to export")
    stale = (not os.path.exists(weights)
             or os.path.getmtime(weights) < os.path.getmtime(params))
    if stale:
        why = "missing" if not os.path.exists(weights) else "older than params.pkl"
        if verbose:
            print(f"[sil] {os.path.basename(weights)} {why} -- exporting")
        # --no-golden on purpose: sim/sil/golden/*.json are COMMITTED vectors
        # for the harness's reference run (and the firmware ctests consume
        # them).  A referee pass must not silently re-point them at whatever
        # run it is scoring.
        _run([sys.executable,
              os.path.join(ROOT, "tools", "export_policy_weights.py"),
              "--run", run_name, "--no-golden"],
             f"exporting {run_name} weights", cwd=ROOT)
        if not os.path.exists(weights):
            raise SystemExit(f"[sil] export produced no {weights}")

    # (c) the nominal calibration ---------------------------------------
    cal_path = os.path.join(H.CAL_DIR, "cal_nominal.json")
    if not os.path.exists(cal_path):
        H.write_cal_files()

    lib = H.SilLib(lib_path, weights, cal_path, gait_hz=H.DEFAULT_GAIT_HZ)
    info = dict(lib=os.path.relpath(lib_path, ROOT),
                weights=os.path.relpath(weights, ROOT),
                cal=os.path.relpath(cal_path, ROOT),
                gait_hz=lib.gait_hz, spec=lib.spec_string(),
                in_order=lib.in_order, out_order=lib.out_order,
                vel_sign_magnitude=bool(lib.vel_sign_magnitude))
    if verbose:
        print(f"[sil] lib {info['lib']}  weights {info['weights']}  "
              f"cal {info['cal']}")
        print(f"[sil] spec {info['spec']}")
        print(f"[sil] probe: in={info['in_order']} out={info['out_order']} "
              f"vel_sign_magnitude={info['vel_sign_magnitude']}")
    return H, lib, info


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
        # Green floor marker at the episode's start position (harmless in every
        # scenario; the visible "did it return?" cue for square_return /
        # circle_return / turn_180 / stand). Purely a render decoration.
        env.mark_xy = (float(env.data.qpos[0]), float(env.data.qpos[1]))
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
        # Feed the TRAINING-correct stacking (SIL harness finding,
        # 2026-07-30): env.step() already returned [f_t, f_{t-1}, f_{t-2}]
        # and pushed f_t, so re-reading env._obs() here prepends a fresh
        # duplicate of f_t onto a ring that already holds it -- the policy
        # got [f_t, f_t, f_{t-1}] (plus a REDRAWN IMU-noise realization).
        # Firmware action divergence vs brax: 0.48 through the old path,
        # 4.9e-3 (tick quantization only) through the training stacking.
        # The new command for this tick belongs in the head frame only, so
        # patch the cmd channels in place instead of rebuilding the frame.
        obs = self.obs.copy()
        nc = getattr(env, "_ncmd", 0)
        if nc:
            fw = env.obs_frame
            obs[fw - nc:fw] = env._cmd
        self.obs_fed = obs             # exposed for the SIL parity tests
        a = self.act(obs)
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
            foot_err=float(info["foot_err"]),
            foot_clear=float(info.get("foot_clear", 0.0)),
            foot_dx=float(info.get("foot_dx", float("nan"))),
            foot_dz=float(info.get("foot_dz", float("nan"))),
            cmd_lift=float(info.get("cmd_lift", 0.0)),
            com_stance=float(info.get("com_stance", float("nan"))),
            recovered=float(info.get("recovered", 1.0))))
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
        # user criterion (2026-07-18): the lifted foot must be >= 3 cm off
        # the ground -- grade the fraction of the window with real clearance
        clear = (np.mean([r["foot_clear"] >= 0.03 for r in judge_win])
                 if judge_win else float("nan"))
        drift = _drift_rms(lift_win)
        wob = _wob(lift_win)
        watts = _mean(lift_win, "watts")
        # knee-articulation quality (user 2026-07-23): planar CoM offset
        # from the stance sole -- a knee-flexion lift keeps this small, a
        # stuck-out leg lurches the CG forward. Continuous metric only for
        # now (the training kernel targets it; threshold once we see the
        # achievable range).
        com_off = _mean(judge_win, "com_stance")
        # sensible minimums (user 2026-07-19): balancing also means STAYING
        # put -- a hop-around that keeps the foot up is not a balance
        success = ((not fell) and contact < 0.05
                   and not math.isnan(clear) and clear >= 0.90
                   and not math.isnan(drift) and drift < 0.10)
        return dict(success=success,
                    metrics=dict(lifted_contact=contact, clear_frac=clear,
                                 drift_rms=drift, wobble=wob, watts=watts,
                                 com_off=com_off),
                    headline=f"clear {clear*100:.0f}%")
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
        clear = (np.mean([r["foot_clear"] >= 0.03 for r in circ])
                 if circ else float("nan"))
        # sensible minimums (user 2026-07-19): the foot must actually TRACE
        # a circle -- >= 3 cm mean radius around its own centroid AND at
        # least one full sweep. (precision_v4 'passed' with 12-19 cm
        # tracking error: lifted and hovering, barely circling.)
        r_traced = float("nan")
        sweep = 0.0
        pts = [(r["foot_dx"], r["foot_dz"]) for r in circ
               if not math.isnan(r["foot_dx"])]
        if len(pts) >= 10:
            p = np.asarray(pts)
            c = p.mean(axis=0)
            d = p - c
            r_traced = float(np.hypot(d[:, 0], d[:, 1]).mean())
            ang = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
            sweep = float(abs(ang[-1] - ang[0]))
        success = ((not fell) and touchdown < 0.10
                   and not math.isnan(clear) and clear >= 0.80
                   and not math.isnan(r_traced) and r_traced >= 0.03
                   and sweep >= 2 * math.pi)
        return dict(success=success,
                    metrics=dict(foot_err=foot_err, touchdown_frac=touchdown,
                                 lifted_contact=touchdown, clear_frac=clear,
                                 traced_radius=r_traced,
                                 sweep_turns=sweep / (2 * math.pi)),
                    headline=f"r={r_traced*100:.1f}cm" if not math.isnan(
                        r_traced) else "n/a")
    return build, evaluate


def scen_line_1m():
    def build():
        ev = {}

        def ctrl(t, gt, ev):
            if t < 1.0:
                return (0, 0, 0, 1, 0)
            # spawn-relative (line_rough, 2026-07-30): the mosaic spawns
            # anywhere, so the finish line is start+1 m and lateral drift is
            # measured from the start lane. Identical numbers on the flat
            # plane, where the spawn IS the origin.
            if "x0" not in ev:
                ev.update(x0=gt["x"], y0=gt["y"])
            if ev.get("cross_t") is None:
                if gt["x"] - ev["x0"] >= 1.0:
                    ev.update(cross_t=t, cross_x=gt["x"], cross_y=gt["y"],
                              cross_yaw=gt["yaw"])
                    return (0, 0, 0, 1, 0)
                return (0.4, 0, 0, 1, 0)
            return (0, 0, 0, 1, 0)
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        crossed = "cross_t" in ev
        t2m = (ev["cross_t"] - 1.0) if crossed else float("nan")
        y_at = (abs(ev["cross_y"] - ev.get("y0", 0.0))
                if crossed else float("nan"))
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


def scen_goal_home():
    """Goal layer (task #15, 2026-07-29): drive the circle_return arc, then
    HOME closed-loop -- an outer P-controller reads ground truth each step
    and steers the velocity commands back to the start point, switching to
    an omnidirectional body-frame creep inside 0.3 m (bearing-chasing at
    close range orbits the goal) and latching stand under 5 cm. The policy
    must track the creep band (0.05-0.25 m/s -- trained since loco_v6creep;
    loco_v5t had never seen commands below 0.3 and generalization alone
    plateaued at 6/8 with 1-3 cm misses). Gate: return < 10 cm, no fall.
    Deploy caveat recorded in docs: this uses ground-truth position; the
    hardware needs an odometry source."""
    period = 2 * math.pi / 0.7

    def build():
        ev = {}

        def ctrl(t, gt, ev):
            if t < 1.0:
                return (0, 0, 0, 1, 0)
            if "start" not in ev:
                ev["start"] = (gt["x"], gt["y"])
            if t - 1.0 < period:
                return (0.35, 0, 0.7, 1, 0)
            dx = ev["start"][0] - gt["x"]
            dy = ev["start"][1] - gt["y"]
            dist = math.hypot(dx, dy)
            if dist < 0.05:
                ev["homed"] = True
            if ev.get("homed"):
                return (0, 0, 0, 1, 0)
            if dist < 0.30:
                c, s = math.cos(gt["yaw"]), math.sin(gt["yaw"])
                bx = c * dx + s * dy
                by = -s * dx + c * dy
                k = min(max(0.8 * dist, 0.08), 0.25) / max(dist, 1e-6)
                return (k * bx, k * by, 0, 1, 0)
            bearing = math.atan2(dy, dx)
            herr = _wrap(bearing - gt["yaw"])
            wz = max(-0.6, min(0.6, 1.2 * herr))
            vx = (min(max(0.9 * dist, 0.12), 0.4)
                  * max(0.0, math.cos(herr)))
            return (vx, 0, wz, 1, 0)
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        final = rows[-1] if rows else None
        sx, sy = ev.get("start", (0.0, 0.0))
        ret_err = (math.hypot(final["x"] - sx, final["y"] - sy)
                   if final else float("nan"))
        circ = _win(rows, 1.0, 1.0 + period)
        radius = (float(np.mean([math.hypot(r["x"] - np.mean([q["x"] for q in circ]),
                                            r["y"] - np.mean([q["y"] for q in circ]))
                                 for r in circ])) if circ else float("nan"))
        success = ((not fell) and ret_err < 0.10
                   and not math.isnan(radius) and radius >= 0.3)
        return dict(success=success,
                    metrics=dict(return_err=ret_err, radius=radius),
                    headline=f"home {ret_err*100:.0f}cm")
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


def scen_recover_fallen(deadline=6.0):
    """Start fallen (env recover_mix=1.0; start pose set per-scenario via
    ENV_EXTRA -- ragdoll for the full claim, sit for the curriculum stage);
    command is stand. Success = first stand within the deadline AND still
    tall at the end."""
    def build():
        ev = {}

        def ctrl(t, gt, ev):
            return (0, 0, 0, 1, 0)
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        t_up = next((r["t"] for r in rows if r["recovered"] > 0.5), None)
        hold = _win(rows, 9.0, 12.0)
        held = _mean(hold, "height") if hold else float("nan")
        success = bool((not fell) and t_up is not None and t_up <= deadline
                       and hold and held >= 0.85 * N)
        return dict(success=success,
                    metrics=dict(
                        time_to_stand=(t_up if t_up is not None
                                       else float("nan")),
                        end_height=held),
                    headline=(f"up in {t_up:.1f}s" if t_up is not None
                              else "no-stand"))
    return build, evaluate


def scen_march():
    """March in place: alternating leg lifts every 0.7 s with a height bob
    (knee-articulation exercise, user 2026-07-20). Success: >= 3 clean
    lifted intervals PER LEG (clearance >= 3 cm), no fall, stays put."""
    def build():
        ev = {}

        def ctrl(t, gt, ev):
            if t < 1.0:
                return (0, 0, 0, 1, 0)
            phase = (t - 1.0) % 1.4
            lift = -1 if phase < 0.7 else 1
            bob = 0.03 * abs(math.sin(math.pi * (phase % 0.7) / 0.7))
            return (0, 0, 0, 1, lift, 0, bob)
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        win = _win(rows, 1.0, 9.4)                 # 6 full L/R cycles
        drift = _drift_rms(win)
        # count clean lifted intervals per side: consecutive stretches of
        # >= 0.3 s with the commanded swing foot clear >= 3 cm
        clean = {"L": 0, "R": 0}
        run_len = 0
        prev_side = None
        for r in win:
            side = "L" if r["cmd_lift"] < -0.5 else (
                "R" if r["cmd_lift"] > 0.5 else None)
            good = side is not None and r["foot_clear"] >= 0.03
            if good and side == prev_side:
                run_len += 1
            else:
                if prev_side and run_len >= 5:      # 5 rows ~ 0.3 s
                    clean[prev_side] += 1
                run_len = 1 if good else 0
            prev_side = side if good else None
        if prev_side and run_len >= 5:
            clean[prev_side] += 1
        success = ((not fell) and clean["L"] >= 3 and clean["R"] >= 3
                   and not math.isnan(drift) and drift < 0.15)
        return dict(success=success,
                    metrics=dict(clean_lifts_l=clean["L"],
                                 clean_lifts_r=clean["R"], drift_rms=drift),
                    headline=f"lifts {clean['L']}L/{clean['R']}R")
    return build, evaluate


def scen_march_knee(settle=1.0):
    """KNEE-HIGH march in place (user 2026-08-01): alternating exaggerated
    knee lifts, the swing sole raised toward the height of the OPPOSITE KNEE
    (env.march_clear -- measured on the plant: 10.4 cm on bimo_biped_v3yaw,
    1.7x the 6 cm nominal lift_height), zero net translation.

    The command script is the training command pattern, replayed: vx/vy/wz
    pinned to 0, the lift channel alternating on the env's own march clock
    (the pinned march_hz cadence when the run trained with one -- it arrives
    through config.json -- else the gait clock, same left-swings-first
    convention as w_feet_phase and the training-time march_mix draw), and c6
    raising the swing-foot target from lift_height up to knee height over
    each swing. No new command channel -- the obs contract is untouched,
    which is the whole point of the encoding.

    Success (user criteria): >= 6 alternating lifts whose peak swing-sole
    clearance exceeds 60% of the knee-high target, total XY drift < 15 cm,
    no fall. Headline: MEDIAN peak clearance across the counted swings."""
    def build():
        ev = {}

        def ctrl(t, gt, ev):
            env = ev["_env"]
            if t < settle:
                return (0, 0, 0, 1, 0)
            # Replay the env's OWN march evolution, whichever clock the run
            # trained on -- march_hz comes through config.json into the eval
            # env, so a run trained at a pinned cadence is graded at that
            # cadence rather than at the referee's guess.
            if getattr(env, "march_hz", 0.0) > 0.0:
                # integer-exact, exactly as the env computes it (the flip
                # must not straddle a float zero crossing -- see env)
                n = env._step_i % env._march_period
                left = n < env._march_half
                ph = 2 * math.pi * n / env._march_period
            else:
                ph = (env._gait_phase if env.gait_clock
                      else 2 * math.pi * 0.7 * (t - settle))
                left = math.sin(ph) >= 0
            lift = -1 if left else 1
            dz = env._march_dz * abs(math.sin(ph))
            return (0, 0, 0, 1, lift, 0, dz)
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        env = ev["_env"]
        tgt = float(getattr(env, "march_clear", 0.0))
        win = _win(rows, settle, 1e9)
        # segment into maximal runs of a constant COMMANDED lift side; each
        # run is one swing, scored by its peak achieved sole clearance
        swings = []          # (side, peak_clear)
        side = None
        peak = 0.0
        for r in win:
            s = "L" if r["cmd_lift"] < -0.5 else (
                "R" if r["cmd_lift"] > 0.5 else None)
            if s != side:
                if side is not None:
                    swings.append((side, peak))
                side, peak = s, 0.0
            if s is not None:
                peak = max(peak, r["foot_clear"])
        if side is not None:
            swings.append((side, peak))
        swings = [(s, p) for s, p in swings if s is not None]
        # the window always ends mid-swing -- drop that partial one; and if
        # the episode ended in a FALL, drop the swing it fell during too:
        # a toppling robot's swing sole sails past knee height on the way
        # over (2x the target, measured), which would otherwise turn the
        # headline into a fall artifact. Neither trim can rescue a pass:
        # 10 s is ~20 swings at a pinned 1 Hz cadence (~30 on the gait
        # clock) against a bar of 6.
        if swings:
            swings = swings[:-1]
        if fell and swings:
            swings = swings[:-1]
        # count only swings that ALTERNATE (a run of same-side swings would
        # be a segmentation artifact, not a march) and clear 60% of target
        thresh = 0.60 * tgt
        good = []
        prev = None
        for s, p in swings:
            if p >= thresh and s != prev:
                good.append(p)
            prev = s
        n_good = len(good)
        peaks = [p for _, p in swings]
        med_peak = float(np.median(good)) if good else (
            float(np.median(peaks)) if peaks else float("nan"))
        # total XY drift: max excursion from where the march started
        ref = next(((r["x"], r["y"]) for r in rows if r["t"] >= settle),
                   (rows[0]["x"], rows[0]["y"]) if rows else (0.0, 0.0))
        drift = (max(math.hypot(r["x"] - ref[0], r["y"] - ref[1])
                     for r in win) if win else float("nan"))
        success = bool((not fell) and n_good >= 6
                       and not math.isnan(drift) and drift < 0.15)
        return dict(success=success,
                    metrics=dict(knee_lifts=n_good, swings=len(swings),
                                 peak_clear_med=med_peak,
                                 peak_clear_max=(max(peaks) if peaks
                                                 else float("nan")),
                                 knee_target=tgt,
                                 clear_frac_of_knee=(med_peak / tgt
                                                     if tgt > 0
                                                     else float("nan")),
                                 drift=drift,
                                 com_off=_mean(win, "com_stance")),
                    headline=(f"peak {med_peak*100:.1f}cm"
                              if not math.isnan(med_peak) else "n/a"))
    return build, evaluate


def scen_sway():
    """Lateral sway: track vy = 0.12 sin(2 pi t / 1.6) for 6 s with both
    feet planted-ish (hip-roll exercise). Success: lateral pelvis
    oscillation amplitude >= 2 cm, net drift < 0.15 m, no fall."""
    def build():
        ev = {}

        def ctrl(t, gt, ev):
            if t < 1.0:
                return (0, 0, 0, 1, 0)
            if t < 7.0:
                vy = 0.12 * math.sin(2 * math.pi * (t - 1.0) / 1.6)
                return (0, vy, 0, 1, 0)
            return (0, 0, 0, 1, 0)
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        win = _win(rows, 1.0, 7.0)
        ys = [r["y"] for r in win]
        amp = (max(ys) - min(ys)) / 2 if ys else float("nan")
        drift = abs(ys[-1] - ys[0]) if ys else float("nan")
        success = ((not fell) and not math.isnan(amp) and amp >= 0.02
                   and drift < 0.15)
        return dict(success=success,
                    metrics=dict(sway_amp=amp, net_drift=drift),
                    headline=f"amp {amp*100:.1f}cm")
    return build, evaluate


def _handover_setup(env, drv, settle_s=1.5):
    """Recreate the BENCH state, then hand the plant to the policy (#29).

    The armed ground sessions start from a rigid bench hold: every joint at
    its kJointDefault zero (the `bench` CLI's servo hold), fully settled, and
    the policy takes the bus at t=0 with NO history of its own actions -- the
    firmware fills the history ring with the very first frame it assembles
    (obs::History::fill, ctrl_task.cpp / sil_lib.cpp). The standard referee
    episodes never visit this regime: reset() adds joint noise and the first
    second runs UNDER the policy. Two on-robot falls within seconds of the
    handover (2026-08-31) are the reason this exists.

    Runs after Driver's env.reset() (so the per-episode DR draws are the
    normal ones) and before the first policy tick:
      1. pin the exact bench pose (joints = kJointDefault zeros, zero vel),
      2. settle under the bench's rigid servo hold -- the same per-substep
         PD-with-torque-envelope that step() applies, target pinned at the
         default pose (mirrors the reset() settle for staged getup poses),
      3. prime the referee bookkeeping and the obs history from the SETTLED
         frame, exactly the way g_hist.fill primes the firmware's.
    Draws no RNG in the settle, so the per-seed DR draws stay untouched.
    """
    import mujoco
    d = env.data
    d.qpos[:] = env.model.qpos0
    d.qpos[env._jqpos] = env._default          # kJointDefault zeros
    d.qvel[:] = 0.0
    d.ctrl[:] = env._default
    mujoco.mj_forward(env.model, d)
    n = int(round(settle_s / env.sim_dt))
    if env._servo is not None:
        kp, kd, stall, w0 = env._servo         # this episode's DR-scaled servo
        for _ in range(n):
            q = d.qpos[env._jqpos]
            qd = d.qvel[env._jqvel]
            cap = stall * np.clip(1.0 - np.abs(qd) / w0, 0.0, 1.0)
            err = env._default - q
            if env._lash_rad > 0.0:
                err = np.sign(err) * np.maximum(
                    np.abs(err) - 0.5 * env._lash_rad, 0.0)
            d.qfrc_applied[env._jqvel] = np.clip(kp * err - kd * qd, -cap, cap)
            mujoco.mj_step(env.model, d)
        d.qfrc_applied[env._jqvel] = 0.0
    else:
        for _ in range(n):                     # ideal actuators read data.ctrl
            mujoco.mj_step(env.model, d)
    mujoco.mj_forward(env.model, d)
    # the handover tick: stand command in the frame the ring is filled with
    # (the benched link commands nothing -- the watchdog's defaults are stand)
    env.set_command(0, 0, 0, 1, 0)
    env._prev_action[:] = 0.0
    env._best_h = float(d.qpos[2])
    env._last_target = env._default.copy()
    env._ctrl_buf = [env._default.copy() for _ in range(env.action_latency + 1)]
    if env.obs_hist_len > 1:
        # firmware priming, mirrored: ONE frame, copied into every slot
        # (g_hist.fill + build feeds the policy [f0, f0, f0])
        env._obs_hist = []
        first = env._obs()
        env._obs_hist = [first.copy() for _ in range(env.obs_hist_len - 1)]
        drv.obs = np.concatenate([first] + list(env._obs_hist))
    else:
        drv.obs = env._obs()
    # referee bookkeeping restarts at the settled pose
    env.mark_xy = (float(d.qpos[0]), float(d.qpos[1]))
    drv._prev_sole = [d.geom_xpos[g][:2].copy() for g in env._sole_gids]


def scen_handover_stand():
    """Bench->policy handover, judged like stand_10s (issue #29): 10 s of
    stand command from a cold takeover -- rigid bench pose, freshly-filled
    history, no settle under the policy. Success: no fall, drift < 0.15 m.
    --scenarios-only; never part of the default suite (see EXTRA_SCENARIOS)."""
    def build():
        ev = {"_setup": _handover_setup}

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


def scen_stand_off():
    """Torque-off standing (user 2026-07-23: 'stand still with the servos
    powered off to conserve power'). Settle 1.5 s powered, then RELEASE servo
    torque (set_torque_enabled(False)) for the rest of the episode. Grade:
    still standing at the end (height/up_z) AND drift < 0.10 m from where the
    torque was cut. Continuous metrics contrast powered vs off wobble and
    confirm the power story (watts_off ~ 0). The passive resistance while off
    is env.off_frictionloss -- an ESTIMATE of unpowered STS3215 backdrive
    friction (see the sensitivity sweep in the report)."""
    cut_t = 1.5

    def build():
        ev = {}

        def ctrl(t, gt, ev):
            env = ev["_env"]
            if t >= cut_t and env._torque_on:
                env.set_torque_enabled(False)      # release: limp servos
            return (0, 0, 0, 1, 0)
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        powered = _win(rows, 0.0, cut_t)
        off = [r for r in rows if r["t"] >= cut_t]
        # reference = torso planar position at the instant torque was cut
        ref = next(((r["x"], r["y"]) for r in rows if r["t"] >= cut_t),
                   (rows[0]["x"], rows[0]["y"]) if rows else (0.0, 0.0))
        drift = (max(math.hypot(r["x"] - ref[0], r["y"] - ref[1]) for r in off)
                 if off else float("nan"))
        t_end = rows[-1]["t"] if rows else 0.0
        end_win = _win(rows, t_end - 1.0, 1e9)          # last 1 s
        end_h = _mean(end_win, "height")
        end_up = _mean(end_win, "up_z")
        standing = (not fell) and not math.isnan(end_h) and end_h >= 0.85 * N \
            and not math.isnan(end_up) and end_up >= 0.9
        success = bool(standing and not math.isnan(drift) and drift < 0.10)
        return dict(success=success,
                    metrics=dict(drift=drift, end_height=end_h, end_up=end_up,
                                 wob_powered=_wob(powered), wob_off=_wob(off),
                                 watts_off=_mean(off, "watts")),
                    headline=f"drift {drift*100:.1f}cm" if not math.isnan(drift)
                    else "n/a")
    return build, evaluate


def scen_turn_180():
    """Turn in place to face backward, then hold (user 2026-07-23: 'work on
    turning to face a different direction'). No hip-yaw joint exists, so
    this grades friction-pivot stepping: commanded wz=0.7 for pi/0.7 s
    (exactly 180 deg), then stand. Minimums: final facing within 15 deg of
    reversed AND stays in place (max 0.3 m excursion) AND no fall."""
    w_cmd = 0.7
    turn_t = math.pi / w_cmd                        # ~4.49 s

    def build():
        ev = {}

        def ctrl(t, gt, ev):
            if t < 1.0:
                return (0, 0, 0, 1, 0)
            if "yaw0" not in ev:
                ev["yaw0"] = gt["yaw"]
                ev["x0"], ev["y0"] = gt["x"], gt["y"]
            if t - 1.0 < turn_t:
                return (0, 0, w_cmd, 1, 0)
            return (0, 0, 0, 1, 0)
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        final = rows[-1] if rows else None
        yaw0 = ev.get("yaw0", 0.0)
        x0, y0 = ev.get("x0", 0.0), ev.get("y0", 0.0)
        turned = _wrap(final["yaw"] - yaw0) if final else float("nan")
        head_err = abs(_wrap(turned - math.pi)) if final else float("nan")
        after = _win(rows, 1.0, 1e9)
        exc = (max(math.hypot(r["x"] - x0, r["y"] - y0) for r in after)
               if after else float("nan"))
        success = ((not fell) and not math.isnan(head_err)
                   and head_err <= math.radians(15.0) and exc <= 0.3)
        return dict(success=success,
                    metrics=dict(head_err_deg=math.degrees(head_err)
                                 if not math.isnan(head_err) else None,
                                 turned_deg=math.degrees(turned)
                                 if not math.isnan(turned) else None,
                                 excursion=exc),
                    headline=(f"hErr {math.degrees(head_err):.0f}deg "
                              f"exc {exc*100:.0f}cm"
                              if not math.isnan(head_err) else "n/a"))
    return build, evaluate



# --- dynamic scenarios (2026-08-06) -----------------------------------------
# The eleven original scenarios are quasi-static: a command is set and held,
# and nothing argues back. These three ask the policy to REACT -- to a shove,
# to a changing speed command, and to a target that keeps moving. They are the
# cheap third of the dynamic-scenario plan (user 2026-08-06); the ball and the
# 1v1 need real env work, these need none.

def scen_push_gauntlet():
    """Walk a straight line while being shoved at random gait phase.

    push_prob/kick_range come from ENV_EXTRA, so the shoves land wherever they
    land -- that is the point. Averaged over 8 seeds this grades whether the
    IMU-driven recovery actually holds a heading under disturbance, which
    --push-prob as a training knob never reported on.
    """
    def build():
        ev = {}

        def ctrl(t, gt, ev):
            if t < 1.0:
                return (0, 0, 0, 1, 0)
            if "x0" not in ev:
                ev.update(x0=gt["x"], y0=gt["y"], yaw0=gt["yaw"])
            return (0.4, 0, 0, 1, 0)
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        after = _win(rows, 1.0, 1e9)
        if not after or "x0" not in ev:
            return dict(success=False, metrics={}, headline="n/a")
        dist = math.hypot(after[-1]["x"] - ev["x0"], after[-1]["y"] - ev["y0"])
        lateral = max(abs(r["y"] - ev["y0"]) for r in after)
        head_err = abs(_wrap(after[-1]["yaw"] - ev["yaw0"]))
        # survived, kept going, and did not get knocked off the lane
        success = ((not fell) and dist >= 2.0 and lateral <= 0.5
                   and head_err <= math.radians(35.0))
        return dict(success=success,
                    metrics=dict(distance=dist, lateral=lateral,
                                 head_err_deg=math.degrees(head_err)),
                    headline=f"d={dist:.1f}m lat {lateral*100:.0f}cm")
    return build, evaluate


def scen_speed_ladder():
    """Commanded speed climbs 0.1 -> 1.2 m/s in rungs; grade the tracking.

    A single held speed never shows whether the gait can RE-ORGANISE -- the
    transitions are where a two-beat gait either restructures or falls apart
    (cf. periodic reward composition, Siekmann et al. 2021). Error is measured
    only in the back half of each rung, after the transient.
    """
    RUNGS = (0.1, 0.3, 0.6, 0.9, 1.2)
    HOLD = 4.0
    SETTLE = 1.0

    def build():
        ev = {"rungs": RUNGS, "hold": HOLD, "settle": SETTLE}

        def ctrl(t, gt, ev):
            if t < SETTLE:
                return (0, 0, 0, 1, 0)
            i = int((t - SETTLE) // HOLD)
            if i >= len(RUNGS):
                return (0, 0, 0, 1, 0)
            return (RUNGS[i], 0, 0, 1, 0)
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        errs, per_rung = [], []
        for i, v in enumerate(RUNGS):
            t0 = SETTLE + i * HOLD
            win = _win(rows, t0 + HOLD * 0.5, t0 + HOLD)   # back half only
            if not win:
                continue
            got = sum(r["planar"] for r in win) / len(win)
            per_rung.append(got)
            errs.append(abs(got - v))
        if not errs:
            return dict(success=False, metrics={}, headline="n/a")
        mae = sum(errs) / len(errs)
        top = per_rung[-1] if per_rung else float("nan")
        success = (not fell) and mae <= 0.20
        return dict(success=success,
                    metrics=dict(speed_mae=mae, top_speed=top),
                    headline=f"mae {mae:.2f}m/s top {top:.2f}")
    return build, evaluate


def scen_pursuit():
    """Chase a target that keeps moving -- continuous heading re-planning.

    goal_home is a fixed point; this one runs away. The outer loop here is
    deliberately dumb (proportional heading + distance) so the score reflects
    the POLICY's turn-while-walking authority, not a clever planner.
    """
    SETTLE = 1.0
    SPEED = 0.18          # target's own speed, m/s -- catchable, not trivial

    def _target(t):
        # a lazy arc away from the spawn: forward, then curving left
        tt = max(0.0, t - SETTLE)
        r = 1.2
        th = SPEED * tt / r
        return (r * math.sin(th), r * (1.0 - math.cos(th)))

    def build():
        ev = {}

        def ctrl(t, gt, ev):
            if t < SETTLE:
                return (0, 0, 0, 1, 0)
            if "x0" not in ev:
                ev.update(x0=gt["x"], y0=gt["y"], yaw0=gt["yaw"])
            tx, ty = _target(t)
            # target in the spawn frame -> world
            gx = ev["x0"] + tx * math.cos(ev["yaw0"]) - ty * math.sin(ev["yaw0"])
            gy = ev["y0"] + tx * math.sin(ev["yaw0"]) + ty * math.cos(ev["yaw0"])
            dx, dy = gx - gt["x"], gy - gt["y"]
            dist = math.hypot(dx, dy)
            bearing = _wrap(math.atan2(dy, dx) - gt["yaw"])
            ev.setdefault("d", []).append(dist)
            wz = max(-1.0, min(1.0, 1.5 * bearing))
            # slow down when badly mis-aimed, so it turns instead of arcing wide
            vx = 0.0 if abs(bearing) > 1.2 else max(0.0, min(0.9, 1.2 * dist))
            return (vx, 0, wz, 1, 0)
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        d = ev.get("d") or []
        if not d:
            return dict(success=False, metrics={}, headline="n/a")
        tail = d[-max(1, len(d) // 4):]          # last quarter of the chase
        final = sum(tail) / len(tail)
        best = min(d)
        success = (not fell) and final <= 0.40
        return dict(success=success,
                    metrics=dict(final_gap=final, best_gap=best),
                    headline=f"gap {final*100:.0f}cm best {best*100:.0f}cm")
    return build, evaluate


def scen_reversal():
    """Command latency, the responsiveness headline (user 2026-08-06: the
    demo sentence is 'responds to a reversal in X s without a stumble').
    Walk forward 0.5, snap the command to backward -0.3, then to stop; each
    latency is the time from the flip until the heading-frame velocity stays
    inside the tracking band for 0.5 s continuous."""
    V_FWD, V_BACK = 0.5, -0.3
    SETTLE, T_FWD, T_BACK, T_STOP = 1.0, 4.0, 4.0, 3.0

    def build():
        ev = {}

        def ctrl(t, gt, ev):
            if t < SETTLE:
                return (0, 0, 0, 1, 0)
            ev.setdefault("yaw0", gt["yaw"])
            if t < SETTLE + T_FWD:
                return (V_FWD, 0, 0, 1, 0)
            if t < SETTLE + T_FWD + T_BACK:
                return (V_BACK, 0, 0, 1, 0)
            return (0, 0, 0, 1, 0)
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        if not rows:
            return dict(success=False, metrics={}, headline="n/a")
        yaw0 = ev.get("yaw0", rows[0]["yaw"])
        c, s = math.cos(yaw0), math.sin(yaw0)
        ts = [r["t"] for r in rows]
        px = [r["x"] * c + r["y"] * s for r in rows]
        # smoothed forward velocity along the initial heading (~0.3 s + 0.3 s
        # central window; stride-to-stride speed pulses would false-trip a
        # narrow band otherwise)
        k = 5
        v = []
        for i in range(len(rows)):
            j0, j1 = max(0, i - k), min(len(rows) - 1, i + k)
            dt = ts[j1] - ts[j0]
            v.append((px[j1] - px[j0]) / dt if dt > 1e-9 else 0.0)

        def lock(t_flip, t_end, target, tol):
            run = None
            for i, r in enumerate(rows):
                if r["t"] < t_flip:
                    continue
                if r["t"] > t_end:
                    break
                if abs(v[i] - target) <= tol:
                    if run is None:
                        run = r["t"]
                    if r["t"] - run >= 0.5 - 1e-9:
                        return run - t_flip
                else:
                    run = None
            return float("nan")

        t_go = lock(SETTLE, SETTLE + T_FWD, V_FWD, 0.12)
        t_rev = lock(SETTLE + T_FWD, SETTLE + T_FWD + T_BACK, V_BACK, 0.12)
        t_stop = lock(SETTLE + T_FWD + T_BACK,
                      SETTLE + T_FWD + T_BACK + T_STOP, 0.0, 0.15)
        success = bool((not fell)
                       and not math.isnan(t_go)
                       and not math.isnan(t_rev) and t_rev <= 2.0
                       and not math.isnan(t_stop) and t_stop <= 2.0)
        return dict(success=success,
                    metrics=dict(t_go=t_go, t_reverse=t_rev, t_stop=t_stop),
                    headline=(f"rev {t_rev:.1f}s stop {t_stop:.1f}s"
                              if not math.isnan(t_rev) else "no-lock"))
    return build, evaluate


def scen_metronome():
    """March to a metronome whose tempo CHANGES: 1.4 -> 1.0 -> 1.7 Hz full
    L/R cycles, 6 cycles each (user 2026-08-06: cadence/dance as the
    natural-responsiveness demo). The command encoding is the training march
    (c4 alternating, c6 = march_dz*|sin ph|) but the clock is the REFEREE's,
    with continuous phase across the tempo changes -- so the score is tempo
    tracking, not replay. 1.4 and 1.7 Hz sit inside the gait-clock training
    draw (1.25-1.75 Hz); 1.0 Hz is outside it and reported as a stretch
    metric (cadence_err_slow), not required for the pass."""
    SEGS = ((1.4, 6), (1.0, 6), (1.7, 6))     # (full-cycle Hz, cycles)
    SETTLE = 1.0

    def build():
        ev = {"ph": 0.0, "tp": SETTLE}

        def ctrl(t, gt, ev):
            if t < SETTLE:
                return (0, 0, 0, 1, 0)
            env = ev["_env"]
            hz, t0 = 0.0, SETTLE
            for f, cyc in SEGS:
                t1 = t0 + cyc / f
                if t < t1:
                    hz = f
                    break
                t0 = t1
            if hz == 0.0:
                return (0, 0, 0, 1, 0)
            ev["ph"] += 2 * math.pi * hz * (t - ev["tp"])
            ev["tp"] = t
            lift = -1 if math.sin(ev["ph"]) >= 0 else 1   # left swings first
            dz = getattr(env, "_march_dz", 0.0) * abs(math.sin(ev["ph"]))
            return (0, 0, 0, 1, lift, 0, dz)
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        bounds, t0 = [], SETTLE
        for f, cyc in SEGS:
            t1 = t0 + cyc / f
            bounds.append((f, t0, t1))
            t0 = t1
        errs = []
        for f, a, b in bounds:
            P = 1.0 / f
            per = []
            for key in ("con_l", "con_r"):
                # liftoff = contact falling edge; first cycle of the segment
                # is the transient and is skipped
                edges, prev = [], None
                for r in rows:
                    if (prev is not None and prev and not r[key]
                            and a <= r["t"] <= b):
                        edges.append(r["t"])
                    prev = r[key]
                evs = [t for t in edges if t >= a + P]
                if len(evs) >= 3:
                    per.append(float(np.median(np.diff(evs))))
            errs.append(float(np.mean([abs(p - P) / P for p in per]))
                        if per else float("nan"))
        in_band = [errs[0], errs[2]]
        cadence_err = (float(np.mean(in_band))
                       if not any(math.isnan(e) for e in in_band)
                       else float("nan"))
        ref = next(((r["x"], r["y"]) for r in rows if r["t"] >= SETTLE),
                   (0.0, 0.0))
        drift = (max(math.hypot(r["x"] - ref[0], r["y"] - ref[1])
                     for r in rows if r["t"] >= SETTLE)
                 if rows else float("nan"))
        success = bool((not fell) and not math.isnan(cadence_err)
                       and errs[0] <= 0.15 and errs[2] <= 0.15
                       and not math.isnan(drift) and drift < 0.15)
        return dict(success=success,
                    metrics=dict(cadence_err=cadence_err,
                                 cadence_err_slow=errs[1], drift=drift),
                    headline=(f"cadErr {cadence_err*100:.0f}%"
                              if not math.isnan(cadence_err) else "no-lock"))
    return build, evaluate


def scen_squat_reps():
    """Three squat reps: crouch to 0.70 height for 1.5 s, back to full for
    1.5 s, repeated (user 2026-08-06: leg-workout moves as scenarios). 0.70
    is the lower edge of the cmd-crouch DR the marathon loco runs train with,
    so the depth is in-distribution; the REPS are the new content --
    crouch_hold tests one descent, this tests cyclic knee flexion and
    recovery."""
    DEPTH, HOLD, REPS, SETTLE = 0.70, 1.5, 3, 1.0

    def build():
        ev = {}

        def ctrl(t, gt, ev):
            if t < SETTLE:
                return (0, 0, 0, 1, 0)
            if (t - SETTLE) >= REPS * 2 * HOLD:
                return (0, 0, 0, 1, 0)
            ph = (t - SETTLE) % (2 * HOLD)
            return (0, 0, 0, DEPTH if ph < HOLD else 1, 0)
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        herrs, recs = [], []
        for i in range(REPS):
            t0 = SETTLE + i * 2 * HOLD
            down = _win(rows, t0 + HOLD - 0.8, t0 + HOLD)
            up = _win(rows, t0 + 2 * HOLD - 0.8, t0 + 2 * HOLD)
            herrs.append(abs(_mean(down, "height") - DEPTH * N))
            recs.append(_mean(up, "height"))
        reps_ok = sum(1 for h, rc in zip(herrs, recs)
                      if not math.isnan(h) and h <= 0.035
                      and not math.isnan(rc) and rc >= 0.92 * N)
        drift = _drift_rms(_win(rows, SETTLE, 1e9))
        success = bool((not fell) and reps_ok == REPS
                       and not math.isnan(drift) and drift < 0.10)
        hv = [h for h in herrs if not math.isnan(h)]
        rv = [r for r in recs if not math.isnan(r)]
        return dict(success=success,
                    metrics=dict(reps_ok=reps_ok,
                                 depth_err=(float(np.mean(hv)) if hv
                                            else float("nan")),
                                 recovery=(float(np.mean(rv)) if rv
                                           else float("nan")),
                                 drift_rms=drift),
                    headline=f"{reps_ok}/{REPS} reps")
    return build, evaluate


def scen_weight_shift():
    """Weight-shift drill: alternate 1.5 s single-leg holds, L then R, two
    full rounds with 0.75 s both-feet transitions (user 2026-08-06: workout
    drills; hip-roll + lateral CoM authority over the abduction range opened
    2026-08-02). Command is the balance-lift encoding at 0.85 crouch, same
    as crouch_leg; a hold is clean when the lifted foot stays off the floor
    through the middle of the hold. The transitions are the point -- the
    single-leg HOLD was already graded by balance_L/R."""
    HOLD, GAP, ROUNDS, SETTLE = 1.5, 0.75, 2, 1.0
    CYCLE = 2 * (HOLD + GAP)

    def build():
        ev = {}

        def ctrl(t, gt, ev):
            if t < SETTLE:
                return (0, 0, 0, 1, 0)
            tt = t - SETTLE
            if tt >= ROUNDS * CYCLE:
                return (0, 0, 0, 1, 0)
            ph = tt % CYCLE
            if ph < HOLD:
                return (0, 0, 0, 0.85, -1)
            if ph < HOLD + GAP:
                return (0, 0, 0, 1, 0)
            if ph < 2 * HOLD + GAP:
                return (0, 0, 0, 0.85, 1)
            return (0, 0, 0, 1, 0)
        return ctrl, ev

    def evaluate(rows, ev, fell, N, shared):
        holds_ok, cons = 0, []
        for i in range(ROUNDS):
            for side, off in (("L", 0.0), ("R", HOLD + GAP)):
                t0 = SETTLE + i * CYCLE + off
                mid = _win(rows, t0 + 0.4, t0 + HOLD)
                c = _frac(mid, _swing_key(side))
                cons.append(c)
                if not math.isnan(c) and c < 0.20:
                    holds_ok += 1
        drift = _drift_rms(_win(rows, SETTLE, 1e9))
        success = bool((not fell) and holds_ok >= 3
                       and not math.isnan(drift) and drift < 0.10)
        cv = [c for c in cons if not math.isnan(c)]
        return dict(success=success,
                    metrics=dict(holds_ok=holds_ok,
                                 lifted_contact=(float(np.mean(cv)) if cv
                                                 else float("nan")),
                                 drift_rms=drift),
                    headline=f"{holds_ok}/{2 * ROUNDS} holds")
    return build, evaluate


# name -> (episode_seconds, factory, is_locomotion)
def _registry():
    # (single-leg crouch scenarios removed 2026-07-18, user call;
    # scen_crouch_leg kept in source for reference only)
    reg = {}
    for s in ("L", "R"):
        reg[f"balance_{s}"] = (12.0, scen_balance(s), False)
        reg[f"circle_air_{s}"] = (12.0, scen_circle_air(s), False)
        reg[f"sidestep_{s}"] = (12.0, scen_sidestep(s), True)
    reg["line_1m"] = (14.0, scen_line_1m(), True)
    reg["backward_1m"] = (14.0, scen_backward_1m(), True)
    reg["square_return"] = (45.0, scen_square_return(), True)
    reg["circle_return"] = (16.0, scen_circle_return(), True)
    reg["goal_home"] = (24.0, scen_goal_home(), True)
    reg["line_rough"] = (14.0, scen_line_1m(), True)
    reg["turn_180"] = (10.0, scen_turn_180(), True)
    reg["crouch_hold"] = (10.0, scen_crouch_hold(), False)
    reg["march_in_place"] = (11.0, scen_march(), False)
    # knee-high march: 1 s settle + 10 s of marching (user 2026-08-01)
    reg["march_10s"] = (11.0, scen_march_knee(), False)
    reg["hip_sway"] = (9.0, scen_sway(), False)
    # sit -> stand: the curriculum stage (user 2026-07-19); shorter deadline
    # since the hard part (getting onto the feet) starts closer to done
    reg["recover_sit"] = (12.0, scen_recover_fallen(deadline=5.0), False)
    reg["recover_fallen"] = (12.0, scen_recover_fallen(), False)
    reg["stand_10s"] = (11.0, scen_stand_10s(), False)
    # torque-off idle: 1.5 s powered settle + 8 s released (user 2026-07-23)
    reg["stand_off"] = (9.5, scen_stand_off(), False)
    # dynamic trio (2026-08-06)
    reg["push_gauntlet"] = (14.0, scen_push_gauntlet(), True)
    reg["speed_ladder"] = (22.0, scen_speed_ladder(), True)
    reg["pursuit"] = (24.0, scen_pursuit(), True)
    # responsiveness + workout drills (2026-08-06, natural/responsive plan)
    reg["reversal"] = (12.5, scen_reversal(), True)
    reg["metronome"] = (15.5, scen_metronome(), False)
    reg["squat_reps"] = (10.5, scen_squat_reps(), False)
    reg["weight_shift"] = (10.5, scen_weight_shift(), False)
    # bench->policy handover probe (issue #29). Registered so --scenarios can
    # reach it, but EXCLUDED from the default suite (not in ORDER / any
    # FAMILY_SCENARIOS list; see EXTRA_SCENARIOS): the 144-seed totals of
    # every existing scorecard must stay comparable, and a subset run writes
    # a _partial card anyway.
    reg["handover_stand"] = (10.0, scen_handover_stand(), False)
    return reg


# per-scenario env overrides (env_cache keys on this too)
ENV_EXTRA = {
    "recover_fallen": dict(recover_mix=1.0,
                           recover_start_mix=(1.0, 0.0, 0.0, 0.0)),
    "recover_sit": dict(recover_mix=1.0,
                        recover_start_mix=(0.0, 0.0, 0.0, 1.0)),
    # rough-ground claim (loco_v7knee): the line walk on the 0-20 mm tiled
    # mosaic; the spawn draw puts each seed on a different tile
    "line_rough": dict(terrain_mosaic=True),
    # the whole point of the gauntlet: shove it, hard, often
    "push_gauntlet": dict(push_prob=0.08, kick_range=(0.6, 1.4)),
}

# specialist-family scenario filters (progress review 2026-07-22): a run
# whose config records train.family gets scored only on its own scenarios
FAMILY_SCENARIOS = {
    "loco": ["line_1m", "line_rough", "backward_1m", "sidestep_L",
             "sidestep_R", "turn_180", "square_return", "circle_return",
             "goal_home", "stand_10s", "stand_off",
             "push_gauntlet", "speed_ladder", "pursuit",
             "reversal", "metronome", "squat_reps", "weight_shift"],
    "skills": ["balance_L", "balance_R", "circle_air_L", "circle_air_R",
               "march_in_place", "march_10s", "hip_sway", "crouch_hold",
               "stand_10s", "metronome", "squat_reps", "weight_shift"],
    "getup": ["recover_sit", "recover_fallen"],
}

ORDER = ["balance_L", "balance_R", "circle_air_L", "circle_air_R",
         "march_in_place", "march_10s", "hip_sway",
         "line_1m", "backward_1m", "sidestep_L", "sidestep_R",
         "turn_180", "square_return", "circle_return", "goal_home",
         "line_rough", "push_gauntlet", "speed_ladder", "pursuit",
         "reversal", "metronome", "squat_reps", "weight_shift",
         "crouch_hold",
         "recover_sit", "recover_fallen", "stand_10s", "stand_off"]

# scenarios that exist ONLY behind the --scenarios filter (issue #29): never
# part of the default suite, so the comparable 144-card totals never move.
EXTRA_SCENARIOS = ["handover_stand"]

# base-name -> (metric key, formatter) for the md headline column, formatted
# from the across-seed MEAN of that metric
HEADLINE = {
    "goal_home": ("return_err", lambda v: f"home {v*100:.0f}cm"),
    "push_gauntlet": ("distance", lambda v: f"d={v:.1f}m"),
    "speed_ladder": ("speed_mae", lambda v: f"mae {v:.2f}m/s"),
    "pursuit": ("final_gap", lambda v: f"gap {v*100:.0f}cm"),
    "reversal": ("t_reverse", lambda v: f"rev {v:.1f}s"),
    "metronome": ("cadence_err", lambda v: f"cadErr {v*100:.0f}%"),
    "squat_reps": ("reps_ok", lambda v: f"{v:.0f}/3 reps"),
    "weight_shift": ("holds_ok", lambda v: f"{v:.0f}/4 holds"),
    "balance": ("clear_frac", lambda v: f"clear {v*100:.0f}%"),
    "circle_air": ("traced_radius", lambda v: f"r={v*100:.1f}cm"),
    "recover_fallen": ("time_to_stand", lambda v: f"up in {v:.1f}s"),
    "recover_sit": ("time_to_stand", lambda v: f"up in {v:.1f}s"),
    "line_1m": ("time_to_1m", lambda v: f"t={v:.1f}s"),
    "line_rough": ("time_to_1m", lambda v: f"t={v:.1f}s"),
    "backward_1m": ("time", lambda v: f"t={v:.1f}s"),
    "sidestep": ("time", lambda v: f"t={v:.1f}s"),
    "turn_180": ("head_err_deg", lambda v: f"hErr {v:.0f}deg"),
    "square_return": ("return_err", lambda v: f"ret {v*100:.0f}cm"),
    "circle_return": ("return_err", lambda v: f"ret {v*100:.0f}cm"),
    "crouch_hold": ("height_err", lambda v: f"hErr {v*1000:.0f}mm"),
    "march_in_place": ("clean_lifts_l", lambda v: f"{v:.0f} clean lifts/leg"),
    "march_10s": ("peak_clear_med", lambda v: f"peak {v*100:.1f}cm"),
    "hip_sway": ("sway_amp", lambda v: f"amp {v*100:.1f}cm"),
    "stand_10s": ("drift", lambda v: f"drift {v*100:.1f}cm"),
    "stand_off": ("drift", lambda v: f"drift {v*100:.1f}cm"),
    "handover_stand": ("drift", lambda v: f"drift {v*100:.1f}cm"),
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
    ev["_env"] = env          # scenarios that drive the env directly (torque-off)
    drv = Driver(env, act, seed, record)
    if ev.get("_setup"):
        # scenario-owned start-state hook, after reset() and before the first
        # policy tick (handover_stand's bench hold + history prime, #29)
        ev["_setup"](env, drv)
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
    p.add_argument("--sil", action="store_true",
                   help="route every tick through the REAL firmware stack "
                        "(libctrl_sil); writes scorecard_sil.{md,json}")
    p.add_argument("--act-lag-hz", type=float, default=0.0,
                   help="apply the measured servo action-chain lag to the "
                        "plant (3-stage cascade pole, Hz; 2.0 = the "
                        "2026-08-31 bench measurement). Writes "
                        "scorecard_lag.{md,json} -- the lag-less columns "
                        "stay untouched for historical comparability")
    args = p.parse_args()
    suffix = "_sil" if args.sil else ""
    if args.act_lag_hz > 0.0:
        suffix = f"{suffix}_lag"
    # a --scenarios subset must not clobber the run's full reel either --
    # the scorecard learned this 2026-08-03 (guard below), but the movie
    # path didn't and a stand-only render overwrote two full reels
    # (2026-08-12). One suffix, decided before ANY artifact path is built.
    if args.scenarios:
        suffix = f"{suffix}_partial"
    stack = "SIL (firmware stack)" if args.sil else "python policy"

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
    fam = (cfg.get("train", {}) or {}).get("family", "all")
    if not args.scenarios and fam in FAMILY_SCENARIOS:
        names = [n for n in ORDER if n in FAMILY_SCENARIOS[fam]]
        print(f"[family={fam}] scoring {len(names)} scenarios")
    if args.scenarios:
        want = [s.strip() for s in args.scenarios.split(",") if s.strip()]
        names = [n for n in ORDER + EXTRA_SCENARIOS if n in want]
        unknown = [w for w in want if w not in reg]
        if unknown:
            print(f"warning: unknown scenarios ignored: {unknown}", file=sys.stderr)

    # one env per distinct (episode length, scenario override) pair
    env_cache = {}

    def get_env(secs, name=""):
        key = (secs, name if name in ENV_EXTRA else "")
        if key not in env_cache:
            env_cache[key] = make_env(cfg, secs, args.nominal, xml,
                                      extra=ENV_EXTRA.get(key[1]),
                                      act_lag_hz=args.act_lag_hz)
        return env_cache[key]

    obs_size = get_env(12.0).observation_space.shape[0]
    act_size = get_env(12.0).action_space.shape[0]
    mass = float(get_env(12.0).model.body_mass.sum())
    N = float(get_env(12.0)._nominal_h)

    # act(obs) -> action.  Either brax's policy, or -- with --sil -- the
    # firmware's own obs assembler + gait clock + MLP + calibration, driven
    # through the C ABI.  Everything downstream (scenarios, seeds, scoring)
    # is identical, which is what makes the two columns comparable.
    sil_info = None
    if args.sil:
        H, sil_lib, sil_info = sil_setup(args.run_name, run_dir)
        act = None

        def act_for(env):
            # one adapter per episode: it owns the library's history ring and
            # gait clock, and resets both in begin_episode()
            return H.SilActAdapter(sil_lib, env, log=False)
    else:
        act = load_policy(run_dir, obs_size, act_size)

        def act_for(env):
            return act

    cond = "nominal (no DR)" if args.nominal else \
        "hardware-claim (GoPro 154 g, 4 ms latency, 0.7 deg backlash, DR, IMU obs)"
    if args.act_lag_hz > 0.0:
        cond += (f"; MEASURED servo lag {args.act_lag_hz:g} Hz "
                 "(3-stage act-lag cascade)")
    print(f"CPU precision referee: {args.run_name}  [{stack}]  [{cond}]  "
          f"{args.episodes} seeds/scenario, plant {os.path.basename(xml)}\n")

    scorecard = {}
    all_shared = {k: [] for k in
                  ("wobble_rms", "mean_watts", "foot_slip")}
    loco_shared = {k: [] for k in ("cot", "symmetry")}
    tot_succ = tot_runs = tot_fall = 0
    scen_all_pass = 0
    md_rows = []
    mov_writer, mov_path = None, None
    # reel chapter index: the .mov is takes back to back, so build_report
    # (and anyone slicing a clip out of it) needs to know where each
    # scenario starts. Written next to the movie as reel_index<suffix>.json.
    reel_index, reel_frames = [], 0

    for name in names:
        secs, factory, is_loco = reg[name]
        env = get_env(secs, name)
        results = [run_one(env, act_for(env), factory, seed=100 * i + 7,
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
                # one .mov per referee run, scenarios back to back, in
                # scorecard order (user 2026-08-03: no per-scenario gifs).
                # Captioned with the laptop-reel style (video_annot, user
                # 2026-08-05 and again 2026-08-12): scenario + seed verdict
                # + headline + timecode on every frame, honesty rules and
                # all -- a FAIL take is labeled FAIL, never left implicit.
                r0 = results[0]
                verdict = "PASS" if r0["success"] else "FAIL"
                seed_note = f"seed 1/{args.episodes}"
                head0 = r0.get("headline") or ""
                title = f"{name} · {seed_note}"
                frame_dt = 3 * env.control_dt        # every 3rd control step
                if mov_writer is None:
                    import imageio
                    mov_path = os.path.join(
                        run_dir, f"{args.run_name}{suffix}.mov")
                    mov_writer = imageio.get_writer(
                        mov_path, fps=MOV_FPS, codec="libx264", quality=8,
                        macro_block_size=2)
                for j, f in enumerate(frames):
                    mov_writer.append_data(caption(
                        f, title, verdict, head0, t=j * frame_dt))
                reel_index.append(dict(
                    scenario=name, verdict=verdict, headline=head0,
                    frames=len(frames),
                    start=round(reel_frames / MOV_FPS, 3),
                    end=round((reel_frames + len(frames)) / MOV_FPS, 3)))
                reel_frames += len(frames)

    if mov_writer is not None:
        mov_writer.close()
        print(f"  render -> {mov_path}")
        idx_path = os.path.join(run_dir, f"reel_index{suffix}.json")
        with open(idx_path, "w") as f:
            json.dump(dict(movie=os.path.basename(mov_path), fps=MOV_FPS,
                           chapters=reel_index), f, indent=1)
        print(f"  reel index -> {idx_path}")

    # summary shared aggregates
    def _m(vals):
        a = np.array([v for v in vals if v is not None
                      and not (isinstance(v, float) and math.isnan(v))], float)
        return float(a.mean()) if a.size else None

    summary = dict(
        conditions=cond, claim=stack, stack=("sil" if args.sil else "python"),
        episodes=args.episodes, plant=os.path.basename(xml),
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
    if sil_info:
        summary["sil"] = sil_info
    scorecard["summary"] = summary

    json_path = os.path.join(run_dir, f"scorecard{suffix}.json")
    with open(json_path, "w") as f:
        json.dump(scorecard, f, indent=2)

    # --- human table ---
    lines = []
    lines.append(f"# Precision scorecard - {args.run_name} [{stack}]")
    lines.append(f"_claim: **{stack}**; {cond}; {args.episodes} "
                 f"seeds/scenario; plant {os.path.basename(xml)}_")
    if sil_info:
        lines.append("")
        lines.append(f"_every tick assembled, normalized and inferred by the "
                     f"firmware's own code: `{sil_info['lib']}`, weights "
                     f"`{sil_info['weights']}`, calibration "
                     f"`{sil_info['cal']}`, gait clock "
                     f"{sil_info['gait_hz']:.2f} Hz. See docs/sil-harness.md._")
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
    sym = summary["symmetry_locomotion"]
    if sym is not None:
        overall += f"; gait asym {sym*100:.0f}%"
    lines.append(overall)
    md = "\n".join(lines) + "\n"
    md_path = os.path.join(run_dir, f"scorecard{suffix}.md")
    with open(md_path, "w") as f:
        f.write(md)
    print(md)
    print(f"wrote {os.path.relpath(md_path, ROOT)} and "
          f"{os.path.relpath(json_path, ROOT)}")


if __name__ == "__main__":
    main()
