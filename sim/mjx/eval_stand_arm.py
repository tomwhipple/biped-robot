"""Standing referee for the FIRST policy test: arm on the floor, zero command.

The first armed hardware test of a legs policy is "stand under the policy with
a zero command", started with the robot ALREADY STANDING on the floor (no
support; Tom spotting) -- after `home`, or in a pose a few degrees off it. This
grades the candidates on exactly that, on two plants:

  train   the referee's hardware claim (eval_precision.make_env: DR, 4 ms
          latency, 0.7 deg backlash, IMU noise/misalignment, the run's own
          zero-offset DR) -- the distribution the policy trained on
  fit     the fitted plant (fit:<band> = one of its band variants)
          (experiments/one-leg-stand/simfit_plant.json, PR
          #179): the best model of the real robot -- roll play 0.96 / 1.55
          deg (ankle / hip rolls), backlash 0.07 deg, 81 mm soles, carpet
          contact, torso x0.70 with its CoM 6.8 mm right, the L outer sole edge
          13.7 mm in and the L ankle-roll zero 2.1 deg off. Deterministic
          plant, plus IMU noise and a per-seed IMU mount error / gyro bias.

Both plants run the action chain the robot runs: the firmware's 10 Hz C2
shaper in series with the servos' ~12 Hz lag (the sim-fit's joint tracking)
is one 3-stage cascade at --lag-hz (5.5 Hz: the stage time constants add).

Scenarios (all at the stand command (0, 0, 0, 1, 0, 0, 0)):

  stand_10s  eval_precision's: env.reset() start, 10 s, the plant's own
             random 5 N shoves on `train` (the referee's claim)
  arm_home   the arm handover as the firmware does it: the robot settles
             1.5 s on the floor under the servos' hold at the calibrated zero
             (`home`), then the loop takes over -- history filled with ONE
             frame, prev_action 0, the shaper seeded at the measured pose and
             the 1 s takeover ramp from the measured pose (ctrl_task.cpp
             kArmRampMs) -- and runs 10 s
  arm_off    the same from a pose OFF home: every joint +-4 deg (uniform, per
             seed), held and settled the same way
  push       arm_home, then four torso velocity kicks (training's push law,
             walker_env push_kick) of --push-mps m/s at t = 3, 5.5, 8, 10.5 s
             after the arm: forward, back, left, right; 12.5 s

  walk_small arm_home, then 0.15 m/s forward from 2 s to 6 s, then stand
             (small steps, for the SIL parity)

--sil runs the REAL firmware stack in the policy's place (sim/sil/harness.py
SilActAdapter over libctrl_sil of the plant's policy profile; shaper off, the
lag cascade stands for it) and logs per-tick parity against the python policy;
--pin-clock gives a python column with the same (firmware) gait clock to set
beside it.

--sensor-delay 1 feeds the policy sensor slots (q, dq, up, gyro) one tick old
(prev_action, phase and command stay current): a stale-read stress.

Per episode: fell, drift (max planar excursion from the arm point), yaw drift,
peak tilt in the first 2 s and overall, gyro rms in the first 2 s, steps
(foot lift-offs >= 3 ticks), peak sole height, mean |servo torque|, mean
power, action-rate rms, joint-speed rms, and the largest single-tick change of
the written target in the first 2 s.

  CUDA_VISIBLE_DEVICES= JAX_PLATFORMS=cpu .venv/bin/python sim/mjx/eval_stand_arm.py \\
      --runs robot_legs_walk14_s128r24,robot_legs_walk13_fit_s1_s128r24 \\
      [--plants train,fit] [--scenarios stand_10s,arm_home,arm_off,push] \\
      [--seeds 8] [--jobs 24] [--sensor-delay 0] [--out stand_arm.json]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
RUNS = os.path.join(HERE, "..", "runs")
FIT_JSON = os.path.join(ROOT, "experiments", "one-leg-stand", "simfit_plant.json")

STAND = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
SETTLE_S = 1.5
RAMP_TICKS = 50            # ctrl_task.cpp kArmRampMs 1000 at 50 Hz
OFF_DEG = 4.0              # --off-deg
PREFALL_DEG = 10.0         # tilted this far at the arm tick: the HOLD tipped it, not the policy
PUSH_T = (3.0, 5.5, 8.0, 10.5)
PUSH_DIR = ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0))


def _xml(cfg):
    xml = cfg.get("xml_path")
    if not os.path.isabs(xml) and not os.path.exists(xml):
        xml = os.path.join(HERE, "..", os.path.basename(xml))
    return xml


def apply_fitted_plant(env, pl):
    """simfit_sim.make_env's plant edits on an already-built BimoWalkerEnv
    (domain_rand off, so a reset never rewrites them)."""
    import mujoco
    m = env.model
    lash = {}
    for n in env._servo_names:
        if n[2:] in ("hip_roll", "ankle_roll"):
            lash[n] = pl["roll_lash_hip"] if n[2:] == "hip_roll" else pl["roll_lash"]
    ref = max(lash.values())
    env.roll_lash_deg_max = None
    env._roll_lash_rad = math.radians(ref)
    env._roll_mask = np.array([lash.get(n, 0.0) / ref for n in env._servo_names])
    name = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
    pads = [g for g in range(m.ngeom) if "_pad_" in name(g)]
    for side in "LR":
        ps = [g for g in pads if name(g).startswith(side)]
        ys = np.array([m.geom_pos[g][1] for g in ps])
        yc, half = 0.5 * (ys.max() + ys.min()), 0.5 * (ys.max() - ys.min())
        outer = 1.0 if side == "L" else -1.0
        for g in ps:
            m.geom_pos[g][1] = yc + (m.geom_pos[g][1] - yc) * (0.5 * pl["sole_w"]) / half
            if side == "L" and pl["sole_out_dL"] and (m.geom_pos[g][1] - yc) * outer > 0:
                m.geom_pos[g][1] -= (outer * pl["sole_out_dL"] * (m.geom_pos[g][1] - yc)
                                     * outer / (0.5 * pl["sole_w"]))
    for g in pads + [m.geom("floor").id]:
        m.geom_solref[g] = [pl["floor_tc"], 1.0]
    b = env._torso_bid
    m.body_mass[b] *= pl["torso_scale"]
    m.body_inertia[b] *= pl["torso_scale"]
    m.body_ipos[b][1] += pl["torso_com_y"]
    zoff = np.zeros(env._n_servo)
    zoff[env._servo_names.index("L_ankle_roll")] = math.radians(pl["zoff_La"])
    return zoff


_CACHE = {}


def _sil(run, xml):
    """The real firmware stack (sim/sil/harness.py -> libctrl_sil of the
    profile whose compiled spec is this plant's), shaper OFF: the plant's
    --lag-hz cascade already stands for shaper + servo, as for the python run."""
    if "sil" in _CACHE:
        return _CACHE["sil"]
    import eval_precision as EP
    prof = EP.sil_profile_for(xml)
    H, lib, info = EP.sil_setup(run, os.path.join(RUNS, run), verbose=False, profile=prof)
    lib.set_shaper(0.0)
    _CACHE["sil"] = (H, lib, info)
    return _CACHE["sil"]


def _setup(run, plant, lag_hz, ep_s):
    key = (run, plant, lag_hz, ep_s)
    if key in _CACHE:
        return _CACHE[key]
    os.environ.setdefault("JAX_PLATFORMS", "cpu")
    os.environ.setdefault("MUJOCO_GL", "disabled")
    sys.path.insert(0, HERE)
    sys.path.insert(0, os.path.join(HERE, ".."))
    import eval_precision as EP
    run_dir = os.path.join(RUNS, run)
    cfg = json.load(open(os.path.join(run_dir, "config.json")))
    xml = _xml(cfg)
    if plant == "train":
        env = EP.make_env(cfg, ep_s, nominal=False, xml=xml, act_lag_hz=lag_hz)
        env.push_prob_claim = env.push_prob
        zoff = None
    else:
        # "fit" or "fit:<band>" -- a simfit_plant.json band variant (the
        # fit's own uncertainty range, one parameter at an end of it)
        fj = json.load(open(FIT_JSON))
        pl = dict(fj["plant"])
        if ":" in plant:
            pl.update(fj["band"][plant.split(":", 1)[1]])
        extra = dict(latency_ms=4.0, backlash_deg=pl["backlash"], backlash_deg_max=None,
                     roll_lash_deg=pl["roll_lash"], roll_lash_deg_max=None)
        env = EP.make_env(cfg, ep_s, nominal=True, xml=xml, act_lag_hz=lag_hz, extra=extra)
        env.push_prob_claim = 0.0
        zoff = apply_fitted_plant(env, pl)
    env.render_mode = None
    act_size = int(env.action_space.shape[0])
    act = EP.load_policy(run_dir, int(env.observation_space.shape[0]), act_size)
    _CACHE[key] = (env, act, zoff)
    return _CACHE[key]


def _place_and_hold(env, q_pol, settle_s):
    """Stand the robot on the floor at joint pose q_pol (policy order, the
    servo-side/believed frame) and let it settle under the servos' hold, through
    env.step's own actuator path (latency, lag, quantization, play)."""
    import mujoco
    m, d = env.model, env.data
    phys = q_pol + (env._zero_off if env._zero_off is not None else 0.0)
    d.qpos[:] = m.qpos0
    d.qpos[env._jqpos] = phys
    d.qpos[3:7] = [1, 0, 0, 0]
    d.qvel[:] = 0.0
    mujoco.mj_kinematics(m, d)
    pads = [g for gs in env._pad_gids for g in gs]
    zmin = min(d.geom_xpos[g][2] - m.geom_size[g][0] for g in pads)
    d.qpos[2] -= zmin - 0.0005
    mujoco.mj_forward(m, d)
    tgt = env._to_servo(q_pol)
    env._lag_y = np.stack([np.asarray(tgt, np.float64)] * 3)
    env._last_target = tgt + (env._zero_off if env._zero_off is not None else 0.0)
    env._shaft = None
    orig = env._action_to_ctrl
    env._action_to_ctrl = lambda a: q_pol
    push = env.push_prob
    env.push_prob = 0.0
    for _ in range(int(round(settle_s / env.control_dt))):
        env.step(np.zeros(env.action_space.shape[0], np.float32))
    env.push_prob = push
    env._action_to_ctrl = orig


def episode(job):
    run, plant, scen, seed, o = job
    run_s = 12.5 if scen == "push" else 10.0
    walk = scen == "walk_small"
    ep_s = run_s + SETTLE_S + 0.5
    env, act, zoff = _setup(run, plant, o["lag_hz"], ep_s)
    rng = np.random.default_rng(1000 + seed)
    if plant.startswith("fit"):
        env.domain_rand = False
    env.push_prob = env.push_prob_claim if scen == "stand_10s" else 0.0
    obs, _ = env.reset(seed=seed)
    if plant.startswith("fit"):
        env._zero_off = zoff.copy()
        # the robot's IMU: `imu mount` calibrated to ~1 deg, a small gyro bias
        ang = math.radians(rng.normal(0.0, 1.0))
        ax = rng.normal(size=3)
        ax /= np.linalg.norm(ax)
        k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        env._imu_R = np.eye(3) + math.sin(ang) * k + (1 - math.cos(ang)) * k @ k
        env._imu_gyro_bias = rng.uniform(-0.01, 0.01, 3)
        env.domain_rand = True      # _obs's IMU noise (reset already ran without DR)
    env.imu_noise = float(env.imu_noise) * o["imu_noise_x"]
    nj = env._nq_act
    fw = env.obs_frame
    # frame layout: q(nj) dq(nj) up(3) linvel(3) gyro(3) prev(nj) h(1) phase(2) cmd
    sens = np.r_[0:2 * nj + 9]
    env.set_command(*STAND)
    arm_pose = None
    if scen != "stand_10s":
        q0 = np.zeros(nj) + np.asarray(env._default)
        if scen == "arm_off":
            q0 = q0 + np.radians(rng.uniform(-o["off_deg"], o["off_deg"], nj))
            q0 = np.clip(q0, env._lo, env._hi)
        _place_and_hold(env, q0, SETTLE_S)
        # the arm tick (ctrl_task.cpp): fresh history of ONE frame, prev 0,
        # shaper (lag cascade) seeded at the MEASURED pose, ramp from it
        env._prev_action[:] = 0.0
        env.set_command(*STAND)
        env._obs_hist = []                 # _obs() is the bare frame only then
        first = env._obs()
        env._obs_hist = [first.copy() for _ in range(env.obs_hist_len - 1)]
        obs = np.concatenate([first] + list(env._obs_hist))
        qm = first[:nj].astype(np.float64)                    # what the servos read
        env._lag_y = np.stack([env._to_servo(qm)] * 3)
        arm_pose = qm.copy()
    adapter = None
    if o["sil"] or o["pin_clock"]:
        # the firmware's gait clock (the run's rate, phase 0 at the arm), so the SIL
        # and python columns see the same phase (harness.pin_gait_clock)
        sys.path.insert(0, os.path.join(ROOT, "sim", "sil"))
        cfg_xml = _xml(json.load(open(os.path.join(RUNS, run, "config.json"))))
        if o["sil"]:
            H, lib, _ = _sil(run, cfg_xml)
        else:
            os.environ.setdefault("BIMO_POLICY", "v6legs")
            import harness as H
        H.pin_gait_clock(env)
        obs = np.concatenate([env._obs_hist[0]] + list(env._obs_hist))
        if o["sil"]:
            adapter = H.SilActAdapter(lib, env, py_act=act)
            adapter.begin_episode()
    policy = adapter if adapter is not None else act
    up_arm = float(env.data.sensordata[env._up_id + 2])
    tilt_arm = math.degrees(math.acos(max(-1.0, min(1.0, up_arm))))
    base_a2c = env._action_to_ctrl
    tick = {"i": 0}

    def ramped(a):
        tgt = base_a2c(a)
        i = tick["i"]
        if arm_pose is not None and i < RAMP_TICKS:
            r = (i + 1) / RAMP_TICKS
            tgt = arm_pose + r * (tgt - arm_pose)
        return tgt
    env._action_to_ctrl = ramped

    d = env.data
    x0, y0 = float(d.qpos[0]), float(d.qpos[1])
    yaw0 = math.atan2(2 * (d.qpos[3] * d.qpos[6] + d.qpos[4] * d.qpos[5]),
                      1 - 2 * (d.qpos[5] ** 2 + d.qpos[6] ** 2))
    n = int(round(run_s / env.control_dt))
    pushes = {int(round(t / env.control_dt)): k for k, t in enumerate(PUSH_T)} if scen == "push" else {}
    hist = [obs[:fw].copy()]
    fell = False
    rows = []
    prev_a = np.zeros(env.action_space.shape[0])
    prev_tgt = None
    air = [0, 0]
    steps = 0
    z0 = float(getattr(env, "_sole_z0", 0.0))
    peak_sole = 0.0
    push_ok = []
    for i in range(n):
        tick["i"] = i
        o_in = obs.copy()
        if o["sensor_delay"] > 0 and len(hist) > 1:
            # fractional delay: linear blend of the two bracketing frames
            # (0.5 ~ a sensor averaged over the previous tick, like the gyro)
            kd = int(math.floor(o["sensor_delay"]))
            fr = o["sensor_delay"] - kd
            for h in range(env.obs_hist_len):
                a0 = hist[max(len(hist) - 1 - h - kd, 0)][sens]
                a1 = hist[max(len(hist) - 2 - h - kd, 0)][sens]
                o_in[h * fw + sens] = (1.0 - fr) * a0 + fr * a1
        if walk:
            # small steps: stand 2 s, 0.15 m/s forward 4 s, stand again
            env.set_command(*((0.15,) + STAND[1:] if 2.0 <= i * env.control_dt < 6.0 else STAND))
        nc = env._ncmd
        o_in[fw - nc:fw] = env._cmd          # this tick's command in the head frame (Driver)
        a = policy(o_in)
        if i in pushes:
            dx, dy = PUSH_DIR[pushes[i]]
            d.qvel[0] += o["push_mps"] * dx
            d.qvel[1] += o["push_mps"] * dy
        obs, _, term, _, info = env.step(a)
        hist.append(obs[:fw].copy())
        if len(hist) > 6:
            hist.pop(0)
        tgt = np.asarray(d.ctrl).copy()
        cl, cr = env._foot_contacts()
        for s, c in enumerate((cl, cr)):
            if not c:
                air[s] += 1
            else:
                if air[s] >= 3:
                    steps += 1
                air[s] = 0
        sz = max(float(d.geom_xpos[g][2]) - z0 for g in env._sole_gids)
        peak_sole = max(peak_sole, min(sz, 1.0))
        up_z = float(info["up_z"])
        rows.append((i * env.control_dt, float(d.qpos[0]) - x0, float(d.qpos[1]) - y0,
                     math.degrees(math.acos(max(-1.0, min(1.0, up_z)))),
                     float(np.sum(d.qvel[3:5] ** 2)),
                     float(np.mean(np.abs(env._servo_tau))), float(info["power_w"]),
                     float(np.sum((a - prev_a) ** 2)),
                     float(np.mean(d.qvel[env._jqvel] ** 2)),
                     float(np.max(np.abs(tgt - prev_tgt))) if prev_tgt is not None else 0.0))
        prev_a = np.asarray(a)
        prev_tgt = tgt
        if term:
            fell = True
            break
    if scen == "push":
        for t in PUSH_T:
            push_ok.append(bool(rows[-1][0] >= t + 1.5))
    env._action_to_ctrl = base_a2c
    R = np.array(rows)
    t = R[:, 0]
    first2 = t < 2.0
    yaw1 = math.atan2(2 * (d.qpos[3] * d.qpos[6] + d.qpos[4] * d.qpos[5]),
                      1 - 2 * (d.qpos[5] ** 2 + d.qpos[6] ** 2))
    sil = {}
    if adapter is not None:
        rr = [r for r in adapter.rows if not r.get("skipped")]
        sil = dict(net_err=float(max(r["net_err"] for r in rr)),
                   train_err_max=float(max(r["train_action_err"] for r in rr)),
                   train_err_mean=float(np.mean([r["train_action_err"] for r in rr])),
                   q_err=float(max(r["obs_block_err"]["q"] for r in rr)),
                   obs_err=float(max(r["obs_frame_err"] for r in rr)))
    return dict(
        sil=sil, x_end=float(R[-1, 1]),
        run=run, plant=plant, scen=scen, seed=seed, fell=fell,
        tilt_arm=tilt_arm, prefall=bool(tilt_arm > PREFALL_DEG),
        t_end=float(t[-1] + env.control_dt),
        drift=float(np.max(np.hypot(R[:, 1], R[:, 2]))),
        yaw_deg=abs(math.degrees(math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0)))),
        tilt2=float(np.max(R[first2, 3])), tilt=float(np.max(R[:, 3])),
        gyro2=float(np.sqrt(np.mean(R[first2, 4]))),
        steps=steps, peak_sole_cm=100 * peak_sole,
        tau=float(np.mean(R[:, 5])), watts=float(np.mean(R[:, 6])),
        arate=float(np.sqrt(np.mean(R[:, 7]))), qd_rms=float(np.sqrt(np.mean(R[:, 8]))),
        dtgt2_deg=float(np.degrees(np.max(R[first2, 9]))) if first2.any() else 0.0,
        pushes_ok=sum(push_ok) if scen == "push" else None)


def summarize(res, runs, plants, scens):
    out = {}
    for r in runs:
        for p in plants:
            for s in scens:
                eps = [e for e in res if e["run"] == r and e["plant"] == p and e["scen"] == s]
                pre = sum(e["prefall"] for e in eps)
                eps = [e for e in eps if not e["prefall"]]
                if not eps:
                    continue
                mean = lambda k: float(np.mean([e[k] for e in eps]))
                worst = lambda k: float(np.max([e[k] for e in eps]))
                out[(r, p, s)] = dict(
                    n=len(eps), prefall=pre, falls=sum(e["fell"] for e in eps),
                    drift=mean("drift"), drift_w=worst("drift"), yaw=mean("yaw_deg"),
                    tilt2=mean("tilt2"), tilt2_w=worst("tilt2"), tilt=mean("tilt"),
                    gyro2=mean("gyro2"), steps=mean("steps"), sole=mean("peak_sole_cm"),
                    tau=mean("tau"), watts=mean("watts"), arate=mean("arate"),
                    qd=mean("qd_rms"), dtgt2=worst("dtgt2_deg"),
                    pushes=(sum(e["pushes_ok"] for e in eps) if s == "push" else None),
                    x_end=mean("x_end"),
                    net_err=(max(e["sil"]["net_err"] for e in eps) if eps[0]["sil"] else None),
                    train_err_max=(max(e["sil"]["train_err_max"] for e in eps) if eps[0]["sil"] else None),
                    train_err_mean=(float(np.mean([e["sil"]["train_err_mean"] for e in eps]))
                                    if eps[0]["sil"] else None),
                    q_err=(max(e["sil"]["q_err"] for e in eps) if eps[0]["sil"] else None))
    return out


def table(summ, runs, plants, scens):
    L = ["| run | plant | scenario | falls | drift cm (worst) | yaw deg | tilt 0-2 s deg (worst) "
         "| tilt deg | gyro 0-2 s | steps | sole cm | tau Nm | W | act-rate | qd rms | max dtgt 0-2 s deg | pushes |",
         "|" + "---|" * 17]
    for r in runs:
        for p in plants:
            for s in scens:
                v = summ.get((r, p, s))
                if not v:
                    continue
                push = f"{v['pushes']}/{4 * v['n']}" if v["pushes"] is not None else "-"
                pf = f" (+{v['prefall']} tipped in the hold)" if v["prefall"] else ""
                L.append(f"| {r} | {p} | {s} | {v['falls']}/{v['n']}{pf} | {100 * v['drift']:.1f} "
                         f"({100 * v['drift_w']:.1f}) | {v['yaw']:.1f} | {v['tilt2']:.1f} "
                         f"({v['tilt2_w']:.1f}) | {v['tilt']:.1f} | {v['gyro2']:.2f} | "
                         f"{v['steps']:.1f} | {v['sole']:.1f} | {v['tau']:.3f} | {v['watts']:.0f} | "
                         f"{v['arate']:.3f} | {v['qd']:.2f} | {v['dtgt2']:.2f} | {push} |")
    sil = [(k, v) for k, v in summ.items() if v.get("net_err") is not None]
    if sil:
        L += ["", "| run | plant | scenario | x end cm | SIL net err (max) | SIL vs training-obs action err "
              "max / mean | q obs err (max, rad) |", "|---|---|---|---|---|---|---|"]
        for (r, p, s), v in sil:
            L.append(f"| {r} | {p} | {s} | {100 * v['x_end']:.1f} | {v['net_err']:.2e} | "
                     f"{v['train_err_max']:.2e} / {v['train_err_mean']:.2e} | {v['q_err']:.2e} |")
    return "\n".join(L)


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--runs", required=True)
    p.add_argument("--plants", default="train,fit")
    p.add_argument("--scenarios", default="stand_10s,arm_home,arm_off,push")
    p.add_argument("--seeds", type=int, default=8)
    p.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 4))
    p.add_argument("--lag-hz", type=float, default=5.5)
    p.add_argument("--push-mps", type=float, default=0.4)
    p.add_argument("--sensor-delay", type=float, default=0.0, help="policy sensor slots this many ticks old (fractional ok)")
    p.add_argument("--imu-noise-x", type=float, default=1.0)
    p.add_argument("--sil", action="store_true",
                   help="the firmware stack (libctrl_sil, the plant's policy profile) acts; "
                        "logs per-tick parity against the python policy (implies --pin-clock)")
    p.add_argument("--pin-clock", action="store_true",
                   help="the firmware's gait clock (1.5 Hz from phase 0 at the arm) instead "
                        "of training's random draw -- for a python column to set beside --sil")
    p.add_argument("--off-deg", type=float, default=OFF_DEG, help="arm_off: per-joint offset range, deg")
    p.add_argument("--out", default=None, help="write the per-episode results + table (json)")
    a = p.parse_args()
    runs = a.runs.split(",")
    plants = a.plants.split(",")
    scens = a.scenarios.split(",")
    o = dict(lag_hz=a.lag_hz, push_mps=a.push_mps, sensor_delay=a.sensor_delay,
             imu_noise_x=a.imu_noise_x, off_deg=a.off_deg, sil=a.sil,
             pin_clock=a.pin_clock or a.sil)
    jobs = [(r, pl, s, k, o) for r in runs for pl in plants for s in scens for k in range(a.seeds)]
    # one env + jit per (run, plant) per worker: hand each worker whole groups
    jobs.sort(key=lambda j: (j[0], j[1]))
    with ProcessPoolExecutor(a.jobs) as ex:
        res = list(ex.map(episode, jobs, chunksize=max(1, len(jobs) // (4 * a.jobs))))
    summ = summarize(res, runs, plants, scens)
    tab = table(summ, runs, plants, scens)
    cond = (f"lag {a.lag_hz:g} Hz, sensor delay {a.sensor_delay:g} tick(s), IMU noise x{a.imu_noise_x:g}, "
            f"{'SIL (firmware stack) ' if a.sil else ''}{'clock pinned, ' if o['pin_clock'] else ''}"
            f"push kicks {a.push_mps:g} m/s, arm_off +-{a.off_deg:g} deg, {a.seeds} seeds; "
            f"episodes tilted > {PREFALL_DEG:g} deg at the arm tick are counted apart")
    print(cond)
    print(tab)
    if a.out:
        with open(a.out, "w") as f:
            json.dump(dict(conditions=cond, options=o, episodes=res, table=tab), f, indent=1)


if __name__ == "__main__":
    main()
