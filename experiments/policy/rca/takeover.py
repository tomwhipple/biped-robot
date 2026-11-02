"""SIM REPRO: replay the 2026-11-01 takeover on a sim plant through the firmware path
(history of one frame, prev 0, firmware gait clock, 10 Hz shaper seeded at the measured
pose, 1 s ramp from it), from the recorded arm pose, for 10 s at the stand command."""
import json, math, os, sys
import numpy as np
from scipy.signal import butter, filtfilt, hilbert
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import rec, simh
import harness as H   # sim/sil/harness.py (pin_gait_clock)

RAMP = 50
# eval_stand_arm's push law: kicks at 3, 5.5, 8 s after the arm, fwd / back / left
PUSHES = {150: (1.0, 0.0), 275: (-1.0, 0.0), 400: (0.0, 1.0)}


def flail_metrics(t, a, gyro, tilt, goal=None, q=None):
    """Same numbers for real and sim. a: (n,12) actions; gyro (n,3); tilt deg."""
    n = len(a)
    ar = np.r_[0, np.sqrt(np.sum(np.diff(a, axis=0) ** 2, 1))]
    b, aa = butter(3, [6 / 25, 13 / 25], "band")
    hf = lambda x: np.abs(hilbert(filtfilt(b, aa, x, axis=0), axis=0)) if len(x) > 30 else np.zeros_like(x)
    eg = hf(gyro[:, 0])
    w = lambda lo, hi: slice(int(lo * 50), min(int(hi * 50), n))
    out = dict(
        arate_0_1=float(ar[w(0.2, 1)].mean()), arate_1_2=float(ar[w(1, 2)].mean()),
        arate_2_3=float(ar[w(2, 3)].mean()), arate_3_5=float(ar[w(3, 5)].mean()) if n > 160 else float("nan"),
        arate_all=float(ar.mean()),
        sat_frac=float(np.mean(np.abs(a) > 0.9)),
        gx_hf_1_2=float(eg[w(1, 2)].mean()), gx_hf_2_3=float(eg[w(2, 3)].mean()),
        gx_hf_end=float(eg[w(max(0, n / 50 - 2), n / 50)].mean()),
        tilt_max=float(np.max(tilt)), t_end=n / 50.0)
    out["flail"] = bool(out["arate_all"] > 0.5 or out["sat_frac"] > 0.05)
    # onset: first time the 0.5 s running mean of the per-tick action change passes 0.5
    k = 25
    rm = np.convolve(ar, np.ones(k) / k, mode="valid") if n > k else np.zeros(1)
    hit = np.where(rm > 0.5)[0]
    out["t_onset"] = float((hit[0] + k) / 50.0) if len(hit) else float("nan")
    return out


def real_metrics():
    r = rec.load(); n = 498
    a = rec.angles_to_action(r["nom"][:n])
    tilt = np.degrees(np.arccos(np.clip(r["up"][:n, 2], -1, 1)))
    m = flail_metrics(None, a, r["gyro"][:n], tilt)
    m["flail"] = True
    return m, r


def episode(plant="fit", seed=0, servo=None, model=None, shaper_hz=10.0, run_s=10.0,
            sensor_delay=0, imu_noise_x=1.0, q0=None, run=simh.RUN_NAME, record=False,
            obs_hook=None, settle_s=1.5, pin_clock=True, out_gain=1.0, push_mps=0.0):
    env, act, zoff = simh.build(plant, run=run, ep_s=run_s + settle_s + 1.0)
    rng = np.random.default_rng(1000 + seed)
    if plant.startswith("fit"):
        env.domain_rand = False
    env.push_prob = 0.0
    env.reset(seed=seed)
    if plant.startswith("fit"):
        env._zero_off = zoff.copy()
        ang = math.radians(rng.normal(0.0, 1.0)); ax = rng.normal(size=3); ax /= np.linalg.norm(ax)
        k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        env._imu_R = np.eye(3) + math.sin(ang) * k + (1 - math.cos(ang)) * k @ k
        env._imu_gyro_bias = rng.uniform(-0.01, 0.01, 3)
        env.domain_rand = True
    env.imu_noise = float(env.imu_noise) * imu_noise_x
    if model:
        simh.apply_model(env, model)
        if model.get("alpha"):
            simh.set_series_encoder(env, model["alpha"])
        if model.get("play") is not None:
            simh.set_play(env, model["play"], model.get("encoder_side", True), model.get("dq_mode", "fd"))
    nj = env._nq_act; fw = env.obs_frame
    env.set_command(*simh.STAND)
    if q0 is None:
        q0 = rec.load()["q"][0]
    simh.place_and_hold(env, np.clip(q0, env._lo, env._hi), settle_s)
    env._prev_action[:] = 0.0
    env.set_command(*simh.STAND)
    if pin_clock:
        H.pin_gait_clock(env)
    env._obs_hist = []
    first = env._obs()
    env._obs_hist = [first.copy() for _ in range(env.obs_hist_len - 1)]
    obs = np.concatenate([first] + list(env._obs_hist))
    qm = first[:nj].astype(np.float64)
    sh = simh.Shaper(shaper_hz); sh.reset(qm)
    sv = servo or simh.Servo(); sv.reset(qm)
    hold = qm.copy()
    cur = {"t": hold}
    env._action_to_ctrl = lambda a_: cur["t"]
    n = int(round(run_s / env.control_dt))
    A, G, TL, Q, GO, F = [], [], [], [], [], []
    hist = [obs[:fw].copy()]
    sens = np.r_[0:2 * nj + 9]
    for i in range(n):
        o_in = obs.copy()
        if sensor_delay > 0 and len(hist) > sensor_delay:
            for h in range(env.obs_hist_len):
                o_in[h * fw + sens] = hist[max(len(hist) - 1 - h - sensor_delay, 0)][sens]
        if obs_hook:
            obs_hook(o_in, env)
        a = np.clip(act(o_in), -1, 1)
        a_cmd = a
        if out_gain != 1.0:
            # loop-gain probe: the policy's action DEVIATION from its own 0.5 s mean is
            # amplified on the way to the servos (prev_action stays the raw output) --
            # a plant k x more responsive at every frequency above ~0.3 Hz
            ema = a.copy() if i == 0 else ema + (1 - math.exp(-0.02 / 0.5)) * (a - ema)
            a_cmd = np.clip(ema + out_gain * (a - ema), -1, 1)
        ang = rec.action_to_angles(a_cmd)
        tgt = sh(ang)
        if i < RAMP:
            tgt = hold + (i + 1) / RAMP * (tgt - hold)
        cur["t"] = sv(tgt)
        if push_mps > 0 and i in PUSHES:
            dx, dy = PUSHES[i]
            env.data.qvel[0] += push_mps * dx; env.data.qvel[1] += push_mps * dy
        obs, _, term, _, info = env.step(a)
        hist.append(obs[:fw].copy()); hist = hist[-8:]
        d = env.data
        A.append(a); G.append(np.asarray(o_in[30:33])); GO.append(cur["t"].copy())
        Q.append(np.asarray(obs[:nj]))
        TL.append(math.degrees(math.acos(max(-1.0, min(1.0, float(info["up_z"]))))))
        if term:
            break
    A, G, TL = np.array(A), np.array(G), np.array(TL)
    m = flail_metrics(None, A, G, TL)
    m.update(fell=bool(len(A) < n), seed=seed, plant=plant)
    if record:
        m["trace"] = dict(a=A, gyro=G, tilt=TL, q=np.array(Q), goal=np.array(GO))
    return m
