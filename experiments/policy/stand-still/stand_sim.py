"""Zero-command stand in sim through the firmware path, with the 2026-11-02 bench settings
(obsfreeze gain=0.5 -> gyro obs x0.5, shape 6 -> 6 Hz C2 shaper) on the measured actuator
(rca/simh.MEAS_SERVO + JF_PLANT on the fitted plant), and the stand-still candidates:

  clock      the policy's trained clock (trained_hz: 1.5 Hz unless the run set --gait-freq)
             "run"        the firmware today: phase advances at that clock at the stand command
             "freeze"     phase held (at `phase0`, else wherever the clock was at the arm tick)
             "slow"       phase advances at `clock_hz` (below the trained clock)
             "ramp"       clock frequency ramps trained -> 0 over `ramp_s` once the stand gate opens
             "coast"      at a plain stand the clock keeps its rate until the phase reaches
                          `stand_phase`, then holds there (the firmware `standclock coast`)
  The phase is advanced HERE (env._gait_freq = 0, so walker_env never moves it), the
  firmware's way: advanced before the frame it feeds. `cmd_schedule` [(t_s, cmd7), ...]
  changes the command mid-episode (the walk -> stop probe); the stand gate is ctrl_task's.
  act_filter None | ("lpf", hz) first-order low-pass on the action at zero command
             | ("db", deg) per-joint deadband (play operator) on the shaped target
             | ("notch", q) notches at the clock and its first harmonic on the action
  prev_action always stays the RAW policy output (what the policy saw in training).

Episode = rca/takeover.episode's arm path (history of the first frame, prev 0, shaper
seeded at measured q, 1 s ramp from the arm pose) for run_s, then (hold_s > 0) the goal
is FROZEN at the last written target for hold_s: the sim's static-hold power at the same
pose, the analogue of the robot's soft-STOP hold."""
from __future__ import annotations
import math, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import jitter as J  # noqa: E402  (puts rca/ on sys.path)
import rec, simh, ingredients  # noqa: E402
import harness as H  # noqa: E402  (sim/sil/harness.py, via simh)

RAMP = 50
PUSHES = {150: (1.0, 0.0), 275: (-1.0, 0.0), 400: (0.0, 1.0)}   # takeover.PUSHES
CLOCK_HZ = 1.5                                 # a run without its own gait_freq_range
_Q0 = {}


def trained_hz(env):
    """The clock env's run trained at (config.json's gait_freq_range midpoint, the value
    gen_obs_spec.py emits as obs::kGaitFreqHz)."""
    return float(getattr(env, "deploy_gait_freq", CLOCK_HZ))


def arm_pose():
    """The robot's measured pose at the 2026-11-02 stand2 arm tick (policy order, rad)."""
    if "q" not in _Q0:
        r = J._window(rec.load(J.STAND2), J.T_ARM, J.T_ARM + 1.0)
        _Q0["q"] = r["q"][0]
    return _Q0["q"]


class Notch:
    """Biquad notches, by default at the 1.5 Hz clock and 3 Hz (RBJ), per channel, fs 50 Hz."""
    def __init__(self, q=2.0, freqs=(1.5, 3.0)):
        self.c = []
        for f0 in freqs:
            w = 2 * math.pi * f0 / 50.0; al = math.sin(w) / (2 * q); cw = math.cos(w)
            b = np.array([1, -2 * cw, 1]) / (1 + al); a = np.array([1, -2 * cw / (1 + al), (1 - al) / (1 + al)])
            self.c.append((b, a))
        self.z = None

    def __call__(self, x):
        if self.z is None:
            self.z = [[x.copy(), x.copy(), x.copy(), x.copy()] for _ in self.c]   # x1 x2 y1 y2
        y = x
        for (b, a), z in zip(self.c, self.z):
            x0 = y
            y = b[0] * x0 + b[1] * z[0] + b[2] * z[1] - a[1] * z[2] - a[2] * z[3]
            z[1], z[0], z[3], z[2] = z[0], x0.copy(), z[2], y.copy()
        return y


def stand_gate(c):
    """ctrl_task.cpp's plain-stand gate (the kClockStandFreeze / flail-guard test)."""
    return abs(c[0]) <= 0.05 and abs(c[1]) <= 0.05 and abs(c[2]) <= 0.05 and abs(c[4]) < 0.5


def _wrap(p):
    return (p + math.pi) % (2 * math.pi) - math.pi


def advance_clock(clk, mode, v, cmd, dt):
    """One tick of the candidate clock at clk["hz"] (the policy's; CLOCK_HZ if absent); returns
    the phase the next frame carries."""
    hz = clk.get("hz", CLOCK_HZ)
    stand = stand_gate(cmd)
    if not stand:
        clk["held"] = False; clk["f"] = hz
        clk["ph"] = _wrap(clk["ph"] + 2 * math.pi * hz * dt)
        return clk["ph"]
    if mode == "run":
        f = hz
    elif mode == "slow":
        f = float(v["clock_hz"])
    elif mode == "freeze":
        f = 0.0
    elif mode == "ramp":
        clk["f"] = max(0.0, clk["f"] - hz * dt / float(v.get("ramp_s", 2.0))); f = clk["f"]
    elif mode == "coast":
        if clk["held"]:
            return clk["ph"]
        step = 2 * math.pi * hz * dt
        to_go = (float(v["stand_phase"]) - clk["ph"]) % (2 * math.pi)
        if 2 * math.pi - to_go <= float(v.get("coast_back", 0.0)):  # just past it: hold, don't run another cycle
            clk["ph"] = _wrap(float(v["stand_phase"])); clk["held"] = True
            return clk["ph"]
        if to_go <= step:
            clk["ph"] = _wrap(float(v["stand_phase"])); clk["held"] = True
            return clk["ph"]
        f = hz
    else:
        raise ValueError(mode)
    clk["ph"] = _wrap(clk["ph"] + 2 * math.pi * f * dt)
    return clk["ph"]


def episode(v, plant="fit", seed=0, run_s=20.0, hold_s=2.0, record=False):
    """v: variant dict. Keys (all optional): run, typed (servo dict), model, shaper_hz (6),
    gyro_obs (0.5), obs (extra ingredients.obs_hook spec), out_gain, push, clock, phase0,
    clock_hz, ramp_s, act_filter, sensor_delay. Returns metrics (+ trace if record)."""
    run = v.get("run", simh.RUN_NAME)
    settle_s = 1.5
    env, act, zoff = simh.build(plant, run=run, ep_s=run_s + hold_s + settle_s + 1.0)
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
    model = v.get("model", simh.JF_PLANT if "typed" in v else None)
    if model:
        simh.apply_model(env, model)
    nj = env._nq_act
    env.set_command(*simh.STAND)
    simh.place_and_hold(env, np.clip(arm_pose(), env._lo, env._hi), settle_s)
    env._prev_action[:] = 0.0
    env.set_command(*simh.STAND)
    sched = list(v.get("cmd_schedule") or [])
    if sched:
        env.set_command(*sched[0][1])
    hz = trained_hz(env)
    w0 = H.pin_gait_clock(env, hz)
    clock = v.get("clock", "run")
    ph = float(v["phase0"]) if v.get("phase0") is not None and clock != "run" else float(w0)
    env._gait_freq = 0.0                      # the clock is advanced below, not by walker_env
    env._gait_phase = ph
    clk = dict(ph=ph, f=hz, hz=hz, held=False)
    env._obs_hist = []
    first = env._obs()
    env._obs_hist = [first.copy() for _ in range(env.obs_hist_len - 1)]
    obs = np.concatenate([first] + list(env._obs_hist))
    qm = first[:nj].astype(np.float64)
    sh = simh.Shaper(v.get("shaper_hz", 6.0)); sh.reset(qm)
    sv = simh.TypedServo(**v["typed"]) if "typed" in v else simh.Servo(**v.get("servo", {}))
    sv.reset(qm)
    ospec = dict(v.get("obs") or {})
    ospec["gyro_gain"] = ospec.get("gyro_gain", 1.0) * v.get("gyro_obs", 0.5)
    hook = ingredients.obs_hook(ospec) if ospec["gyro_gain"] != 1.0 or len(ospec) > 1 else None
    out_gain = v.get("out_gain", 1.0); push = v.get("push", 0.0)
    filt = v.get("act_filter")
    if filt and filt[0] == "notch":
        notch = Notch(q=filt[1], freqs=(hz, 2 * hz))
    hold = qm.copy(); cur = {"t": hold}
    env._action_to_ctrl = lambda a_: cur["t"]
    n = int(round(run_s / env.control_dt)); nh = int(round(hold_s / env.control_dt))
    A, GO, GY, PW, TL, PH = [], [], [], [], [], []
    a_f = None; db_p = None; fell = False
    for i in range(n + nh):
        holding = i >= n
        if not holding:
            o_in = obs.copy()
            if hook:
                hook(o_in, env)
            a = np.clip(act(o_in), -1, 1)
            a_cmd = a
            if out_gain != 1.0:
                ema = a.copy() if i == 0 else ema + (1 - math.exp(-0.02 / 0.5)) * (a - ema)
                a_cmd = np.clip(ema + out_gain * (a - ema), -1, 1)
            if filt and filt[0] == "lpf":
                kf = 1 - math.exp(-2 * math.pi * filt[1] * 0.02)
                a_f = a_cmd.copy() if a_f is None else a_f + kf * (a_cmd - a_f)
                a_cmd = a_f
            elif filt and filt[0] == "notch":
                a_cmd = notch(a_cmd)
            tgt = sh(rec.action_to_angles(a_cmd))
            if filt and filt[0] == "db":
                h = 0.5 * math.radians(filt[1])
                db_p = tgt.copy() if db_p is None else np.clip(db_p, tgt - h, tgt + h)
                tgt = db_p.copy()
            if i < RAMP:
                tgt = hold + (i + 1) / RAMP * (tgt - hold)
            last_tgt = tgt
        else:
            tgt = last_tgt                      # static hold at the last written goal
            a = env._prev_action.copy()
        cur["t"] = sv(tgt)
        if push > 0 and i in PUSHES:
            dx, dy = PUSHES[i]
            env.data.qvel[0] += push * dx; env.data.qvel[1] += push * dy
        t_next = (i + 1) * env.control_dt
        for ts, c in sched:
            if abs(ts - t_next) < 1e-6:
                env.set_command(*c)
        env._gait_phase = advance_clock(clk, clock, v, env._cmd, env.control_dt)
        obs, _, term, _, info = env.step(a)
        d = env.data
        A.append(a); GO.append(cur["t"].copy()); GY.append(np.array(d.qvel[3:6]))
        PW.append(float(info.get("power_w", np.nan))); PH.append(env._gait_phase)
        TL.append(math.degrees(math.acos(max(-1.0, min(1.0, float(info["up_z"]))))))
        if term:
            fell = True
            break
    A, GO, GY, PW, TL = map(np.array, (A, GO, GY, PW, TL))
    m = dict(seed=seed, plant=plant, fell=fell, t_end=len(A) / 50.0, tilt_max=float(TL.max()))
    w = slice(int(5.0 * 50), min(n, len(A)))     # past ramp + settle, before the hold
    if not fell and n - 250 > 100:
        m.update(J.jitter(GO[w], GY[w]))
        m["power_w"] = float(PW[w].mean())
        m["power_hold_w"] = float(PW[n + nh // 2:].mean()) if len(PW) >= n + nh else float("nan")
        m["arate"] = float(np.mean(np.sqrt(np.sum(np.diff(A[w], axis=0) ** 2, 1))))
    if record:
        m["trace"] = dict(a=A, goal=GO, gyro=GY, power=PW, tilt=TL, phase=np.array(PH))
    return m
