"""Grade policies through the deployed firmware chain on the FAST measured actuator, with the
flail guard judged on every tick (2026-11-02, shaper-train).

  python grade.py [seeds] [out.json] [variant-set]          (CPU: CUDA_VISIBLE_DEVICES=)
  RUNS=a,b python grade.py 8 out.json runs                  # each run at its trained pole

Every variant: NO obsfreeze (gyro obs x1), the C2 shaper at `shaper_hz`, standclock coast
to pi/2 (the firmware's default target), fast STS3215 (reg 85 = 0 / 81 = 1: dead 20 ms,
tau 33 ms) on the joint-fit plant -- stand_sim.episode's arm path (history of the first
frame, prev 0, shaper seeded at measured q, 1 s takeover ramp from the stand2 arm pose).

Probes (falls / flail-guard trips out of the seeds):
  stand            20 s at the zero command + a 2 s static hold: goal jitter, body rate,
                   power proxy, and the SQUEEZE: sum over joints of |written goal - q|,
                   deg -- the robot's armed-stand cost is targets the legs cannot reach
  x1.5             loop gain x1.5 (the action's deviation from its 0.5 s mean), 20 s
  push 0.4         kicks at 3 / 5.5 / 8 s
  walk 0.3 -> stop 4 s at 0.3 m/s then the stand (training's walk band is |vx| >= 0.3)
  stride1 <v>      link/sources.py `stride1`: stand, at t = 2 s vx <v> for one gait cycle
                   (0.667 s at 1.5 Hz; every cycle-based probe follows the policy's trained
                   clock, Stride.at), then the stand -- the robot's first walk command
                   (walk16_act, v 0.6) tripped the guard on body rate 0.28 s in
  stride1 0.6 mu<m> the same on a slippery floor: effective contact friction mu (and a low
                   torsion) on the floor AND the pads (set_contact) -- walk18_shp10sq on the
                   robot (2026-11-03, smooth board) yawed -19 deg in 180 ms and fell
  steps5 0.3 refit vx 0.3 for 2.5 gait cycles from t = 2 s (the robot's steps5), refit actuator
  ... dqlag        the probe with the ROBOT's joint-velocity observation: the STS3215 reg 58
                   is the mean joint velocity over the last 40 ms in 50 steps/s steps
                   (hw_sessions/2026-11-05/kick_analysis.md section 3) -- walker_env
                   dq_obs_window 2, dq_obs_quantum 50. DQ_OBS_WINDOW=N or lo,hi (and
                   DQ_OBS_QUANTUM=Q, default 50) applies it to every probe instead.
  march5 <robot>   `link/sources.py march5` at the policy's clock: the lift channel alternating
                   on the gait clock for 5 full swings (a half-way first one, then R L R L R)
                   with c6 = 2 cm |sin|, then the stand. Per FOOT and per STEP the peak lift
                   (the bar: >= 3 cm on every step), per side the weight-transfer shift while
                   that foot is up (shift_when_up, bar |shift| >= 0.6), the torso's horizontal
                   drift and yaw over the run, falls and guard trips. The commanded channels
                   a run never saw move (normaliser std 0: c4 and c6 of every run that trained
                   no march) are PINNED to their trained value as the firmware does
                   (policy::pinFrozen), so such a student is shown what the robot would do:
                   stand. MARCH_PIN=0 feeds the raw command instead (a student out of
                   distribution by ~1e6 sigma).
The flail guard (firmware/components/flailguard, kLimits): rate EMA(|gyro|, 0.2 s) >
1.5 rad/s; tilt > 20 deg for 200 ms; chatter EMA(mean |d raw target / dt|, 0.1 s) >
7 rad/s; pose (plain stand, after the ramp) any written target > 15 deg from the stand pose
for 300 ms. A trip is counted (first one, with its time); the episode runs on, so a fall is
also reported."""
from __future__ import annotations
import json, math, os, sys
from concurrent.futures import ProcessPoolExecutor
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SS = os.path.join(HERE, "..", "stand-still")
sys.path.insert(0, SS)
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "link"))
import stand_sim as S  # noqa: E402
import simh, rec  # noqa: E402

# runs trained in this worktree (sim/runs) are found too
_RUNS_HERE = os.path.join(rec.ROOT, "sim", "runs")
_run_root0 = simh.run_root


def _run_root(run):
    if os.path.exists(os.path.join(_RUNS_HERE, run, "params.pkl")):
        return _RUNS_HERE
    return _run_root0(run)


simh.run_root = _run_root

if os.environ.get("GRADE_XML"):     # grade every run on one plant (e.g. bimo_biped_v6legs_meas.xml)
    _GX = os.environ["GRADE_XML"]
    simh.ESA._xml = lambda cfg: _GX if os.path.isabs(_GX) else os.path.join(rec.ROOT, "sim", _GX)

FAST = dict(simh.MEAS_SERVO, dead15=20.0, tau15=33.0)
# the actuator refit (experiments/policy/actuator-refit, 2026-11-03): the hanging 60 deg max-speed
# moves + the robot's stride1 swing (R hip pitch -31 deg at +180 ms) -- hip pitch / knee
# torque-speed envelope at the robot's 12.0 V, and the fast servo's dead time under the
# stride's large goal steps (40 ms; the 7 deg stepresp steps give 18-27)
REFIT_PLANT = dict(supply_v=12.0, stall_scale=dict(hip_pitch=2.0, knee=2.0),
                   w0_scale=dict(hip_pitch=1.325, knee=1.281))
REFIT_SERVO = dict(FAST, dead15=40.0)
STAND_CMD = simh.STAND
P2 = math.pi / 2
# the robot's dq obs (reg 58): mean velocity over 2 ticks (40 ms), 50 steps/s quantum
DQ_ROBOT = dict(window=2, quantum=50.0)
DT = 0.02
FG = dict(rate=1.5, rate_tau=0.2, tilt=20.0, tilt_ticks=10, chatter=7.0, chatter_tau=0.1,
          pose_deg=15.0, pose_ticks=15)


def walk(vx):
    return (vx, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)


class Stride(list):
    """stride()'s schedule [(t_s, cmd), ...] at a clock: `cycles` gait cycles of vx from t0, plus
    extra_s of absolute time (how far the robot's command misses the cycle). The list is the 1.5 Hz
    one; episode() re-times it to the graded policy's clock with at()."""
    def __init__(self, vx, t0=2.0, cycles=1.0, extra_s=0.0, hz=S.CLOCK_HZ):
        super().__init__([(0.0, STAND_CMD), (t0, walk(vx)), (t0 + (cycles / hz + extra_s), STAND_CMD)])
        self.spec = (vx, t0, cycles, extra_s)

    def at(self, hz):
        return Stride(*self.spec, hz=hz)


def stride(vx, t0=2.0, cycles=1.0, extra_s=0.0):
    return Stride(vx, t0, cycles, extra_s)


class March(list):
    """link/sources.py march5's schedule [(t_s, cmd), ...], one entry per control tick: `n` full
    swings (after the half-way first one) of the lift channel alternating on the gait clock at
    `hz`, c6 = dz |sin|, from t0, then the stand. The list is the 1.5 Hz one; episode() re-times
    it to the graded policy's clock with at() (the march IS clock-locked: the clock runs from the
    stand phase pi/2 the tick the lift is commanded, so time maps to phase). The commands come
    from sources.march_cmd, the function the robot's script sends."""
    def __init__(self, n=5, t0=2.0, hz=S.CLOCK_HZ, dz=0.02):
        import sources as L
        T = L.march_swings_s(n, hz)
        sched = [(0.0, STAND_CMD)]
        for k in range(int(round(T / DT))):
            lift, c6 = L.march_cmd(k * DT, hz, dz)
            sched.append((t0 + k * DT, (0.0, 0.0, 0.0, 1.0, lift, 0.0, c6)))
        sched.append((t0 + int(round(T / DT)) * DT, STAND_CMD))
        super().__init__(sched)
        self.spec = (n, t0, dz)
        self.hz, self.t0, self.n = hz, t0, n

    def at(self, hz):
        return March(*self.spec[:2], hz=hz, dz=self.spec[2])


def march(n=5, t0=2.0, dz=None):
    return March(n, t0, dz=float(os.environ.get("MARCH_DZ", "0.02")) if dz is None else dz)


PROBES = {
    "stand": dict(run_s=20.0, hold_s=2.0),
    "x1.5": dict(out_gain=1.5, run_s=20.0),
    "push 0.4": dict(push=0.4, run_s=12.0),
    "walk 0.3 -> stop": dict(cmd_schedule=[(0.0, walk(0.3)), (4.0, STAND_CMD)], run_s=14.0),
    "walk 0.2 -> stop": dict(cmd_schedule=[(0.0, walk(0.2)), (4.0, STAND_CMD)], run_s=14.0),
    "walk 0.15 -> stop": dict(cmd_schedule=[(0.0, walk(0.15)), (4.0, STAND_CMD)], run_s=14.0),
    "x2": dict(out_gain=2.0, run_s=20.0),
    "push 0.5": dict(push=0.5, run_s=12.0),
    "slow servo": dict(typed=dict(simh.MEAS_SERVO), run_s=20.0),
    "stride1 0.6": dict(cmd_schedule=stride(0.6), run_s=8.0),
    "stride1 0.4": dict(cmd_schedule=stride(0.4), run_s=8.0),
    "stride1 0.3": dict(cmd_schedule=stride(0.3), run_s=8.0),
    # the low-friction walk-start gate (experiments/policy/friction-dr): effective contact mu
    # and torsion forced on the floor AND the pads -- the robot's 2026-11-03 stride1 spun -19 deg
    # in 180 ms on a smooth board (TPU sole, mu ~0.4-0.6)
    "stride1 0.6 mu0.4": dict(cmd_schedule=stride(0.6), run_s=8.0, contact=dict(mu=0.4, torsion=0.005)),
    "stride1 0.6 mu0.3": dict(cmd_schedule=stride(0.6), run_s=8.0, contact=dict(mu=0.3, torsion=0.002)),
    # the same on the REFIT actuator, where walk18_shp10sq's leg swings as fast as on the robot
    "stride1 0.6 refit": dict(cmd_schedule=stride(0.6), run_s=8.0, plant=REFIT_PLANT, typed=REFIT_SERVO),
    "stride1 0.6 mu0.4 refit": dict(cmd_schedule=stride(0.6), run_s=8.0, plant=REFIT_PLANT, typed=REFIT_SERVO,
                                    contact=dict(mu=0.4, torsion=0.005)),
}

# 2026-11-05 (stride first, measured-sole plant): the stride ladder on the REFIT actuator. One
# stride at the slower deployable commands (stride1_04 / stride1_03 in link/sources.py), "a
# few strides" (3 gait cycles), and walk -> stop -- each at mu 1 and 0.4. Every probe also
# reports walk_dx_2s5: a policy that passes the guard by NOT walking (a strong gyro hinge
# taught that, dx2.5 0.06 m) must not count -- see walks().
_R = dict(plant=REFIT_PLANT, typed=REFIT_SERVO)
_MU = dict(contact=dict(mu=0.4, torsion=0.005))
for _v in (0.4, 0.3):
    PROBES[f"stride1 {_v} refit"] = dict(cmd_schedule=stride(_v), run_s=8.0, **_R)
    PROBES[f"stride1 {_v} mu0.4 refit"] = dict(cmd_schedule=stride(_v), run_s=8.0, **_R, **_MU)
PROBES["steps5 0.3 refit"] = dict(cmd_schedule=stride(0.3, cycles=2.5), run_s=10.0, **_R)
PROBES["stride1 0.3 refit dqlag"] = dict(PROBES["stride1 0.3 refit"], dq_obs=DQ_ROBOT)
PROBES["steps5 0.3 refit dqlag"] = dict(PROBES["steps5 0.3 refit"], dq_obs=DQ_ROBOT)


class YawKicks(dict):
    """{tick: dw}: one kick of dw rad/s per half gait cycle (in single support; a kick in double support
    dies in the two feet's grip), `lead` s after the walk command at t0. Calibrated on the robot's steps5
    run (experiments/policy/yaw-gap/README.md). The dict is the 1.5 Hz one; episode() re-times it with at()."""
    def __init__(self, dw, t0=2.0, n=2, lead=0.25, hz=S.CLOCK_HZ):
        super().__init__({int(round((t0 + lead + k * 0.5 / hz) / DT)): dw for k in range(n)})
        self.spec = (dw, t0, n, lead)

    def at(self, hz):
        return YawKicks(*self.spec, hz=hz)


def yaw_kicks(dw, t0=2.0, n=2, lead=0.25):
    return YawKicks(dw, t0, n, lead)


# The robot's command does not end on the sim's tick grid: stride1_03 on 2026-11-06 stopped at +700 ms, 40 ms
# after the coasting clock had passed the stand phase, so the clock ran another whole cycle under the stand
# command (experiments/policy/yaw-gap/README.md). "late <ms>" ends the command that long after the cycle.
for _ms in (20, 40, 60, 100):
    _c = stride(0.3, extra_s=_ms / 1e3)
    PROBES[f"stride1 0.3 refit dqlag late{_ms}"] = dict(PROBES["stride1 0.3 refit dqlag"], cmd_schedule=_c)
for _ms in (-20, -40, -60):
    _c = stride(0.3, extra_s=_ms / 1e3)
    PROBES[f"stride1 0.3 refit dqlag early{-_ms}"] = dict(PROBES["stride1 0.3 refit dqlag"], cmd_schedule=_c)
# 2026-11-06, the robot's own peak speed: hip pitch 246-254 and knee 281-303 deg/s in all three flight
# records (11-03 walk18, 11-06 omni5 t1, t2), where the refit runs 360+. With this cap and 60 ms of dead
# time the joints and the final yaw come close, not the body rate (1.4 - 2.1 rad/s; robot 3.07): the `dq5`,
# `dq6` and `dead80` probes grade the delay margin. experiments/policy/yaw-gap/README.md
ROBOT_SPEED_PLANT = dict(supply_v=12.0, stall_scale=dict(hip_pitch=2.0, knee=2.0),
                         w0_scale=dict(hip_pitch=0.87, knee=1.0))
ROBOT_SPEED_SERVO = dict(FAST, dead15=60.0)
_RS = dict(plant=ROBOT_SPEED_PLANT, typed=ROBOT_SPEED_SERVO, dq_obs=DQ_ROBOT)
PROBES["stride1 0.3 robot"] = dict(cmd_schedule=stride(0.3), run_s=8.0, **_RS)
PROBES["stride1 0.3 robot early40"] = dict(cmd_schedule=stride(0.3, extra_s=-0.04), run_s=8.0, **_RS)
PROBES["stride1 0.3 robot late40"] = dict(cmd_schedule=stride(0.3, extra_s=0.04), run_s=8.0, **_RS)
PROBES["steps5 0.3 robot"] = dict(cmd_schedule=stride(0.3, cycles=2.5), run_s=10.0, **_RS)
PROBES["back3 -0.3 robot"] = dict(cmd_schedule=stride(-0.3, cycles=1.5), run_s=10.0, **_RS)
PROBES["walk 0.3 -> stop robot"] = dict(PROBES["walk 0.3 -> stop"], **_RS)
for _w in (5, 6):
    PROBES[f"steps5 0.3 robot dq{_w}"] = dict(PROBES["steps5 0.3 robot"], dq_obs=dict(DQ_ROBOT, window=_w))
# the first walking milestone (#261): march in place, alternating full lifts, no velocity command.
# 3.7 s of march at the slow-gait line's 0.75 Hz; the run is long enough for the slowest trained clock (0.6 Hz: 4.6 s)
PROBES["march5 robot"] = dict(cmd_schedule=march(5), run_s=10.0, march=True, **_RS)
PROBES["march5 refit"] = dict(cmd_schedule=march(5), run_s=10.0, march=True, **_R)
PROBES["steps5 0.3 robot dead80"] = dict(PROBES["steps5 0.3 robot"], typed=dict(ROBOT_SPEED_SERVO, dead15=80.0))
# what a firmware coast that holds when just past the stand phase would do (not on the robot)
for _ms in (20, 40, 60, 100):
    PROBES[f"stride1 0.3 refit dqlag late{_ms} hold"] = dict(PROBES[f"stride1 0.3 refit dqlag late{_ms}"], coast_back=1.0)
for _dw in (-1.0, -2.0, -3.0):
    PROBES[f"stride1 0.3 refit dqlag yk{-_dw:g}"] = dict(PROBES["stride1 0.3 refit dqlag"], yaw_kicks=yaw_kicks(_dw))
    PROBES[f"steps5 0.3 refit dqlag yk{-_dw:g}"] = dict(PROBES["steps5 0.3 refit dqlag"], yaw_kicks=yaw_kicks(_dw, n=5, lead=0.06))
for _v in (0.3, 0.2):
    _c3 = stride(_v, cycles=3.0)
    PROBES[f"stride3 {_v} refit"] = dict(cmd_schedule=_c3, run_s=10.0, **_R)
    PROBES[f"stride3 {_v} mu0.4 refit"] = dict(cmd_schedule=_c3, run_s=10.0, **_R, **_MU)
    PROBES[f"walk {_v} -> stop refit"] = dict(PROBES[f"walk {_v} -> stop"], **_R)
    PROBES[f"walk {_v} -> stop mu0.4 refit"] = dict(PROBES[f"walk {_v} -> stop"], **_R, **_MU)

# 2026-11-05 (Tom: "policies that respond to arbitrary directions: forward, backward, sidestep
# (both ways), turn (both ways)"): a 4 s command each, from the stand at t = 2 s, then the stand.
# The deployed walk25_msole_d13_gh2h14_s1 had silently kept only forward (back -0.01 m, side L
# +0.01 m, turns 8 - 16 deg, side R FELL 3/4) -- walk21_fricact did all six. directions()
# reports travel along each command; DIR_IDEAL is the 4 s ideal.
def _dir(vx=0.0, vy=0.0, wz=0.0):
    return (vx, vy, wz, 1.0, 0.0, 0.0, 0.0)


DIRS = {"dir fwd 0.3": _dir(vx=0.3), "dir back -0.25": _dir(vx=-0.25),
        "dir side L 0.15": _dir(vy=0.15), "dir side R -0.15": _dir(vy=-0.15),
        "dir turn L 0.5": _dir(wz=0.5), "dir turn R -0.5": _dir(wz=-0.5)}
DIR_IDEAL = {"dir fwd 0.3": ("dx", 1.2), "dir back -0.25": ("dx", -1.0),
             "dir side L 0.15": ("dy", 0.6), "dir side R -0.15": ("dy", -0.6),
             "dir turn L 0.5": ("dyaw", 115.0), "dir turn R -0.5": ("dyaw", -115.0)}
for _n, _c in DIRS.items():
    PROBES[_n] = dict(cmd_schedule=[(0.0, STAND_CMD), (2.0, _c), (6.0, STAND_CMD)], run_s=10.0)

WALKS_DX = 0.3      # m in 2.5 s on "walk 0.3 -> stop" (fast servo); the 11-05 walkers did 0.45 - 0.6


def directions(episodes, variant):
    """Per direction probe: (metric, mean travel, ideal, falls) -- dx / dy in m, dyaw in deg."""
    out = {}
    for p, (k, ideal) in DIR_IDEAL.items():
        e = [x for x in episodes if x["variant"] == variant and x["probe"] == p]
        if e:
            vals = [x.get(k) for x in e if x.get(k) is not None and x.get(k) == x.get(k)]
            out[p] = (k, float(np.mean(vals)) if vals else float("nan"), ideal, sum(bool(x["fell"]) for x in e))
    return out


def walks(episodes, variant):
    """True if the variant actually walks: mean walk_dx_2s5 on `walk 0.3 -> stop` >= WALKS_DX."""
    dx = [e.get("walk_dx_2s5") for e in episodes
          if e["variant"] == variant and e["probe"] == "walk 0.3 -> stop"]
    dx = [d for d in dx if d is not None and d == d]
    return bool(dx) and float(np.mean(dx)) >= WALKS_DX


def set_contact(env, mu=None, torsion=None):
    """Force the EFFECTIVE foot/floor contact friction: MuJoCo takes the elementwise MAX of
    the two geoms' friction, so the floor alone cannot go below the pads' 1.0 -- set the
    floor, every sole pad and the reference soles together (sliding mu, torsional m)."""
    mm = env.model
    gids = [mm.geom("floor").id] + [g for pads in env._pad_gids for g in pads] + list(env._sole_gids)
    for g in gids:
        if mu is not None:
            mm.geom_friction[g, 0] = float(mu)
        if torsion is not None:
            mm.geom_friction[g, 1] = float(torsion)


def body_yaw(q):
    w, x, y, z = (float(c) for c in q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class FlailGuard:
    """flailguard::Detector with kLimits, one 20 ms tick at a time."""
    def __init__(self, stand_pose):
        self.stand = np.asarray(stand_pose, float)
        self.rate = 0.0; self.chat = 0.0; self.prev_raw = None
        self.n_tilt = 0; self.n_pose = 0
        self.trip = None; self.peak_rate = 0.0; self.peak_chat = 0.0
        self.rax = np.zeros(3); self.peak_rax = np.zeros(3)
        self.first = {}                     # criterion -> (t_s, value): each one's first trip

    def __call__(self, i, raw, target, gyro, tilt_deg, stand_cmd, armed):
        a_r = 1 - math.exp(-DT / FG["rate_tau"]); a_c = 1 - math.exp(-DT / FG["chatter_tau"])
        self.rate += a_r * (float(np.linalg.norm(gyro)) - self.rate)
        self.rax += a_r * (np.abs(gyro) - self.rax)          # per axis, for the report
        self.peak_rax = np.maximum(self.peak_rax, self.rax)
        if self.prev_raw is not None:
            self.chat += a_c * (float(np.mean(np.abs(raw - self.prev_raw))) / DT - self.chat)
        self.prev_raw = raw.copy()
        self.n_tilt = self.n_tilt + 1 if tilt_deg > FG["tilt"] else 0
        off = np.degrees(np.abs(target - self.stand)).max()
        self.n_pose = self.n_pose + 1 if (stand_cmd and armed and off > FG["pose_deg"]) else 0
        self.peak_rate = max(self.peak_rate, self.rate); self.peak_chat = max(self.peak_chat, self.chat)
        for name, hit, val in (("pose", self.n_pose >= FG["pose_ticks"], off),
                               ("rate", self.rate > FG["rate"], self.rate),
                               ("tilt", self.n_tilt >= FG["tilt_ticks"], tilt_deg),
                               ("chatter", self.chat > FG["chatter"], self.chat)):
            if hit and name not in self.first:
                self.first[name] = (round(i * DT, 2), round(float(val), 3))
                if self.trip is None:
                    self.trip = (name, round(i * DT, 2), round(float(val), 3))


def _decel_peaks(qv, dt, win=5):
    """Peak joint DECELERATION (rad/s^2) per joint from physics-substep joint velocities
    qv (T, n): acceleration as the velocity change over `win` substeps (a ~10 ms window,
    so a single-substep contact spike does not count), deceleration where it opposes the
    velocity."""
    if len(qv) <= win:
        return np.zeros(qv.shape[1] if qv.ndim == 2 else 0)
    acc = (qv[win:] - qv[:-win]) / (win * dt)
    vm = 0.5 * (qv[win:] + qv[:-win])
    dec = np.where(acc * vm < 0, np.abs(acc), 0.0)
    return dec.max(0)


def _dq_env():
    """DQ_OBS_WINDOW=N or lo,hi (DQ_OBS_QUANTUM=Q, default 50): every probe's dq obs."""
    w = os.environ.get("DQ_OBS_WINDOW")
    if not w:
        return None
    return dict(window=w, quantum=float(os.environ.get("DQ_OBS_QUANTUM", "50")))


def at_clock(v, hz):
    """The variant/probe with its cycle-based schedule and kicks re-timed to clock hz."""
    v = dict(v)
    for k in ("cmd_schedule", "yaw_kicks"):
        if hasattr(v.get(k), "at"):
            v[k] = v[k].at(hz)
    return v


_FROZEN = {}


def frozen_cmd(run, env):
    """{command channel: its trained value} for the channels the run never saw move: normaliser
    std at or below policy::kFrozenStdMax (1e-5) -- the slots policy::pinFrozen feeds exactly
    their trained value on the robot. Read off the run's params.pkl (current frame)."""
    if run not in _FROZEN:
        import pickle
        with open(os.path.join(simh.run_root(run), run, "params.pkl"), "rb") as f:
            norm = pickle.load(f)[0]
        std, mean = np.asarray(norm.std).ravel(), np.asarray(norm.mean).ravel()
        off = env.obs_frame - 7                  # the 7 command channels end the frame
        _FROZEN[run] = {k: float(mean[off + k]) for k in range(7) if std[off + k] <= 1e-5}
    return _FROZEN[run]


def episode(v, seed=0, run_s=20.0, hold_s=0.0):
    """stand_sim.episode with the flail guard, the squeeze and walk progress recorded.
    v: run, shaper_hz, out_gain, push, cmd_schedule, clock ('coast'), stand_phase,
    limit (the firmware motion limiter, 'vmax,amax[,jmax]' deg units; after the shaper and
    the takeover ramp, as in ctrl_task)."""
    run = v["run"]; settle_s = 1.5
    env, act, zoff = simh.build("fit", run=run, ep_s=run_s + hold_s + settle_s + 1.0)
    # the policy's own clock: the phase, and the cycle-based command and kick times with it
    hz = S.trained_hz(env)
    v = at_clock(v, hz)
    rng = np.random.default_rng(1000 + seed)
    env.domain_rand = False; env.push_prob = 0.0
    # the dq obs: the probe's (dq_obs), else DQ_OBS_WINDOW, else as the run trained (the
    # env is cached per worker, so set it every episode)
    if not hasattr(env, "_dq_cfg0"):
        env._dq_cfg0 = dict(window=env.dq_obs_window, quantum=env.dq_obs_quantum)
    dq = v.get("dq_obs") or _dq_env() or env._dq_cfg0
    env.set_dq_obs(dq["window"], dq["quantum"])
    env.reset(seed=seed)
    env._zero_off = zoff.copy()
    ang = math.radians(rng.normal(0.0, 1.0)); ax = rng.normal(size=3); ax /= np.linalg.norm(ax)
    k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    env._imu_R = np.eye(3) + math.sin(ang) * k + (1 - math.cos(ang)) * k @ k
    env._imu_gyro_bias = rng.uniform(-0.01, 0.01, 3)
    env.domain_rand = True
    simh.apply_model(env, simh.JF_PLANT)
    if v.get("plant"):
        simh.apply_model(env, dict(simh.JF_PLANT, **v["plant"]))
    if v.get("contact"):
        set_contact(env, **v["contact"])
    nj = env._nq_act
    env.set_command(*STAND_CMD)
    simh.place_and_hold(env, np.clip(S.arm_pose(), env._lo, env._hi), settle_s)
    env._prev_action[:] = 0.0
    sched = list(v.get("cmd_schedule") or [])
    frozen = {}
    if v.get("march") and os.environ.get("MARCH_PIN", "1") != "0":
        # the firmware pins a channel the run never saw move to its trained value, before the
        # clock gate and the policy read it (ctrl_task.cpp pinFrozen)
        frozen = frozen_cmd(run, env)
        sched = [(ts, tuple(frozen.get(k, c[k]) for k in range(7))) for ts, c in sched]
    env.set_command(*(sched[0][1] if sched else STAND_CMD))
    w0 = S.H.pin_gait_clock(env, hz)
    clock = v.get("clock", "coast")
    env._gait_freq = 0.0
    env._gait_phase = float(w0)
    clk = dict(ph=float(w0), f=hz, hz=hz, held=False)
    env._obs_hist = []
    first = env._obs()
    env._obs_hist = [first.copy() for _ in range(env.obs_hist_len - 1)]
    obs = np.concatenate([first] + list(env._obs_hist))
    qm = first[:nj].astype(np.float64)
    sh = simh.Shaper(v["shaper_hz"]); sh.reset(qm)
    lim = simh.Limiter(v["limit"]) if v.get("limit") else None
    if lim is not None:
        lim.reset(qm)
    sv = simh.TypedServo(**v.get("typed", FAST)); sv.reset(qm)
    out_gain = v.get("out_gain", 1.0); push = v.get("push", 0.0)
    hold = qm.copy(); cur = {"t": hold}
    env._action_to_ctrl = lambda a_: cur["t"]
    fg = FlailGuard(env._default)
    n = int(round(run_s / DT)); nh = int(round(hold_s / DT))
    fell = False; ema = None; last_tgt = hold
    x0 = float(env.data.qpos[0])
    # physics-substep joint velocities and body yaw rate (the stride's deceleration / spin)
    import mujoco as _mj
    QV, WZ = [], []
    _step0 = _mj.mj_step

    def _rec_step(m_, d_):
        _step0(m_, d_)
        QV.append(np.array(d_.qvel[env._sqvel])); WZ.append(float(d_.qvel[5]))
    _mj.mj_step = _rec_step
    try:
        res = _episode_loop(env, act, obs, sh, lim, sv, fg, sched, clk, clock, v, hold, cur,
                            out_gain, push, n, nh, last_tgt, x0, nj)
    finally:
        _mj.mj_step = _step0
    GO, GY, PW, TL, SQ, X, fell, ext = res
    m = _episode_metrics(env, GO, GY, PW, TL, SQ, X, fell, fg, sched, n, nh, seed,
                         np.array(QV), np.array(WZ), v=v, hz=hz, **ext)
    if v.get("march"):
        m["frozen_cmd"] = sorted(frozen)          # the channels pinned to their trained value
    return m


def _episode_loop(env, act, obs, sh, lim, sv, fg, sched, clk, clock, v, hold, cur,
                  out_gain, push, n, nh, last_tgt, x0, nj):
    import mujoco
    GO, GY, PW, TL, SQ, X, YW, QM, QD, LM, AC = [], [], [], [], [], [], [], [], [], [], []
    Y = []; y0 = float(env.data.qpos[1])
    FT, C4 = [], []
    fell = False; ema = None
    for i in range(n + nh):
        holding = i >= n
        if not holding:
            a = np.clip(act(obs.copy()), -1, 1)
            a_cmd = a
            if out_gain != 1.0:
                ema = a.copy() if ema is None else ema + (1 - math.exp(-DT / 0.5)) * (a - ema)
                a_cmd = np.clip(ema + out_gain * (a - ema), -1, 1)
            raw = rec.action_to_angles(a_cmd)
            tgt = sh(raw)
            if i < S.RAMP:
                tgt = hold + (i + 1) / S.RAMP * (tgt - hold)
            if lim is not None:
                tgt, _ = lim(tgt)
            last_tgt = tgt
        else:
            tgt = last_tgt; raw = last_tgt; a = env._prev_action.copy()
        cur["t"] = sv(tgt)
        if i in v.get("yaw_kicks", ()):         # extra body yaw rate at swing onset, rad/s
            env.data.qvel[5] += v["yaw_kicks"][i]
        if push > 0 and i in S.PUSHES:
            dx, dy = S.PUSHES[i]
            env.data.qvel[0] += push * dx; env.data.qvel[1] += push * dy
        for ts, c in sched:                     # on the tick grid (stride1's 0.667 s)
            if int(round(ts / DT)) == i + 1:
                env.set_command(*c)
        env._gait_phase = S.advance_clock(clk, clock, v, env._cmd, DT)
        obs, _, term, _, info = env.step(a)
        d = env.data
        gyro = np.array(d.qvel[3:6])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(info["up_z"])))))
        if not holding:
            fg(i, raw, tgt, gyro, tilt, S.stand_gate(env._cmd), i >= S.RAMP)
        q_meas = obs[:nj].astype(np.float64)
        GO.append(tgt.copy()); GY.append(gyro); PW.append(float(info.get("power_w", np.nan)))
        TL.append(tilt); SQ.append(np.degrees(np.abs(tgt - q_meas))); X.append(float(d.qpos[0]) - x0)
        Y.append(float(d.qpos[1]) - y0)
        # soles' height, ground x/y (L then R) and the whole-body CoM x/y: per-foot lift and the
        # weight transfer; and the lift channel the tick was commanded with
        FT.append((float(d.geom_xpos[env._sole_gids[0]][2]), float(d.geom_xpos[env._sole_gids[1]][2]),
                   float(d.geom_xpos[env._sole_gids[0]][0]), float(d.geom_xpos[env._sole_gids[0]][1]),
                   float(d.geom_xpos[env._sole_gids[1]][0]), float(d.geom_xpos[env._sole_gids[1]][1]),
                   float(d.subtree_com[env._torso_bid][0]), float(d.subtree_com[env._torso_bid][1])))
        C4.append(float(env._cmd[4]))
        YW.append(body_yaw(d.qpos[3:7])); QM.append(q_meas)
        QD.append(np.array(d.qvel[env._sqvel])); AC.append(np.array(a, float)); mujoco.mj_subtreeVel(env.model, d)
        LM.append(np.array(d.subtree_angmom[env._torso_bid]))
        if term:
            fell = True
            break
    return GO, GY, PW, TL, SQ, X, fell, dict(YW=YW, QM=QM, QD=QD, LM=LM, AC=AC, Y=Y, FT=FT, C4=C4)


def _shift_frac(ft):
    """CoM position along the line between the soles, per tick: +1 = over the L foot, -1 = over
    the R foot, 0 = between them (ft: the FT rows, tick x 8)."""
    L, R, C = ft[:, 2:4], ft[:, 4:6], ft[:, 6:8]
    mid = 0.5 * (L + R)
    u = L - R
    half = 0.5 * np.linalg.norm(u, axis=1)
    u = u / np.maximum(2 * half, 1e-6)[:, None]
    return np.sum((C - mid) * u, axis=1) / np.maximum(half, 1e-6)


UP_CM = 0.01     # a foot is "up" above 1 cm


def feet_metrics(FT, i0=75):
    """Per-foot clearance and lateral weight transfer after the takeover ramp (the 11-09 robot:
    the R foot never lifted, the CoM stayed 0.14-0.23 of the way toward the stance foot):
    lift_cm = [L, R] peak sole height above the stand height; shift_when_up = [L-up, R-up]: how far
    the CoM sits toward the other (stance) foot while a foot is up > 1 cm (L up: the most negative
    value, toward R; R up: the most positive, toward L) -- the bar is |shift| >= 0.6."""
    ft = np.asarray(FT, float)
    if len(ft) <= 100:
        return {}
    z0 = np.median(ft[:50, 0:2], axis=0)
    lift = ft[i0:, 0:2] - z0
    f = _shift_frac(ft[i0:])
    up = lift > UP_CM
    return dict(lift_cm=(lift.max(0) * 100).round(2).tolist(),
                lift_steps=[int(((lift[1:, k] > UP_CM) & (lift[:-1, k] <= UP_CM)).sum()) for k in (0, 1)],
                shift_frac=[round(float(f.max()), 2), round(float(f.min()), 2)],
                shift_when_up=[round(float(f[up[:, 0]].min()), 2) if up[:, 0].any() else None,
                               round(float(f[up[:, 1]].max()), 2) if up[:, 1].any() else None])


MARCH_BAR_CM = 3.0     # Tom, #137, 2026-11-10: every step lifts the foot >= 3 cm
MARCH_SHIFT_BAR = 0.6


def march_metrics(FT, C4, X, Y, YW, sched, hz, fell):
    """The march probe: per step and per foot the peak lift, the weight transfer, the drift.

    A step is one run of the commanded lift channel (c4 -1 = L, +1 = R) inside the march window;
    the first run is the half-way L swing the clock's coast phase starts in (reported as `lead`),
    the rest are full swings. Each is scored by its commanded foot's peak height above the stand
    height. Drift and yaw run from the march's first tick to the end of the episode (the 2.7 s
    after it included: the stand that follows). Per-seed pass = no fall, every full step
    >= MARCH_BAR_CM, and |shift| >= MARCH_SHIFT_BAR on both sides."""
    ft = np.asarray(FT, float)
    c4 = np.asarray(C4, float)
    marching = [ts for ts, c in sched if abs(c[4]) >= 0.5]
    if len(ft) <= 100 or not marching:
        return {}
    # an entry at ts is set before tick ts/DT - 1's step and acts from that tick
    i0 = int(round(marching[0] / DT)) - 1
    i1 = int(round(marching[-1] / DT)) - 1
    z0 = np.median(ft[:50, 0:2], axis=0)
    lift = ft[:, 0:2] - z0
    steps = []                                           # (foot, peak cm, first tick)
    k = i0
    n = len(c4)
    while k < min(i1 + 1, n):
        side = 0 if c4[k] < -0.5 else (1 if c4[k] > 0.5 else None)
        j = k
        while j + 1 < n and c4[j + 1] == c4[k]:
            j += 1
        if side is not None:
            steps.append((side, float(lift[k:j + 2, side].max()) * 100.0, k))
        k = j + 1
    lead, full = steps[:1], steps[1:]
    win = slice(i0, min(n, i1 + int(0.7 / DT)))          # the march plus the last foot's landing
    f = _shift_frac(ft[win])
    up = lift[win] > UP_CM
    shift_l = float(f[up[:, 0]].min()) if up[:, 0].any() else float("nan")
    shift_r = float(f[up[:, 1]].max()) if up[:, 1].any() else float("nan")
    xs, ys = np.asarray(X, float), np.asarray(Y, float)
    yw = np.degrees(np.unwrap(np.asarray(YW, float)))
    end = len(xs) - 1
    peaks_l = [p for s, p, _ in full if s == 0]
    peaks_r = [p for s, p, _ in full if s == 1]
    m = dict(march_steps=[[("L", "R")[s], round(p, 2)] for s, p, _ in full],
             march_lead_cm=round(lead[0][1], 2) if lead else None,
             march_n=len(full), march_min_cm=round(min((p for _, p, _ in full), default=0.0), 2),
             march_min_l_cm=round(min(peaks_l, default=0.0), 2), march_min_r_cm=round(min(peaks_r, default=0.0), 2),
             march_all_cm=[round(float(x), 2) for x in (lift[win].max(0) * 100)],
             march_shift_l=round(shift_l, 2), march_shift_r=round(shift_r, 2),
             march_drift_m=round(float(np.hypot(xs[end] - xs[i0], ys[end] - ys[i0])), 3),
             march_dx=round(float(xs[end] - xs[i0]), 3), march_dy=round(float(ys[end] - ys[i0]), 3),
             march_yaw_deg=round(float(yw[end] - yw[i0]), 1))
    n_asked = int(round(2 * hz * len(marching) * DT - 0.5))      # the full swings the schedule asks for
    m["march_steps_ok"] = int(sum(p >= MARCH_BAR_CM for _, p, _ in full))
    m["march_shift_ok"] = bool(abs(shift_l) >= MARCH_SHIFT_BAR and abs(shift_r) >= MARCH_SHIFT_BAR)
    m["march_pass"] = bool(not fell and len(full) >= n_asked and m["march_steps_ok"] == len(full)
                           and m["march_shift_ok"])
    m["march_lifts_pass"] = bool(not fell and len(full) >= n_asked and m["march_steps_ok"] == len(full))
    return m


def _episode_metrics(env, GO, GY, PW, TL, SQ, X, fell, fg, sched, n, nh, seed, QV, WZ,
                     v=None, hz=S.CLOCK_HZ, YW=(), QM=(), QD=(), LM=(), AC=(), Y=(), FT=(), C4=()):
    v = v or {}
    GO, GY, PW, TL, SQ = map(np.array, (GO, GY, PW, TL, SQ))
    # the training smoothness terms as graded (env_mjx w_qacc / w_action_acc): per-tick means of
    # sum qdd^2 (servo DOFs, rad/s^2) and sum (a_t - 2 a_t-1 + a_t-2)^2 over the policy part
    qd_all = np.array(QD); ac = np.array(AC); npol = min(n, len(ac))
    qdd_all = np.diff(qd_all, axis=0) / DT
    m_smooth = dict(qacc_sq=float(np.mean(np.sum(qdd_all[:npol] ** 2, axis=1))) if len(qdd_all) else float("nan"),
                    action_acc=float(np.mean(np.sum(np.diff(ac[:npol], 2, axis=0) ** 2, axis=1))) if npol > 2 else float("nan"),
                    action_rate=float(np.mean(np.sum(np.diff(ac[:npol], axis=0) ** 2, axis=1))) if npol > 1 else float("nan"))
    m = dict(seed=seed, fell=fell, t_end=len(GO) * DT, tilt_max=float(TL.max()), gait_hz=hz, **m_smooth,
             trip=fg.trip, trips_by=fg.first, peak_rate=fg.peak_rate, peak_rate_xyz=fg.peak_rax.tolist(), peak_chatter=fg.peak_chat,
             dx=float(X[-1]),
             dy=float(Y[-1]) if len(Y) else float("nan"),
             dyaw=float(np.degrees(np.unwrap(np.asarray(YW, float))[-1] - np.unwrap(np.asarray(YW, float))[0]))
             if len(YW) > 1 else float("nan"))
    m.update(feet_metrics(FT))
    if v.get("march"):
        m.update(march_metrics(FT, C4, X, Y, YW, list(v["cmd_schedule"]), hz, fell))   # the command asked for, before any pinning
    # the written goals' own peak acceleration (what the limiter bounds), deg/s^2
    if len(GO) > 2:
        ga = np.abs(np.diff(GO, 2, axis=0)) / DT ** 2
        m["goal_acc_peak_dps2"] = float(np.degrees(ga.max()))
    sub = env.sim_dt
    walks0 = [ts for ts, c in sched if not S.stand_gate(c)]
    if len(QV):
        # from the first walk command to 2.5 s after the last command change (the walk
        # and the stop that ends it; the whole run without a walk)
        k_per = env.n_substeps
        i0 = int(round(walks0[0] / DT)) * k_per if walks0 else 0
        t1 = max(ts for ts, _ in sched) + 2.5 if walks0 else None
        i1 = min(len(QV), int(round(t1 / DT)) * k_per) if walks0 else len(QV)
        dec = _decel_peaks(QV[i0:i1], sub)
        m["joint_decel_peak_dps2"] = float(np.degrees(dec.max())) if dec.size else float("nan")
        m["joint_decel_by"] = dict(zip(rec.POL, np.round(np.degrees(dec), 0).tolist()))
        m["joint_vel_peak_dps"] = float(np.degrees(np.abs(QV[i0:i1]).max()))
        wz = WZ[i0:i1]
        m["yaw_rate_peak"] = float(np.abs(wz).max())
        m["yaw_deg_2s5"] = float(np.degrees(np.sum(wz) * sub))
    walks = [ts for ts, c in sched if not S.stand_gate(c)]
    if walks:
        t_walk = walks[0]; i0 = int(round(t_walk / DT))
        seg = slice(i0, min(len(TL), i0 + int(2.5 / DT)))
        m["walk_dx_2s5"] = float(X[min(len(X) - 1, i0 + int(2.5 / DT))] - X[i0]) if len(X) > i0 else float("nan")
        m["walk_tilt_max"] = float(TL[seg].max()) if len(TL) > i0 else float("nan")
        # the robot's 2026-11-03 stride1 spin: yaw -19 deg at +180 ms, |gyro z| 3.9 rad/s.
        # the command is set after tick i0 - 1's action, so tick i0 writes the first walk goal
        # (the robot's +0 ms: the first goal off the stand); yaw from the tick before it
        yw = np.degrees(np.unwrap(np.array(YW)))
        j0 = i0
        if len(yw) > j0 + 9:
            y_ref = yw[j0 - 1] if j0 > 0 else yw[0]
            m["walk_yaw_180ms"] = float(yw[j0 + 9] - y_ref)
            m["walk_yaw_max"] = float(np.abs(yw[j0:j0 + int(1.0 / DT)] - y_ref).max())
            m["walk_gz_max"] = float(np.abs(GY[j0:j0 + int(1.0 / DT), 2]).max())
            # smoothness (Tom 2026-11-03: "avoid big jerks, especially when a joint comes to a
            # stop"): qdd = the tick's mean joint acceleration (rad/s^2, servo DOFs, 50 Hz
            # differences of qvel -- a lower bound on the substep peak); deceleration = the part
            # of qdd that opposes the joint's motion. start = the first 300 ms of the walk
            # command, stop = 300 ms .. 1.2 s at the 1.5 Hz clock, longer at a slower one (the swing
            # ends, the stride's last step, the stop)
            qd = np.array(QD); lm = np.array(LM)
            qdd = np.diff(qd, axis=0, prepend=qd[:1]) / DT
            dec = np.maximum(-np.sign(np.vstack([qd[:1], qd[:-1]])) * qdd, 0.0)
            w0, w3, w12 = j0, min(len(qd), j0 + int(0.3 / DT)), min(len(qd), j0 + int(1.2 * S.CLOCK_HZ / hz / DT))
            ref = lm[j0 - 1] if j0 > 0 else lm[0]
            m["walk_qdd_start"] = float(np.abs(qdd[w0:w3]).max())
            m["walk_qdd_stop"] = float(np.abs(qdd[w3:w12]).max()) if w12 > w3 else float("nan")
            m["walk_decel_start"] = float(dec[w0:w3].max())
            m["walk_decel_stop"] = float(dec[w3:w12].max()) if w12 > w3 else float("nan")
            m["walk_wz_300ms"] = float(np.abs(GY[w0:w3, 2]).max())
            m["walk_dL_300ms"] = float(np.linalg.norm(lm[w0:w3] - ref, axis=1).max())
            m["walk_dLz_300ms"] = float(np.abs(lm[w0:w3, 2] - ref[2]).max())
            if v.get("trace"):
                m["trace"] = dict(gz=GY[j0:j0 + 30, 2].tolist(), yaw=(yw[j0:j0 + 30] - y_ref).tolist(),
                                  goal=np.degrees(np.array(GO[j0:j0 + 30])).tolist(),
                                  q=np.degrees(np.array(QM[j0:j0 + 30])).tolist())
    w = slice(int(5.0 / DT), min(n, len(GO)))
    if not fell and n - 250 > 100:
        m.update(S.J.jitter(GO[w], GY[w]))
        m["power_w"] = float(PW[w].mean())
        m["power_hold_w"] = float(PW[n + nh // 2:].mean()) if nh and len(PW) >= n + nh else float("nan")
        sq = SQ[w]
        m["squeeze_deg"] = float(sq.sum(1).mean())
        m["squeeze_joint"] = dict(zip(rec.POL, np.round(sq.mean(0), 3).tolist()))
        if nh and len(SQ) >= n + nh:
            m["squeeze_hold_deg"] = float(SQ[n + nh // 2:].sum(1).mean())
    return m


W16 = "robot_legs_walk16_act_s128r24"


def runs_set(runs, hz):
    V = {f"walk16_act @ shape 12 (as on the robot)": dict(run=W16, shaper_hz=12.0),
         f"walk16_act @ shape {hz:g}": dict(run=W16, shaper_hz=hz)}
    for r in runs:
        V[f"{r} @ shape {hz:g}"] = dict(run=r, shaper_hz=hz)
    return V


def job(args):
    vname, pname, seed, v, p = args
    spec = dict(v, stand_phase=P2, clock="coast")
    spec.update({k: x for k, x in p.items() if k not in ("run_s", "hold_s")})
    try:
        m = episode(spec, seed=seed, run_s=p["run_s"], hold_s=p.get("hold_s", 0.0))
    except Exception as e:  # noqa: BLE001 -- one broken run must not lose the table
        m = dict(seed=seed, error=repr(e), fell=True, trip=("error", 0, 0))
    m.update(variant=vname, probe=pname)
    return m


def _f(x, nd=3):
    return "—" if x is None or (isinstance(x, float) and not np.isfinite(x)) else f"{x:.{nd}f}"


def summarize(res, variants, probes):
    rows = []
    for vn in variants:
        row = dict(variant=vn)
        for pn in probes:
            e = [r for r in res if r["variant"] == vn and r["probe"] == pn]
            if not e:
                continue
            ok = [r for r in e if not r["fell"]]
            trips = [r["trip"] for r in e if r.get("trip")]
            row[pn] = dict(n=len(e), falls=sum(r["fell"] for r in e), trips=len(trips),
                           trip_kinds=sorted({t[0] for t in trips}),
                           by_kind={k: sum(1 for r in e if k in (r.get("trips_by") or {}))
                                    for k in ("rate", "pose", "tilt", "chatter")},
                           first_trip_s=min((t[1] for t in trips), default=None),
                           peak_rate=float(np.mean([r.get("peak_rate", np.nan) for r in e])),
                           peak_rate_max=float(np.max([r.get("peak_rate", np.nan) for r in e])))
            for key in ("goal_rms_mean", "goal_rms_max", "rate_x", "rate_y", "rate_z", "power_w",
                        "power_hold_w", "squeeze_deg", "squeeze_hold_deg", "walk_dx_2s5",
                        "walk_tilt_max", "tilt_max", "goal_acc_peak_dps2", "joint_decel_peak_dps2",
                        "joint_vel_peak_dps", "yaw_rate_peak", "yaw_deg_2s5", "walk_yaw_180ms",
                        "walk_yaw_max", "walk_gz_max", "walk_qdd_start", "walk_qdd_stop", "walk_decel_start",
                        "walk_decel_stop", "walk_wz_300ms", "walk_dL_300ms", "walk_dLz_300ms", "qacc_sq", "action_acc", "action_rate",
                        "march_min_cm", "march_min_l_cm", "march_min_r_cm", "march_shift_l", "march_shift_r",
                        "march_drift_m", "march_yaw_deg", "march_steps_ok", "march_n"):
                vals = [r[key] for r in (ok if key != "tilt_max" else e) if key in r]
                if vals:
                    row[pn][key] = float(np.nanmean(vals))
            if any("march_pass" in r for r in e):
                row[pn]["march_pass"] = sum(bool(r.get("march_pass")) for r in e)
                row[pn]["march_lifts_pass"] = sum(bool(r.get("march_lifts_pass")) for r in e)
                row[pn]["frozen_cmd"] = sorted({tuple(r.get("frozen_cmd") or ()) for r in e})
        rows.append(row)
    return rows


def table(rows, probes):
    L = ["| variant | stand goal jitter mean / max (deg) | body rate x / y / z | power / hold (W proxy) | "
         "squeeze / hold (deg, sum 12 joints) | " + " | ".join(p for p in probes if p != "stand") + " |",
         "|---|---|---|---|---|" + "---|" * (len(probes) - 1)]
    for r in rows:
        s = r.get("stand", {})
        cells = []
        for p in probes:
            if p == "stand" or p not in r:
                continue
            c = r[p]
            txt = f"{c['falls']}/{c['trips']}"
            bk = {k: n for k, n in c.get("by_kind", {}).items() if n}
            if bk:
                txt += " [" + " ".join(f"{k} {n}" for k, n in bk.items()) + "]"
            if p.startswith("march"):
                txt += (f" (pass {c.get('march_pass', 0)}/{c['n']}, lifts>=3cm {c.get('march_lifts_pass', 0)}/{c['n']}, "
                        f"min lift L/R {_f(c.get('march_min_l_cm'), 1)}/{_f(c.get('march_min_r_cm'), 1)} cm, "
                        f"shift L/R {_f(c.get('march_shift_l'), 2)}/{_f(c.get('march_shift_r'), 2)}, "
                        f"drift {_f(c.get('march_drift_m'), 2)} m, yaw {_f(c.get('march_yaw_deg'), 0)} deg"
                        + (f", pinned c{list(c['frozen_cmd'][0])}" if c.get("frozen_cmd") and c["frozen_cmd"][0] else "")
                        + ")")
            if p.startswith("stride1") or p.startswith("walk"):
                txt += f" (rate {c['peak_rate']:.2f}/{c['peak_rate_max']:.2f}, dx {_f(c.get('walk_dx_2s5'), 2)}"
                if p.startswith("stride1"):
                    txt += (f", yaw180 {_f(c.get('walk_yaw_180ms'), 1)}, wz300 {_f(c.get('walk_wz_300ms'), 2)}, "
                            f"dL300 {_f(c.get('walk_dL_300ms'), 3)}, qdd {_f(c.get('walk_qdd_start'), 0)}/"
                            f"{_f(c.get('walk_qdd_stop'), 0)}, dec {_f(c.get('walk_decel_start'), 0)}/"
                            f"{_f(c.get('walk_decel_stop'), 0)}")
                txt += ")"
            cells.append(txt)
        L.append(f"| {r['variant']} | {_f(s.get('goal_rms_mean'))} / {_f(s.get('goal_rms_max'))} "
                 f"({s.get('falls', '?')}/{s.get('trips', '?')}) | "
                 f"{_f(s.get('rate_x'))} / {_f(s.get('rate_y'))} / {_f(s.get('rate_z'))} | "
                 f"{_f(s.get('power_w'), 2)} / {_f(s.get('power_hold_w'), 2)} | "
                 f"{_f(s.get('squeeze_deg'), 2)} / {_f(s.get('squeeze_hold_deg'), 2)} | " + " | ".join(cells) + " |")
    L.append("")
    L.append("Cells: falls/flail-guard trips out of the seeds [episodes in which each guard "
             "criterion would trip, judged separately]; walk cells add the guard's peak "
             "EMA body rate (mean/max over seeds, trip at 1.5 rad/s) and the torso's forward "
             "travel over the 2.5 s after the walk command (m); stride1 cells add the body yaw "
             "180 ms after the command (deg; the robot 2026-11-03: -19), the peak body yaw rate "
             "(rad/s) and whole-body angular momentum change (N m s) in its first 300 ms, and the "
             "peak joint acceleration / deceleration (rad/s^2, 50 Hz qvel differences) at the "
             "start (0-300 ms) / the swing ends and stop (0.3-1.2 s). `mu` probes force the "
             "effective contact friction (sliding mu, torsion) on the floor and the pads. march5 cells: "
             "pass = no fall, every one of the 5 full steps lifts its foot >= 3 cm AND the CoM goes >= 0.6 "
             "of the way over the stance foot on both sides; min lift = the lowest step's peak per foot "
             "(cm, mean over seeds); shift L/R = CoM toward the stance foot while the L / R foot is up "
             "(-1 = over R, +1 = over L); drift and yaw from the march's start to the end of the run; "
             "pinned = the command channels the run never saw move, fed their trained value as the "
             "firmware does.")
    return "\n".join(L)


def main():
    seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "grade.json")
    which = sys.argv[3] if len(sys.argv) > 3 else "runs"
    hz = float(os.environ.get("SHAPE_HZ", "10"))
    runs = [r for r in os.environ.get("RUNS", "").split(",") if r]
    if which == "runs":
        variants = runs_set(runs, hz)
    elif which == "poles":      # does the sim reproduce the robot's shaper sweep (stand5-7)?
        variants = {f"{r} @ shape {p:g}": dict(run=r, shaper_hz=p)
                    for r in (runs or [W16]) for p in (0.0, 20.0, 15.0, 12.0, 10.0, 8.0)}
    else:
        raise SystemExit(f"unknown set {which}")
    only = os.environ.get("ONLY")
    probes = {k: p for k, p in PROBES.items() if not only or k in only.split(",")}
    jobs = [(vn, pn, s, v, p) for vn, v in variants.items() for pn, p in probes.items()
            for s in range(seeds)]
    with ProcessPoolExecutor(max_workers=int(os.environ.get("WORKERS", "24"))) as ex:
        res = list(ex.map(job, jobs, chunksize=1))
    rows = summarize(res, variants, probes)
    json.dump(dict(rows=rows, episodes=res, fg=FG, servo=FAST, shape_hz=hz), open(out, "w"),
              indent=1, default=str)
    md = table(rows, list(probes))
    if any(p in probes for p in DIRS):
        for vn in variants:
            dd = directions(res, vn)
            md += "\n\n- " + vn + " directions (4 s, ideal in brackets): " + "; ".join(
                f"{p[4:]} {k} {m:+.2f} [{i:+.2f}]" + (f" FALLS {f}" if f else "")
                for p, (k, m, i, f) in dd.items())
    if "walk 0.3 -> stop" in probes:
        md += "\n\n" + "\n".join(
            f"- {vn}: {'WALKS' if walks(res, vn) else 'does NOT walk'} "
            f"(walk 0.3 -> stop dx2.5 >= {WALKS_DX} m)" for vn in variants)
    open(os.path.splitext(out)[0] + ".md", "w").write(md + "\n")
    print(md)


if __name__ == "__main__":
    main()
