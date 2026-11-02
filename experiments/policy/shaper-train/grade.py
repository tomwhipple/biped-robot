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
                   (0.667 s at 1.5 Hz), then the stand -- the robot's first walk command
                   (walk16_act, v 0.6) tripped the guard on body rate 0.28 s in
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

FAST = dict(simh.MEAS_SERVO, dead15=20.0, tau15=33.0)
STAND_CMD = simh.STAND
P2 = math.pi / 2
DT = 0.02
FG = dict(rate=1.5, rate_tau=0.2, tilt=20.0, tilt_ticks=10, chatter=7.0, chatter_tau=0.1,
          pose_deg=15.0, pose_ticks=15)


def walk(vx):
    return (vx, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)


def stride(vx, t0=2.0, cyc=1.0 / S.CLOCK_HZ):
    return [(0.0, STAND_CMD), (t0, walk(vx)), (t0 + cyc, STAND_CMD)]


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
}


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


def episode(v, seed=0, run_s=20.0, hold_s=0.0):
    """stand_sim.episode with the flail guard, the squeeze and walk progress recorded.
    v: run, shaper_hz, out_gain, push, cmd_schedule, clock ('coast'), stand_phase."""
    run = v["run"]; settle_s = 1.5
    env, act, zoff = simh.build("fit", run=run, ep_s=run_s + hold_s + settle_s + 1.0)
    rng = np.random.default_rng(1000 + seed)
    env.domain_rand = False; env.push_prob = 0.0
    env.reset(seed=seed)
    env._zero_off = zoff.copy()
    ang = math.radians(rng.normal(0.0, 1.0)); ax = rng.normal(size=3); ax /= np.linalg.norm(ax)
    k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    env._imu_R = np.eye(3) + math.sin(ang) * k + (1 - math.cos(ang)) * k @ k
    env._imu_gyro_bias = rng.uniform(-0.01, 0.01, 3)
    env.domain_rand = True
    simh.apply_model(env, simh.JF_PLANT)
    nj = env._nq_act
    env.set_command(*STAND_CMD)
    simh.place_and_hold(env, np.clip(S.arm_pose(), env._lo, env._hi), settle_s)
    env._prev_action[:] = 0.0
    sched = list(v.get("cmd_schedule") or [])
    env.set_command(*(sched[0][1] if sched else STAND_CMD))
    w0 = S.H.pin_gait_clock(env, S.CLOCK_HZ)
    clock = v.get("clock", "coast")
    env._gait_freq = 0.0
    env._gait_phase = float(w0)
    clk = dict(ph=float(w0), f=S.CLOCK_HZ, held=False)
    env._obs_hist = []
    first = env._obs()
    env._obs_hist = [first.copy() for _ in range(env.obs_hist_len - 1)]
    obs = np.concatenate([first] + list(env._obs_hist))
    qm = first[:nj].astype(np.float64)
    sh = simh.Shaper(v["shaper_hz"]); sh.reset(qm)
    sv = simh.TypedServo(**v.get("typed", FAST)); sv.reset(qm)
    out_gain = v.get("out_gain", 1.0); push = v.get("push", 0.0)
    hold = qm.copy(); cur = {"t": hold}
    env._action_to_ctrl = lambda a_: cur["t"]
    fg = FlailGuard(env._default)
    n = int(round(run_s / DT)); nh = int(round(hold_s / DT))
    GO, GY, PW, TL, SQ, X = [], [], [], [], [], []
    fell = False; ema = None; last_tgt = hold
    x0 = float(env.data.qpos[0])
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
            last_tgt = tgt
        else:
            tgt = last_tgt; raw = last_tgt; a = env._prev_action.copy()
        cur["t"] = sv(tgt)
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
        if term:
            fell = True
            break
    GO, GY, PW, TL, SQ = map(np.array, (GO, GY, PW, TL, SQ))
    m = dict(seed=seed, fell=fell, t_end=len(GO) * DT, tilt_max=float(TL.max()),
             trip=fg.trip, trips_by=fg.first, peak_rate=fg.peak_rate, peak_rate_xyz=fg.peak_rax.tolist(), peak_chatter=fg.peak_chat,
             dx=float(X[-1]))
    walks = [ts for ts, c in sched if not S.stand_gate(c)]
    if walks:
        t_walk = walks[0]; i0 = int(round(t_walk / DT))
        seg = slice(i0, min(len(TL), i0 + int(2.5 / DT)))
        m["walk_dx_2s5"] = float(X[min(len(X) - 1, i0 + int(2.5 / DT))] - X[i0]) if len(X) > i0 else float("nan")
        m["walk_tilt_max"] = float(TL[seg].max()) if len(TL) > i0 else float("nan")
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
                        "walk_tilt_max", "tilt_max"):
                vals = [r[key] for r in (ok if key != "tilt_max" else e) if key in r]
                if vals:
                    row[pn][key] = float(np.nanmean(vals))
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
            if p.startswith("stride1") or p.startswith("walk"):
                txt += f" (rate {c['peak_rate']:.2f}/{c['peak_rate_max']:.2f}, dx {_f(c.get('walk_dx_2s5'), 2)})"
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
             "travel over the 2.5 s after the walk command (m).")
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
    open(os.path.splitext(out)[0] + ".md", "w").write(md + "\n")
    print(md)


if __name__ == "__main__":
    main()
