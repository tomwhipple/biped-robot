"""Stand-still candidates x probes x seeds on the measured actuator, in parallel.

  python stand_candidates.py [seeds] [out.json] [variant-set]      (CPU: CUDA_VISIBLE_DEVICES=)

Probes: stand (20 s jitter + power + 2 s static hold), loop gain x1.25 / x1.5 (20 s),
push 0.4 m/s (kicks at 3 / 5.5 / 8 s), and the hard set (reg-58 dq, joint-fit actuator,
gyro path x1.5, x2). Writes json + a markdown table next to it."""
from __future__ import annotations
import json, math, os, sys
from concurrent.futures import ProcessPoolExecutor
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import stand_sim as S  # noqa: E402
import simh  # noqa: E402

# The STS3215 dead time is a RAM register (Tom 2026-11-02, R hip pitch stepresp): reg 85 = 50
# (factory) 63-78 ms; reg 85 = 0 18-24 ms; reg 81 = 1 (factory 20) also halves the rise (t90
# ~95 ms vs ~200). The 2026-11-02 stand ran SLOW (factory). Planned: 85 = 0, 81 = 1 at boot.
SERVOS = {
    "slow": dict(simh.MEAS_SERVO),                                  # dead 68 ms, tau 55 ms
    # 85 = 0 + 81 = 1, the robot's config since 2026-11-02 ~16:20 UTC (servo-ram-regw). Re-measured
    # (rca/stepresp.py on the loaded stepresp, double support, hard floor): STS3215 dead 18-27 ms
    # (median 21), tau63 30-60 ms (median ~37), peak <= 1.13; STS3250 unchanged.
    "fast": dict(simh.MEAS_SERVO, dead15=20.0, tau15=33.0),
    "fast85": dict(simh.MEAS_SERVO, dead15=20.0),                   # 85 = 0 only
}
SERVO = os.environ.get("SERVO", "slow")
MEAS = dict(typed=SERVOS[SERVO])
WALK = (0.15, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
PROBES = {
    "stand": dict(MEAS, run_s=20.0, hold_s=2.0),
    "x1.25": dict(MEAS, out_gain=1.25, run_s=20.0, hold_s=0.0),
    "x1.5": dict(MEAS, out_gain=1.5, run_s=20.0, hold_s=0.0),
    "push 0.4": dict(MEAS, push=0.4, run_s=12.0, hold_s=0.0),
    "walk 0.15 -> stop": dict(MEAS, cmd_schedule=[(0.0, WALK), (4.0, simh.STAND)], run_s=14.0, hold_s=0.0),
}
PROBES_HARD = {
    "reg-58 dq": dict(MEAS, obs=dict(dq_delay=1, dq_quant=1), run_s=20.0, hold_s=0.0),
    "slow servo" if SERVO != "slow" else "fast servo":
        dict(typed=SERVOS["fast" if SERVO == "slow" else "slow"], run_s=20.0, hold_s=0.0),
    "gyro path x1.5": dict(MEAS, obs=dict(gyro_gain=1.5), run_s=20.0, hold_s=0.0),
    "x2": dict(MEAS, out_gain=2.0, run_s=20.0, hold_s=0.0),
    "push 0.5": dict(MEAS, push=0.5, run_s=12.0, hold_s=0.0),
}

P2 = math.pi / 2
FW = {   # firmware-side candidates for the CURRENT student (all with gyro x0.5 + shaper 6)
    "as run (clock 1.5 Hz)": dict(),
    "freeze @ arm phase": dict(clock="freeze"),
    "freeze @ 0": dict(clock="freeze", phase0=0.0),
    "freeze @ +pi/2": dict(clock="freeze", phase0=P2),
    "freeze @ pi": dict(clock="freeze", phase0=math.pi - 1e-6),
    "freeze @ -pi/2": dict(clock="freeze", phase0=-P2),
    "ramp 1.5->0 Hz over 2 s": dict(clock="ramp", ramp_s=2.0),
    "slow clock 0.5 Hz": dict(clock="slow", clock_hz=0.5),
    "slow clock 0.25 Hz": dict(clock="slow", clock_hz=0.25),
    "action LPF 2 Hz": dict(act_filter=("lpf", 2.0)),
    "action LPF 1 Hz": dict(act_filter=("lpf", 1.0)),
    "target deadband 0.5 deg": dict(act_filter=("db", 0.5)),
    "target deadband 1 deg": dict(act_filter=("db", 1.0)),
    "action notch 1.5+3 Hz": dict(act_filter=("notch", 2.0)),
}
COAST = {
    "as run (clock 1.5 Hz)": dict(),
    "freeze @ stop phase": dict(clock="freeze"),
    **{f"coast -> {k}": dict(clock="coast", stand_phase=p) for k, p in
       (("pi", math.pi), ("-pi/2", -P2), ("-3pi/4", -3 * math.pi / 4), ("3pi/4", 3 * math.pi / 4))},
    "ramp 1.5->0 Hz over 2 s": dict(clock="ramp", ramp_s=2.0),
    "action notch 1.5+3 Hz": dict(act_filter=("notch", 2.0)),
    "target deadband 1 deg": dict(act_filter=("db", 1.0)),
}
PHASES = {f"freeze @ {k}/8 turn": dict(clock="freeze", phase0=_p) for k, _p in
          ((k, -math.pi + 2 * math.pi * k / 8 + 1e-6) for k in range(8))}
NOKNOBS = dict(gyro_obs=1.0, shaper_hz=10.0)     # stock firmware: obsfreeze gain=1, shape 10
KNOBS = {
    "knobs (gyro x0.5, shape 6), clock 1.5 Hz": dict(),
    "NO knobs (gyro x1, shape 10), clock 1.5 Hz": dict(NOKNOBS),
    "NO knobs, freeze @ stop phase": dict(NOKNOBS, clock="freeze"),
    "NO knobs, coast -> pi": dict(NOKNOBS, clock="coast", stand_phase=math.pi),
    "NO knobs, coast -> -pi/2": dict(NOKNOBS, clock="coast", stand_phase=-P2),
    "knobs, coast -> pi": dict(clock="coast", stand_phase=math.pi),
    "knobs, coast -> -pi/2": dict(clock="coast", stand_phase=-P2),
    "NO knobs, gyro x0.5 only, coast -> pi": dict(gyro_obs=0.5, shaper_hz=10.0, clock="coast", stand_phase=math.pi),
}
def students(runs):
    """Per run, stock firmware (gyro x1, shape 10): the clock as the firmware runs it for that
    run (a clock_stand_freeze run holds -- here at the arm phase and at pi), plus coast -> pi."""
    V = {}
    for r in runs:
        V[f"{r} :: clock 1.5 Hz"] = dict(NOKNOBS, run=r)
        V[f"{r} :: freeze @ arm phase"] = dict(NOKNOBS, run=r, clock="freeze")
        V[f"{r} :: coast -> pi"] = dict(NOKNOBS, run=r, clock="coast", stand_phase=math.pi)
    return V


SETS = {"fw": FW, "knobs": KNOBS, "coast": COAST, "phases": PHASES}


def job(args):
    vname, pname, seed, v, p = args
    spec = dict(v); spec.update({k: x for k, x in p.items() if k not in ("run_s", "hold_s")})
    if "obs" in v and "obs" in p:
        spec["obs"] = dict(v["obs"], **p["obs"])
    m = S.episode(spec, seed=seed, run_s=p["run_s"], hold_s=p["hold_s"])
    m.update(variant=vname, probe=pname)
    return m


def summarize(res, variants, probes):
    rows = []
    for vn in variants:
        for pn in probes:
            e = [r for r in res if r["variant"] == vn and r["probe"] == pn]
            if not e:
                continue
            ok = [r for r in e if not r["fell"]]
            f = lambda k: float(np.mean([r[k] for r in ok])) if ok and k in ok[0] else float("nan")
            rows.append(dict(variant=vn, probe=pn, n=len(e), falls=len(e) - len(ok),
                             goal_rms=f("goal_rms_mean"), goal_max=f("goal_rms_max"),
                             g15=f("goal_1.5hz_mean"), g3=f("goal_3hz_mean"),
                             rx=f("rate_x"), ry=f("rate_y"), rz=f("rate_z"),
                             power=f("power_w"), power_hold=f("power_hold_w"), arate=f("arate"),
                             power_dyn=float(np.mean([r["power_w"] - r["power_hold_w"] for r in ok]))
                             if ok and "power_hold_w" in ok[0] else float("nan"),
                             tilt=float(np.mean([r["tilt_max"] for r in e]))))
    return rows


def table(rows, robot=None):
    L = ["| variant | probe | falls | goal jitter mean / max (deg) | 1.5 / 3 Hz (deg) | body rate x / y / z (rad/s) | power / static hold (W, sim proxy) | tilt max |",
         "|---|---|---|---|---|---|---|---|"]
    if robot:
        L.append(f"| **robot 2026-11-02 stand2** (as run) | real | 0/1 | {robot['goal_rms_mean']:.3f} / {robot['goal_rms_max']:.3f} | "
                 f"{robot['goal_1.5hz_mean']:.3f} / {robot['goal_3hz_mean']:.3f} | {robot['rate_x']:.3f} / {robot['rate_y']:.3f} / "
                 f"{robot['rate_z']:.3f} | {robot['power_w']:.2f} / {robot['power_hold_w']:.2f} (motor current x V) | - |")
    for r in rows:
        if r["probe"] == "stand":
            L.append(f"| {r['variant']} | {r['probe']} | {r['falls']}/{r['n']} | {r['goal_rms']:.3f} / {r['goal_max']:.3f} | "
                     f"{r['g15']:.3f} / {r['g3']:.3f} | {r['rx']:.3f} / {r['ry']:.3f} / {r['rz']:.3f} | "
                     f"{r['power']:.2f} / {r['power_hold']:.2f} | {r['tilt']:.1f} |")
        else:
            L.append(f"| {r['variant']} | {r['probe']} | {r['falls']}/{r['n']} | {r['goal_rms']:.3f} / {r['goal_max']:.3f} | "
                     f"{r['g15']:.3f} / {r['g3']:.3f} | {r['rx']:.3f} / {r['ry']:.3f} / {r['rz']:.3f} | {r['power']:.2f} / - | {r['tilt']:.1f} |")
    return "\n".join(L)


def compact(rows, probes):
    """One row per variant: stand jitter/power, falls per probe."""
    vs = list(dict.fromkeys(r["variant"] for r in rows))
    L = ["| variant | goal jitter mean / max (deg) | 1.5 / 3 Hz (deg) | body rate x / y / z (rad/s) | power / static hold / dynamic (W, sim proxy) | "
         + " | ".join(f"falls {p}" for p in probes if p != "stand") + " | stand falls |",
         "|---" * (5 + len(probes)) + "|"]
    for vn in vs:
        by = {r["probe"]: r for r in rows if r["variant"] == vn}
        s = by.get("stand")
        if not s:
            continue
        L.append(f"| {vn} | {s['goal_rms']:.3f} / {s['goal_max']:.3f} | {s['g15']:.3f} / {s['g3']:.3f} | "
                 f"{s['rx']:.3f} / {s['ry']:.3f} / {s['rz']:.3f} | {s['power']:.2f} / {s['power_hold']:.2f} / {s['power_dyn']:.2f} | "
                 + " | ".join(f"{by[p]['falls']}/{by[p]['n']}" if p in by else "-" for p in probes if p != "stand")
                 + f" | {s['falls']}/{s['n']} |")
    return "\n".join(L)


def run(variants, probes, seeds, jobs, out):
    js = [(vn, pn, s, v, p) for vn, v in variants.items() for pn, p in probes.items() for s in range(seeds)]
    with ProcessPoolExecutor(jobs) as ex:
        res = list(ex.map(job, js, chunksize=1))
    rows = summarize(res, variants, probes)
    json.dump(dict(rows=rows, episodes=res), open(out, "w"), indent=1, default=float)
    return rows


if __name__ == "__main__":
    import jitter as J
    seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "fw_candidates.json")
    vset = SETS[sys.argv[3]] if len(sys.argv) > 3 and sys.argv[3] in SETS else FW
    if len(sys.argv) > 3 and sys.argv[3] == "students":
        vset = students(os.environ["RUNS"].split(","))
    probes = dict(PROBES, **PROBES_HARD) if os.environ.get("HARD") else PROBES
    if os.environ.get("ONLY"):
        probes = {k: probes[k] for k in os.environ["ONLY"].split(",")}
    rows = run(vset, probes, seeds, int(os.environ.get("JOBS", 12)), out)
    robot = J.robot()
    md = f"servo: {SERVO} ({SERVOS[SERVO]})\n\n## Compact\n\n" + compact(rows, list(probes)) + "\n\n## Full\n\n" + table(rows, robot) + "\n"
    open(out.replace(".json", ".md"), "w").write(md)
    print(md)
