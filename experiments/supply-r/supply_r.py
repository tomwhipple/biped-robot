#!/usr/bin/env python3
"""Estimate the servo supply's series resistance R from recorded flight data (#6).

    .venv/bin/python experiments/supply-r/supply_r.py extract     # cache every .frec (slow, once)
    .venv/bin/python experiments/supply-r/supply_r.py segments    # per-minute idle V: which supply
    .venv/bin/python experiments/supply-r/supply_r.py fit [--boot 2000] [--json out.json]

Data: hw_sessions/**/*.frec on mira (gitignored, read only). Each flight tick carries, for every
fitted servo, registers 56-70 from ONE sync read (firmware ctrl_task `syncReadFeedback`): position,
speed (reg 58, steps/s), load (reg 60, PWM duty in 0.1 %), voltage (reg 62, 0.1 V), current
(reg 69, 6.5 mA units). Only ticks where every leg servo answered are used, so a tick's per-servo
currents are summed across servos read in the same sync read.

The model fitted is the sim's (sim/supply_sag.py):  V_servo = V0 - R * I_bus.

I_bus per tick is estimated three ways, and R is fitted against each:
  bridge  the sim's own current model, from the logged load: reg 60 is PWM duty d (signed), and the
          sim's envelope gives the torque at duty d and speed w as tau = stall(V) (d - w / w0(V))
          (sim/sts_servo_model.py: duty = tau / stall + w / w0), with stall and w0 of each servo's
          physical type at 12 V scaled by V / 12 (STS3215 2.94 N m, 4.04 rad/s; STS3250 per
          walker_env.SERVO_TYPES), V the servo's own reading. Then supply_sag.bus_current(tau, w, V)
          with current_budget.SERVO_ELEC: the lossless bridge, capped at the real stall current.
  motor   current_budget's upper model on the same tau, w (the motor's current at full duty).
  reg69   the servos' own present-current reading, summed |reg 69| x 6.5 mA. What it senses is not
          documented; at the bus level it is 0.8-1.3x the bridge model (`ratio`).
The R to use is the reg69 fit. The two load-based estimates are noisy per tick (while a joint moves
they price current reg 69 does not see), and noise in the regressor biases R low; the reg69 bus
current is on the bridge model's scale, so its R pairs with the sim's current as it is.
(docs/design-v6/2026-11-22-supply-resistance-estimate.md.)

V_servo: the mean of the leg servos' reg 62 in that tick (0.1 V steps; the mean of 12 dithers the
quantisation). V0 is not assumed: it is fitted per "block" (a run of ticks within one supply
configuration and one boot, cut every BLOCK_S seconds), which absorbs a pack's slow discharge and the
bench supply's set point, so R comes only from the fast co-variation of V with I inside a block.
Shared vs branch: V_A (mean of port A's servos, the right leg) and V_B (left leg) are fitted jointly as
  V_A = V0A - R_s (I_A + I_B) - R_b I_A,   V_B = V0B - R_s (I_A + I_B) - R_b I_B   (V0A, V0B per block)
(R_s: supply + leads + board inlet, FET, switch; R_b: one General Driver port + its chain's leads).
Confidence intervals: a block bootstrap (resampling whole blocks, so serial correlation inside a
block is kept), 2.5-97.5 percentiles.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "sim"))

HW = os.path.expanduser("~/code/robot/hw_sessions")
CACHE = os.environ.get("SUPPLY_R_CACHE", "/tmp/s6r/cache")
LEGS = ("R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle",
        "L_hip_roll", "L_hip_pitch", "L_knee", "L_ankle",
        "R_hip_yaw", "L_hip_yaw", "R_ankle_roll", "L_ankle_roll")
STEP_RAD = 2.0 * math.pi / 4096.0
REG69_A = 0.0065
BLOCK_S = 20.0


# ----------------------------------------------------------------------------- extract
def extract_one(path):
    import flight_reader as R
    import flightrec as F
    out = os.path.join(CACHE, os.path.basename(path).replace(".frec", ".npz"))
    if os.path.exists(out):
        return out
    rows = []
    with open(path, "rb") as f:                      # streamed: the biggest session is 5 GB
        if f.read(len(R.FILE_MAGIC)) != R.FILE_MAGIC:
            raise ValueError(path)
        f.readline()
        while True:
            hdr = f.read(R.REC.size)
            if len(hdr) < R.REC.size:
                break
            n, rx_us = R.REC.unpack(hdr)
            data = f.read(n)
            if len(data) < n:
                break
            # a tick with fewer than 12 joints cannot be used: skip it before decoding
            if n < F.HEADER_LEN + F.TICK_FIXED_LEN + 12 * F.JOINT_LEN:
                continue
            try:
                t = F.decode(data)
            except F.DecodeError:
                continue
            if t["type"] != F.T_TICK:
                continue
            js = {j["bus"]: j for j in t["joints"]}
            if not all(b in js and js[b]["flags"] & F.JF_FB_OK for b in range(12)):
                continue
            jj = [js[b] for b in range(12)]
            rows.append((t["boot_id"], t["t_boot_us"], t["t_utc_us"], rx_us, t["source"],
                         t["ctrl"]["vbat_mv"],
                         *[j["voltage_dv"] for j in jj], *[j["load"] for j in jj],
                         *[j["speed"] for j in jj], *[j["current"] for j in jj],
                         *[j["temp_c"] for j in jj]))
    a = np.array(rows, dtype=np.float64).reshape(-1, 6 + 60)
    os.makedirs(CACHE, exist_ok=True)
    np.savez_compressed(out, a=a, src=path)
    return out


def cmd_extract(a):
    from multiprocessing import Pool
    fs = sorted(glob.glob(os.path.join(HW, "**", "*.frec"), recursive=True))
    with Pool(a.workers) as p:
        for o in p.imap_unordered(extract_one, fs):
            print(o, flush=True)


def load_all(merge=False):
    """{file stem: dict of arrays} for every cached session with complete-feedback ticks. merge:
    one stream {"all": ...}, sorted by time, with ticks that two files share (the snapshot copies
    hw_sessions/2026-11-10/m271 and 2026-11-11/bench re-record ticks of the 11-09 sessions) kept once."""
    out = {}
    arrs = []
    for f in sorted(glob.glob(os.path.join(CACHE, "*.npz"))):
        z = np.load(f, allow_pickle=True)
        a = z["a"]
        if len(a) == 0:
            continue
        if merge:
            arrs.append(a)
            continue
        d = _arrays(a, str(z["src"]))
        out[os.path.basename(f)[:-4]] = d
    if merge:
        a = np.concatenate(arrs)
        _, first = np.unique(a[:, :2], axis=0, return_index=True)    # (boot id, boot clock)
        a = a[np.sort(first)]
        t = np.where(a[:, 2] > 0, a[:, 2], a[:, 3])
        a = a[np.argsort(t, kind="stable")]
        return {"all": _arrays(a, "merged")}
    return out


def _arrays(a, src):
    return dict(src=src, boot=a[:, 0].astype(np.int64), t_boot=a[:, 1] / 1e6,
                t_utc=np.where(a[:, 2] > 0, a[:, 2], a[:, 3]) / 1e6, source=a[:, 4],
                vbat=a[:, 5] / 1000.0, V=a[:, 6:18] / 10.0, load=a[:, 18:30],
                speed=a[:, 30:42], cur=a[:, 42:54], temp=a[:, 54:66])


# ----------------------------------------------------------------------------- current models
def currents(d):
    """Per tick and servo: (bridge, motor, reg69) current, A, shape (n, 12)."""
    import current_budget as CB
    import sts_servo_model as SM
    import walker_env as W
    typ = [CB.servo_type(n) for n in LEGS]
    stall12 = np.array([SM.STALL_NM_12V * W.SERVO_TYPES[t][0] for t in typ])
    w012 = np.array([W._STS_NOLOAD_12V * W.SERVO_TYPES[t][1] for t in typ])
    istall12, kt, kcu = np.array([CB.SERVO_ELEC[t] for t in typ]).T
    V = d["V"]
    # Reg 60's sign is OPPOSITE to reg 58's: at full duty while moving (|load| >= 900, |w| > 1 rad/s)
    # the two have opposite signs in 100 % of the 916 such samples on record (`ratio`). So the duty in
    # the speed's convention is -load, and then tau = stall (d - w / w0) and tau * w is the power.
    duty = np.clip(-d["load"] / 1000.0, -1.0, 1.0)
    w = d["speed"] * STEP_RAD
    stall = stall12 * V / 12.0
    w0 = w012 * V / 12.0
    tau = stall * (duty - w / w0)
    bridge = np.minimum(SM.I_IDLE + (np.maximum(tau * w, 0.0) + kcu * tau ** 2) / V, istall12 * V / 12.0)
    motor = np.minimum(SM.I_IDLE + (SM.I_NOLOAD - SM.I_IDLE) * np.clip(np.abs(w) / w0, 0, 1)
                       + np.abs(tau) / kt, istall12 * V / 12.0)
    reg69 = np.abs(d["cur"]) * REG69_A
    return dict(bridge=bridge, motor=motor, reg69=reg69)


PORT_A = np.array([n.startswith("R_") for n in LEGS])     # H5: the right leg (current_budget PORT_A_IDS)


# ----------------------------------------------------------------------------- configurations
def config_of(d, stem):
    """Which supply powered a tick, from the session notes (hw_sessions/*/NOTES.md), checked against
    the idle voltage (`segments`). Returns an array of labels per tick."""
    t = d["t_utc"]
    lab = np.full(len(t), "bench", dtype=object)
    for lo, hi in PACK_WINDOWS:
        lab[(t >= lo) & (t < hi)] = "pack850"
    return lab


def _utc(s):
    import datetime as dt
    return dt.datetime.strptime(s, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=dt.timezone.utc).timestamp()


# The v5 pack (Tattu 850 mAh 3S 75C, XT30) on the robot, no tether, General Driver H1:
#  2026-11-06 ~19:20Z-22:31Z (NOTES 11-06: "BATTERY (no tether)", the *_battery sessions);
#  2026-11-09 23:13Z - 2026-11-11 01:04Z (the 11-09 *_battery / prec16 sessions; m271 NOTES:
#  "battery in, no tether" ... "01:04Z robot REBOOTED (battery alarm) and is now on BENCH POWER").
# Everything else is the lab supply on its leads into H1 (NOTES: "bench supply", "bench power 12.0 V").
#  2026-11-11 16:35Z-17:10Z: the bench NOTES say "bench supply", but from 16:39Z ("floor (board), USB
#  unplugged") the idle bus voltage reads 12.52 V and falls steadily to 11.88 V by 17:06Z with the
#  servos idle -- a pack discharging, not the lab supply (which reads 12.02 V idle in every other
#  session). Counted as the pack; `segments` shows the evidence.
PACK_WINDOWS = [(_utc("2026-11-06T19:00:00Z"), _utc("2026-11-06T23:00:00Z")),
                (_utc("2026-11-09T23:00:00Z"), _utc("2026-11-11T01:04:00Z")),
                (_utc("2026-11-11T16:35:00Z"), _utc("2026-11-11T17:10:00Z"))]


def cmd_segments(a):
    D = load_all()
    for stem, d in D.items():
        I = currents(d)["reg69"].sum(1)
        Vm = d["V"].mean(1)
        lab = config_of(d, stem)
        m = np.floor(d["t_utc"] / 60.0)
        for k in np.unique(m):
            s = m == k
            idle = s & (I < 0.1)
            if s.sum() < 50:
                continue
            import datetime as dt
            ts = dt.datetime.fromtimestamp(k * 60, dt.timezone.utc).strftime("%m-%dT%H:%M")
            vi = np.median(Vm[idle]) if idle.any() else float("nan")
            print(f"{stem[:40]:40s} {ts} {lab[s][0]:8s} n {s.sum():5d} Vidle {vi:6.3f} "
                  f"Vmin {Vm[s].min():5.2f} Imax {I[s].max():4.2f} vbat {np.median(d['vbat'][s]):5.2f}")


# ----------------------------------------------------------------------------- fit
LAG = 0          # V taken LAG ticks after the current (a robustness check: the registers' timing)
_CUR = {}


def design(d, lab, model, cfg):
    """Rows for one session's ticks of configuration cfg: (block id, V_A, V_B, Vmean, I_A, I_B)."""
    if id(d) not in _CUR:
        _CUR[id(d)] = currents(d)
    I = _CUR[id(d)][model]
    if LAG:
        I = np.roll(I, LAG, axis=0)
    keep = lab == cfg
    # Exclude obviously broken reads: a servo voltage far from its tick's median (a brown-out
    # or a bad read), and ticks with any servo below 9 V (the bench never sags that far: those
    # are power-off ramps when the supply is switched off with the board alive).
    Vmed = np.median(d["V"], axis=1)
    ok = keep & (np.abs(d["V"] - Vmed[:, None]).max(1) < 0.6) & (d["V"].min(1) > 9.0)
    VA = d["V"][:, PORT_A].mean(1)
    VB = d["V"][:, ~PORT_A].mean(1)
    IA = I[:, PORT_A].sum(1)
    IB = I[:, ~PORT_A].sum(1)
    return ok, VA, VB, d["V"].mean(1), IA, IB


def blocks(t, boot, ok):
    """Block ids: a new block every BLOCK_S seconds or on a boot change or a > 2 s gap."""
    idx = np.where(ok)[0]
    b = np.full(len(t), -1, dtype=np.int64)
    if len(idx) == 0:
        return b
    tt = t[idx]
    brk = np.r_[True, (np.diff(tt) > 2.0) | (np.diff(boot[idx]) != 0)]
    seg = np.cumsum(brk)
    t0 = tt[np.r_[np.where(brk)[0]]][seg - 1]
    b[idx] = seg * 100000 + np.floor((tt - t0) / BLOCK_S).astype(np.int64)
    return b


def fit_within(y, x, g):
    """OLS of y on x (columns) with a fixed intercept per group g (demeaned within groups)."""
    u, inv = np.unique(g, return_inverse=True)
    cnt = np.bincount(inv)

    def dm(v):
        return v - (np.bincount(inv, v) / cnt)[inv]
    Y = dm(y)
    X = np.column_stack([dm(c) for c in x.T])
    beta, *_ = np.linalg.lstsq(X, Y, rcond=None)
    return beta, X, Y


def collect(D, model, cfg, min_imax):
    rows = []
    gid = 0
    for stem, d in D.items():
        lab = config_of(d, stem)
        ok, VA, VB, Vm, IA, IB = design(d, lab, model, cfg)
        b = blocks(d["t_utc"], d["boot"], ok)
        for k in np.unique(b[b >= 0]):
            s = b == k
            # A block only informs R if its current moves: keep blocks whose current spans min_imax.
            Ib = IA[s] + IB[s]
            if Ib.max() - Ib.min() < min_imax or s.sum() < 50:
                continue
            gid += 1
            rows.append(np.column_stack([np.full(s.sum(), gid), VA[s], VB[s], Vm[s], IA[s], IB[s]]))
    return np.concatenate(rows) if rows else np.zeros((0, 6))


def boot_ci(M, fn, nboot, rng):
    g = M[:, 0].astype(int)
    ug = np.unique(g)
    parts = {k: M[g == k] for k in ug}
    est = []
    for _ in range(nboot):
        pick = rng.choice(ug, size=len(ug), replace=True)
        Mb = np.concatenate([np.column_stack([np.full(len(parts[k]), i), parts[k][:, 1:]])
                             for i, k in enumerate(pick)])
        est.append(fn(Mb))
    est = np.array(est)
    return np.percentile(est, [2.5, 97.5], axis=0)


def fit_total(M):
    beta, *_ = fit_within(M[:, 3], (M[:, 4] + M[:, 5])[:, None], M[:, 0])
    return np.array([-beta[0]])


def fit_split(M):
    # stack A and B rows: V_side = V0(block, side) - R_s (IA+IB) - R_b I_side; each side keeps its
    # own intercept per block (the two legs' servos read their supply with different offsets)
    g = np.r_[2 * M[:, 0], 2 * M[:, 0] + 1]
    y = np.r_[M[:, 1], M[:, 2]]
    It = M[:, 4] + M[:, 5]
    X = np.column_stack([np.r_[It, It], np.r_[M[:, 4], M[:, 5]]])
    beta, *_ = fit_within(y, X, g)
    return -beta


def cmd_fit(a):
    D = load_all(merge=True)
    rng = np.random.default_rng(0)
    res = {}
    global BLOCK_S, LAG
    BLOCK_S, LAG = a.block, a.lag
    for cfg in a.cfgs.split(","):
        for model in a.models.split(","):
            M = collect(D, model, cfg, a.min_span)
            if len(M) < 100:
                print(f"{cfg} {model}: not enough data ({len(M)} ticks)")
                continue
            r = fit_total(M)[0]
            ci = boot_ci(M, fit_total, a.boot, rng)[:, 0]
            rs = fit_split(M)
            cis = boot_ci(M, fit_split, a.boot, rng)
            It = M[:, 4] + M[:, 5]
            nb = len(np.unique(M[:, 0]))
            res[f"{cfg}/{model}"] = dict(n_ticks=len(M), n_blocks=nb, R=r, R_ci=list(ci),
                                         R_shared=rs[0], R_shared_ci=list(cis[:, 0]),
                                         R_branch=rs[1], R_branch_ci=list(cis[:, 1]),
                                         I_p50=float(np.median(It)), I_p99=float(np.percentile(It, 99)),
                                         I_max=float(It.max()))
            print(f"{cfg:8s} {model:6s} n {len(M):7d} blocks {nb:4d}  R {r:.3f} [{ci[0]:.3f}, {ci[1]:.3f}]"
                  f"   shared {rs[0]:.3f} [{cis[0,0]:.3f}, {cis[1,0]:.3f}]  branch {rs[1]:.3f} "
                  f"[{cis[0,1]:.3f}, {cis[1,1]:.3f}]   I p50 {np.median(It):.2f} p99 "
                  f"{np.percentile(It, 99):.2f} max {It.max():.2f} A", flush=True)
    if a.json:
        json.dump(res, open(a.json, "w"), indent=1)


def cmd_ratio(a):
    """How the servos' own current reading (reg 69) compares with the sim's bridge model on the same
    ticks: per |load| bin for servos holding still (|w| < 0.05 rad/s, where tau = stall x duty is
    exact in the envelope model), and summed over the bus in ticks where the bus draws > 1 A."""
    d = load_all(merge=True)["all"]
    C = currents(d)
    m = (np.abs(d["load"]) >= 900) & (np.abs(d["speed"] * STEP_RAD) > 1.0)
    print(f"load vs speed sign at >= 90 % duty, |w| > 1 rad/s: {m.sum()} samples, same sign in "
          f"{np.mean(np.sign(d['load'][m]) == np.sign(d['speed'][m])):.1%}")
    L, w = np.abs(d["load"]), np.abs(d["speed"] * STEP_RAD)
    still = w < 0.05
    print("holding still, per servo: |load| bin -> median reg69 / bridge / motor (A), n")
    for lo, hi in ((100, 200), (200, 400), (400, 700), (700, 1001)):
        k = still & (L >= lo) & (L < hi)
        if k.sum() >= 20:
            print(f"  {lo:4d}-{hi:4d}: {np.median(C['reg69'][k]):.3f} / {np.median(C['bridge'][k]):.3f} / "
                  f"{np.median(C['motor'][k]):.3f}   n {k.sum()}")
    lab = config_of(d, "all")
    for cfg in ("bench", "pack850"):
        r69 = C["reg69"].sum(1)
        br = C["bridge"].sum(1) - 12 * 0.030      # reg 69 does not see the servos' 30 mA idle
        mo = C["motor"].sum(1) - 12 * 0.030
        for thr in (0.5, 1.0, 2.0):
            k = (lab == cfg) & (r69 > thr)
            if k.sum() < 20:
                continue
            print(f"{cfg}: bus reg69 > {thr} A: n {k.sum()}, sum reg69 / sum bridge = "
                  f"{r69[k].sum() / br[k].sum():.2f}, / sum motor = {r69[k].sum() / mo[k].sum():.2f}")


def main():
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest="cmd", required=True)
    e = sp.add_parser("extract")
    e.add_argument("--workers", type=int, default=2)
    sp.add_parser("segments")
    f = sp.add_parser("fit")
    f.add_argument("--boot", type=int, default=500)
    f.add_argument("--min-span", type=float, default=0.5, help="A: a block's current must span this")
    f.add_argument("--json", default=None)
    f.add_argument("--block", type=float, default=BLOCK_S, help="s per V0 block")
    f.add_argument("--lag", type=int, default=0, help="ticks V lags I by (robustness)")
    f.add_argument("--cfgs", default="bench,pack850")
    f.add_argument("--models", default="reg69,bridge,motor")
    r = sp.add_parser("ratio")
    a = ap.parse_args()
    {"extract": cmd_extract, "segments": cmd_segments, "fit": cmd_fit, "ratio": cmd_ratio}[a.cmd](a)


if __name__ == "__main__":
    main()
