"""The body path's loop gain while walking, read against a known excitation (#259, #137).

walk_gain.py found the robot's joints answer their goals while walking as the x1.5 grade plant's do (1.00), but could
not read the body path (torso roll / pitch / yaw against the goals): the track gait's moves are large and slow, and the
robot's body moves a lot in ways the plant does not (coherence 0.0-0.16). `track_host.py --id-wiggle AMP[:CHANNEL]`
adds a small known multisine to one joint channel's goals in the double supports. This tool reads the response to it,
with the excitation u as the instrument:

    z = y_simW - y_sim0       the plant's response to the excitation: the robot's written goals replayed open loop with
                              the excitation (simW) and with it taken out again (sim0)
    e = y_robot - y_sim0      what the robot did beyond the plant's response to its gait
    w = h * u                 the part of z that u drives: h a short FIR fitted from u to z (the open-loop replays also
                              drift apart once nudged; that part is not driven by u and stays out of w)
    g = sum(e w) / sum(z w)   over the excited double supports, all band-passed BAND (zero-phase); coherence
                              (sum e w)^2 / (sum e^2 sum w^2): the share of e the excitation explains

u is independent of the gait and of the robot's own motion, so they add noise to g, not bias; the CI comes from
resampling whole double supports. The reading is not G itself (sub-degree goals meet the servos' deadband, so the
response is not linear): `simcheck` is the calibration, reading -> G.

    python wiggle_gain.py simcheck [--n 48] [--amp 1.0] [--channel roll] [--gains 1.0,1.25,1.5] [--workers 8]
    python wiggle_gain.py robot --log experiments/roll-balance/results/track_<stamp>.jsonl [--log ...]

`simcheck`: real track walks (walk_gain.py extract's segments, WALK_GAIN_DIR) with the excitation added to their
written goals in their double supports, as track_host.py builds it (10 Hz keyframes, ticks, the firmware's
Catmull-Rom). The stand-in robot is the real robot's record PLUS the plant's response to the excitation at a known G
(the out_gain law on the excitation; on the whole gait it tips the open-loop replay within seconds): so the noise is
the robot's own. It also runs the robot path's alignment of the excitation to the written goals, from a start 0.4 s
off. `robot`: one or more runs' track logs (their `id_wiggle` row) and flight records -> the same estimate.
CPU only. Record: docs/design-v6/2026-11-26-walking-loop-gain.md; results: wiggle_gain_simcheck.md.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "roll-balance"))
import walk_gain as WG  # noqa: E402

DT = WG.DT
BAND = (0.7, 3.5)                       # Hz: the excitation's lines (1.07-2.87) and the spread of its 1.3 s windows
ALIGN_BAND = (1.5, 3.5)                 # Hz: the alignment's: above most of the gait's own moves in the goals
AMP_OK = (0.8, 1.2)                     # the aligned window's fitted amplitude (1 = the logged excitation, as written)
SUB_BANDS = {"0.7-2": (0.7, 2.0), "2-3.5": (2.0, 3.5)}
BODY = {"roll": ("roll", "gyro_x"), "pitch": ("pitch", "gyro_y"), "yaw": ("gyro_z",)}
ALL_BODY = ("roll", "gyro_x", "pitch", "gyro_y", "gyro_z")
LAGS = np.arange(-100, 101, 20)         # ms: the robot may answer a little late or early
H_LAGS_MS = np.arange(-100, 401, 20)   # ms: the plant's response to the excitation, as an FIR on it
STEP_END_S = 0.1                        # dwell: the step end frame is 0.1 s after the touchdown


def _th():
    import track_host as TH
    return TH


def law_gain(G, f):
    """|1 + (G - 1) HP(f)|: what grade's out_gain does to a sine of f Hz (HP: x - its 0.5 s EMA, at DT)."""
    k = 1 - math.exp(-DT / 0.5)
    zi = np.exp(-2j * math.pi * np.asarray(f, float) * DT)
    lp = k / (1 - (1 - k) * zi)
    return np.abs(1 + (G - 1) * (1 - lp))


def catmull_at(t_kf, v, tq):
    """The firmware's Catmull-Rom (track_host.catmull_play) at arbitrary query times tq (same unit as t_kf); 0
    outside [t_kf[0], t_kf[-1]]."""
    t = np.asarray(t_kf, float)
    v = np.asarray(v, float)
    tq = np.asarray(tq, float)
    i = np.clip(np.searchsorted(t, tq, side="right") - 1, 0, len(t) - 2)
    p0, p1, p2 = v[np.maximum(i - 1, 0)], v[i], v[i + 1]
    p3 = v[np.minimum(i + 2, len(v) - 1)]
    u = (tq - t[i]) / np.maximum(t[i + 1] - t[i], 1e-9)
    out = 0.5 * (2 * p1 + (-p0 + p2) * u + (2 * p0 - 5 * p1 + 4 * p2 - p3) * u ** 2 + (-p0 + 3 * p1 - 3 * p2 + p3) * u ** 3)
    return np.where((tq >= t[0]) & (tq <= t[-1]), out, 0.0)


def pattern(channel):
    """The channel's joint signs in the policy order (walk_gain.POL), and how many joints it has."""
    js = _th().WIGGLE_JOINTS[channel]
    p = np.array([js.get(n, 0) for n in WG.POL], float)
    return p, int(np.count_nonzero(p))


def body(gyro, up):
    """Body channels as walk_gain.channels() computes them (rad, rad/s)."""
    return dict(roll=-np.arcsin(np.clip(up[:, 1], -1, 1)), pitch=np.arctan2(up[:, 0], up[:, 2]),
                gyro_x=gyro[:, 0], gyro_y=gyro[:, 1], gyro_z=gyro[:, 2])


def zoh_goals(r, T):
    """The record's written goals on the grid T, zero-order hold, policy order (walk_gain.replay_one's)."""
    goal = r["goal"].copy()
    pos = r["pos"]
    for j in range(12):
        last = pos[np.argmax(np.isfinite(pos[:, j])), j]
        for i in range(len(goal)):
            if np.isfinite(goal[i, j]):
                last = goal[i, j]
            else:
                goal[i, j] = last
    gi = np.searchsorted(r["t"], T, side="right") - 1
    return goal[np.clip(gi, 0, len(r["t"]) - 1)][:, WG.B2P]


def replay_goals(goals, q0, servo="fast"):
    """walk_gain.replay_one's open-loop replay of a goal array (policy order, DT grid) from the pose q0."""
    WG.set_env()
    import grade  # noqa: F401
    import simh
    env, sv = WG.build_plant(len(goals) * DT + 10.0, servo=servo)
    simh.place_and_hold(env, np.clip(q0, env._lo, env._hi), 1.5)
    sv.reset(goals[0])
    cur = {"t": goals[0].copy()}
    env._action_to_ctrl = lambda a_: cur["t"]
    a0 = np.zeros(env.action_space.shape[0], np.float32)
    nj = env._nq_act
    d = env.data
    Q, GY, UP = [], [], []
    fell = False
    for k in range(len(goals)):
        cur["t"] = sv(goals[k])
        obs, _, _, _, _ = env.step(a0)
        Q.append(obs[:nj].astype(np.float64))
        GY.append(np.array(d.qvel[3:6]))
        w, x, y, z = (float(c) for c in d.qpos[3:7])
        UP.append((2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)))
        if UP[-1][2] < math.cos(math.radians(WG.FALL_DEG)):
            fell = True
            break
    return dict(q=np.array(Q), gyro=np.array(GY), up=np.array(UP), fell=fell, n=len(Q))


def excite_windows(ds, post_s=None, pre_s=None):
    """The excitation windows (s, on the segment's clock) of the double supports ds [(t_touchdown_or_start, t_liftoff)]
    as track_host.wiggle_windows builds them: from the step end frame (0.1 s after the touchdown) + pre_s to post_s
    before the lift-off, kept when both ramps fit."""
    TH = _th()
    post_s = TH.WIGGLE_POST_S if post_s is None else post_s
    pre_s = TH.WIGGLE_PRE_S if pre_s is None else pre_s
    out = []
    for a, b in ds:
        a_, b_ = a + STEP_END_S + pre_s, b - post_s
        if b_ - a_ >= 2 * TH.WIGGLE_RAMP_S + 0.1:
            out.append((a_, b_))
    return out


def template(frames, t_end_ms, kf_ms=100.0):
    """The logged excitation (track_host's `frames`: [t_ms, ticks] of every keyframe with an offset) on the full 10 Hz
    keyframe grid of the track (0 elsewhere), deg: (keyframe times ms, values deg)."""
    TH = _th()
    t = np.arange(0.0, max(t_end_ms, max((f[0] for f in frames), default=0.0)) + 2 * kf_ms, kf_ms)
    v = np.zeros(len(t))
    for tm, n in frames:
        v[int(round(tm / kf_ms))] = n / TH.TICKS_PER_DEG
    return t, v


def align(c, T, t_kf, v_kf, windows_ms, delta0, search0=1.5, search=(-0.1, 3.2), min_r=0.3):
    """Where each excitation window sits on the record's clock: c is the written goals' channel combination on the grid
    T (s), the template (t_kf ms, v_kf deg) is on the track's clock. The track clock runs with the wall clock except while
    the lift gate parks it (at a gate frame, where the excitation is 0), so each window k has its own offset
    delta_k = t_record - t_track (s), delta_k >= delta_{k-1}. Found by the cross-correlation of the two, band-passed
    (ALIGN_BAND), the first window within +-search0 of delta0, the next ones within `search` of the last one found;
    refined to 1 ms. A window whose best correlation is under min_r is not found (NaN).
    -> [(delta_k, peak correlation, fitted amplitude: 1 = the template's own)]."""
    from scipy.signal import butter, sosfiltfilt
    cb = WG.band(c[:, None], ALIGN_BAND)[:, 0]
    tau = np.arange(t_kf[0], t_kf[-1], 2.0)                     # ms: the template, band-passed on a 2 ms grid
    ub = sosfiltfilt(butter(2, ALIGN_BAND, btype="band", fs=500.0, output="sos"), catmull_at(t_kf, v_kf, tau))

    def corr(d, a, b):
        m = (T >= a / 1e3 + d - 0.3) & (T <= b / 1e3 + d + 0.3)
        if m.sum() < 20:
            return None
        u = np.interp(1e3 * (T[m] - d), tau, ub, left=0.0, right=0.0)
        x = cb[m] - cb[m].mean()
        den = math.sqrt(float(u @ u) * float(x @ x))
        return (float(x @ u) / den, float(x @ u) / float(u @ u)) if den > 0 else None

    out, prev = [], None
    for a, b in windows_ms:
        lo, hi = ((delta0 - search0, delta0 + search0) if prev is None else (prev + search[0], prev + search[1]))
        best = None
        for d in np.arange(lo, hi + 1e-9, 0.02):
            r_ = corr(d, a, b)
            if r_ is not None and (best is None or r_[0] > best[1]):
                best = (d, r_[0], r_[1])
        for step in (0.004, 0.001):
            if best is None:
                break
            for d in np.arange(best[0] - 6 * step, best[0] + 6 * step + 1e-12, step):
                r_ = corr(d, a, b)
                if r_ is not None and r_[0] > best[1]:
                    best = (d, r_[0], r_[1])
        if best is None or best[1] < min_r:
            out.append((float("nan"), best[1] if best else 0.0, 0.0))
            continue
        out.append((float(best[0]), float(best[1]), float(best[2])))
        prev = best[0]
    return out


def u_on_grid(T, t_kf, v_kf, windows_ms, deltas):
    """The excitation (deg) on the record's grid T from the aligned windows: each window's part of the template at
    its own offset."""
    u = np.zeros(len(T))
    for (a, b), (d, _, _) in zip(windows_ms, deltas):
        if not np.isfinite(d):
            continue
        tq = 1e3 * (T - d)
        m = (tq >= a - 150.0) & (tq <= b + 150.0)
        u[m] += catmull_at(t_kf, v_kf, tq[m])
    return u


def bp(x, band=BAND):
    return WG.band(np.asarray(x, float)[:, None], band)[:, 0]


def lagmat(x, lags_ms=H_LAGS_MS):
    """Columns x(t - lag) for each lag (ms), zero-padded."""
    n = len(x)
    out = np.zeros((n, len(lags_ms)))
    for k, L in enumerate(lags_ms):
        s_ = int(round(L / 1e3 / DT))
        if s_ >= 0:
            out[s_:, k] = x[:n - s_]
        else:
            out[:n + s_, k] = x[-s_:]
    return out


def fit_response(us, zs, ms, band=BAND):
    """The plant's response to the excitation as a short FIR h on it (H_LAGS_MS): least squares of the band-passed
    z = y_simW - y_sim0 on the band-passed, lagged excitation u (deg), over the masked samples of every segment. z also
    holds what the open-loop replays do apart once the excitation has nudged them (their gaits drift differently); that
    part is not driven by u and does not go into h."""
    A, y = [], []
    for u, z, m in zip(us, zs, ms):
        A.append(lagmat(bp(u, band))[m])
        y.append(bp(z, band)[m])
    A, y = np.vstack(A), np.concatenate(y)
    return np.linalg.lstsq(A, y, rcond=None)[0]


def instrument(u, h, band=BAND):
    """w = h * u: the plant's response to the excitation alone, from the excitation (the instrument)."""
    return lagmat(bp(u, band)) @ h


def sums(eb, zb, w, m, lags=LAGS):
    """For one window (mask m) and every lag of the robot (late = +): sum e w, sum z w, sum e e, sum w w, with e, z
    band-passed and w the instrument. Shape (4, len(lags))."""
    idx = np.where(m)[0]
    S = np.zeros((4, len(lags)))
    for k, lag in enumerate(lags):
        j = idx + int(round(lag / 1e3 / DT))
        ok = (j >= 0) & (j < len(eb))
        a, ww = eb[j[ok]], w[idx[ok]]
        S[:, k] = (float(a @ ww), float(zb[idx[ok]] @ ww), float(a @ a), float(ww @ ww))
    return S


def pool(S, lags=LAGS):
    """The instrumental-variable gain from summed sums(): g = sum(e w) / sum(z w) at the lag where the excitation
    explains the largest share of e (the coherence, (sum e w)^2 / (sum e e sum w w)), and at lag 0."""
    C, Zw, E, W2 = S.sum(0) if S.ndim == 3 else S
    with np.errstate(invalid="ignore", divide="ignore"):
        coh = np.where((E > 0) & (W2 > 0), C * np.abs(C) / (E * W2), -np.inf)
    k = int(np.argmax(coh))
    k0 = int(np.where(np.asarray(lags) == 0)[0][0])
    return dict(g=float(C[k] / Zw[k]), coh=float(max(coh[k], 0.0)), lag=float(lags[k]), g0=float(C[k0] / Zw[k0]),
                coh0=float(max(coh[k0], 0.0)))


def boot(S, nb=1000, seed=0, key="g0"):
    """90 % CI of the gain by resampling the units (rows of S: double supports)."""
    rng = np.random.default_rng(seed)
    pick = rng.integers(0, len(S), (nb, len(S)))
    gs = [pool(S[p_].sum(0))[key] for p_ in pick]
    return float(np.percentile(gs, 5)), float(np.percentile(gs, 95))


def read_channel(segs, band=BAND):
    """segs: [(u, e, z, [window masks])] of one channel -> (S per window (n, 4, lags), h). The instrument's FIR is
    fitted over every window of every segment first."""
    h = fit_response([x[0] for x in segs], [x[2] for x in segs],
                     [np.any(x[3], 0) if len(x[3]) else np.zeros(len(x[0]), bool) for x in segs], band)
    S = []
    for u, e, z, wins in segs:
        eb, zb, w = bp(e, band), bp(z, band), instrument(u, h, band)
        for m in wins:
            if m.sum() >= 20:
                S.append(sums(eb, zb, w, m))
    return np.array(S), h


# ----------------------------------------------------------------------------------------------------------------
# simcheck
# ----------------------------------------------------------------------------------------------------------------
def ds_spans(r, T):
    """The double supports of a walk segment, (start, end) s on T's clock, that end in a lift-off."""
    W = WG.windows(r, dict(t=T))
    out = []
    for i, (kind, _, i0, i1) in enumerate(W):
        if kind == "ds" and i + 1 < len(W) and W[i + 1][0] == "ss" and W[i + 1][2] == i1:
            out.append((T[i0] - T[0], T[i1 - 1] - T[0]))
    return out


def _simcheck_one(args):
    key, amp, channel, seed, gains, out_dir, shift_s = args
    WG.set_env()
    TH = _th()
    r = dict(np.load(os.path.join(WG.OUT, "robot", key + ".npz"), allow_pickle=False))
    T = np.arange(r["t"][0], r["t"][-1], DT)
    G0 = zoh_goals(r, T)
    pos = r["pos"]
    q0 = pos[int(np.argmax(np.all(np.isfinite(pos), 1)))][WG.B2P]
    spans = ds_spans(r, T)
    win_s = excite_windows(spans)
    if not win_s:
        return key, None
    spec = TH.wiggle_spec(amp, channel, seed)
    win_ms = [(1e3 * a, 1e3 * b) for a, b in win_s]
    t_kf = np.arange(0.0, 1e3 * (T[-1] - T[0]) + 1e-6, 100.0)
    n, _ = TH.wiggle_ticks(t_kf, win_ms, spec)
    v = n / TH.TICKS_PER_DEG
    u = catmull_at(t_kf, v, 1e3 * (T - T[0]))
    pat, nj = pattern(channel)
    GW = G0 + np.radians(u)[:, None] * pat[None, :]
    sv = str(r["servo"])
    sims = {"sim0": replay_goals(G0, q0, sv), "simW": replay_goals(GW, q0, sv)}
    for g in gains:
        if g != 1.0:
            # the stand-in robot's body at G: the out_gain law on the excitation (the robot's own gait as recorded is in
            # the disturbance below; the law on the whole gait tips the open-loop replay within seconds)
            ug = u + (g - 1.0) * WG.ema_hp(u[:, None])[:, 0]
            sims[f"g{g:g}"] = replay_goals(G0 + np.radians(ug)[:, None] * pat[None, :], q0, sv)
    n_ok = min(s_["n"] for s_ in sims.values())
    if n_ok < int(10.0 / DT):                 # a replay that fell in the first seconds: nothing to read
        return key, None
    # the robot's own disturbance on the plant's grid: robot - sim0 (walk_gain.channels: the robot at t + DT)
    s0 = dict(t=T[:n_ok], q=sims["sim0"]["q"][:n_ok], gyro=sims["sim0"]["gyro"][:n_ok], up=sims["sim0"]["up"][:n_ok])
    ch = WG.channels(r, s0)
    dist = {c: ch[c][0] - ch[c][1] for c in ALL_BODY}
    # usable: as walk_gain (both upright, the plant's attitude within 6 deg of the robot's)
    upr = np.stack([np.interp(T[:n_ok], r["t"], r["up"][:, i]) for i in range(3)], 1)
    tr = np.degrees(np.arccos(np.clip(upr[:, 2], -1, 1)))
    usable = (tr < 20) & (np.degrees(np.abs(dist["roll"] - np.median(dist["roll"][:50]))) < 6) & \
             (np.degrees(np.abs(dist["pitch"] - np.median(dist["pitch"][:50]))) < 6)
    for s_ in sims.values():
        usable &= np.degrees(np.arccos(np.clip(s_["up"][:n_ok, 2], -1, 1))) < 20
    # the robot path's alignment, on the written goals with the excitation, from a start 0.4 s off
    c = (GW[:n_ok] * pat[None, :]).sum(1) / nj
    dl = align(np.degrees(c), T[:n_ok] - T[0], t_kf, v, win_ms, delta0=shift_s)
    out = dict(T=T[:n_ok] - T[0], u=u[:n_ok], usable=usable, win_ms=np.array(win_ms), align=np.array(dl),
               GW=GW[:n_ok], q0=q0, kf_t=t_kf, kf_n=n, servo=sv)
    if "g1.25" in sims:                       # the stand-in robot's encoders at G 1.25, for the robot path's check
        out["rob_q"] = sims["g1.25"]["q"][:n_ok]
    for name, s_ in sims.items():
        for cn, y in body(s_["gyro"][:n_ok], s_["up"][:n_ok]).items():
            out[f"{name}:{cn}"] = y
        out[f"{name}:fell"] = s_["fell"]
    for cn, y in dist.items():
        out[f"dist:{cn}"] = y
    # the joint channel (the servo path): the channel's encoders, combined as the goals
    for name, s_ in sims.items():
        out[f"{name}:joint"] = (s_["q"][:n_ok] * pat[None, :]).sum(1) / nj
    os.makedirs(out_dir, exist_ok=True)
    np.savez_compressed(os.path.join(out_dir, key + ".npz"), **out)
    return key, len(win_ms)


def excited_mask(T, win_ms):
    """Samples inside an excitation window, plus 0.3 s after it (the response rings on)."""
    m = np.zeros(len(T), bool)
    for a, b in win_ms:
        m |= (T >= a / 1e3) & (T <= b / 1e3 + 0.3)
    return m


def per_window(T, win_ms):
    return [((T >= a / 1e3) & (T <= b / 1e3 + 0.3)) for a, b in win_ms]


def cmd_simcheck(a):
    out_dir = os.path.join(a.out or WG.OUT, f"wiggle_{a.channel}_a{a.amp:g}_s{a.seed}")
    keys = sorted(os.path.basename(p)[:-4] for p in glob.glob(os.path.join(WG.OUT, "sim", "walk_*.npz")))
    keys = [k for k in keys if os.path.exists(os.path.join(WG.OUT, "robot", k + ".npz"))]
    keys = keys[:: max(1, len(keys) // a.n)][:a.n]
    gains = [float(x) for x in a.gains.split(",")]
    todo = [k for k in keys if a.force or not os.path.exists(os.path.join(out_dir, k + ".npz"))]
    print(f"{len(keys)} walk segments ({len(todo)} to replay), excitation {a.amp:g} deg on {a.channel}, seed {a.seed}, "
          f"G {gains}", flush=True)
    if todo:
        with ProcessPoolExecutor(a.workers) as ex:
            for k, nw in ex.map(_simcheck_one, [(k, a.amp, a.channel, a.seed, gains, out_dir, 0.4) for k in todo]):
                print(f"  {k}: {nw} windows", flush=True)
    D = []
    for k in keys:
        p = os.path.join(out_dir, k + ".npz")
        if os.path.exists(p):
            D.append(dict(np.load(p, allow_pickle=False)))
    report(D, gains, a)
    if a.robot_path and 1.25 in gains:
        robot_path_check(D[:: max(1, len(D) // a.robot_path)][:a.robot_path], a)


def _robot_path_one(d):
    T = d["T"]
    wl = dict(channel=str(d["channel"]), windows_ms=d["win_ms"].tolist(),
              frames=[[float(t), int(n)] for t, n in zip(d["kf_t"], d["kf_n"]) if n])
    rob = {c: d[f"g1.25:{c}"] + d[f"dist:{c}"] for c in ALL_BODY}
    return analyze_run(T, d["GW"], rob, d["rob_q"], d["q0"], wl, delta0=0.4, servo=str(d["servo"]))


def robot_path_check(D, a):
    """The `robot` command's own path (alignment from the written goals, the excitation taken out again, two fresh
    replays) on stand-in robots at G 1.25: it must read what the simcheck reads."""
    for d in D:
        d["channel"] = a.channel
    per_ch = {c: [] for c in list(ALL_BODY) + ["joint"]}
    with ProcessPoolExecutor(a.workers) as ex:
        for _dl, segs in ex.map(_robot_path_one, D):
            for cn, sg in segs.items():
                per_ch[cn].append(sg)
    print(f"\nThe robot command's path on {len(D)} of the walks, the stand-in robot at G 1.25 (from the written goals, "
          f"a start 0.4 s off):")
    print_reading({c: per_ch[c] for c in list(BODY[a.channel]) + ["joint"]}, _th().wiggle_spec(a.amp, a.channel,
                                                                                               a.seed)["freqs_hz"])


def report(D, gains, a):
    TH = _th()
    spec = TH.wiggle_spec(a.amp, a.channel, a.seed)
    lg = {g: law_gain(g, spec["freqs_hz"]) for g in gains}
    nwin = sum(len(d["win_ms"]) for d in D)
    exc = sum(float(np.sum(np.diff(d["win_ms"], axis=1))) for d in D) / 1e3
    print(f"\n{len(D)} walks, {nwin} excited double supports, {exc:.0f} s excited; amplitude {a.amp:g} deg on "
          f"{a.channel} ({', '.join(TH.WIGGLE_JOINTS[a.channel])}), lines {spec['freqs_hz']} Hz; band {BAND} Hz")
    al = np.concatenate([d["align"] for d in D])
    ok = np.isfinite(al[:, 0])
    err = np.abs(al[ok, 0]) * 1e3                 # the truth is 0: the excitation was built on the record's clock
    print(f"alignment (the robot path, from a start 0.4 s off): {ok.sum()} of {len(al)} windows found, |error| median "
          f"{np.median(err):.1f} ms, 95 % {np.percentile(err, 95):.1f} ms, max {err.max():.1f}; correlation median "
          f"{np.median(al[ok, 1]):.2f}, fitted amplitude median {np.median(al[ok, 2]):.3f}")
    rows, sess = [], {}
    chans = list(BODY[a.channel]) + ["joint"]           # the other body channels hardly see this excitation
    print("\n| channel | true G (the law at the lines) | reading g at lag 0 [90 % CI] | coherence | best lag: g (ms) | "
          "0.7-2 Hz: g (coh) | 2-3.5 Hz: g (coh) | rms: excitation response / disturbance |")
    print("|---|---|---|---|---|---|---|---|")
    for c in chans:
        for g in gains:
            rob = "simW" if g == 1.0 else f"g{g:g}"
            res = {}
            for bn, b in [("all", BAND)] + list(SUB_BANDS.items()):
                segs = []
                for d in D:
                    T = d["T"]
                    y0, yW, yR = d[f"sim0:{c}"], d[f"simW:{c}"], d[f"{rob}:{c}"]
                    dist = d[f"dist:{c}"] if c != "joint" else np.zeros_like(y0)
                    wins = [m & d["usable"] for m in per_window(T, d["win_ms"])]
                    segs.append((d["u"], yR + dist - y0, yW - y0, wins))
                S, h = read_channel(segs, b)
                p_ = pool(S)
                if bn == "all":
                    p_["ci"] = boot(S)
                    p_["n"] = len(S)
                    num = den = 0.0
                    for d, (u, _e, _z, wins) in zip(D, segs):
                        m_ = np.any(wins, 0) if wins else np.zeros(len(u), bool)
                        num += float(np.sum(instrument(u, h, b)[m_] ** 2))
                        if c != "joint":
                            den += float(np.sum(bp(d[f"dist:{c}"], b)[m_] ** 2))
                    p_["snr"] = math.sqrt(num / den) if den > 0 else None
                    sess[(c, g)] = S
                res[bn] = p_
            rr = res["all"]
            snr = f"{rr['snr']:.2f}" if rr["snr"] is not None else "-"
            print(f"| {c} | {g:g} ({lg[g].min():.2f}-{lg[g].max():.2f}) | **{rr['g0']:.2f}** [{rr['ci'][0]:.2f}, "
                  f"{rr['ci'][1]:.2f}] | {rr['coh0']:.2f} | {rr['g']:.2f} ({rr['lag']:+.0f}) | {res['0.7-2']['g0']:.2f} "
                  f"({res['0.7-2']['coh0']:.2f}) | {res['2-3.5']['g0']:.2f} ({res['2-3.5']['coh0']:.2f}) | {snr} |")
            rows.append(dict(channel=c, G=g, law=[float(x) for x in lg[g]], **rr, sub={k_: res[k_] for k_ in SUB_BANDS}))
    # a robot session: K double supports drawn from the pool (an 8-step walk has 9), the reading's 5-95 % spread
    rng = np.random.default_rng(0)
    print("\nA robot session of K excited double supports (an 8-step walk has 9): the reading's 5-95 % range over 500 "
          "draws of K from the pool\n")
    print("| channel | K | " + " | ".join(f"G {g:g}" for g in gains) + " |\n|---|---|" + "---|" * len(gains))
    for c in BODY[a.channel]:
        for K in (18, 36, 54):
            cells = []
            for g in gains:
                S = sess[(c, g)]
                if len(S) < K:
                    cells.append("-")
                    continue
                v = [pool(S[rng.choice(len(S), K, replace=False)].sum(0))["g0"] for _ in range(500)]
                cells.append(f"{np.percentile(v, 5):.2f}-{np.percentile(v, 95):.2f}")
            print(f"| {c} | {K} | " + " | ".join(cells) + " |")
    fell = sum(bool(d[k_]) for d in D for k_ in d if k_.endswith(":fell"))
    print(f"\nreplays that fell (cut there): {fell} of {sum(1 for d in D for k_ in d if k_.endswith(':fell'))}")
    # what the excitation alone does to the body (the plant's FIR response to it, roll channel), and at each lift gate's
    # look: the roll rate over the 150 ms before the gate frame (post_s after the window's end)
    segs = [(d["u"], d["simW:gyro_x"] - d["sim0:gyro_x"], d["simW:gyro_x"] - d["sim0:gyro_x"],
             [m & d["usable"] for m in per_window(d["T"], d["win_ms"])]) for d in D]
    _, hx = read_channel(segs, (0.3, 8.0))
    segs = [(d["u"], d["simW:roll"] - d["sim0:roll"], d["simW:roll"] - d["sim0:roll"],
             [m & d["usable"] for m in per_window(d["T"], d["win_ms"])]) for d in D]
    _, hr = read_channel(segs, (0.3, 8.0))
    rates, rolls = [], []
    for d in D:
        T = d["T"]
        wx, wr = instrument(d["u"], hx, (0.3, 8.0)), instrument(d["u"], hr, (0.3, 8.0))
        rolls.append(np.degrees(wr[excited_mask(T, d["win_ms"])]))
        for _a, b_ in d["win_ms"]:
            m = (T >= b_ / 1e3 + TH.WIGGLE_POST_S - 0.15) & (T <= b_ / 1e3 + TH.WIGGLE_POST_S)
            if m.any():
                rates.append(float(np.max(np.abs(np.degrees(wx[m])))))
    rolls = np.concatenate(rolls)
    print(f"the excitation's own torso roll in the plant (its fitted response): rms {np.std(rolls):.3f} deg, peak "
          f"{np.max(np.abs(rolls)):.2f} deg; its roll rate in the 150 ms before each gate frame: median "
          f"{np.median(rates):.2f}, max {max(rates):.2f} deg/s (the gate holds above 15)")
    if a.json:
        json.dump(dict(spec=spec, walks=len(D), windows=nwin, excited_s=exc, rows=rows), open(a.json, "w"), indent=1)


# ----------------------------------------------------------------------------------------------------------------
# robot
# ----------------------------------------------------------------------------------------------------------------
def read_run(log_path):
    """A track log of an --id-wiggle run -> (stamp, the excitation log, t of `track play` (s, the log's clock), lead ms)."""
    rows = [json.loads(x) for x in open(log_path) if x.strip()]
    wl = next((r["id_wiggle"] for r in reversed(rows) if r.get("id_wiggle")), None)   # the summary's, else the first row's
    if wl is None:
        raise SystemExit(f"{log_path}: no id_wiggle row (a run of track_host.py --id-wiggle)")
    play = next(r for r in rows if str(r.get("cmd", "")).startswith("track play"))
    lead = float(str(play["cmd"]).split()[-1])
    stamp = os.path.basename(log_path)[6:21]
    return stamp, wl, float(play["t"]), lead


def robot_body(r, T):
    """The robot's body channels and encoders on the plant's grid (walk_gain.channels: sampled at T + DT)."""
    Tq = T + DT
    okr = np.all(np.isfinite(r["pos"]), 1)
    q = np.stack([np.interp(Tq, r["tfb"][okr], r["pos"][okr, b]) for b in WG.B2P], 1)
    gy = np.stack([np.interp(Tq, r["t"], r["gyro"][:, i]) for i in range(3)], 1)
    up = np.stack([np.interp(Tq, r["t"], r["up"][:, i]) for i in range(3)], 1)
    return body(gy, up), q


def analyze_run(T, GW, rob, q_rob, q0, wl, delta0, servo="fast", search0=3.0):
    """One --id-wiggle run on the record's grid T (s from its start): GW the written goals (policy order, rad), rob the
    robot's body channels and q_rob its encoders on T. The logged excitation is aligned to the written goals, taken out
    of them for sim0, both replayed. -> (alignment, {channel: (u, e, z, [window masks])}) for read_channel()."""
    pat, nj = pattern(wl["channel"])
    c = np.degrees((GW * pat[None, :]).sum(1) / nj)
    win_ms = [tuple(w) for w in wl["windows_ms"]]
    t_kf, v_kf = template(wl["frames"], win_ms[-1][1] + 1000.0)
    dl = align(c, T, t_kf, v_kf, win_ms, delta0=delta0, search0=search0)
    # a window whose written goals do not carry the logged excitation at its own size (a stall, a refused line, a
    # match on the gait's own moves) is left out
    dl = [x if AMP_OK[0] <= x[2] <= AMP_OK[1] else (float("nan"), x[1], x[2]) for x in dl]
    u = u_on_grid(T, t_kf, v_kf, win_ms, dl)
    G0 = GW - np.radians(u)[:, None] * pat[None, :]
    sW, s0 = replay_goals(GW, q0, servo), replay_goals(G0, q0, servo)
    n = min(sW["n"], s0["n"])
    bW, b0 = body(sW["gyro"][:n], sW["up"][:n]), body(s0["gyro"][:n], s0["up"][:n])
    usable = (np.degrees(np.arccos(np.clip(sW["up"][:n, 2], -1, 1))) < 20) & \
             (np.degrees(np.arccos(np.clip(s0["up"][:n, 2], -1, 1))) < 20)
    wins = []
    for (a_, b_), (d_, _, _) in zip(win_ms, dl):
        if np.isfinite(d_):
            wins.append((T[:n] >= a_ / 1e3 + d_) & (T[:n] <= b_ / 1e3 + d_ + 0.3) & usable)
    out = {}
    for cn in ALL_BODY:
        out[cn] = (u[:n], rob[cn][:n] - b0[cn], bW[cn] - b0[cn], wins)
    jq = lambda q: (q[:n] * pat[None, :]).sum(1) / nj  # noqa: E731
    out["joint"] = (u[:n], jq(q_rob) - jq(s0["q"]), jq(sW["q"]) - jq(s0["q"]), wins)
    return dl, out


def print_reading(per_ch, freqs):
    G_law = {g: law_gain(g, freqs) for g in (1.25, 1.5)}
    print(f"\nthe out_gain law at the lines: G 1.25 -> {G_law[1.25].min():.2f}-{G_law[1.25].max():.2f}, "
          f"G 1.5 -> {G_law[1.5].min():.2f}-{G_law[1.5].max():.2f}; the reading -> G: wiggle_gain_simcheck.md")
    print("\n| channel | windows | g at lag 0 [90 % CI] | coherence | best lag: g (ms) |\n|---|---|---|---|---|")
    res = {}
    for cn, segs in per_ch.items():
        S, _ = read_channel(segs) if segs else (np.zeros((0, 4, len(LAGS))), None)
        if len(S) < 5:
            print(f"| {cn} | {len(S)} | too few | | |")
            continue
        p_ = pool(S)
        ci = boot(S)
        res[cn] = dict(p_, ci=ci, n=len(S))
        print(f"| {cn} | {len(S)} | **{p_['g0']:.2f}** [{ci[0]:.2f}, {ci[1]:.2f}] | {p_['coh0']:.2f} | "
              f"{p_['g']:.2f} ({p_['lag']:+.0f}) |")
    return res


def cmd_robot(a):
    """One or more --id-wiggle runs on the robot (one channel): the flight record's window of each, the excitation
    aligned to its written goals, the plant replayed with (simW) and without (sim0) it, the estimate pooled."""
    WG.set_env()
    per_ch = {c: [] for c in list(ALL_BODY) + ["joint"]}
    chans, freqs = set(), None
    for lp in a.log:
        stamp, wl, t_play, lead = read_run(lp)
        chans.add(wl["channel"])
        freqs = wl["freqs_hz"]
        t0, tmax, _, note = WG.track_window(stamp, lp)
        fr = a.frec or WG.frec_for(t0)
        r = WG.read_windows(fr, [("run", t0 - 2.0, t0 + tmax + 4.0)])["run"]
        i0, i1, fell = WG.active_span(r)
        r = {k: (v[i0:i1 + 1] if isinstance(v, np.ndarray) else v) for k, v in r.items()}
        T = np.arange(r["t"][0], r["t"][-1], DT)
        GW = zoh_goals(r, T)
        rob, q_rob = robot_body(r, T)
        q0 = r["pos"][int(np.argmax(np.all(np.isfinite(r["pos"]), 1)))][WG.B2P]
        dl, segs = analyze_run(T - T[0], GW, rob, q_rob, q0, wl, delta0=t0 + t_play + lead / 1e3 - T[0])
        ok = [x for x in dl if np.isfinite(x[0])]
        print(f"{os.path.basename(lp)} ({note}; {wl['channel']} {wl['amp_deg']:g} deg, seed {wl['seed']}): "
              f"{len(ok)} of {len(dl)} windows aligned (fitted amplitude {AMP_OK[0]:g}-{AMP_OK[1]:g}), correlation "
              f"median {np.median([x[1] for x in dl] or [0]):.2f}, fitted amplitude median "
              f"{np.median([x[2] for x in dl] or [0]):.3f}; robot fell: {fell}", flush=True)
        for cn, sg in segs.items():
            per_ch[cn].append(sg)
    if len(chans) > 1:
        raise SystemExit(f"one channel per reading, got {sorted(chans)}: read the runs of each channel apart")
    res = print_reading(per_ch, freqs)
    if a.json:
        json.dump(res, open(a.json, "w"), indent=1)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("simcheck")
    s.add_argument("--n", type=int, default=48, help="walk segments (every k-th of walk_gain's)")
    s.add_argument("--amp", type=float, default=1.0)
    s.add_argument("--channel", default="roll", choices=["roll", "pitch", "yaw"])
    s.add_argument("--seed", type=int, default=1)
    s.add_argument("--gains", default="1.0,1.25,1.5")
    s.add_argument("--workers", type=int, default=8)
    s.add_argument("--force", action="store_true")
    s.add_argument("--json")
    s.add_argument("--out", help="where the replays go (default: walk_gain's build dir)")
    s.add_argument("--robot-path", type=int, default=12, help="walks to run the robot command's path on (0: none)")
    rb = sub.add_parser("robot")
    rb.add_argument("--log", action="append", required=True, help="a track_<stamp>.jsonl of an --id-wiggle run")
    rb.add_argument("--frec", help="the flight record (default: the latest one created before the run)")
    rb.add_argument("--json")
    a = ap.parse_args(argv)
    WG.set_env()
    {"simcheck": cmd_simcheck, "robot": cmd_robot}[a.cmd](a)


if __name__ == "__main__":
    main()
