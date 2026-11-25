#!/usr/bin/env python3
"""The late left-stance tips (#297): learned trim drift (A) or a push between the feet in double support (B)?

    python tip_causes.py extract --day 20261116 [--from HHMM --to HHMM]   # cache the robot side per run (build/)
    python tip_causes.py table   --day 20261116 [--from ... --to ...]     # one row per single support -> CSV
    python tip_causes.py report  CSV [CSV ...] [--since 20261116T1826]     # the A-vs-B comparisons, printed

A sibling of stance_tips.py (same runs, same single supports, same tip rule: lean toward the swing foot >= 7 deg; the
records docs/design-v6/2026-11-24-left-stance-tips.md and 2026-11-25-left-stance-tips-a-vs-b.md). This one keeps every
field the flight record has for the roll chain and the step-to-step state, so the two candidates can be tested apart.
Per single support (s = +1 left stance, -1 right; "toward the swing foot" = +):

  A  the firmware's learned stance-hip trim T (balance/stance_trim.h) drifting within a walk
     T_lift, T_other      T of the stance side at lift-off, and of the other side at its last touchdown, deg (hike sign)
     T_LmR, dT            T L - T R; T_lift minus that of the previous stance on this side
     hike_04, hike_08     the trim's hike (bal.u) at 0.4 / 0.8 s after lift-off, deg
     hip_corr_04, _08     the stance hip roll's goal - nominal there (the correction as written; equals the hike)
  B  the double support (DS) before the lift-off. The trim's correction is 0 there (res_hip_pre: the largest
     |goal - nominal| of either hip roll over the last 0.25 s, deg). Windows: td = 0.1-0.35 s after the previous
     touchdown, mid = 1.0-0.75 s and pre = 0.25-0 s before the lift-off, raise = 0.3-0.5 s after it.
     ld_{L,R}_{hip,ank}_<w>   roll loads, joint frame (STS counts; + = pushed toward + joint angle)
     sq_hip_<w>, sq_ank_<w>   (L - R) / 2 of those loads: the differential part (the feet pushed apart / together)
     cm_hip_<w>, cm_ank_<w>   (L + R) / 2: the common part (the pelvis pushed sideways)
     dev_*, wdev_<w>      pos - goal per roll joint; wdev = that of L hip roll minus R's, deg (+ = the hips read wider
                          than written; x ~121 mm hip roll to sole it is the stance-width error)
     tilt_<w>, tiltc_<w>  (hip + ankle roll) of L minus R from the encoders / the goals, deg: the soles' relative tilt
     width_<w>, widthn_<w>   hip roll L - R from the encoders / the track's nominal, deg
     roll_<w>             body roll (IMU), deg
     ds_roll_peak, ds_gyro_peak, ds_t_peak, ds_roll_end   the weight shift's roll transient ("snap"): over the DS
                          from 0.3 s after the previous touchdown to the lift-off, the peak roll toward the coming swing
                          foot relative to the window's start (deg), the peak rate (deg/s), its time (s from lift-off),
                          and the roll at lift-off relative to the start (deg)
     shift_t10, _t50, _t90   when the stance hip roll's nominal (the track) has done 10 / 50 / 90 % of its DS move, s
                          after the previous touchdown (t10: how soon the shift starts after the landing)
     prev_e_td, prev_hike_td, prev_lean_in, prev_lean_out   the previous (other side's) single support at its
                          touchdown: its error toward its swing foot and its hike (deg), its TS lean_in / lean_out
  stance_tips.py's names: e_05, gyro_06, e_09, beyond_06, beyond_08 (the same), and ld_st_hip, ld_st_ank (stance roll
     loads in the raise), ld_sw_hip (swing hip roll load, pre) in the joint frame x s (stance_tips.py's are raw counts).
  confounders: temp_* (deg C) of the four roll servos and the stance knee / ankle pitch at lift-off, v_servo (V),
     vbat (V), ds_s (the DS length), the step command.
Joint frame: obs::busStepsToAngle's (steps - zero) * dir; + hip roll moves the foot to the robot's left (+y).
The cache (build/tip-causes/<stamp>.npz) holds per tick: t, bal (state, side, ref, err, T, hike), roll, gyro_x,
vbat, and for 12 leg servos nominal / goal / pos / load / current / temp / voltage. One pass per flight record.
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "tools"))
import stance_tips as ST  # noqa: E402

CACHE = os.path.join(ROOT, "build", "tip-causes")
OUT = os.path.join(HERE, "results", "tip_causes")
LEG = ("L_hip_roll", "L_ankle_roll", "R_hip_roll", "R_ankle_roll", "L_hip_pitch", "L_knee", "L_ankle",
       "R_hip_pitch", "R_knee", "R_ankle", "L_hip_yaw", "R_hip_yaw")
JF = ("nominal", "goal", "pos", "load", "current", "temp_c", "voltage_dv")
DEG_PER_STEP = 360.0 / 4096.0
HELLO_LEN = 36 + 116 + 17 * 5


# ------------------------------------------------------------------------------------------------
# pure parts (tests/test_tip_causes.py)
# ------------------------------------------------------------------------------------------------
def joint_deg(steps, zero, direction):
    """Bus steps -> joint-frame degrees (obs::busStepsToAngle)."""
    return (np.asarray(steps, float) - zero) * DEG_PER_STEP * direction


def modes(left, right):
    """(differential, common) of a left / right pair: ((L - R) / 2, (L + R) / 2)."""
    return (left - right) / 2.0, (left + right) / 2.0


def auc(pos, neg):
    """P(a random pos > a random neg), ties 1/2 (the Mann-Whitney AUC); NaN without both."""
    pos = np.asarray([x for x in pos if np.isfinite(x)], float)
    neg = np.asarray([x for x in neg if np.isfinite(x)], float)
    if not len(pos) or not len(neg):
        return float("nan")
    gt = (pos[:, None] > neg[None, :]).sum() + 0.5 * (pos[:, None] == neg[None, :]).sum()
    return float(gt / (len(pos) * len(neg)))


def perm_p_auc(pos, neg, n=20000, seed=0):
    """Two-sided permutation p of |AUC - 0.5|."""
    pos = [x for x in pos if np.isfinite(x)]
    neg = [x for x in neg if np.isfinite(x)]
    if not pos or not neg:
        return float("nan")
    a0 = abs(auc(pos, neg) - 0.5)
    allv = np.array(pos + neg, float)
    r = np.argsort(np.argsort(allv, kind="mergesort"), kind="mergesort").astype(float)
    # average ranks for ties
    _, inv, cnt = np.unique(allv, return_inverse=True, return_counts=True)
    sums = np.zeros(len(cnt))
    np.add.at(sums, inv, r)
    r = (sums / cnt)[inv]
    rng = np.random.default_rng(seed)
    k, m = len(pos), len(neg)
    hits = 0
    for _ in range(n):
        idx = rng.permutation(k + m)[:k]
        u = r[idx].sum() - k * (k - 1) / 2.0
        hits += abs(u / (k * m) - 0.5) >= a0 - 1e-12
    return (hits + 1) / (n + 1)


# ------------------------------------------------------------------------------------------------
# the record
# ------------------------------------------------------------------------------------------------
def scan(path, windows):
    """One pass over a flight record: per window (t0, t1) UTC s, its ticks as a dict of arrays. Also the
    calibration (zero, dir per bus) of the boot that wrote them."""
    import flightrec as F
    names = list(F.BUS_JOINT_NAMES)
    bus = [names.index(k) for k in LEG]
    order = sorted(range(len(windows)), key=lambda i: windows[i][0])
    out = [dict(rows=[], cal=None) for _ in windows]
    cal = {}
    lo = min(w[0] for w in windows) - 2.0
    hi = max(w[1] for w in windows) + 2.0
    with open(path, "rb") as f:
        f.read(len(ST.FREC_MAGIC))
        f.readline()
        while True:
            h = f.read(ST.REC.size)
            if len(h) < ST.REC.size:
                break
            n, rx = ST.REC.unpack(h)
            t = rx / 1e6
            if t < lo:
                if n == HELLO_LEN:                       # keep the calibration current; skip everything else
                    buf = f.read(n)
                    if buf[3] == F.T_HELLO:
                        try:
                            p = F.decode(buf)
                            cal[p["boot_id"]] = (p["zero"], p["dir"])
                        except F.DecodeError:
                            pass
                else:
                    f.seek(n, 1)
                continue
            if t > hi:
                break
            buf = f.read(n)
            try:
                p = F.decode(buf)
            except F.DecodeError:
                continue
            if p["type"] == F.T_HELLO:
                cal[p["boot_id"]] = (p["zero"], p["dir"])
                continue
            if p["type"] != F.T_TICK:
                continue
            tt = p["t_utc_us"] / 1e6 if p["t_utc_us"] else t
            hit = [i for i in order if windows[i][0] <= tt <= windows[i][1]]
            if not hit:
                continue
            js = {j["bus"]: j for j in p["joints"]}
            b = p["bal"]
            row = [tt, b["state"], b["side"], math.degrees(b["ref"]), math.degrees(b["err"]),
                   math.degrees(b["integral"]), math.degrees(b["u"]), math.degrees(p["roll"]),
                   math.degrees(p["gyro"][0]), p["ctrl"]["vbat_mv"] / 1000.0]
            for bb in bus:
                j = js.get(bb)
                ok = j is not None and j["flags"] & F.JF_FB_OK
                if j is None:
                    row += [np.nan] * len(JF)
                else:     # the written goals always; the feedback only when read
                    row += [j[k] if (ok or k in ("nominal", "goal")) else np.nan for k in JF]
            for i in hit:
                out[i]["rows"].append(row)
                if out[i]["cal"] is None and p["boot_id"] in cal:
                    out[i]["cal"] = cal[p["boot_id"]]
    res = []
    for o in out:
        a = np.array(o["rows"], float) if o["rows"] else np.zeros((0, 10 + len(LEG) * len(JF)))
        a = a[np.argsort(a[:, 0])] if len(a) else a
        zero, d_ = o["cal"] if o["cal"] else ([2048] * F.MAX_JOINTS, [1] * F.MAX_JOINTS)
        res.append(dict(a=a, zero=np.array([zero[b] for b in bus], float), dir=np.array([d_[b] for b in bus], float)))
    return res


def extract(stamps):
    os.makedirs(CACHE, exist_ok=True)
    by_frec = {}
    for st in stamps:
        if os.path.exists(os.path.join(CACHE, f"{st}.npz")):
            continue
        try:
            rows = [json.loads(line) for line in open(ST.track_path(st)) if line.strip()]
        except FileNotFoundError:
            continue
        t0 = ST.stamp_utc(st)
        t1 = t0 + max(r.get("t", 0.0) for r in rows) + 6.0
        path = ST.frec_for(t0)
        if path:
            by_frec.setdefault(path, []).append((st, t0 - 3.0, t1))
    for path, runs in by_frec.items():
        print(f"{os.path.basename(path)}: {len(runs)} runs", flush=True)
        got = scan(path, [(a, b) for _, a, b in runs])
        for (st, _, _), g in zip(runs, got):
            np.savez_compressed(os.path.join(CACHE, f"{st}.npz"), **g)


def load(st):
    z = np.load(os.path.join(CACHE, f"{st}.npz"))
    return z["a"], z["zero"], z["dir"]


def col(name, field):
    return 10 + LEG.index(name) * len(JF) + JF.index(field)


def run_rows(st):
    rows = [json.loads(line) for line in open(ST.track_path(st)) if line.strip()]
    summ = rows[-1] if rows and rows[-1].get("summary") else {}
    ts, period = ST.parse_ts(rows)
    a, zero, dir_ = load(st)
    if not len(a) or period is None:
        return []
    t = a[:, 0]
    jd = {}
    for k, name in enumerate(LEG):
        for fld in ("nominal", "goal", "pos"):
            jd[(name, fld)] = joint_deg(a[:, col(name, fld)], zero[k], dir_[k])
        jd[(name, "load")] = a[:, col(name, "load")] * dir_[k]       # load follows (pos - goal): joint frame
    ch = ST.chains(a[:, 1])
    cmd = {s["step"]: s for s in (summ.get("cmd_walk") or {}).get("steps", [])}
    note = summ.get("note", "")
    is_cmd = bool(summ.get("cmd_walk"))

    def wm(y, t_ref, lo, hi):
        return ST.window_mean(t, y, t_ref, lo, hi)

    def at(y, tt):
        ok = np.isfinite(y)
        return float(np.interp(tt, t[ok], y[ok])) if ok.any() else float("nan")

    dev = {n: jd[(n, "pos")] - jd[(n, "goal")] for n in LEG[:4]}
    res = {n: jd[(n, "goal")] - jd[(n, "nominal")] for n in LEG[:4]}
    tilt_enc = (jd[("L_hip_roll", "pos")] + jd[("L_ankle_roll", "pos")]) - (jd[("R_hip_roll", "pos")] + jd[("R_ankle_roll", "pos")])
    tilt_cmd = (jd[("L_hip_roll", "goal")] + jd[("L_ankle_roll", "goal")]) - (jd[("R_hip_roll", "goal")] + jd[("R_ankle_roll", "goal")])
    tilt_nom = (jd[("L_hip_roll", "nominal")] + jd[("L_ankle_roll", "nominal")]) - (jd[("R_hip_roll", "nominal")] + jd[("R_ankle_roll", "nominal")])
    width = jd[("L_hip_roll", "pos")] - jd[("R_hip_roll", "pos")]
    width_nom = jd[("L_hip_roll", "nominal")] - jd[("R_hip_roll", "nominal")]
    wdev = dev["L_hip_roll"] - dev["R_hip_roll"]
    enc = {"L": jd[("L_hip_roll", "pos")] + jd[("L_ankle_roll", "pos")],
           "R": jd[("R_hip_roll", "pos")] + jd[("R_ankle_roll", "pos")]}
    out, count, last_T, prev_lift_T = [], {"L": 0, "R": 0}, {"L": float("nan"), "R": float("nan")}, {}
    for n in sorted(ts):
        x = ts[n]
        side = x["side"]
        count[side] += 1
        s = dict(stamp=st, note=note, cmd=int(is_cmd), period=round(period, 3),
                 timing="fast" if period < ST.FAST_PERIOD_S else "normal", n=n, side=side, k=count[side],
                 late=int(count[side] >= ST.LATE_K), tip=int(x["lean_in"] >= ST.TIP_DEG), lean_in=x["lean_in"],
                 T_mid=x["T"], step=json.dumps((cmd.get(n) or {}).get("step_mm_deg")))
        if n - 1 >= len(ch):
            out.append(s)
            continue
        i0, _, i_td = ch[n - 1]
        tl = t[i0]
        sign = 1.0 if side == "L" else -1.0
        other = "R" if side == "L" else "L"
        ok = np.isfinite(enc[side])
        s.update(ST.step_metrics(t[ok], a[ok, 4], a[ok, 8], enc[side][ok], tl, sign))
        s["lift_utc"] = tl
        T_lift = float(a[min(i0 + 2, len(a) - 1), 5])
        s["T_lift"] = T_lift
        s["T_other"] = last_T[other]
        s["T_LmR"] = T_lift - last_T[other] if side == "L" else last_T[other] - T_lift
        s["dT"] = T_lift - prev_lift_T.get(side, float("nan"))
        prev_lift_T[side] = T_lift
        # the trim's last value on this side (end of this chain), for the next step's T_other
        j_end = i_td if i_td is not None else min(i0 + 100, len(a) - 1)
        last_T[side] = float(a[j_end - 1, 5])
        s["hike_04"] = at(a[:, 6], tl + 0.4)
        s["hike_08"] = at(a[:, 6], tl + 0.8)
        st_hip = f"{side}_hip_roll"
        s["hip_corr_04"] = sign * at(res[st_hip], tl + 0.4)
        s["hip_corr_08"] = sign * at(res[st_hip], tl + 0.8)
        # the double support before this lift-off: from the previous chain's touchdown
        td_prev = None
        if n - 2 >= 0 and n - 2 < len(ch) and ch[n - 2][2] is not None:
            td_prev = t[ch[n - 2][2]]
        s["ds_s"] = tl - td_prev if td_prev is not None else float("nan")
        # the weight shift's timing in the DS, from the stance hip roll's nominal (the track): when it has done 10 %,
        # 50 % and 90 % of its move, s after the previous touchdown (t10 = how soon the shift starts after landing)
        if td_prev is not None:
            m = (t >= td_prev) & (t <= tl)
            y = jd[(st_hip, "nominal")][m]
            if m.sum() > 5 and np.isfinite(y).all() and abs(y[-1] - y[0]) > 2.0:
                f = (y - y[0]) / (y[-1] - y[0])
                for q in (10, 50, 90):
                    s[f"shift_t{q}"] = float(t[m][np.argmax(f >= q / 100.0)] - td_prev)
                # what the trim is doing when the shift gets going (the last chain's fade: state 7, its hike, deg)
                i10 = min(int(np.searchsorted(t, td_prev + s["shift_t10"])), len(a) - 1)
                s["fading_t10"] = int(a[i10, 1] == ST.FADE)
                s["hike_t10"] = float(abs(a[i10, 6]))
                # around the shift's fastest part: the servos' lag behind the goals (max |pos - goal|, deg) and the
                # stance hip roll goal's rate (deg/s), over t50 - 0.35 .. t50 + 0.15 s
                tm = td_prev + s["shift_t50"]
                mm = (t >= tm - 0.35) & (t <= tm + 0.15)
                if mm.sum() > 4:
                    for n_ in ("L_hip_roll", "R_hip_roll", "L_ankle_roll", "R_ankle_roll"):
                        s[f"lag_mid_{n_[0]}_{'hip' if 'hip' in n_ else 'ank'}"] = float(np.nanmax(np.abs(dev[n_][mm])))
                    s["goal_rate_mid"] = float(np.nanmax(np.abs(np.gradient(jd[(st_hip, "goal")][mm], t[mm]))))
        # the previous single support (the other side's) at its touchdown: its error toward ITS swing foot (the
        # foot that lands here, deg), its hike (deg) and its TS lean_in / lean_out
        if td_prev is not None:
            j = ch[n - 2][2]
            s["prev_e_td"] = float(-sign * a[j, 4])
            s["prev_hike_td"] = float(a[j, 6])
            if n - 1 in ts:
                s["prev_lean_in"], s["prev_lean_out"] = ts[n - 1]["lean_in"], ts[n - 1]["lean_out"]
        s["res_hip_pre"] = float(np.nanmax(np.abs(np.concatenate([
            res["L_hip_roll"][(t >= tl - 0.25) & (t <= tl)], res["R_hip_roll"][(t >= tl - 0.25) & (t <= tl)]]))))
        for tag, ref, lo, hi in (("pre", tl, -0.25, 0.0), ("td", td_prev, 0.1, 0.35), ("mid", tl, -1.0, -0.75),
                                 ("raise", tl, 0.3, 0.5)):
            if ref is None:
                continue
            L = {n_: wm(jd[(n_, "load")], ref, lo, hi) for n_ in LEG[:4]}
            for n_ in LEG[:4]:
                s[f"ld_{n_[0]}_{'hip' if 'hip' in n_ else 'ank'}_{tag}"] = L[n_]
                s[f"dev_{n_[0]}_{'hip' if 'hip' in n_ else 'ank'}_{tag}"] = wm(dev[n_], ref, lo, hi)
            s[f"sq_hip_{tag}"], s[f"cm_hip_{tag}"] = modes(L["L_hip_roll"], L["R_hip_roll"])
            s[f"sq_ank_{tag}"], s[f"cm_ank_{tag}"] = modes(L["L_ankle_roll"], L["R_ankle_roll"])
            s[f"wdev_{tag}"] = wm(wdev, ref, lo, hi)
            s[f"tilt_{tag}"] = wm(tilt_enc, ref, lo, hi)
            s[f"tiltc_{tag}"] = wm(tilt_cmd, ref, lo, hi)
            s[f"tiltn_{tag}"] = wm(tilt_nom, ref, lo, hi)
            s[f"width_{tag}"] = wm(width, ref, lo, hi)
            s[f"widthn_{tag}"] = wm(width_nom, ref, lo, hi)
            s[f"roll_{tag}"] = wm(a[:, 7], ref, lo, hi)
        # the weight shift's roll transient (the "snap"): over the DS from 0.3 s after the previous touchdown (or
        # 1.8 s before the lift-off) to the lift-off, toward the coming swing foot (+), relative to the roll there
        w0 = (td_prev + 0.3) if td_prev is not None else tl - 1.8
        m = (t >= w0) & (t <= tl) & np.isfinite(a[:, 7])
        if m.sum() > 5:
            r0 = sign * a[m, 7][0]
            s["ds_roll_peak"] = float(np.max(sign * a[m, 7]) - r0)
            s["ds_gyro_peak"] = float(np.max(sign * a[m, 8]))
            s["ds_t_peak"] = float(t[m][np.argmax(sign * a[m, 7])] - tl)
            s["ds_roll_end"] = float(sign * wm(a[:, 7], tl, -0.25, 0.0) - r0)
        # the stance side's loads in the raise, as stance_tips.py names them
        s["ld_st_hip"] = s.get(f"ld_{side}_hip_raise", float("nan")) * sign
        s["ld_st_ank"] = s.get(f"ld_{side}_ank_raise", float("nan")) * sign
        s["ld_sw_hip"] = s.get(f"ld_{other}_hip_pre", float("nan")) * sign
        i_t = min(i0, len(a) - 1)
        for n_ in ("L_hip_roll", "L_ankle_roll", "R_hip_roll", "R_ankle_roll", f"{side}_knee", f"{side}_ankle"):
            seg = a[max(0, i_t - 25):i_t + 1, col(n_, "temp_c")]
            s[f"temp_{n_}" if "knee" not in n_ and n_ != f"{side}_ankle" else f"temp_st_{n_[2:]}"] = \
                float(np.nanmedian(seg)) if np.isfinite(seg).any() else float("nan")
        seg = a[max(0, i_t - 25):i_t + 1, col("L_hip_roll", "voltage_dv")]
        s["v_servo"] = float(np.nanmedian(seg)) / 10.0 if np.isfinite(seg).any() else float("nan")
        s["vbat"] = float(np.nanmedian(a[max(0, i_t - 25):i_t + 1, 9]))
        out.append(s)
    return out


def table(stamps):
    steps = []
    for st in stamps:
        if not os.path.exists(os.path.join(CACHE, f"{st}.npz")):
            continue
        try:
            steps += run_rows(st)
        except (ValueError, KeyError, IndexError) as ex:
            print(f"{st}: skipped ({ex})", file=sys.stderr)
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, f"{stamps[0]}_{stamps[-1]}.csv")
    keys = []
    for s in steps:
        keys += [k for k in s if k not in keys]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows([{k: round(v, 4) if isinstance(v, float) else v for k, v in s.items()} for s in steps])
    print(f"{len(steps)} single supports -> {os.path.relpath(path, ROOT)}")
    return path


# ------------------------------------------------------------------------------------------------
# the report (the numbers of the 2026-11-25 record)
# ------------------------------------------------------------------------------------------------
SNAP_DEG = 2.5     # the weight shift's roll transient toward the coming swing foot, deg (ds_roll_peak)
LAG_S = 0.5        # the shift "starts late" when its 10 % point is >= this after the previous touchdown


def ols(rows, y, xs, by_walk=True):
    """Least squares of y on xs (walk fixed effects: everything demeaned per run). (coef, se, r2, n)."""
    Y = np.array([r.get(y, np.nan) for r in rows], float)
    X = np.array([[r.get(x, np.nan) for x in xs] for r in rows], float)
    ok = np.isfinite(Y) & np.isfinite(X).all(1)
    Y, X, g = Y[ok], X[ok], np.array([r["stamp"] for r in rows])[ok]
    groups = sorted(set(g))
    if by_walk:
        for w in groups:
            m = g == w
            Y[m] -= Y[m].mean()
            X[m] -= X[m].mean(0)
    else:
        X = np.c_[np.ones(len(X)), X]
    b = np.linalg.lstsq(X, Y, rcond=None)[0]
    e = Y - X @ b
    dof = max(1, len(Y) - X.shape[1] - (len(groups) if by_walk else 0))
    se = np.sqrt(np.diag(e @ e / dof * np.linalg.pinv(X.T @ X)))
    r2 = 1.0 - (e @ e) / max(1e-12, ((Y - Y.mean()) @ (Y - Y.mean())))
    return b, se, r2, len(Y)


def logit_nll(rows, xs):
    """Minimum negative log-likelihood of tip ~ xs (standardised) and the coefficients' z (scipy's BFGS)."""
    from scipy import optimize
    Y = np.array([r["tip"] for r in rows], float)
    X = np.array([[r.get(x, np.nan) for x in xs] for r in rows], float)
    ok = np.isfinite(X).all(1)
    Y, X = Y[ok], X[ok]
    Z = np.c_[np.ones(len(X)), (X - X.mean(0)) / X.std(0)]
    res = optimize.minimize(lambda b: np.sum(np.logaddexp(0, Z @ b) - Y * (Z @ b)), np.zeros(Z.shape[1]), method="BFGS")
    return res.fun, res.x / np.sqrt(np.diag(res.hess_inv))


def report(rows, since=""):
    from scipy import stats
    rows = [r for r in rows if r["stamp"] >= since and np.isfinite(r.get("ds_roll_peak", np.nan))
            and np.isfinite(r.get("sq_hip_pre", np.nan))]
    med = lambda rs, k: float(np.nanmedian([r.get(k, np.nan) for r in rs])) if rs else float("nan")  # noqa: E731
    snap = lambda r: r["ds_roll_peak"] >= SNAP_DEG  # noqa: E731
    print(f"{len(rows)} single supports, {sum(r['tip'] for r in rows)} tips")
    print("\n1. The weight-shift snap (roll toward the coming swing foot >= %.1f deg in the DS) by stance k" % SNAP_DEG)
    for timing in ("normal", "fast"):
        for side in "LR":
            rs = [r for r in rows if r["timing"] == timing and r["side"] == side]
            cells = []
            for lab, f in (("1", lambda k: k == 1), ("2", lambda k: k == 2), ("3-4", lambda k: 3 <= k <= 4),
                           ("5-6", lambda k: 5 <= k <= 6), ("7+", lambda k: k >= 7)):
                b = [r for r in rs if f(r["k"])]
                cells.append(f"k {lab}: {sum(map(snap, b))}/{len(b)} snap, {sum(r['tip'] for r in b)} tips")
            print(f"   {timing:6s} {side}  " + " | ".join(cells))
    print("\n2. Tips by the shift's start after the previous touchdown (shift_t10), stance k >= 2")
    for side in "LR":
        for timing in ("normal", "fast"):
            for lab, f in ((f">= {LAG_S}", lambda r: r.get("shift_t10", np.nan) >= LAG_S),
                           (f"<  {LAG_S}", lambda r: r.get("shift_t10", np.nan) < LAG_S)):
                b = [r for r in rows if r["side"] == side and r["timing"] == timing and r["k"] >= 2 and f(r)]
                late = [r for r in b if r["k"] >= 5]
                if b:
                    print(f"   {side} {timing:6s} t10 {lab} s: {len(b):4d} stances, snap {100 * np.mean([snap(r) for r in b]):3.0f} %, "
                          f"tips {sum(r['tip'] for r in b):2d}; late (k >= 5) {len(late)}, tips {sum(r['tip'] for r in late)}, "
                          f"T_lift median {med(late, 'T_lift'):.2f}, T_lift >= 6 in {sum(r['T_lift'] >= 6 for r in late)}")
    nl = [r for r in rows if r["timing"] == "normal" and r["side"] == "L"]
    late = [r for r in nl if r["late"]]
    tips = [r for r in late if r["tip"]]
    clean = [r for r in late if not r["tip"]]
    print(f"\n3. Normal-timing late left stances: {len(tips)} tips / {len(late)}; AUC tip vs clean (permutation p), medians")
    for k in ("k", "T_lift", "T_other", "hike_04", "ds_roll_peak", "ds_roll_end", "roll_pre", "sq_hip_pre", "wdev_pre",
              "tilt_pre", "ld_R_hip_pre", "ld_L_ank_raise", "temp_L_hip_roll", "temp_L_ankle_roll", "temp_R_hip_roll",
              "temp_R_ankle_roll", "v_servo", "shift_t10"):
        pv, cv = [r.get(k, np.nan) for r in tips], [r.get(k, np.nan) for r in clean]
        print(f"   {k:18s} AUC {auc(pv, cv):.2f} (p {perm_p_auc(pv, cv, n=4000):.4f})   tips {med(tips, k):+8.2f}   "
              f"clean {med(clean, k):+8.2f}")
    print("\n4. The swing-back e@0.9 s on every normal-timing left stance (walk fixed effects): coef +- se")
    for xs in (["k"], ["T_lift"], ["sq_hip_pre"], ["wdev_pre"], ["sq_hip_pre", "T_lift", "k"],
               ["wdev_pre", "T_lift", "T_other", "k"]):
        b, se, r2, n = ols(nl, "e_09", xs)
        print(f"   e_09 ~ {' + '.join(xs):32s} n {n}, R2 {r2:.2f}: " + ", ".join(f"{x} {bb:+.3f} +- {ss:.3f}"
                                                                         for x, bb, ss in zip(xs, b, se)))
    sn = [r for r in nl if r["k"] >= 2 and snap(r)]
    print(f"\n5. Snap steps only (normal L, k >= 2): {sum(r['tip'] for r in sn)} tips / {len(sn)}; snap k 2-4 vs k >= 5 medians")
    for k in ("ds_roll_peak", "sq_hip_pre", "wdev_pre", "T_lift", "hike_04", "e_09"):
        a_ = [r for r in sn if r["k"] <= 4]
        b_ = [r for r in sn if r["k"] >= 5 and not r["tip"]]
        print(f"   {k:14s} k 2-4 {med(a_, k):+7.2f}   k >= 5 clean {med(b_, k):+7.2f}   tips {med([r for r in sn if r['tip']], k):+7.2f}")
    nll = {tuple(xs): logit_nll(sn, xs) for xs in (["sq_hip_pre"], ["sq_hip_pre", "T_lift"], ["sq_hip_pre", "k"],
                                                     ["sq_hip_pre", "T_lift", "k"])}
    lr = lambda a_, b_: stats.chi2.sf(2 * (nll[a_][0] - nll[b_][0]), 1)  # noqa: E731
    print(f"   logistic tip ~ sq_hip_pre + T_lift + k: z {', '.join(f'{z:+.1f}' for z in nll[('sq_hip_pre', 'T_lift', 'k')][1][1:])}; "
          f"T_lift added to sq + k: p {lr(('sq_hip_pre', 'k'), ('sq_hip_pre', 'T_lift', 'k')):.4f}; "
          f"k added to sq + T_lift: p {lr(('sq_hip_pre', 'T_lift'), ('sq_hip_pre', 'T_lift', 'k')):.3f}")
    print("\n6. The shift's fast part (L stances, k >= 2): medians by the shift's start and the snap")
    ls = [r for r in rows if r["side"] == "L" and r["k"] >= 2 and np.isfinite(r.get("shift_t10", np.nan))]
    for lab, f in (("start < %.1f s, no snap" % LAG_S, lambda r: r["shift_t10"] < LAG_S and not snap(r)),
                   ("start < %.1f s, snap" % LAG_S, lambda r: r["shift_t10"] < LAG_S and snap(r)),
                   ("start >= %.1f s" % LAG_S, lambda r: r["shift_t10"] >= LAG_S)):
        b = [r for r in ls if f(r)]
        print(f"   {lab:24s} n {len(b):4d}: goal rate {med(b, 'goal_rate_mid'):5.1f} deg/s, lag L hip {med(b, 'lag_mid_L_hip'):.2f} "
              f"R hip {med(b, 'lag_mid_R_hip'):.2f} L ank {med(b, 'lag_mid_L_ank'):.2f} deg; trim fading at the shift's 10 %: "
              f"{100 * np.mean([r.get('fading_t10', 0) for r in b]):.0f} %, its hike {med(b, 'hike_t10'):.2f} deg")
    early = [dict(r, snap=float(snap(r))) for r in ls if r["shift_t10"] < LAG_S]
    for xs in (["hike_t10"], ["T_other"], ["prev_hike_td"], ["prev_e_td"], ["k"]):
        b, se, r2, n = ols(early, "snap", xs)
        print(f"   P(snap) ~ {' + '.join(xs):14s} (start < {LAG_S} s, walk fixed effects) n {n}, R2 {r2:.3f}: "
              + ", ".join(f"{x} {bb:+.3f} +- {ss:.3f}" for x, bb, ss in zip(xs, b, se)))
    print("\n7. The DS state (sq_hip_pre) against the previous right stance (normal L, k >= 2, walk fixed effects)")
    nl2 = [r for r in nl if r["k"] >= 2]
    for xs in (["T_other"], ["prev_hike_td"], ["prev_e_td"], ["k"], ["T_other", "prev_e_td", "prev_hike_td", "k"]):
        b, se, r2, n = ols(nl2, "sq_hip_pre", xs)
        print(f"   sq_hip_pre ~ {' + '.join(xs):40s} n {n}, R2 {r2:.2f}: " + ", ".join(
            f"{x} {bb:+.2f} +- {ss:.2f}" for x, bb, ss in zip(xs, b, se)))


def read_csv(path):
    rows = []
    for r in csv.DictReader(open(path)):
        d = {}
        for k, v in r.items():
            try:
                d[k] = float(v) if v not in ("", None) else float("nan")
            except ValueError:
                d[k] = v
        for k in ("n", "k", "late", "tip", "cmd"):
            if k in d and np.isfinite(d[k]):
                d[k] = int(d[k])
        rows.append(d)
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=["extract", "table", "report"])
    ap.add_argument("args", nargs="*")
    ap.add_argument("--day", action="append", default=[])
    ap.add_argument("--from", dest="t_from", default="0000")
    ap.add_argument("--to", dest="t_to", default="2359")
    ap.add_argument("--since", default="", help="report: only runs with stamp >= this (e.g. 20261116T1826)")
    a = ap.parse_args(argv)
    if a.what == "report":
        rows = []
        for path in a.args:
            rows += read_csv(path)
        report(rows, a.since)
        return
    stamps = list(a.args)
    for d in a.day:
        stamps += ST.stamps_for(d, a.t_from, a.t_to)
    stamps = sorted(set(stamps))
    if not stamps:
        ap.error("no runs")
    if a.what == "extract":
        extract(stamps)
    else:
        table(stamps)


if __name__ == "__main__":
    main()
