"""Summarise the x1.5 traces (trace_x15.py): per run x gain, falls, time to fall, and the shape of
the divergence, measured as the difference from the SAME run and seed at gain 1.0 (identical
initial state, so the difference is what the gain did). Prints falls per run x gain; the per-episode table
(onset, growth time, leading joints, axis, foot creep, ...) goes to the csv.

  python analyze.py TRDIR [csv]"""
import glob, os, re, sys
import numpy as np

DT = 0.02
J = ["L_hip_yaw", "L_hip_roll", "L_hip_pitch", "L_knee", "L_ankle", "L_ankle_roll",
     "R_hip_yaw", "R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle", "R_ankle_roll"]
SHORT = ["Lhy", "Lhr", "Lhp", "Lkn", "Lan", "Lar", "Rhy", "Rhr", "Rhp", "Rkn", "Ran", "Rar"]


def hp_rms(a, tau=0.5):
    """rms of a - EMA(a, tau): the part of the action the probe's gain scales."""
    e = a[0].copy(); out = []
    k = 1 - np.exp(-DT / tau)
    for x in a:
        e = e + k * (x - e); out.append(x - e)
    out = np.array(out)
    return np.sqrt((out ** 2).mean(0)), out


def spectrum_peak(x):
    x = x - x.mean()
    F = np.abs(np.fft.rfft(x * np.hanning(len(x))))
    fr = np.fft.rfftfreq(len(x), DT)
    k = 1 + np.argmax(F[1:])
    return float(fr[k])


def one(f, base):
    d = np.load(f)
    QM = np.degrees(d["QM"]); n = len(QM); t = np.arange(n) * DT
    r = dict(fell=bool(d["fell"]), t_end=float(d["t_end"]), trip=str(d["trip"]))
    yaw = np.degrees(np.unwrap(d["YW"])); yaw -= yaw[0]
    r["yaw_absmax"] = float(np.abs(yaw).max())
    r["x_end"] = float(d["X"][-1]); r["y_end"] = float(d["Y"][-1])
    # foot creep on the floor (mm, xy of each sole from t=0) and body yaw at 8 s and at the end
    FT = d["FT"]
    for tag, i in (("8", min(n - 1, int(8.0 / DT))), ("end", n - 1)):
        r["creepL_" + tag] = float(1000 * np.linalg.norm(FT[i, 2:4] - FT[0, 2:4]))
        r["creepR_" + tag] = float(1000 * np.linalg.norm(FT[i, 4:6] - FT[0, 4:6]))
        r["yaw_" + tag] = float(yaw[i])
    r["stance_w_change_end"] = float(1000 * (np.linalg.norm(FT[-1, 2:4] - FT[-1, 4:6]) - np.linalg.norm(FT[0, 2:4] - FT[0, 4:6])))
    go_hr, _ = hp_rms(np.degrees(d["GO"][int(1.5 / DT):]) if n > 100 else np.degrees(d["GO"]))
    r["goal_hp_rms_deg"] = float(np.sqrt((go_hr ** 2).mean()))
    hr, hpo = hp_rms(d["AC"][int(1.5 / DT):] if n > 100 else d["AC"])
    r["act_hp_rms"] = float(np.sqrt((hr ** 2).mean()))
    r["act_hp_top"] = ",".join(f"{SHORT[j]}{hr[j]:.3f}" for j in np.argsort(-hr)[:3])
    if base is None:
        return r
    B = np.degrees(base["QM"])
    m = min(n, len(B))
    dev = QM[:m] - B[:m]
    mx = np.abs(dev).max(1)
    on = np.where(mx > 1.0)[0]
    if len(on) == 0:
        r["onset"] = np.nan
        return r
    k = on[0]; r["onset"] = float(t[k])
    k5 = np.where(mx[k:] > 5.0)[0]
    k5 = k + (k5[0] if len(k5) else m - 1 - k)
    r["t_1to5"] = float((k5 - k) * DT)
    w = slice(k, k5 + 1)
    amp = np.abs(dev[w]).max(0)
    order = np.argsort(-amp)
    r["lead"] = " ".join(f"{SHORT[j]}{'+' if dev[k5, j] >= 0 else '-'}{amp[j]:.1f}" for j in order[:4])
    gy = d["GY"][k:k5 + 1] - base["GY"][k:k5 + 1]
    r["gy_rms_xyz"] = "/".join(f"{x:.2f}" for x in np.sqrt((gy ** 2).mean(0)))
    r["axis"] = "xyz"[int(np.argmax(np.sqrt((gy ** 2).mean(0))))]
    seg = dev[max(0, k - 50):k5 + 1, order[0]]
    r["f_lead_hz"] = spectrum_peak(seg) if len(seg) > 40 else np.nan
    # sign crossings of the lead joint's difference over the growth window (oscillation vs drift)
    s = np.sign(dev[k:k5 + 1, order[0]]); r["n_cross"] = int((np.diff(s) != 0).sum())
    return r


def main():
    files = sorted(glob.glob(os.path.join(sys.argv[1], "*.npz")))
    meta = []
    for f in files:
        m = re.match(r"(.*)_g([\d.]+)_s(\d+)\.npz", os.path.basename(f))
        meta.append((f, m.group(1), float(m.group(2)), int(m.group(3))))
    bases = {(run, s): np.load(f) for f, run, g, s in meta if g == 1.0}
    rows = []
    for f, run, g, s in meta:
        r = one(f, None if g == 1.0 else bases.get((run, s)))
        r.update(run=run, g=g, s=s); rows.append(r)
    import pandas as pd
    df = pd.DataFrame(rows)
    if len(sys.argv) > 2:
        df.to_csv(sys.argv[2], index=False)
    piv = df.pivot_table(index="run", columns="g", values="fell", aggfunc="sum")
    print(piv.fillna(-1).astype(int).to_string())
    return df


if __name__ == "__main__":
    main()
