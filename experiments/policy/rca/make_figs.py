"""Figures for experiments/policy/2026-11-01-first-stand-rca.md (writes figs/*.png)."""
import os, sys, json
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import rec, simh, takeover as T, stepresp as SRP, sim_stepresp as SSR, sweep_sim as SS, sweep_fit as SF
from scipy.signal import butter, filtfilt, hilbert
FIG = os.path.join(HERE, "figs"); os.makedirs(FIG, exist_ok=True)
C = dict(real="#2a78d6", sim="#eb6834", sim2="#1baf7a", py="#e87ba4", grey="#888888")
plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.grid": True, "grid.alpha": 0.25, "legend.frameon": False})


def env_hf(g):
    b, a = butter(3, [6 / 25, 13 / 25], "band")
    return np.abs(hilbert(filtfilt(b, a, g)))


def arate(a):
    r = np.r_[0, np.sqrt(np.sum(np.diff(a, axis=0) ** 2, 1))]
    return np.convolve(r, np.ones(10) / 10, mode="same")


def fig_timeline():
    r = rec.load(); n = 498; t = np.arange(n) * 0.02
    a = rec.angles_to_action(r["nom"][:n])
    sims = [("sim: fitted plant as trained", dict(servo=simh.Servo(lag_hz=12)), C["sim"]),
            ("sim: measured actuator, loop gain x1.25", dict(servo=simh.TypedServo(**simh.MEAS_SERVO),
             model=simh.JF_PLANT, out_gain=1.25), C["sim2"])]
    tr = []
    for lab, kw, col in sims:
        best = None
        for seed in range(8):
            m = T.episode("fit", seed, kw["servo"], kw.get("model"), record=True, out_gain=kw.get("out_gain", 1.0))
            if best is None or (np.isfinite(m["t_onset"]) and abs(m["t_onset"] - 3.3) < abs((best["t_onset"] if np.isfinite(best["t_onset"]) else 99) - 3.3)):
                best = m
        tr.append((lab, best, col))
    fig, ax = plt.subplots(4, 1, figsize=(9, 9), sharex=True)
    ax[0].plot(t, arate(a), color=C["real"], lw=1.4, label="robot 2026-11-01")
    for lab, m, col in tr:
        ts = np.arange(len(m["trace"]["a"])) * 0.02
        ax[0].plot(ts, arate(m["trace"]["a"]), color=col, lw=1.2, label=f"{lab} (seed {m['seed']})")
    ax[0].set_ylabel("action change / tick\n(0.2 s mean)"); ax[0].set_yscale("log"); ax[0].legend(loc="upper left")
    ax[0].axvline(1.0, color=C["grey"], lw=0.8, ls=":"); ax[0].text(1.02, 1.2, "ramp ends", fontsize=7, color=C["grey"])
    for i, nm in enumerate(["roll (x)", "pitch (y)", "yaw (z)"]):
        ax[i + 1].plot(t, r["gyro"][:n, i], color=C["real"], lw=1)
        for lab, m, col in tr:
            ts = np.arange(len(m["trace"]["gyro"])) * 0.02
            ax[i + 1].plot(ts, m["trace"]["gyro"][:, i], color=col, lw=0.9)
        ax[i + 1].set_ylabel(f"gyro {nm}\nrad/s"); ax[i + 1].set_ylim(-3, 3)
    ax[-1].set_xlabel("s after `run`"); ax[-1].set_xlim(0, 6)
    fig.suptitle("First policy stand: growing oscillation, robot vs sim (same arm path, same start pose)", fontsize=10)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "timeline.png"), dpi=110); plt.close(fig)
    return {lab: dict(seed=m["seed"], t_onset=m["t_onset"]) for lab, m, _ in tr}


def fig_fidelity():
    d = np.load(os.path.join(HERE, "fidelity.npz"))
    t, fw, py = d["t"], d["a_fw"], d["a_py"]
    js = ["R_hip_roll", "L_ankle_roll", "R_hip_pitch", "R_knee"]
    fig, ax = plt.subplots(len(js) + 1, 1, figsize=(9, 8), sharex=False)
    for k, jn in enumerate(js):
        j = rec.POL.index(jn)
        ax[k].plot(t, fw[:, j], color=C["real"], lw=1.6, label="firmware (recorded nominal)")
        ax[k].plot(t, py[:, j], color=C["py"], lw=0.8, ls="--", label="python student on the recorded obs")
        ax[k].set_ylabel(jn, fontsize=8); ax[k].set_xlim(0, 10)
    ax[0].legend(loc="upper left")
    ax[-1].hist(d["err"].ravel(), bins=60, color=C["real"])
    ax[-1].set_yscale("log"); ax[-1].set_xlabel("|python - firmware| action, all joints, all 498 ticks")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fidelity.png"), dpi=110); plt.close(fig)


def fig_actuator():
    real_l = SRP.parse(os.path.join(HERE, "data", "2026-11-01_stepresp_loaded.txt"))
    by = {SRP.NAMES[b["id"]]: b for b in real_l}
    fig, ax = plt.subplots(1, 3, figsize=(12, 3.6))
    for k, jn in enumerate(["R_hip_pitch", "R_hip_roll"]):
        b = by[jn]; s = np.array(b["s"][0], float)
        ax[k].plot(s[:, 0], s[:, 1] / b["step"], color=C["real"], lw=1.6, label="robot (loaded stepresp)")
        for lab, kw, model, col in [("sim as trained (no dead time)", dict(dead15=0, tau15=0, dead50=0, tau50=0), None, C["sim"]),
                                    ("sim + measured actuator", simh.MEAS_SERVO, simh.JF_PLANT, C["sim2"])]:
            st = SSR.run(jn, kw, model, trace=True)
            ax[k].plot(np.r_[0, st["t"][:15]], np.r_[0, st["x"][:15]], color=col, lw=1.2, marker="o", ms=2.5, label=lab)
        ax[k].set_title(f"{jn} ({'STS3250' if 'hip_roll' in jn else 'STS3215'}): +30-tick step", fontsize=9)
        ax[k].set_xlabel("ms after the goal write"); ax[k].set_ylabel("fraction of step"); ax[k].set_xlim(0, 300)
    ax[0].legend(loc="lower right", fontsize=7)
    real = json.load(open(os.path.join(HERE, "sweep_real.json")))
    f = [x["f"] for x in real]
    ax[2].plot(f, [x["body_roll"]["per_goal"] for x in real], color=C["real"], marker="o", lw=1.6, label="robot")
    for lab, sv, model, col in [("sim as trained", simh.Servo(), None, C["sim"]),
                                ("sim + measured actuator", simh.TypedServo(**simh.MEAS_SERVO), simh.JF_PLANT, C["sim2"])]:
        rs, t0 = SS.run("fit", sv, model)
        resp = SS.response(rs, t0)
        ax[2].plot(f, [x["body_roll"]["per_goal"] for x in resp], color=col, marker="o", lw=1.2, label=lab)
    ax[2].set_xlabel("roll-sweep frequency, Hz"); ax[2].set_ylabel("body roll / hip-roll goal")
    ax[2].set_title("double-support roll sweep (+-1 deg on 4 roll joints)", fontsize=9); ax[2].legend(fontsize=7)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "actuator.png"), dpi=110); plt.close(fig)


def fig_margin():
    rows = []
    d1 = json.load(open(os.path.join(HERE, "margin.json")))["rows"]
    d2 = json.load(open(os.path.join(HERE, "measured_margin.json")))["rows"]
    d4 = json.load(open(os.path.join(HERE, "jf_margin.json")))["rows"]
    d3 = json.load(open(os.path.join(HERE, "fw_mitigation.json")))["rows"] + json.load(open(os.path.join(HERE, "fw_mitigation2.json")))["rows"]
    def fr(rws, key, plant="fit"):
        e = [r for r in rws if r["variant"] == key and r["plant"] == plant]
        return e[0]["falls"] / e[0]["n"] if e else np.nan
    gains = [1.0, 1.25, 1.5, 2.0, 3.0]
    series = {
        "fitted plant as trained": [0.0, np.nan, fr(d1, "loop gain x1.5"), fr(d1, "loop gain x2"), fr(d1, "loop gain x3")],
        "+ measured actuator": [1 / 8, fr(d2, "measured servo, loop gain x1.25"), fr(d2, "measured servo, loop gain x1.5"), fr(d2, "measured servo, loop gain x2"), np.nan],
        "+ joint-fit actuator (faster bracket)": [0.0, np.nan, fr(d4, "fitted actuator, loop gain x1.5"), fr(d4, "fitted actuator, loop gain x2"), np.nan],
        "+ measured actuator, gyro obs x0.5, shaper 6 Hz": [fr(d3, "gyro x0.5 + shaper 6 :: measured"), fr(d3, "gyro x0.5 + shaper 6 :: measured, loop gain x1.25"),
                                                           fr(d3, "gyro x0.5 + shaper 6 :: measured, loop gain x1.5"), fr(d3, "gyro x0.5 + shaper 6 :: measured, loop gain x2"), np.nan],
    }
    fig, ax = plt.subplots(figsize=(6.5, 3.6))
    for (lab, ys), col in zip(series.items(), [C["sim"], C["real"], C["py"], C["sim2"]]):
        ys = np.array(ys, float); m = np.isfinite(ys)
        ax.plot(np.array(gains)[m], ys[m] * 100, marker="o", color=col, lw=1.6, label=lab)
    ax.set_xscale("log"); ax.set_xticks(gains); ax.set_xticklabels([f"x{g:g}" for g in gains])
    ax.set_xlabel("extra loop gain (policy action deviation amplified on the way to the servos)")
    ax.set_ylabel("falls, % of 8 seeds"); ax.legend(fontsize=7, loc="lower right")
    ax.set_title("Stability margin of walk14_s1_s128r24 at the stand (fitted plant)", fontsize=9)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "margin.png"), dpi=110); plt.close(fig)


if __name__ == "__main__":
    which = sys.argv[1:] or ["fidelity", "margin", "actuator", "timeline"]
    out = {}
    for w in which:
        res = globals()["fig_" + w]()
        if res:
            out[w] = res
    print(json.dumps(out, default=float))
