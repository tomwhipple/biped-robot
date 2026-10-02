"""Sweep roughness at P 128 vs motion smoothness and damping (issue #73),
2026-10-02, v3 rig with the washer weights.

Tom: "test the sweep roughness at P128 with smoother motions and/or
dampening". Session 3 measured 90-degree sweeps at 100 steps/s (one `move`,
ACC 2) as rough from P 96 up (P 128: reported speed -950..1200, ~25 % of the
samples moving backward). Here each gain setting sweeps LEVEL <-> DOWN three
ways:
  A  one `move` at 100 steps/s, ACC 2 (200 steps/s^2)   -- session 3's sweep
  B  one `move` at 100 steps/s, ACC 1 (100 steps/s^2)   -- gentler ramps
  C  host-streamed minimum-jerk targets over the same 10.24 s (avg 100
     steps/s), one `move` per cycle at the trajectory's own speed, the way a
     policy streams targets
at P 32 / D 32 (reference), P 128 with D 32, 64 and 128 (damping).

Metrics per sweep (servo telemetry through the sweep): backward samples
(reported speed against the sweep), jitter = RMS of pos minus its 0.4 s
moving average over the moving part, load range, and for C the RMS tracking
error to the commanded trajectory.

Rules (Tom): the weights stay on; gains are written and torque released only
at DOWN (plumb, no gravity torque); an abort off DOWN holds torque. Run from
the repo root in tmux, with the camera recording:
  LEVEL_T=2507 DOWN_T=3531 .venv/bin/python experiments/plan-b-bench/2026-10-02/sweep_smooth.py
"""
import json, math, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import gain_bench as GB

SID = 30
LEVEL = int(os.environ.get("LEVEL_T", 2507))
DOWN = int(os.environ.get("DOWN_T", 3531))
DOWN_TOL, SLOW, T_SWEEP = 60, 100, 10.24
GAINS = [tuple(int(v) for v in x.split(":")) for x in
         os.environ.get("GAINS", "32:32,128:32,128:64,128:128").split(",")]
PROFILES = os.environ.get("PROFILES", "A,B,C").split(",")
BNO = os.environ.get("BNO", "0") == "1"   # BNO055 on the fork, firmware `bno` (new controller)
OUT = os.path.join(os.getcwd(), "hw_sessions", time.strftime("%Y-%m-%d"), "sweep_smooth")
os.makedirs(OUT, exist_ok=True)
logf = open(os.path.join(OUT, "session.log"), "a")
state = {"torque": False}


def log(s):
    logf.write(s + "\n"); logf.flush()


def say(s):
    print(s, flush=True); log(s)


def go(what):
    say(f"\nNEXT MOTION: {what}\n  type go:")
    a = input("> ").strip(); log(f"< {a!r}")
    if a.lower() != "go":
        raise GB.Abort(f"no go for: {what}")


def move(b, to, spd, acc):
    out = b.cmd(f"move {SID} {int(round(to))} 0 {int(spd)} {int(acc)}", until=GB.MOVE_UNTIL)
    if ": ok" not in out or "clamp" in out:
        raise GB.Abort(f"move {to}: {out.strip()[-90:]!r}")


def at(b):
    return GB.read_pos(b, SID)


def bno_start(b, ms):
    out = b.cmd(f"bno rec {int(ms)}", until=r"bno rec: started.*\n|bno: .*\n|\? \(try", timeout=3)
    if "started" not in out:
        raise GB.Abort(f"bno rec failed: {out.strip()[-90:]!r}")
    return time.monotonic()


def bno_collect(b):
    t_end = time.monotonic() + 15
    while time.monotonic() < t_end:
        st = b.cmd("bno status", until=r"bno: .*\n", timeout=2)
        if "idle" in st:
            break
        time.sleep(0.2)
    out = b.cmd("bno dump", until=r"bno dump end\r?\n|bno: .*\n", timeout=30)
    samples = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 4 and all(x.lstrip("-").isdigit() for x in parts):
            samples.append([int(x) for x in parts])
    return samples


def bno_metrics(samples):
    """3-axis acceleration (mg): RMS above 5 Hz per axis, and the dominant
    frequency of that high-passed signal (FFT)."""
    import numpy as np
    if len(samples) < 64:
        return {"n": len(samples)}
    a = np.array(samples, float)
    t = a[:, 0] * 1e-6
    fs = (len(t) - 1) / (t[-1] - t[0])
    out = {"n": len(samples), "fs_hz": round(float(fs), 1)}
    for i, ax in enumerate("xyz"):
        x = a[:, i + 1] - a[:, i + 1].mean()
        f = np.fft.rfftfreq(len(x), 1 / fs)
        hp = f >= 5.0
        xh = np.fft.irfft(np.where(hp, np.fft.rfft(x), 0), len(x))      # unwindowed: RMS
        out[f"rms_{ax}_mg"] = round(float(np.sqrt(np.mean(xh ** 2))), 2)
        W = np.abs(np.fft.rfft(x * np.hanning(len(x)))) * hp              # windowed: peak
        out[f"peak_{ax}_hz"] = round(float(f[int(np.argmax(W))]), 1)
    return out


def minjerk(t, T):
    x = min(max(t / T, 0.0), 1.0)
    return 10 * x ** 3 - 15 * x ** 4 + 6 * x ** 5, (30 * x ** 2 - 60 * x ** 3 + 30 * x ** 4) / T


def sweep(b, frm, to, prof):
    """One sweep frm -> to; returns telemetry samples (t, pos, spd, load, err, tgt)."""
    tr, t0 = [], time.monotonic()
    if prof in ("A", "B"):
        move(b, to, SLOW, 2 if prof == "A" else 1)
        while time.monotonic() - t0 < abs(to - frm) / SLOW + 1.0 + 2.0:
            r = at(b); r["t"] = time.monotonic() - t0; r["tgt"] = None; tr.append(r)
    else:
        d = to - frm
        while True:
            t = time.monotonic() - t0
            if t > T_SWEEP + 2.0:
                break
            s, sd = minjerk(t, T_SWEEP)
            tgt = frm + d * s
            spd = max(30, min(1000, abs(d * sd) * 1.5))
            if t <= T_SWEEP:
                move(b, tgt, spd, 0)
            r = at(b); r["t"] = time.monotonic() - t0; r["tgt"] = tgt if t <= T_SWEEP else to
            tr.append(r)
    return tr


def metrics(tr, frm, to):
    dirn = 1 if to > frm else -1
    mov = [s for s in tr if abs(s["pos"] - frm) > 15 and abs(s["pos"] - to) > 15]
    back = sum(1 for s in mov if s["spd"] * dirn < 0)
    jit = float("nan")
    if len(mov) > 10:
        ts = [s["t"] for s in mov]; ps = [s["pos"] for s in mov]
        res = []
        for i, t in enumerate(ts):
            w = [ps[k] for k in range(len(ts)) if abs(ts[k] - t) <= 0.2]
            res.append(ps[i] - sum(w) / len(w))
        jit = math.sqrt(sum(r * r for r in res) / len(res))
    trk = [s["pos"] - s["tgt"] for s in tr if s.get("tgt") is not None and s["t"] <= T_SWEEP]
    tail = [s for s in tr if s["t"] >= tr[-1]["t"] - 1.0]
    rate = (len(tr) - 1) / (tr[-1]["t"] - tr[0]["t"]) if len(tr) > 1 else 0
    return {"n": len(tr), "hz": rate, "moving_n": len(mov), "backward": back,
            "jitter_rms": jit, "track_rms": math.sqrt(sum(e * e for e in trk) / len(trk)) if trk else None,
            "load_min": min(s["load"] for s in tr), "load_max": max(s["load"] for s in tr),
            "end_offset": sum(s["pos"] for s in tail) / len(tail) - to,
            "end_range": max(s["pos"] for s in tail) - min(s["pos"] for s in tail),
            "err": max(s["err"] for s in tr)}


def release_at_down(b):
    pos = at(b)["pos"]
    if abs(pos - DOWN) > DOWN_TOL:
        raise GB.Abort(f"not releasing at {pos}: weights on, not within {DOWN_TOL} of plumb {DOWN}")
    GB.release(b, SID); state["torque"] = False
    time.sleep(1.5)
    return at(b)["pos"]


def main():
    say(f"# {time.strftime('%Y-%m-%dT%H:%M:%S')} sweep_smooth.py id {SID} LEVEL {LEVEL} DOWN {DOWN} "
        f"gains {GAINS} profiles {PROFILES}")
    b = GB.Board(GB.find_port(None), log)
    res = {"level": LEVEL, "down": DOWN, "gains": GAINS, "profiles": PROFILES, "rows": []}
    try:
        p0 = at(b)["pos"]
        if GB.read_reg(b, SID, 40) != 0 or abs(p0 - DOWN) > DOWN_TOL:
            raise GB.Abort(f"start refused: pos {p0}; need torque off, hanging within {DOWN_TOL} of {DOWN}")
        if BNO:
            out = b.cmd("bno", until=r"bno: .*\n|\? \(try", timeout=3)
            if "0xA0" not in out or "ACCONLY" not in out:
                raise GB.Abort(f"BNO055 not ready: {out.strip()[-120:]!r}")
            say(f"  {out.strip()}")
        go(f"the whole ladder {GAINS}: per gain, write at plumb, torque on, then per profile "
           f"{PROFILES} raise to level and lower to plumb (90 deg, ~10 s each way), release at plumb")
        for p, d in GAINS:
            say(f"\n== P {p} D {d}")
            if GB.read_gains(b, SID)[:2] != (p, d):
                say(f"  gains {p}/{d}: {GB.write_gains(b, SID, p, d)}")   # at plumb, released
            before = at(b)["pos"]
            b.cmd(f"torque {SID}", until=r"torque \S+: \S+\r?\n"); state["torque"] = True
            if abs(at(b)["pos"] - before) > GB.JUMP_TICKS:
                raise GB.Abort("jump at torque-on")
            move(b, DOWN, SLOW, 2); time.sleep(2.0)
            for prof in PROFILES:
                for frm, to, way in ((DOWN, LEVEL, "raise"), (LEVEL, DOWN, "lower")):
                    if BNO:
                        bno_t = bno_start(b, (T_SWEEP + 4.0) * 1000 if prof == "C"
                                          else (abs(to - frm) / SLOW + 5.0) * 1000)
                    tr = sweep(b, frm, to, prof)
                    m = metrics(tr, frm, to)
                    row = {"p": p, "d": d, "profile": prof, "way": way, "metrics": m, "trace": tr}
                    if BNO:
                        acc = bno_collect(b)
                        row.update({"bno": acc, "bno_host_t0": bno_t, "bno_metrics": bno_metrics(acc)})
                        bm = row["bno_metrics"]
                        say(f"    accel >5 Hz rms x/y/z {bm.get('rms_x_mg')}/{bm.get('rms_y_mg')}/{bm.get('rms_z_mg')} mg, "
                            f"peak {bm.get('peak_x_hz')}/{bm.get('peak_y_hz')}/{bm.get('peak_z_hz')} Hz, "
                            f"{bm.get('n')} samples @ {bm.get('fs_hz')} Hz")
                    res["rows"].append(row)
                    trk = "-" if m["track_rms"] is None else f"{m['track_rms']:.1f}"
                    say(f"  {prof} {way:5s}: backward {m['backward']}/{m['moving_n']}, jitter {m['jitter_rms']:.2f} "
                        f"ticks rms, track {trk}, load {m['load_min']}..{m['load_max']}, end "
                        f"{m['end_offset']:+.1f} (range {m['end_range']}), {m['hz']:.0f} Hz, err {m['err']}")
                    time.sleep(1.5)
            say(f"  released at plumb: pos {release_at_down(b)}")
        if GB.read_gains(b, SID)[:2] != (32, 32):
            GB.write_gains(b, SID, 32, 32)
        say(f"  gains back to {GB.read_gains(b, SID)[:2]}")
    except (GB.Abort, KeyboardInterrupt, Exception) as e:
        say(f"!! STOPPED: {e!r}")
        res["stopped"] = repr(e)
        try:
            if state["torque"]:
                pos = at(b)["pos"]
                if abs(pos - DOWN) <= DOWN_TOL:
                    say(f"  released at plumb: {release_at_down(b)}")
                else:
                    say(f"!! HOLDING TORQUE at {pos} (weights on, off plumb). To finish: "
                        f"`move {SID} {DOWN} 0 {SLOW} 2`, wait, then `release {SID}`.")
                    input("press enter to leave it HELD and close the port > ")
        except Exception as e2:
            say(f"!! could not check/release: {e2!r} -- torque state unknown")
    finally:
        with open(os.path.join(OUT, "sweep_smooth.json"), "w") as f:
            json.dump(res, f, indent=1)
        say(f"# end {time.strftime('%Y-%m-%dT%H:%M:%S')}")
        b.close()


if __name__ == "__main__":
    main()
