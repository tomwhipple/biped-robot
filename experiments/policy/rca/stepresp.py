"""Parse the robot's `stepresp` logs (single max-speed goal write, single-servo feedback every
~3 ms) and fit, per servo, dead time + first-order rise + final fraction, out and back."""
import os, re, sys, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
NAMES = {52: "R_hip_roll", 51: "L_hip_roll", 6: "R_hip_pitch", 2: "L_hip_pitch", 7: "R_knee", 3: "L_knee",
         8: "R_ankle", 4: "L_ankle", 10: "R_hip_yaw", 9: "L_hip_yaw", 11: "R_ankle_roll", 12: "L_ankle_roll"}


def parse(path):
    blocks, cur = [], None
    for line in open(path):
        m = re.search(r"stepresp id (\d+): start (\d+), step ([+-]?\d+) ticks", line)
        if m:
            cur = dict(id=int(m.group(1)), start=int(m.group(2)), step=int(m.group(3)), s={0: [], 1: []})
            blocks.append(cur); continue
        m = re.search(r"SR (\d) (\d+) (-?\d+) (-?\d+) (-?\d+)", line)
        if m and cur is not None:
            ph, t, p, v, ld = map(int, m.groups())
            cur["s"][ph].append((t / 1000.0, p, v, ld))
    # de-duplicate (the log can repeat a block)
    seen, out = set(), []
    for b in blocks:
        k = (b["id"], b["start"], b["step"], len(b["s"][0]))
        if k in seen or not b["s"][0]:
            continue
        seen.add(k); out.append(b)
    return out


def fit(samples, step, back=False):
    a = np.array(samples, float)
    t, p = a[:, 0], a[:, 1]
    # positions are logged relative to the block's start ticks; 'back' runs from wherever
    # 'out' ended toward 0, normalised by the commanded step
    x = (p[0] - p) / step if back else p / step
    moved = np.where(np.abs(x) * abs(step) >= 2)[0]
    dead = float(t[moved[0]]) if len(moved) else float("nan")
    fin = float(np.mean(x[-5:]))
    def tcross(f):
        k = np.where(x >= f * fin)[0]
        return float(t[k[0]]) if len(k) else float("nan")
    t10, t63, t90 = tcross(0.1), tcross(0.632), tcross(0.9)
    tau = t63 - dead if np.isfinite(t63) and np.isfinite(dead) else float("nan")
    return dict(dead_ms=dead, t10_ms=t10, t90_ms=t90, tau_ms=tau, final=fin, peak=float(x.max()),
                load_peak=float(np.max(np.abs(a[:, 3]))))


def main(path):
    res = []
    for b in parse(path):
        for ph in (0, 1):
            if not b["s"][ph]:
                continue
            r = fit(b["s"][ph], b["step"], back=(ph == 1))
            r.update(id=b["id"], joint=NAMES.get(b["id"], str(b["id"])), phase="out" if ph == 0 else "back", step=b["step"])
            res.append(r)
    for r in res:
        print(f"{r['joint']:13s} {r['phase']:4s} step {r['step']:+d}: dead {r['dead_ms']:5.1f} ms  t10 {r['t10_ms']:5.1f}  "
              f"t90 {r['t90_ms']:5.1f}  tau63 {r['tau_ms']:5.1f} ms  final {r['final']:.2f}  peak {r['peak']:.2f}  |load| {r['load_peak']:.0f}")
    return res


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "data", "2026-11-01_stepresp_loaded.txt")
    res = main(path)
    json.dump(res, open(os.path.join(HERE, os.path.basename(path).replace(".txt", ".json")), "w"), indent=1)
