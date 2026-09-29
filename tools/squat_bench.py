"""Open-loop reference squat on the tethered robot: the level-foot family
(hip -theta, knee -2*theta, ankle -theta) streamed as firmware `pose s<ms>`
minimum-jerk moves, N reps, both cameras recording, every hold read back.

    tools/squat_bench.py hw_sessions/2026-09-13/openloop --thetas 39,39,39
    tools/squat_bench.py hw_sessions/2026-09-13/openloop_dry --thetas 15

Per theta: pose down (s<ms>), settle, read every joint + tilt, pose back to
zero, settle, read again. Zeros/dirs come from the robot's own `cal show`
(NVS), asbuilt_cal.h only as the fallback. Guard: torso tilt beyond
--tilt-abort or a joint stalled far from target -> `release` (torque off;
gear friction holds the pose, Tom 2026-09-05) and exit 2. Ends with
`release` (idle = torque released). Opening the port reboots the board to
BENCH (harmless; the WiFi link re-homes it later with RESET SERVOS).
"""
import argparse, csv, math, os, re, subprocess, sys, time
import serial

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument("out", help="output base (no extension)")
ap.add_argument("--thetas", default="39,39,39", help="level-foot squat depth per rep, deg")
ap.add_argument("--poses", default=None,
                help="explicit reps 'hip,knee,ankle;hip,knee,ankle' (deg, negative = flex), "
                     "overrides --thetas; the same triple goes to both legs")
ap.add_argument("--rolls", default=None,
                help="hip-roll reps, ONE leg at a time: 'L5,R5,L10,R10' = abduct (foot outward) that hip by N deg "
                     "at the standing pose, hold, back to zero; overrides --poses/--thetas")
ap.add_argument("--balance", default=None,
                help="one-foot balance, open loop: 'L:12:25' = stance LEFT, lean 12 deg over it (both hips roll "
                     "the same way; no ankle roll, so the whole body leans), lift the RIGHT foot with the level-foot "
                     "family theta=25 (knee -50). Steps: lean, lean+lift, lean, zero. Overrides the other modes")
ap.add_argument("--step", default=None,
                help="one open-loop step: 'R:20:10:200' = swing RIGHT leg, lift theta 20 (knee -40), swing psi +10 deg "
                     "forward (negative = backward), each move 200 ms: swing pose, landing pose, hold, then slow zero. "
                     "The swing KNEE is traced at 50 Hz (firmware T<id>) so its load shows the foot unloading")
ap.add_argument("--rock", default=None,
                help="lateral rock only: 'phi:P:cycles' = both hip rolls alternate -phi/+phi as half-period smooth "
                     "moves (P seconds per cycle), then zero. For measuring whether the real torso sways")
ap.add_argument("--balance-mode", default="stance", choices=("stance", "both"),
                help="stance = roll the stance hip only; both = both hips the same way (parallelogram lean)")
ap.add_argument("--ms", type=int, default=2000, help="minimum-jerk duration per move")
ap.add_argument("--settle", type=float, default=2.0)
ap.add_argument("--tilt-abort", type=float, default=12.0)
ap.add_argument("--cams", default="/dev/video0,/dev/video2")
ap.add_argument("--no-cam", action="store_true")
A = ap.parse_args()

NAMES = ["L_hip_yaw", "L_hip_roll", "L_hip_pitch", "L_knee", "L_ankle",
         "R_hip_yaw", "R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle"]
IDS = [10, 5, 6, 7, 8, 9, 1, 2, 3, 4]
ZERO = [1692, 2418, 2001, 1581, 3535, 1806, 3532, 2479, 2063, 3437]   # asbuilt_cal.h
DIR = [+1, -1, +1, -1, +1, +1, -1, +1, -1, +1]
TPD = 4096 / 360.0
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

log = open(A.out + ".log", "w")
def say(*a):
    s = " ".join(str(x) for x in a); print(s, flush=True); log.write(s + "\n"); log.flush()

def openport():
    for _ in range(60):
        try:
            return serial.Serial("/dev/ttyUSB0", 115200, timeout=0.15)
        except Exception:
            time.sleep(0.5)
    raise SystemExit("no /dev/ttyUSB0 -- tether not connected?")

s = openport(); time.sleep(2.5); s.read(65536); s.write(b"\r\n"); time.sleep(0.4); s.read(65536)

def cmd(c, until, timeout=3.0):
    s.read(65536); s.write(c.encode() + b"\r\n"); t0 = time.time(); o = ""
    while time.time() - t0 < timeout:
        o += s.read(4096).decode(errors="replace")
        if re.search(until, o): break
    log.write(f">>> {c}\n{o}\n"); log.flush()
    return o

def release():
    s.write(b"release\r\n"); s.flush(); time.sleep(0.5)
    say("!! RELEASED (torque off)")

# --- calibration from the robot itself -------------------------------------
o = cmd("cal show", until=r"(NVS|DEFAULTS)")
got = {m.group(1): (int(m.group(2)), int(m.group(3)), int(m.group(4)))
       for m in re.finditer(r"(\w+)\s+id\s+(\d+)\s+zero\s+(\d+)\s+dir\s+([+-]\d)", o)}
if all(n in got for n in NAMES):     # `cal show` lists all 17 bus servos
    for j, n in enumerate(NAMES):
        i, z, d = got[n]
        if (i, z, d) != (IDS[j], ZERO[j], DIR[j]):
            say(f"cal: {n} robot says id {i} zero {z} dir {d:+d} (header {IDS[j]} {ZERO[j]} {DIR[j]:+d}) -- using the robot's")
        IDS[j], ZERO[j], DIR[j] = i, z, d
    say("cal source:", "NVS" if "NVS" in o else "DEFAULTS (not saved!)")
else:
    say(f"cal show named {sum(n in got for n in NAMES)}/{len(NAMES)} leg "
        "joints -- using asbuilt_cal.h values")

def triple(theta):
    return (-theta, -2 * theta, -theta)          # the level-foot family

def targets(tr):
    if isinstance(tr, dict):
        return dict(tr)
    h, k, a = tr[:3]
    rl = tr[3] if len(tr) > 3 else 0.0      # abduction, deg: L +, R - in the sim/model frame (leg_rom.py)
    rr = tr[4] if len(tr) > 4 else 0.0
    return {"L_hip_pitch": h, "L_knee": k, "L_ankle": a, "R_hip_pitch": h, "R_knee": k, "R_ankle": a,
            "L_hip_roll": +rl, "R_hip_roll": -rr}

def ticks(tr):
    off = targets(tr)
    return [int(round(ZERO[j] + DIR[j] * off.get(n, 0.0) * TPD)) for j, n in enumerate(NAMES)]

def readback():
    ang, load = [], []
    for j in range(10):
        o = cmd(f"pos {IDS[j]}", until=r"load\s+-?\d+.*err", timeout=1.5)
        m = re.search(r"pos\s+(-?\d+)\s+spd\s+(-?\d+)\s+load\s+(-?\d+)", o)
        if m:
            ang.append((int(m.group(1)) - ZERO[j]) * DIR[j] / TPD); load.append(int(m.group(3)))
        else:
            ang.append(float("nan")); load.append(0)
    return ang, load

def imu_up():
    o = cmd("imu raw 1", until=r"\|a\|.*g .*\n", timeout=2.0)
    m = re.search(r"a\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)", o)
    return tuple(float(x) for x in m.groups()) if m else None

def tilt(a, b):
    if not a or not b: return float("nan")
    na = math.sqrt(sum(x * x for x in a)); nb = math.sqrt(sum(x * x for x in b))
    return math.degrees(math.acos(max(-1, min(1, sum(x * y for x, y in zip(a, b)) / (na * nb)))))

def label(tr):
    if isinstance(tr, dict):
        return " ".join(f"{k.replace('_hip', '')}={v:g}" for k, v in tr.items() if v)
    return f"hip/knee/ankle {tr[0]:g}/{tr[1]:g}/{tr[2]:g}" + (f" abdL/abdR {tr[3]:g}/{tr[4]:g}" if len(tr) > 4 else "")

def pose(tr, ms=None, trace_id=0):
    tk = ticks(tr)
    tail = f" s{ms or A.ms}" + (f" T{trace_id}" if trace_id else "")
    o = cmd("pose " + " ".join(map(str, tk)) + tail, until=r"(streaming|ok|refus|OFF|usage|busy|answer)", timeout=2.0)
    ok = bool(re.search(r"streaming|ok", o))
    say(f"pose {label(tr) or 'zero'}: {'streaming' if ok else 'REFUSED: ' + o.strip()[:100]}"
        + ("  (CLAMPED by the bench envelope)" if "clamp" in o else ""))
    return ok

sw = None
if A.rock:
    phi, P, cyc = A.rock.split(":"); phi = float(phi); P = float(P); cyc = int(cyc)
    half = int(round(500 * P))
    reps = []
    for c in range(cyc):
        reps.append((f"rock {c+1}-", {"L_hip_roll": -phi, "R_hip_roll": -phi}, half))
        reps.append((f"rock {c+1}+", {"L_hip_roll": +phi, "R_hip_roll": +phi}, half))
elif A.step:
    sw, th, psi, sms = A.step.split(":"); th = float(th); psi = float(psi); sms = int(sms); sw = sw.upper()
    swing = {f"{sw}_hip_pitch": -(th + psi), f"{sw}_knee": -2 * th, f"{sw}_ankle": -(th - psi)}   # sole level
    land = {f"{sw}_hip_pitch": -(5 + psi), f"{sw}_knee": -10, f"{sw}_ankle": -(5 - psi)}
    reps = [("swing", swing, sms), ("land", land, sms)]
elif A.balance:
    side, phi, th = A.balance.split(":"); phi = float(phi); th = float(th); side = side.upper()
    # lean toward the stance foot: rolls L and R both -phi leans LEFT (L adducts, R abducts), both +phi leans RIGHT
    # 2026-09-13 measured: rolling BOTH hips just slides the feet on the pad (torso stayed put on camera).
    # Roll the STANCE hip only: the pelvis rotates about it, the torso tilts over the stance foot (which
    # stays flat), the other hip rises and that foot unloads. Adduction at the stance hip: L -phi / R +phi.
    if A.balance_mode == "both":
        lean = {"L_hip_roll": -phi if side == "L" else +phi, "R_hip_roll": -phi if side == "L" else +phi}
    else:
        lean = {"L_hip_roll": -phi} if side == "L" else {"R_hip_roll": +phi}
    sw = "R" if side == "L" else "L"
    lift = dict(lean); lift.update({f"{sw}_hip_pitch": -th, f"{sw}_knee": -2 * th, f"{sw}_ankle": -th})
    # a slow lean first is pointless (the planted swing foot pins the pelvis): ONE move that shortens the swing leg
    # while the stance hip rolls, so the pelvis rolls about the stance hip as the constraint goes; hold; back down
    reps = ([("lean", lean), ("lean+lift", lift), ("lean", lean)] if A.balance_mode == "both"
            else [("lean+lift", lift), ("lean", lean)])
elif A.rolls:
    reps = []
    for tok in A.rolls.split(","):
        leg, deg = tok.strip()[0].upper(), float(tok.strip()[1:])
        reps.append((0.0, 0.0, 0.0, deg if leg == "L" else 0.0, deg if leg == "R" else 0.0))
elif A.poses:
    reps = [tuple(float(x) for x in r.split(",")) for r in A.poses.split(";")]
else:
    reps = [triple(float(x)) for x in A.thetas.split(",")]
total_s = 3 + len(reps) * 2 * (A.ms / 1000 + A.settle + 2.5) + 4
cams = []
if not A.no_cam:
    for k, dev in enumerate(A.cams.split(",")):
        base = f"{A.out}_cam{dev[-1]}"
        cams.append(subprocess.Popen([os.path.join(ROOT, "tools", "cam_record.sh"), dev, base,
                                      str(int(total_s)), f"cam{dev[-1]}"],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
    time.sleep(1.5)

o = cmd("bench", until=r"\n", timeout=2.0); say("bench:", o.strip()[-80:])   # the armed loop owns the bus until benched
time.sleep(1.0)
o = cmd("torque", until=r"\n", timeout=2.0); say("torque:", o.strip()[-60:])
if "busy" in o:
    say("!! bus still busy after bench -- not sending poses"); sys.exit(2)
ZERO5 = (0.0, 0.0, 0.0, 0.0, 0.0)
if not pose(ZERO5):
    release(); sys.exit(2)
time.sleep(A.ms / 1000 + A.settle)
base = imu_up(); a0, l0 = readback()
say("zero pose: " + " ".join(f"{n} {a:+.1f}" for n, a in zip(NAMES, a0)) + f"  | up {base}")

w = csv.writer(open(A.out + ".csv", "w", newline=""))
w.writerow(["rep", "phase", "hip_cmd", "knee_cmd", "ankle_cmd", "t", "tilt_deg"] + [f"{n}_deg" for n in NAMES] + [f"{n}_load" for n in NAMES])
t_start = time.time(); rc = 0
try:
    if A.rock:
        seq = [(n, d, ms) for n, d, ms in reps] + [("zero", ZERO5, None)]
    elif A.step:
        seq = [(n, d, ms) for n, d, ms in reps] + [("zero", ZERO5, None)]
    elif A.balance:
        seq = [(n, d) for n, d in reps] + [("zero", ZERO5)]
    else:
        seq = []
        for rep, tr in enumerate(reps, 1):
            seq += [(f"rep {rep} down", tr), (f"rep {rep} up", ZERO5)]
    KNEE_ID = {"L": 7, "R": 3}
    for rep, item in enumerate(seq, 1):
        if True:
            phase, th = item[0], item[1]; mv = item[2] if len(item) > 2 and item[2] else None
            tid = KNEE_ID[sw] if (A.step and phase == "swing" and sw) else 0
            if not pose(th, ms=mv, trace_id=tid):
                rc = 2; break
            if A.rock and phase.startswith("rock"):
                time.sleep((mv or A.ms) / 1000 + 0.02); continue
            time.sleep((mv or A.ms) / 1000 + (0.3 if (A.step and phase == "swing") else A.settle))
            if tid:
                time.sleep(1.4)
                to = cmd("trace", until=r"trace end", timeout=6.0)
                rows = [tuple(int(x) for x in mm.groups()) for mm in re.finditer(r"T (\d+) (-?\d+) (-?\d+) (-?\d+)", to)]
                with open(A.out + f"_trace_{phase}.csv", "w") as f:
                    f.write("t_ms,pos,spd,load\n" + "".join(f"{a},{b},{c},{d}\n" for a, b, c, d in rows))
                if rows:
                    loads = [abs(r[3]) for r in rows]
                    say(f"  knee trace ({len(rows)} samples): load min {min(loads)} max {max(loads)} "
                        f"first {loads[0]} last {loads[-1]}  | samples with |load| < 16: {sum(1 for l in loads if l < 16)}")
            up = imu_up(); tl = tilt(base, up); ang, load = readback()
            tg = targets(th)
            w.writerow([rep, phase, tg.get("L_hip_pitch", 0), tg.get("L_knee", 0), tg.get("L_ankle", 0), round(time.time() - t_start, 2), round(tl, 2)]
                       + [round(a, 2) for a in ang] + load)
            err = [abs(a - tg.get(n, 0.0)) for n, a in zip(NAMES, ang)]
            say(f"{phase:12s} cmd [{label(th) or 'zero'}]: tilt {tl:5.1f} deg | "
                f"L roll/hip/knee/ankle {ang[1]:+6.1f}/{ang[2]:+6.1f}/{ang[3]:+6.1f}/{ang[4]:+6.1f}  "
                f"R {ang[6]:+6.1f}/{ang[7]:+6.1f}/{ang[8]:+6.1f}/{ang[9]:+6.1f} | yaw L/R {ang[0]:+5.1f}/{ang[5]:+5.1f} | max err {max(err):.1f} | "
                f"load max {max(abs(x) for x in load)}")
            if tl > A.tilt_abort:
                say(f"!! tilt {tl:.1f} > {A.tilt_abort} -- abort"); rc = 2; break
            if max(err) > 12.0:
                say(f"!! joint error {max(err):.1f} deg -- stalled? abort"); rc = 2; break
finally:
    if rc:
        release()
    else:
        time.sleep(0.5); release()
    for p in cams:
        try: p.wait(timeout=total_s)
        except Exception: p.kill()
    s.close()
say("done rc", rc, "csv", A.out + ".csv")
sys.exit(rc)
