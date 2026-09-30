"""Hip-YAW play on the standing robot, read through the pelvis gyro.

Yaw does not tilt the torso and the encoders sit on the shaft side of the
slop, so the sweep tool cannot see it. Instead: with both feet planted, yaw
BOTH hips the same way and the pelvis must yaw with them (rigid sim: 0.78 deg
per deg); any shortfall + hysteresis is slop (Tom 2026-09-03: "the entire
leg is controlled by the tiny servo axis"). Pelvis yaw = the gyro projected
on the measured gravity axis, bias-corrected from a stationary burst just
before each 1 deg step and integrated over a burst covering the slow move.

    .venv/bin/python tools/yaw_probe.py hw_sessions/<day>/yaw1.csv [--modes both,left,right]

Calibration from the board's `cal show` (blob v3); asbuilt fallback.
Pose strings are 17-wide in servo-ID order.
"""
import serial, time, re, csv, math, sys, argparse, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bus_cal import BusCal, BusCalError, asbuilt_prototype_cal, fetch_cal
ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument('out'); ap.add_argument('--modes', default='both,left,right,opposite')
ap.add_argument('--max', type=float, default=4.0); ap.add_argument('--step', type=float, default=2.0)
ap.add_argument('--spd', type=int, default=15); ap.add_argument('--tilt-abort', type=float, default=6.0)
A = ap.parse_args()
TPD = 4096/360.0
N = int(round(A.max/A.step))
STEPS = [k*A.step for k in range(1, N+1)] + [k*A.step for k in range(N-1, -N-1, -1)] + [k*A.step for k in range(-N+1, 1)]
log = open(A.out.replace('.csv','.log'),'w')
def openport():
    for _ in range(60):
        try: return serial.Serial('/dev/ttyUSB0', 115200, timeout=0.15)
        except Exception: time.sleep(0.5)
    raise SystemExit('serial port never came back')
s = openport(); time.sleep(1.5); s.read(65536); s.write(b'\r\n'); time.sleep(0.4); s.read(65536)
def cmd(c, until, timeout=3.0, tries=4):
    """send a CLI line, wait for the reply pattern, RESEND if it never comes
    (the board drops ~half of all CLI lines). Returns (text, seconds)."""
    global s
    for attempt in range(tries):
        try: s.read(65536)
        except serial.SerialException:
            log.write('!! serial dropped\n'); s.close(); time.sleep(2); s = openport(); time.sleep(2); s.read(65536)
        s.write(c.encode()+b'\r\n'); t0=time.time(); o=''
        while time.time()-t0 < timeout:
            o += s.read(4096).decode(errors='replace')
            if re.search(until, o): break
        dt = time.time()-t0
        log.write(f'>>> {c} [try {attempt+1}, {dt:.2f}s]\n{o}'); log.flush()
        if re.search(until, o): time.sleep(0.15); return o, dt
    return o, 0.0
def burst(n):
    """n raw IMU samples (the board streams ~10/s) -> (accel rows, gyro rows, seconds)."""
    global s
    pat = re.compile(r'a\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+\|a\|\s+[\d.]+\s+g\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)')
    for attempt in range(3):
        try: s.read(65536)
        except serial.SerialException:
            log.write('!! serial dropped\n'); s.close(); time.sleep(2); s = openport(); time.sleep(2); s.read(65536)
        s.write(f'imu raw {n}\r\n'.encode()); t0=time.time(); o=''; t_first=None
        while time.time()-t0 < n/7.0 + 2.5:
            chunk = s.read(4096).decode(errors='replace'); o += chunk
            if chunk and t_first is None and 'a ' in o: t_first = time.time()
            if len(pat.findall(o)) >= n: break
        rows = pat.findall(o); dt = (time.time() - (t_first or t0))
        log.write(f'>>> imu raw {n} [try {attempt+1}, {len(rows)} rows, {dt:.2f}s]\n{o}'); log.flush()
        if len(rows) >= max(3, n//2): break
        time.sleep(0.3)
    acc = [tuple(float(x) for x in r[:3]) for r in rows]; gyr = [tuple(float(x) for x in r[3:]) for r in rows]
    return acc, gyr, dt
def unit(v):
    n = math.sqrt(sum(x*x for x in v)); return tuple(x/n for x in v)
def mean(rows): return tuple(sum(r[i] for r in rows)/len(rows) for i in range(3))
def dot(a,b): return sum(x*y for x,y in zip(a,b))
def pose_ticks(yawL, yawR):
    """17-wide bus-order pose: yaw L by yawL, R by yawR, hold others at zero."""
    return CAL.pose_ticks({'L_hip_yaw': yawL, 'R_hip_yaw': yawR})
def servo_pos(i):
    o,_ = cmd(f'pos {i}', until=r'load\s+-?\d+.*err'); m = re.search(r'pos\s+(-?\d+)', o)
    return int(m.group(1)) if m else None
def pose(yawL, yawR):
    o,_ = cmd('pose ' + ' '.join(map(str, pose_ticks(yawL, yawR))) + f' {A.spd}', until=r'(ok|refus|OFF|usage|busy)')
    return o

# Calibration from the board (`cal show`, blob v3, 17-servo bus map).
# cmd() returns (reply, seconds); bus_cal's callback wants just the reply.
CAL, CAL_SOURCE = fetch_cal(lambda c, until: cmd(c, until)[0])
log.write(f'# cal source: {CAL_SOURCE}\n'); log.flush()
print(f'cal source: {CAL_SOURCE}', flush=True)
if not (CAL.by_name('L_hip_yaw').fitted and CAL.by_name('R_hip_yaw').fitted):
    print('!! a hip yaw is NOT FITTED on this robot -- refusing', flush=True); s.close(); sys.exit(2)
# torque pre-check: goals at zero must be accepted (never auto-enable torque from here)
o = pose(0.0, 0.0)
if 'ok' not in o: print('!! pose refused:', o.strip()[:120]); sys.exit(2)
# sample-rate + gravity axis from a long stationary burst
acc, gyr, dt = burst(30)
if len(gyr) < 15: print('!! short burst', len(gyr)); sys.exit(2)
rate = len(gyr)/dt; up = unit(mean(acc)); base_up = up
print(f'imu burst: {len(gyr)} samples in {dt:.2f}s -> {rate:.0f} Hz; gravity axis (sensor frame) {tuple(round(x,3) for x in up)}', flush=True)
w = csv.writer(open(A.out,'w',newline='')); w.writerow(['mode','cmd_deg','dyaw_deg','cum_yaw_deg','bias_rad_s','samples','burst_s','tilt_deg','L_yaw_ticks','L_yaw_err','R_yaw_ticks','R_yaw_err'])
MODES = {'both':(1,1), 'left':(1,0), 'right':(0,1), 'opposite':(1,-1)}
for mode in A.modes.split(','):
    sL, sR = MODES[mode]; cum = 0.0
    pose(0.0, 0.0); time.sleep(1.0)
    print(f'== mode {mode}: L x{sL}, R x{sR}', flush=True)
    prev = 0.0
    for deg in STEPS:
        acc0, g0, _ = burst(12)
        bias = sum(dot(g, up) for g in g0)/len(g0)
        pose(deg*sL, deg*sR)
        # window: the move (|step| deg at spd ticks/s) plus ~1.2 s of settle, at ~10 samples/s
        need = int((abs(deg-prev)*TPD/A.spd + 1.2) * 10) + 2
        acc1, g1, dt1 = burst(need)
        if len(g1) < need//2: print('   !! short burst, skipping step', flush=True); continue
        dts = dt1/len(g1)
        dyaw = math.degrees(sum(dot(g, up) - bias for g in g1) * dts)
        cum += dyaw
        tilt = math.degrees(math.acos(max(-1,min(1,dot(unit(mean(acc1)), base_up)))))
        tk = pose_ticks(deg*sL, deg*sR); pL, pR = servo_pos(10), servo_pos(9)
        eL = (pL - tk[0]) if pL is not None else None; eR = (pR - tk[5]) if pR is not None else None
        w.writerow([mode, deg, f'{dyaw:.3f}', f'{cum:.3f}', f'{bias:.5f}', len(g1), f'{dt1:.2f}', f'{tilt:.2f}', pL, eL, pR, eR])
        print(f'   cmd {deg:+.1f}  step {deg-prev:+.1f} -> pelvis {dyaw:+.2f} deg  cum {cum:+.2f}  (bias {bias:+.4f}, tilt {tilt:.1f})  servo err L {eL} R {eR} ticks', flush=True)
        prev = deg
        if tilt > A.tilt_abort: print('!! tilt guard', flush=True); pose(0.0, 0.0); sys.exit(2)
    pose(0.0, 0.0); time.sleep(1.0)
    print(f'   back at zero; net pelvis yaw after the loop {cum:+.2f} deg', flush=True)
o,_ = cmd('home', until=r'(HOLDING|readback|REFUSED|failed)', timeout=8.0); print(o.strip().splitlines()[-1] if o.strip() else 'home: no reply', flush=True)
s.close()
