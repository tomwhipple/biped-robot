"""Gyro SCALE check on one clamped leg (2026-09-03, Tom: "hand input will not
produce accurate results"): the stance ankle pitches the whole robot; the
accelerometer at rest before/after is the truth for the tilt change; the 250 Hz
ring integral through the move is the gyro's answer. Ratio = scale error.
    .venv/bin/python tools/imu_scale_check.py --leg L   (left foot CLAMPED)

Calibration from the board's `cal show` (blob v3, 17-servo bus map);
asbuilt fallback. Pose strings are 17-wide in servo-ID order.
"""
import serial, time, re, math, argparse, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bus_cal import BusCal, BusCalError, asbuilt_prototype_cal, fetch_cal
ap=argparse.ArgumentParser(); ap.add_argument('--leg', choices=['L','R'], required=True, help='clamped (stance) leg'); ap.add_argument('--joint', default='ankle', choices=['ankle','hip_roll','hip_pitch'], help='stance joint to drive'); ap.add_argument('--amp', type=float, default=8.0); ap.add_argument('--ms', type=int, default=2000)
A=ap.parse_args()
TPD=4096/360; free='R' if A.leg=='L' else 'L'; ankle=f'{A.leg}_{A.joint}'
s=serial.Serial('/dev/ttyUSB0',115200,timeout=0.15); time.sleep(1.5); s.read(65536); s.write(b'\r\n'); time.sleep(0.4); s.read(65536)
def cmd(c, until, timeout=4.0, tries=3):
    for _ in range(tries):
        s.read(65536); s.write(c.encode()+b'\r\n'); t0=time.time(); o=''
        while time.time()-t0<timeout:
            o+=s.read(4096).decode(errors='replace')
            if re.search(until,o): time.sleep(0.1); return o
    return o

CAL, CAL_SOURCE = fetch_cal(lambda c, until: cmd(c, until))
print(f'cal source: {CAL_SOURCE}', flush=True)
if not (CAL.by_name(ankle).fitted and CAL.by_name(f'{free}_hip_pitch').fitted):
    print('!! a needed joint is NOT FITTED on this robot -- refusing', flush=True); s.close(); sys.exit(2)
def pose(off, ms):
    t = CAL.pose_ticks(off)
    return cmd('pose '+' '.join(map(str,t))+f' s{ms}', until=r'(streaming|refus|OFF)')
def base(ank=0.0):
    abd=10.0 if free=='L' else -10.0
    return {f'{free}_hip_pitch':-25.0, f'{free}_knee':-50.0, f'{free}_ankle':-25.0, f'{free}_hip_roll':abd, ankle:ank}
pat=re.compile(r'a\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+\|a\|')
def accel_dir(n=12):
    s.read(65536); s.write(f'imu raw {n}\r\n'.encode()); t0=time.time(); o=''
    while time.time()-t0<n/7.0+2.5:
        o+=s.read(4096).decode(errors='replace')
        if len(pat.findall(o))>=n: break
    rows=[tuple(float(x) for x in r) for r in pat.findall(o)]; m=[sum(r[i] for r in rows)/len(rows) for i in range(3)]; nn=math.sqrt(sum(x*x for x in m)); return [x/nn for x in m]
def ang(a,b): return math.degrees(math.acos(max(-1,min(1,sum(x*y for x,y in zip(a,b))))))
print(cmd('home', r'(streaming|REFUSED)').strip().splitlines()[0][:30]); time.sleep(3.5)
print('lift free leg:', pose(base(0.0), 3000).strip()[:20]); time.sleep(4.5)
print(f"stance {A.leg}, {A.joint} {A.amp:+.0f}/{-A.amp:+.0f}/0 deg over {A.ms} ms each")
for tgt in (A.amp, -A.amp, 0.0):
    a0=accel_dir(); pose(base(tgt), A.ms); time.sleep(A.ms/1000.0+0.4)
    ro=cmd('imu ring 2500', r'envelope.*\n'); m=re.search(r'integral over the window \(deg, body frame\): x (-?[\d.]+) y (-?[\d.]+) z (-?[\d.]+)', ro)
    time.sleep(1.5); a1=accel_dir()
    d_acc=ang(a0,a1)
    if m:
        gx,gy,gz=(float(m.group(i)) for i in (1,2,3)); gmag=math.sqrt(gx*gx+gy*gy+gz*gz)
        print(f"  {A.joint} -> {tgt:+.0f}: ACCEL tilt change {d_acc:5.2f} deg | GYRO integral body x {gx:+6.2f} y {gy:+6.2f} z {gz:+6.2f} (|.| {gmag:5.2f})  ratio gyro/accel {gmag/d_acc if d_acc>0.3 else float('nan'):.2f}", flush=True)
    else: print(f"  {A.joint} -> {tgt:+.0f}: ACCEL tilt change {d_acc:5.2f} deg | ring: no reply", flush=True)
pose(base(0.0), 2000); time.sleep(3); print(cmd('home', r'(streaming|REFUSED)').strip().splitlines()[0][:30]); time.sleep(3.5); print(cmd('release', r'ok').strip()[:16]); s.close()
