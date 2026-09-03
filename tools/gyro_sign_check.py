"""Gyro SIGN vs up-vector check, robot standing on both feet, no arming
(2026-09-03: hiding only the gyro from the policy turns a fall into an 8 s
stand, so the rate's sign/frame is the suspect). Both ankles (or both hip
rolls) lean the whole body a few degrees over 2 s; `imu` before/after gives
the same body-frame `up` the policy sees, `imu ring 2500` gives the body-frame
gyro integral through the move. For a world-fixed vector seen from the body,
The sim's up is framezaxis = body z in WORLD at yaw 0 (column 3 of R), so
near upright d up_x = +int(gyro_y), d up_y = -int(gyro_x). Verified in sim on
bimo_biped_v5body (ankle +6: d_up (-0.11, 0), int y -6.28 deg) and on the
robot after the yaw-strip fix (d_up (-0.100, -0.005), int y -6.55 deg).
    .venv/bin/python tools/gyro_sign_check.py --joint ankle --amp 6"""
import serial, time, re, argparse
ap=argparse.ArgumentParser(); ap.add_argument('--joint', default='ankle', choices=['ankle','hip_roll','hip_pitch','knee']); ap.add_argument('--amp', type=float, default=6.0); ap.add_argument('--ms', type=int, default=2000); ap.add_argument('--no-release', action='store_true')
A=ap.parse_args()
CAL=[("L_hip_yaw",10,1692,+1),("L_hip_roll",5,2418,-1),("L_hip_pitch",6,2001,+1),("L_knee",7,1581,-1),("L_ankle",8,3535,+1),("R_hip_yaw",9,1806,+1),("R_hip_roll",1,3532,-1),("R_hip_pitch",2,2479,+1),("R_knee",3,2063,-1),("R_ankle",4,3437,+1)]
TPD=4096/360
s=serial.Serial('/dev/ttyUSB0',115200,timeout=0.15); time.sleep(1.5); s.read(65536); s.write(b'\r\n'); time.sleep(0.4); s.read(65536)
def cmd(c, until, timeout=4.0, tries=3):
    for _ in range(tries):
        s.read(65536); s.write(c.encode()+b'\r\n'); t0=time.time(); o=''
        while time.time()-t0<timeout:
            o+=s.read(4096).decode(errors='replace')
            if re.search(until,o): time.sleep(0.1); return o
    return o
def pose(off, ms):
    t=[int(round(z+d*off.get(n,0)*TPD)) for n,i,z,d in CAL]; return cmd('pose '+' '.join(map(str,t))+f' s{ms}', until=r'(streaming|refus|OFF)')
UP=re.compile(r'up\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)')
def up_avg(n=5):
    acc=[0.0,0.0,0.0]; k=0
    for _ in range(n):
        m=UP.search(cmd('imu', r'levelled'))
        if m: k+=1; acc=[a+float(m.group(i+1)) for i,a in enumerate(acc)]
        time.sleep(0.15)
    return [a/k for a in acc] if k else None
def lean(a):
    # same WORLD sense on both legs: ankles share sign; hip rolls are +L/-R abduction, so L +a with R +a rolls both the same way
    if A.joint=='hip_roll': return {'L_hip_roll':a,'R_hip_roll':a}
    return {f'L_{A.joint}':a, f'R_{A.joint}':a}
print(cmd('home', r'(streaming|REFUSED)').strip().splitlines()[0][:30]); time.sleep(3.5)
print(f"both-{A.joint} lean {A.amp:+.0f} then back, {A.ms} ms each; up = policy's body-frame up, integral = policy's body-frame gyro")
for tgt in (A.amp, 0.0):
    u0=up_avg(); pose(lean(tgt), A.ms); time.sleep(A.ms/1000.0+0.4)
    ro=cmd('imu ring 2500', r'envelope.*\n'); m=re.search(r'integral over the window \(deg, body frame\): x (-?[\d.]+) y (-?[\d.]+) z (-?[\d.]+)', ro)
    time.sleep(1.5); u1=up_avg()
    if not (u0 and u1 and m): print(f"  -> {tgt:+.0f}: missing data (up0 {u0} up1 {u1} ring {bool(m)})"); continue
    du=[b-a for a,b in zip(u0,u1)]; gx,gy,gz=(float(m.group(i)) for i in (1,2,3))
    import math
    print(f"  -> {tgt:+.0f}: up {u0[0]:+.3f} {u0[1]:+.3f} {u0[2]:+.3f} -> {u1[0]:+.3f} {u1[1]:+.3f} {u1[2]:+.3f}  d_up x {du[0]:+.3f} y {du[1]:+.3f} ({math.degrees(math.asin(max(-1,min(1,math.hypot(du[0],du[1]))))):.1f} deg) | gyro integral deg x {gx:+.2f} y {gy:+.2f} z {gz:+.2f}")
    print(f"     sim convention (column 3, yaw 0): d_up_x sign = +(int y) sign -> {'MATCH' if du[0]*gy>0 else 'MISMATCH'} ; d_up_y sign = -(int x) sign -> {'MATCH' if du[1]*gx<0 else 'MISMATCH'}  (only the axis that moved matters)", flush=True)
time.sleep(3.0)
if not A.no_release: print(cmd('release', r'ok').strip()[:16])
s.close()
