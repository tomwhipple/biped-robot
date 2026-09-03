"""Range of motion on the bench: the level-foot knee-bend family.

Pose family (torso level, foot level, foot stays under the hip):
    hip_pitch = -theta, knee = -2*theta, ankle = -theta
    lift height ~ (thigh+shank) * (1 - cos theta)   (theta 45 deg -> ~6 cm)
--legs L|R lifts one foot (other foot CLAMPED, per Tom 2026-09-03);
--legs both is the crouch/squat, unclamped, with a spotter.
Every step: `pose` (one sync write), settle, then per-joint position error and
load for the moving joints, torso tilt vs the start pose, free-leg loads, and a
camera frame. Aborts back to zero on tilt > --tilt-abort or a stalled joint.
"""
import serial, time, re, csv, math, sys, argparse, subprocess, os
ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument('out'); ap.add_argument('--legs', choices=['L','R','both'], required=True)
ap.add_argument('--thetas', default='10,20,30,40,45,40,30,20,10,0')
ap.add_argument('--spd', type=int, default=60); ap.add_argument('--settle', type=float, default=2.0)
ap.add_argument('--smooth-ms', type=int, default=3000, help='firmware pose s<ms>: minimum-jerk stream from the housekeeping loop (default; 0 = constant speed)')
ap.add_argument('--stream-opts', default='', help='extra tokens after s<ms>, e.g. "h90 a10" (speed headroom pct, servo acc register)')
ap.add_argument('--unison-ms', type=int, default=0, help='use the firmware pose t<ms> unison mode (per-servo speeds) instead of one speed')
ap.add_argument('--tilt-abort', type=float, default=8.0); ap.add_argument('--stall-ticks', type=int, default=40)
ap.add_argument('--roll', type=float, default=0.0, help='hip roll (abduction, +) on the lifted leg for clearance')
ap.add_argument('--cam', default='/dev/video0')
ap.add_argument('--burst', type=int, default=0, help='IMU raw samples to stream right after each pose (~11/s): captures the move + ringing; gyro rows go to <out>_gyro.csv')
A = ap.parse_args()
CAL = [("L_hip_yaw",10,1692,+1),("L_hip_roll",5,2418,-1),("L_hip_pitch",6,2001,+1),("L_knee",7,1581,-1),("L_ankle",8,3535,+1),
       ("R_hip_yaw",9,1806,+1),("R_hip_roll",1,3532,-1),("R_hip_pitch",2,2479,+1),("R_knee",3,2063,-1),("R_ankle",4,3437,+1)]
TPD = 4096/360.0
log = open(A.out.replace('.csv','.log'),'w')
def openport():
    for _ in range(60):
        try: return serial.Serial('/dev/ttyUSB0', 115200, timeout=0.15)
        except Exception: time.sleep(0.5)
    raise SystemExit('serial port never came back')
s = openport(); time.sleep(1.5); s.read(65536); s.write(b'\r\n'); time.sleep(0.4); s.read(65536)
def cmd(c, until, timeout=3.0, tries=4):
    global s
    for attempt in range(tries):
        try: s.read(65536)
        except serial.SerialException:
            log.write('!! serial dropped\n'); s.close(); time.sleep(2); s = openport(); time.sleep(2); s.read(65536)
        s.write(c.encode()+b'\r\n'); t0=time.time(); o=''
        while time.time()-t0 < timeout:
            o += s.read(4096).decode(errors='replace')
            if re.search(until, o): break
        log.write(f'>>> {c} [try {attempt+1}]\n{o}'); log.flush()
        if re.search(until, o): time.sleep(0.2); return o
    return o
def pos(i):
    o = cmd(f'pos {i}', until=r'load\s+-?\d+.*err'); m = re.search(r'pos\s+(-?\d+)\s+spd\s+(-?\d+)\s+load\s+(-?\d+)', o)
    return (int(m.group(1)), int(m.group(2)), int(m.group(3))) if m else None
def imu():
    o = cmd('imu raw 1', until=r'\|a\|.*g .*\n'); m = re.search(r'a\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)', o)
    return tuple(float(x) for x in m.groups()) if m else None
def ang(a,b):
    na=math.sqrt(sum(x*x for x in a)); nb=math.sqrt(sum(x*x for x in b))
    return math.degrees(math.acos(max(-1,min(1,sum(x*y for x,y in zip(a,b))/(na*nb)))))
def offsets(theta):
    off = {}
    for side in (['L','R'] if A.legs=='both' else [A.legs]):
        off[f'{side}_hip_pitch'] = -theta; off[f'{side}_knee'] = -2*theta; off[f'{side}_ankle'] = -theta
        if A.legs != 'both': off[f'{side}_hip_roll'] = A.roll
    return off
def ticks(off): return [int(round(z + d*off.get(n,0.0)*TPD)) for n,i,z,d in CAL]
def pose(off):
    tail = (f's{A.smooth_ms} {A.stream_opts}'.strip()) if A.smooth_ms else f't{A.unison_ms}' if A.unison_ms else f'{A.spd}'
    return cmd('pose ' + ' '.join(map(str, ticks(off))) + f' {tail}', until=r'(ok|refus|OFF|usage|busy|answer)')
def burst(n):
    pat = re.compile(r'a\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+\|a\|\s+[\d.]+\s+g\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)')
    s.read(65536); s.write(f'imu raw {n}\r\n'.encode()); t0=time.time(); o=''
    while time.time()-t0 < n/7.0 + 2.5:
        o += s.read(4096).decode(errors='replace')
        if len(pat.findall(o)) >= n: break
    rows = pat.findall(o); dt = time.time()-t0
    log.write(f'>>> imu raw {n} [{len(rows)} rows, {dt:.2f}s]\n{o}'); log.flush()
    return [tuple(float(x) for x in r) for r in rows], dt
def ring_stats(rows, dt):
    """gyro magnitude per sample; peak, RMS of the second half (settled?), dominant
    zero-crossing frequency of the largest-variance gyro axis."""
    if len(rows) < 8: return None
    g = [r[3:] for r in rows]; n=len(g); per = dt/n
    var = [sum((x[k]-sum(y[k] for y in g)/n)**2 for x in g)/n for k in range(3)]
    k = max(range(3), key=lambda i: var[i]); x=[r[k] for r in g]; m=sum(x)/n; d=[v-m for v in x]
    zc = sum(1 for i in range(1,n) if d[i-1]*d[i] < 0); f = zc/2/(n*per)
    half = d[n//2:]; rms2 = math.sqrt(sum(v*v for v in half)/len(half))
    return dict(axis='xyz'[k], peak=max(abs(v) for v in d), rms_tail=rms2, f_hz=f, n=n, hz=n/dt)
def frame(tag):
    f = A.out.replace('.csv', f'_{tag}.jpg')
    subprocess.run(['ffmpeg','-loglevel','error','-y','-f','v4l2','-video_size','1280x720','-input_format','mjpeg','-i',A.cam,'-frames:v','1',f], timeout=20)
    return os.path.basename(f) if os.path.exists(f) else ''
moving = [c for c in CAL if c[0] in offsets(1.0)]
watch = [c for c in CAL if c[0] not in offsets(1.0) and c[0].endswith(('_ankle','_knee','_hip_pitch'))]
o = pose(offsets(0.0))
if not re.search(r'ok|streaming', o): print('!! pose refused:', o.strip()[:120]); sys.exit(2)
time.sleep(1.5); base = imu(); print(f'start pose ok; tilt reference {base}', flush=True)
gw = None
if A.burst:
    gw = csv.writer(open(A.out.replace('.csv','_gyro.csv'),'w',newline='')); gw.writerow(['theta','k','ax','ay','az','gx','gy','gz'])
w = csv.writer(open(A.out,'w',newline='')); w.writerow(['theta','tilt_deg','frame','ring_axis','ring_peak_rad_s','ring_rms_tail','ring_f_hz'] + [f'{c[0]}_err' for c in moving] + [f'{c[0]}_load' for c in moving] + [f'{c[0]}_load' for c in watch])
prev_theta = 0.0
for theta in [float(x) for x in A.thetas.split(',')]:
    off = offsets(theta); tk = ticks(off); o = pose(off)
    if not re.search(r'ok|streaming', o): print(f'!! pose refused at theta {theta}: {o.strip()[:100]}'); break
    ring = None
    if A.burst:
        rows, bdt = burst(A.burst)
        for k, r in enumerate(rows): gw.writerow([theta, k] + list(r))
        ring = ring_stats(rows, bdt)
    else:
        time.sleep(((A.smooth_ms or A.unison_ms)/1000.0 if (A.smooth_ms or A.unison_ms) else 2*abs(theta-prev_theta)*TPD/A.spd) + A.settle)
    prev_theta = theta
    errs=[]; loads=[]
    for n,i,z,d in moving:
        p = pos(i); j = [c[0] for c in CAL].index(n)
        errs.append(p[0]-tk[j] if p else None); loads.append(p[2] if p else None)
    wl = [ (pos(i) or (None,None,None))[2] for n,i,z,d in watch ]
    a = imu(); tilt = ang(base, a) if (a and base) else float('nan'); fr = frame(f't{int(theta):02d}')
    rs = [ring['axis'], f"{ring['peak']:.3f}", f"{ring['rms_tail']:.4f}", f"{ring['f_hz']:.2f}"] if ring else ['','','','']
    w.writerow([theta, f'{tilt:.2f}', fr] + rs + errs + loads + wl)
    print((f"  ring: axis {ring['axis']} peak {ring['peak']:.2f} rad/s, tail RMS {ring['rms_tail']:.3f}, ~{ring['f_hz']:.1f} Hz over {ring['n']} samples @ {ring['hz']:.0f} Hz" if ring else ''), flush=True)
    print(f'theta {theta:4.0f}  knee {-2*theta:+.0f}  tilt {tilt:4.1f}  err ' + ' '.join(f'{c[0]}={e}' for c,e in zip(moving,errs)) + '  load ' + ' '.join(f'{c[0]}={l}' for c,l in zip(moving,loads)) + ('  other-leg ' + ' '.join(f'{c[0]}={l}' for c,l in zip(watch,wl)) if watch else ''), flush=True)
    if tilt > A.tilt_abort: print('!! tilt guard -- back to zero', flush=True); pose(offsets(0.0)); sys.exit(2)
    if any(e is not None and abs(e) > A.stall_ticks for e in errs): print('!! stalled joint -- back to zero', flush=True); pose(offsets(0.0)); sys.exit(2)
pose(offsets(0.0)); time.sleep(2.0); print('back at zero', flush=True)
s.close()
