"""Right/left ANKLE gap probe on a clamped single-leg stance.

Lift the free leg clear (level-foot family + abduction), then sweep the
stance ankle 0 -> +A -> -A -> 0 with the smooth streamer, tracing that ankle
servo at 50 Hz (pose/speed/load) and reading the 250 Hz torso ringing meter
after each segment. Reports, per segment: the sample where the servo load
changes sign, the shaft position step across it (a jump = gearbox lash inside
the servo; a glide = play outside it), and the torso's peak rate.
    .venv/bin/python tools/ankle_probe.py hw_sessions/<day>/ankleR.csv --leg R
"""
import serial, time, re, csv, math, sys, argparse
ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument('out'); ap.add_argument('--leg', choices=['L','R'], required=True, help='stance (clamped) leg')
ap.add_argument('--amp', type=float, default=3.0); ap.add_argument('--seg-ms', type=int, default=6000)
A = ap.parse_args()
CAL = [("L_hip_yaw",10,1692,+1),("L_hip_roll",5,2418,-1),("L_hip_pitch",6,2001,+1),("L_knee",7,1581,-1),("L_ankle",8,3535,+1),
       ("R_hip_yaw",9,1806,+1),("R_hip_roll",1,3532,-1),("R_hip_pitch",2,2479,+1),("R_knee",3,2063,-1),("R_ankle",4,3437,+1)]
TPD = 4096/360.0
free = 'L' if A.leg == 'R' else 'R'
ankle_name = f'{A.leg}_ankle'; ankle_id = [c[1] for c in CAL if c[0] == ankle_name][0]
log = open(A.out.replace('.csv','.log'),'w')
s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.15); time.sleep(1.5); s.read(65536); s.write(b'\r\n'); time.sleep(0.4); s.read(65536)
def cmd(c, until, timeout=3.0, tries=3):
    for attempt in range(tries):
        s.read(65536); s.write(c.encode()+b'\r\n'); t0=time.time(); o=''
        while time.time()-t0 < timeout:
            o += s.read(4096).decode(errors='replace')
            if re.search(until, o): break
        log.write(f'>>> {c} [try {attempt+1}]\n{o}'); log.flush()
        if re.search(until, o): time.sleep(0.15); return o
    return o
def ticks(off): return [int(round(z + d*off.get(n,0.0)*TPD)) for n,i,z,d in CAL]
def base(ankle_deg=0.0):
    off = {f'{free}_hip_pitch': -25.0, f'{free}_knee': -50.0, f'{free}_ankle': -25.0, f'{free}_hip_roll': 10.0, ankle_name: ankle_deg}
    return off
def pose_once(off, ms, trace=False):
    tail = f's{ms}' + (f' T{ankle_id}' if trace else '')
    o = cmd('pose ' + ' '.join(map(str, ticks(off))) + ' ' + tail, until=r'(streaming|ok|refus|OFF|usage|busy|answer)', timeout=1.5, tries=1)
    return o
print(cmd('home', r'(streaming|REFUSED)', 4.0).strip().splitlines()[0][:70]); time.sleep(3.5)
print('lifting the free leg:', pose_once(base(0.0), 4000).strip()[:40]); time.sleep(5.5)
w = csv.writer(open(A.out,'w',newline='')); w.writerow(['segment','target_deg','flip_ms','pos_before','pos_after','pos_step_ticks','load_before','load_after','torso_peak_rad_s','torso_env'])
for seg, tgt in enumerate([A.amp, -A.amp, 0.0]):
    o = pose_once(base(tgt), A.seg_ms, trace=True); print(f'== segment {seg}: {ankle_name} -> {tgt:+.1f} deg over {A.seg_ms} ms  ({o.strip()[:30]})', flush=True)
    time.sleep(A.seg_ms/1000.0 + 1.0)
    ro = cmd('imu ring 2500', until=r'envelope.*\n', timeout=3.0); rm = re.search(r'peak ([\d.]+) rad/s', ro); env = re.search(r'envelope[^:]*:(.*)', ro)
    time.sleep(0.5)
    to = cmd('trace', until=r'trace end', timeout=6.0)
    rows = [tuple(int(x) for x in m.groups()) for m in re.finditer(r'T (\d+) (-?\d+) (-?\d+) (-?\d+)', to)]
    tf = A.out.replace('.csv', f'_seg{seg}_trace.csv')
    with open(tf,'w',newline='') as fh: tw=csv.writer(fh); tw.writerow(['ms','pos','spd','load']); [tw.writerow(r) for r in rows]
    flip = None
    for a, b in zip(rows, rows[1:]):
        if a[3] != 0 and b[3] != 0 and (a[3] > 0) != (b[3] > 0): flip = (a, b); break
    if flip:
        a, b = flip
        # shaft step across the flip: compare position 100 ms before and after
        before = [r for r in rows if a[0]-120 <= r[0] <= a[0]]; after = [r for r in rows if b[0] <= r[0] <= b[0]+120]
        pb = before[0][1] if before else a[1]; pa = after[-1][1] if after else b[1]
        print(f'   load flips {a[3]:+d} -> {b[3]:+d} at {b[0]} ms; shaft {pb} -> {pa} ({pa-pb:+d} ticks across the flip)', flush=True)
        w.writerow([seg, tgt, b[0], pb, pa, pa-pb, a[3], b[3], rm.group(1) if rm else '', env.group(1).strip() if env else ''])
    else:
        print('   no load sign flip in this segment', flush=True); w.writerow([seg, tgt, '', '', '', '', '', '', rm.group(1) if rm else '', env.group(1).strip() if env else ''])
    if rm: print(f'   torso peak {float(rm.group(1)):.3f} rad/s; envelope {env.group(1).strip()[:90] if env else ""}', flush=True)
    print('   trace (every 10th): ' + ' '.join(f'{r[0]}:{r[1]}/{r[3]:+d}' for r in rows[::10]), flush=True)
print('lowering the free leg:', pose_once(base(0.0), 3000).strip()[:20]); time.sleep(4)
print(cmd('home', r'(streaming|REFUSED)', 4.0).strip().splitlines()[0][:60]); time.sleep(3.5); print(cmd('release', r'ok').strip()[:20])
s.close()
