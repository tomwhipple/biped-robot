"""Single-joint load sweep on the standing robot: free play = the low-load band."""
import serial, time, re, csv, math, sys
OUT = sys.argv[1]
CAL = [  # (name, id, zero, dir) joint order
 ("L_hip_roll",5,2420,-1),("R_hip_roll",1,3533,-1),
 ("L_hip_pitch",6,2044,+1),("R_hip_pitch",2,2501,+1),
 ("L_ankle",8,3516,+1),("R_ankle",4,3450,+1),
 ("L_knee",7,1634,-1),("R_knee",3,2050,-1)]
TPD = 4096/360.0
STEPS = [x*0.5 for x in range(0,9)] + [x*0.5 for x in range(7,-9,-1)] + [x*0.5 for x in range(-7,1)]
LOAD_ABORT = 250; LOAD_FREE = 60; TILT_ABORT_DEG = 8.0; SPD = 60
def openport():
    for _ in range(60):
        try:
            return serial.Serial('/dev/ttyUSB0', 115200, timeout=0.15)
        except Exception:
            time.sleep(0.5)
    raise SystemExit('serial port never came back')
s = openport()
log = open(OUT.replace('.csv','.log'),'w')
def cmd(c, wait=0.35, until=None, timeout=1.2, tries=4):
    """Send a CLI line; with `until`, wait for the reply pattern and RESEND if
    it does not come (the board drops ~half of all CLI lines, measured
    2026-09-02 -- pos/move/imu are idempotent, so a retry is safe)."""
    global s
    o = ''
    for attempt in range(tries if until else 1):
        try:
            s.read(65536)                   # discard anything stale
        except serial.SerialException:
            print('!! serial dropped -- reopening', flush=True); log.write('!! serial dropped\n')
            try: s.close()
            except Exception: pass
            time.sleep(2); s = openport(); time.sleep(2); s.read(65536); s.write(b'\r\n'); time.sleep(0.5); s.read(65536)
        s.write(c.encode()+b'\r\n'); t=time.time(); o=''
        while time.time()-t < (timeout if until else wait):
            try:
                o += s.read(4096).decode(errors='replace')
            except serial.SerialException:
                print('!! serial dropped mid-read', flush=True); break
            if until and re.search(until, o): break
        if until is None: time.sleep(0.05); o += s.read(65536).decode(errors='replace')
        log.write(f'>>> {c}  [try {attempt+1}]\n{o}'); log.flush()
        if until is None or re.search(until, o): break
    time.sleep(0.25)
    return o
t0=time.time(); buf=b''
while time.time()-t0 < 2: buf += s.read(4096)
cmd('', 0.3)
def pos(i):
    o = cmd(f'pos {i}', until=r'load\s+-?\d+.*err 0x..'); m = re.search(r'pos\s+(-?\d+)\s+spd\s+(-?\d+)\s+load\s+(-?\d+)', o)
    return (int(m.group(1)), int(m.group(2)), int(m.group(3))) if m else None
def imu():
    o = cmd('imu raw 1', until=r'\|a\|.*g .*\n'); m = re.search(r'a\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)', o)
    return tuple(float(x) for x in m.groups()) if m else None
def ang(a,b):
    na=math.sqrt(sum(x*x for x in a)); nb=math.sqrt(sum(x*x for x in b))
    return math.degrees(math.acos(max(-1,min(1,sum(x*y for x,y in zip(a,b))/(na*nb)))))
sc = cmd('scan', until=r'servo\(s\)', timeout=6); print(sc.strip().splitlines()[-1], flush=True)
base = imu(); print('imu baseline accel', base, flush=True)
for name,i,zero,d in CAL:
    o = cmd(f'move {i} {zero} 0 {SPD}', until=r'(: ok|refusing|torque is OFF)')
    if 'torque is OFF' in o or 'refusing' in o:
        print(f'!! {name} id {i}: {o.strip()[:70]} -- torque must be ON (run `home` / `torque` first); stopping', flush=True); s.close(); sys.exit(2)
print('all ten joints accept goals at zero (torque on)', flush=True)
w = csv.writer(open(OUT,'w',newline='')); w.writerow(['joint','id','cmd_deg','goal_ticks','pos_ticks','pos_err','load','spd','tilt_deg','epoch'])
for name,i,zero,d in CAL:
    p0 = pos(i); print(f'== {name} id {i} start pos {p0}', flush=True)
    aborted = None
    for deg in STEPS:
        goal = int(round(zero + d*deg*TPD))
        o = cmd(f'move {i} {goal} 0 {SPD}', until=r'(: ok|refusing|torque is OFF)'); time.sleep(0.45)
        if 'torque is OFF' in o or 'refusing' in o: aborted=f'gate: {o.strip()[:60]}'; break
        p = pos(i); a = imu(); tilt = ang(base,a) if (a and base) else float('nan')
        if p is None: aborted='no feedback'; break
        w.writerow([name,i,deg,goal,p[0],p[0]-goal,p[2],p[1],f'{tilt:.2f}',f'{time.time():.2f}'])
        print(f'  {deg:+5.1f} deg  goal {goal}  pos {p[0]} (err {p[0]-goal:+d})  load {p[2]:+5d}  tilt {tilt:.1f}', flush=True)
        if abs(p[2]) > LOAD_ABORT: aborted=f'load {p[2]} at {deg:+.1f} deg'; break
        if tilt > TILT_ABORT_DEG: aborted=f'tilt {tilt:.1f} deg at {deg:+.1f}'; break
    cmd(f'move {i} {zero} 0 {SPD}', until=r': ok'); time.sleep(0.8); p=pos(i)
    print(f'   back to zero: pos {p}  {"ABORT: "+aborted if aborted else "sweep complete"}', flush=True)
    if aborted and aborted.startswith('tilt'): print('!! tilt guard -- stopping all sweeps'); break
print(cmd('home', until=r'HOLDING', timeout=6).strip(), flush=True); time.sleep(2)
print(cmd('scan', until=r'servo\(s\)', timeout=6).strip(), flush=True)
s.close()
