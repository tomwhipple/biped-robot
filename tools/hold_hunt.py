"""Hold a crouch pose and look for servo hunting: a long gyro burst, then repeated
pos/speed reads of the pitch-chain servos. Robot: one foot clamped."""
import serial, time, re, csv, math, sys, argparse
ap = argparse.ArgumentParser(); ap.add_argument('out'); ap.add_argument('--theta', type=float, default=20.0)
ap.add_argument('--polls', type=int, default=15); ap.add_argument('--smooth-ms', type=int, default=3000)
A = ap.parse_args()
CAL = [("L_hip_yaw",10,1692,+1),("L_hip_roll",5,2418,-1),("L_hip_pitch",6,2001,+1),("L_knee",7,1581,-1),("L_ankle",8,3535,+1),
       ("R_hip_yaw",9,1806,+1),("R_hip_roll",1,3532,-1),("R_hip_pitch",2,2479,+1),("R_knee",3,2063,-1),("R_ankle",4,3437,+1)]
TPD=4096/360.0
s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.15); time.sleep(1.5); s.read(65536); s.write(b'\r\n'); time.sleep(0.4); s.read(65536)
log=open(A.out.replace('.csv','.log'),'w')
def cmd(c, until, timeout=3.0, tries=4):
    for attempt in range(tries):
        s.read(65536); s.write(c.encode()+b'\r\n'); t0=time.time(); o=''
        while time.time()-t0 < timeout:
            o += s.read(4096).decode(errors='replace')
            if re.search(until, o): break
        log.write(f'>>> {c} [try {attempt+1}]\n{o}'); log.flush()
        if re.search(until, o): time.sleep(0.15); return o
    return o
def pose(theta):
    off={}
    for side in 'LR': off[f'{side}_hip_pitch']=-theta; off[f'{side}_knee']=-2*theta; off[f'{side}_ankle']=-theta
    t=[int(round(z+d*off.get(n,0.0)*TPD)) for n,i,z,d in CAL]
    return cmd('pose '+' '.join(map(str,t))+f' s{A.smooth_ms}', until=r'(ok|streaming|refus|OFF)')
def burst(n):
    pat=re.compile(r'g\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)')
    s.read(65536); s.write(f'imu raw {n}\r\n'.encode()); t0=time.time(); o=''
    while time.time()-t0 < n/7.0+2.5:
        o+=s.read(4096).decode(errors='replace')
        if len(pat.findall(o))>=n: break
    return [tuple(float(x) for x in r) for r in pat.findall(o)], time.time()-t0
o=pose(A.theta); print('pose:', o.strip()[:80]); time.sleep(A.smooth_ms/1000+2.0)
w=csv.writer(open(A.out,'w',newline='')); w.writerow(['phase','k','gx','gy','gz'])
g,dt=burst(200); [w.writerow(['hold',k]+list(r)) for k,r in enumerate(g)]
bx=sum(r[0] for r in g)/len(g); bz=sum(r[2] for r in g)/len(g); d=[math.hypot(r[0]-bx,r[2]-bz) for r in g]
print(f'hold gyro: {len(g)} samples {dt:.1f}s  RMS {math.sqrt(sum(v*v for v in d)/len(d)):.4f}  peak {max(d):.3f} rad/s')
pw=csv.writer(open(A.out.replace('.csv','_servo.csv'),'w',newline='')); pw.writerow(['t','joint','id','pos','spd','load'])
watch=[c for c in CAL if c[0].endswith(('_hip_pitch','_knee','_ankle'))]
stats={c[0]:[] for c in watch}; t0=time.time()
for k in range(A.polls):
    for n,i,z,d in watch:
        o=cmd(f'pos {i}', until=r'load\s+-?\d+.*err', timeout=1.5, tries=2); m=re.search(r'pos\s+(-?\d+)\s+spd\s+(-?\d+)\s+load\s+(-?\d+)',o)
        if m:
            p,sp,ld=int(m.group(1)),int(m.group(2)),int(m.group(3)); pw.writerow([f'{time.time()-t0:.1f}',n,i,p,sp,ld]); stats[n].append((p,sp,ld))
for n,v in stats.items():
    if not v: print(f'{n:12s} no reads'); continue
    ps=[x[0] for x in v]; sps=[x[1] for x in v]; lds=[x[2] for x in v]
    print(f'{n:12s} n={len(v):2d}  pos {min(ps)}..{max(ps)} (range {max(ps)-min(ps)})  |spd|>0 in {sum(1 for x in sps if x!=0)}  load {min(lds)}..{max(lds)}')
o=pose(0.0); time.sleep(A.smooth_ms/1000+1.5); print('back at zero'); s.close()
