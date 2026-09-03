"""Raw sensor-frame IMU check: accel rotation axis vs gyro integral through slow hip-pitch moves (2026-09-03)."""
import serial, time, re, math
CAL=[("L_hip_yaw",10,1692,+1),("L_hip_roll",5,2418,-1),("L_hip_pitch",6,2001,+1),("L_knee",7,1581,-1),("L_ankle",8,3535,+1),("R_hip_yaw",9,1806,+1),("R_hip_roll",1,3532,-1),("R_hip_pitch",2,2479,+1),("R_knee",3,2063,-1),("R_ankle",4,3437,+1)]
TPD=4096/360
s=serial.Serial('/dev/ttyUSB0',115200,timeout=0.15); time.sleep(1.5); s.read(65536); s.write(b'\r\n'); time.sleep(0.4); s.read(65536)
def cmd(c, until, timeout=3.0, tries=3):
    for _ in range(tries):
        s.read(65536); s.write(c.encode()+b'\r\n'); t0=time.time(); o=''
        while time.time()-t0<timeout:
            o+=s.read(4096).decode(errors='replace')
            if re.search(until,o): time.sleep(0.1); return o
    return o
pat=re.compile(r'a\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+\|a\|\s+[\d.]+\s+g\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)')
def burst(n):
    s.read(65536); s.write(f'imu raw {n}\r\n'.encode()); t0=time.time(); o=''
    while time.time()-t0 < n/7.0+2.5:
        o+=s.read(4096).decode(errors='replace')
        if len(pat.findall(o))>=n: break
    return [tuple(float(x) for x in r) for r in pat.findall(o)], time.time()-t0
def mean3(rows, k): return [sum(r[k+i] for r in rows)/len(rows) for i in range(3)]
def unit(v): n=math.sqrt(sum(x*x for x in v)) or 1; return [x/n for x in v]
def pose(deg, ms=3000):
    off={'L_hip_pitch':deg,'R_hip_pitch':deg}; t=[int(round(z+d*off.get(n,0)*TPD)) for n,i,z,d in CAL]
    return cmd('pose '+' '.join(map(str,t))+f' s{ms}', until=r'(streaming|refus|OFF)')
print(cmd('home', r'(streaming|REFUSED)').strip().splitlines()[0][:40]); time.sleep(3.5)
for amp in (+12.0, 0.0, -12.0, 0.0):
    b0,_=burst(15); pose(amp); rows,dt=burst(44); time.sleep(0.3); b1,_=burst(15)
    a0=unit(mean3(b0,0)); a1=unit(mean3(b1,0))
    ax=[a0[1]*a1[2]-a0[2]*a1[1], a0[2]*a1[0]-a0[0]*a1[2], a0[0]*a1[1]-a0[1]*a1[0]]; ang=math.degrees(math.asin(max(-1,min(1,math.sqrt(sum(x*x for x in ax))))))
    g0=mean3(b0,3); g1=mean3(b1,3); bias=[(x+y)/2 for x,y in zip(g0,g1)]; dts=dt/len(rows)
    integ=[math.degrees(sum(r[3+i]-bias[i] for r in rows)*dts) for i in range(3)]
    print(f'hips {amp:+.0f}: ACCEL rotation {ang:5.2f} deg about sensor axis {[round(x,2) for x in unit(ax)]}   GYRO integral (sensor frame, deg) x {integ[0]:+6.2f} y {integ[1]:+6.2f} z {integ[2]:+6.2f}   rest rates before/after x {g0[0]:+.3f}/{g1[0]:+.3f} y {g0[1]:+.3f}/{g1[1]:+.3f}', flush=True)
    time.sleep(0.5)
print(cmd('release', r'ok').strip()[:16]); s.close()
