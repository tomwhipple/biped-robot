"""Quiet-hold gyro drift log per axis, sensor frame and body-frame ring (2026-09-03)."""
import serial, time, re
s=serial.Serial('/dev/ttyUSB0',115200,timeout=0.15); time.sleep(1.5); s.read(65536); s.write(b'\r\n'); time.sleep(0.4); s.read(65536)
def cmd(c, until, timeout=4.0, tries=3):
    for _ in range(tries):
        s.read(65536); s.write(c.encode()+b'\r\n'); t0=time.time(); o=''
        while time.time()-t0<timeout:
            o+=s.read(4096).decode(errors='replace')
            if re.search(until,o): time.sleep(0.1); return o
    return o
print(cmd('home', r'(streaming|REFUSED)').strip().splitlines()[0][:30]); time.sleep(4)
pat=re.compile(r'a\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+\|a\|\s+[\d.]+\s+g\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)')
print("robot still, torque holding: sensor-frame gyro (bias-corrected) mean per ~1.5 s burst, rad/s; and body-frame mean from the 250 Hz ring")
for k in range(10):
    s.read(65536); s.write(b'imu raw 15\r\n'); t0=time.time(); o=''
    while time.time()-t0<4: 
        o+=s.read(4096).decode(errors='replace')
        if len(pat.findall(o))>=15: break
    rows=[tuple(float(x) for x in r) for r in pat.findall(o)]
    m=[sum(r[3+i] for r in rows)/len(rows) for i in range(3)] if rows else [float('nan')]*3
    ro=cmd('imu ring 2500', r'envelope.*\n'); rm=re.search(r'mean rate \(rad/s\) (-?[\d.]+) (-?[\d.]+) (-?[\d.]+)', ro)
    print(f"  t={k*3.5:4.1f}s  sensor x {m[0]:+.4f} y {m[1]:+.4f} z {m[2]:+.4f}   | body ring x {rm.group(1)} y {rm.group(2)} z {rm.group(3)}" if rm else f"  t={k*3.5:4.1f}s sensor {m}", flush=True)
print(cmd('release', r'ok').strip()[:16]); s.close()
