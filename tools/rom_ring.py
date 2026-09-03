"""Per-step ringing metrics from a leg_rom gyro CSV: peak horizontal gyro during
the move window, then RMS in the settled window (bias from the last 2 s)."""
import csv, math, sys
def analyze(f, move_s):
    steps=[]; cur=None
    for r in csv.DictReader(open(f)):
        k=int(r['k'])
        if k==0: cur=(float(r['theta']),[]); steps.append(cur)
        cur[1].append(tuple(float(r[c]) for c in ('gx','gy','gz')))
    out=[]
    for th, rows in steps:
        n=len(rows); hz=n/12.0 if n>60 else 10.0
        bx=sum(g[0] for g in rows[-20:])/20; bz=sum(g[2] for g in rows[-20:])/20
        d=[math.hypot(g[0]-bx,g[2]-bz) for g in rows]
        seg=lambda a,b: d[int(a*hz):int(b*hz)]
        rms=lambda s: math.sqrt(sum(v*v for v in s)/len(s)) if s else float('nan')
        out.append((th, max(seg(0,move_s) or [0]), rms(seg(move_s, move_s+3)), rms(seg(move_s+3, 12))))
    return out
if __name__=='__main__':
    f=sys.argv[1]; move_s=float(sys.argv[2]) if len(sys.argv)>2 else 3.0
    print(f"{f.split('/')[-1]}  (move window {move_s:g} s)")
    print(f"  {'theta':>5s} {'peak move':>10s} {'RMS +0..3s':>11s} {'RMS +3s..end':>13s}")
    for th,pk,r1,r2 in analyze(f, move_s): print(f"  {th:5.0f} {pk:10.3f} {r1:11.4f} {r2:13.4f}")
