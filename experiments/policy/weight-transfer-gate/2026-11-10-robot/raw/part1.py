import sys, re, json, math
sys.path.insert(0,'/home/claw/code/robot/experiments/policy/weight-transfer-gate')
sys.path.insert(0,'/home/claw/code/robot/experiments/one-leg-stand')
import check1_oneleg as C
import one_leg as OL
LOGD='/home/claw/code/robot/hw_sessions/2026-10-25/'
LAT_X=C.LAT_X
BUS=OL.BUS; CAL=OL.CAL
def raw_lines(path):
    out=[]
    for l in open(path,errors='replace'):
        m=re.match(r'^(\d\d:\d\d:\d\d\.\d+Z) > pose (.*)$',l.rstrip('\n'))
        if m: out.append((m.group(1),m.group(2).strip()))
    return out
def ticks_of(cmd): return [int(x) for x in cmd.split()[:17]]
def dur_of(cmd): return int(re.search(r's(\d+)',cmd).group(1))
def deg(side_ticks,base):
    d={}
    for i,n in enumerate(BUS):
        z,s=CAL[n]
        d[n]=round(s*(side_ticks[i]-base[i])*360/4096,2)
    return d
res={}
for side,name in (('R','latR'),('L','latL')):
    L=raw_lines(LOGD+name+'_serial.log')
    log=C.parse_log(LOGD+name+'_serial.log')
    assert len(L)==len(log['moves'])
    assert all(m[3] is None for m in log['moves'])
    stand=L[0]; crouch=L[1]
    base=ticks_of(stand[1])
    rungs=[]; excl=[]
    k=2; prev_shift=None; xi=0
    while k<len(L):
        t,c=L[k]; d=dur_of(c)
        if d==2000:
            x=LAT_X[xi]; xi+=1
            tk=ticks_of(c)
            # the lift = next s1500 whose ticks differ from this shift; lower = s1500 equal to it
            lift=L[k+1]; lower=L[k+2]
            assert dur_of(lift[1])==1500 and ticks_of(lower[1])==tk and dur_of(lower[1])==1500
            ldiff={BUS[i]:ticks_of(lift[1])[i]-tk[i] for i in range(12) if ticks_of(lift[1])[i]!=tk[i]}
            rungs.append(dict(mult=x,shift_mm=round(62*x,1),time=t,cmd=c,dur_ms=d,
                 returned_to='the same shift pose (lower of the lifted foot, EXCLUDED); next rung is commanded directly from it, NOT via crouch',
                 deg_from_stand=deg(tk,base),
                 deg_from_crouch=deg(tk,ticks_of(crouch[1]))))
            excl.append(dict(role='lift 5mm (excluded)',time=lift[0],cmd=lift[1],tick_diff_vs_shift=ldiff))
            excl.append(dict(role='lower (excluded)',time=lower[0],cmd=lower[1]))
            k+=3
        else:
            if k>=len(L)-2: break
            raise SystemExit(f'unexpected {k} {c}')
    tail=L[k:]
    res[side]=dict(log=name+'_serial.log',stand=dict(time=stand[0],cmd=stand[1],deg_from_stand=deg(ticks_of(stand[1]),base)),
        crouch=dict(time=crouch[0],cmd=crouch[1],deg_from_stand=deg(ticks_of(crouch[1]),base)),
        rungs=rungs,excluded=excl,tail=[dict(time=t,cmd=c) for t,c in tail])
json.dump(dict(method="classified by duration and tick diffs: s2000 = shift; following s1500 pose differing from it in R/L hip_pitch,knee,ankle (the foot-lift IK) = lift; next s1500 identical to the shift = lower. NOTES.md ladder 0.5..1.4x of 62 mm; B<L|R> token absent in both logs (0 matches); 'balance' commands are status reads (state off). Degrees = dir*dticks*360/4096 using one_leg.CAL (sign = joint positive direction per CAL dir).",
  bus=BUS,cal=CAL,sides=res),open('shift_ladder.json','w'),indent=1)
# md
o=["# Lateral shift ladder as sent 2026-10-25 (both-feet-down poses only)\n",
"Source: hw_sessions/2026-10-25/latR_serial.log, latL_serial.log. R ladder = pelvis toward the robot's RIGHT foot. Pose strings verbatim (17 ticks + s<ms>).\n",
"Identification: s2000 moves are the shifts (7 per side, matching NOTES' 7 multipliers 0.5,0.75,1.0,1.1,1.2,1.3,1.4 x 62 mm); each is followed by an s1500 pose that differs from it only in R/L hip_pitch, knee, ankle (5 mm foot lift, EXCLUDED) and an s1500 pose byte-identical to the shift (lower, EXCLUDED). No `B<L|R>` token anywhere in either log (grep count 0): no balance loop. Sequence is stand s3000 -> crouch s3000 -> [shift s2000, lift, lower] x7 -> crouch s3000 -> stand s3000 -> home. Rungs are commanded rung-to-rung (shift k -> lift -> lower to shift k -> shift k+1), NOT returning to crouch between rungs.\n",
"Degrees: dir*dticks*360/4096 from one_leg.CAL; sign is the joint-positive direction, so just check magnitudes. Pitch chain = hip_pitch/knee/ankle.\n"]
short=lambda d:' '.join(f"{k.replace('_hip_','_h').replace('_ankle_roll','_ar').replace('_ankle','_a').replace('_knee','_k')}={v:+.1f}" for k,v in d.items() if k in ('R_hip_roll','L_hip_roll','R_ankle_roll','L_ankle_roll','R_hip_pitch','R_knee','R_ankle','L_hip_pitch','L_knee','L_ankle'))
for side in 'RL':
    r=res[side]
    o.append(f"\n## {side} ladder ({r['log']})\n")
    o.append(f"- stand ({r['stand']['time']}): `pose {r['stand']['cmd']}`")
    o.append(f"- crouch ({r['crouch']['time']}): `pose {r['crouch']['cmd']}`\n  - from stand: {short(r['crouch']['deg_from_stand'])}")
    for i,g in enumerate(r['rungs']):
        o.append(f"- rung {i+1}: x{g['mult']} = {g['shift_mm']} mm ({g['time']}): `pose {g['cmd']}`\n  - from stand (deg): {short(g['deg_from_stand'])}\n  - from crouch (deg): {short(g['deg_from_crouch'])}")
    o.append(f"- excluded per rung: lift 5 mm + lower (see json `excluded`); tail: "+'; '.join(f"{t['time']} `{t['cmd'][:30]}...`" for t in r['tail']))
open('shift_ladder.md','w').write('\n'.join(o)+'\n')
