import csv, collections, re, sys
hw=collections.OrderedDict()
for r in csv.DictReader(open('hw_sessions/2026-09-02/play_sweep4.csv')): hw.setdefault(r['joint'],[]).append((float(r['cmd_deg']), float(r['tilt_deg'])))
sim={}
lines=open('hw_sessions/2026-09-02/sim_ref/sim_sweep.txt').read().splitlines()
for i,l in enumerate(lines):
    m=re.match(r'(\w+)\s+tilt max',l)
    if m: sim[m.group(1)]=[(float(a),float(b)) for a,b in re.findall(r'([+-]\d\.\d):(\d+\.\d)', lines[i+1])]
def hyst(tr):
    """max |tilt(down) - tilt(up)| at the same command, and the reversal
    travel: how far the command came back from +4 before tilt dropped by
    more than 0.3 deg from its value at +4."""
    up={c:t for c,t in tr[:9]}; dn={c:t for c,t in tr[8:17]}
    h=max(abs(dn[c]-up[c]) for c in up if c in dn)
    t4=dn[4.0]; rel=next((4.0-c for c,t in tr[8:17] if t < t4-0.3), None)
    return h, rel
print(f"{'joint':12s} {'hw +4':>6s} {'sim +4':>6s} {'hw -4':>6s} {'sim -4':>6s} {'hw hyst':>8s} {'sim hyst':>8s} {'reversal travel hw':>18s}")
for j in hw:
    if j not in sim: continue
    a,b=hw[j],sim[j]; ha,ra=hyst(a); hb,rb=hyst(b)
    def at(tr,c,which):
        seg = tr[:9] if which=='up' else tr[16:25]
        return next((t for cc,t in seg if cc==c), float('nan'))
    print(f"{j:12s} {at(a,4.0,'up'):6.1f} {at(b,4.0,'up'):6.1f} {at(a,-4.0,'dn'):6.1f} {at(b,-4.0,'dn'):6.1f} {ha:8.1f} {hb:8.1f} {('%.1f deg' % ra) if ra else 'none':>18s}")
