# -*- coding: utf-8 -*-
"""Sweep every joint through its FULL ROM and check that no two moving bodies
ever pass through each other.

Run headless (no GUI, no 90 s dispatch limit):
    /Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd cad/freecad_rom_collide.py

Exit status is the point: 0 = every pair stayed clear, 1 = something interfered.
That makes this a CHECK, not a report you have to read. cad/run_checks.sh runs
it next to check_assembly.py. Pass a sample count to trade runtime for
resolution:  freecadcmd cad/freecad_rom_collide.py 25

User, 2026-07-30, on watching cad/freecad_rom_play.py: *"I'd hope that the parts
wouldn't move through one and other."* The player only POSES geometry -- it
proves nothing about interference. check_assembly.py does check clearances, but
at ROM EXTREMES and a handful of combined poses; a sweep can foul somewhere in
the middle and pass both endpoints. This walks the whole travel.

What it reports per pair: the worst (smallest) gap over the sweep, and any
actual overlap volume. Pairs are freecad_pose.PAIRS -- the bodies that move
relative to each other -- plus torso-vs-carrier and carrier-vs-carrier.

Servo mocks are excluded, same as freecad_pose.world_shape: the horn/idler disc
faces touch their cases BY DESIGN, so including them reports a permanent
"collision" that means nothing. Printed parts and fasteners are what get signed
off.
"""
import os
import sys

import FreeCAD as App

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import dimensions as D
import freecad_pose as FP

DOC = os.path.join(HERE, "step", "bimo_v3yaw_articulated.FCStd")
STEPS = 13          # samples across each joint's travel, endpoints included
for _a in sys.argv[1:]:                 # freecadcmd forwards trailing args
    if _a.isdigit():
        STEPS = max(3, int(_a))


# Shapes are extracted ONCE at neutral and then TRANSFORMED per pose. Setting
# Placements and calling doc.recompute() instead would re-run the Assembly
# solver (MbD) on every sample -- ~130 solves, minutes of wall clock, for
# results the solver has no say in anyway (we are driving the bodies, not
# asking the solver where they go).
def _neutral_shapes(doc):
    for t, sign in (("L", 1), ("R", -1)):
        for stem, plc in FP.stage_placements(sign * D.HIP_SEP / 2,
                                             *(0.0,) * 5).items():
            p = doc.getObject("%s_%s" % (stem, t))
            if p is not None:
                p.Placement = plc
    doc.recompute()
    out = {}
    for tag in ("L", "R"):
        for stem in ("yaw", "roll", "thigh", "shin", "foot"):
            p = doc.getObject("%s_%s" % (stem, tag))
            if p is not None:
                out[(stem, tag)] = FP.world_shape(p)
    t = doc.getObject("torso")
    if t is not None:
        out[("torso", "")] = FP.world_shape(t)
    return out


def _posed(neutral, tag, vals):
    """Neutral shapes moved to the pose -- rigid transforms, no solver."""
    out = dict(neutral)
    for t, sign in (("L", 1), ("R", -1)):
        v = vals if t == tag else (0.0,) * 5
        for stem, plc in FP.stage_placements(sign * D.HIP_SEP / 2, *v).items():
            key = (stem, t)
            if key in neutral:
                out[key] = neutral[key].transformed(plc.toMatrix())
    return out


def main():
    doc = App.openDocument(DOC)
    neutral = _neutral_shapes(doc)
    worst = {}          # label -> (gap, volume, pose description)

    for tag in ("L", "R"):
        pairs = [((a, tag), (b, tag), "%s vs %s (%s)" % (a, b, tag))
                 for a, b in FP.PAIRS]
        pairs.append((("torso", ""), ("yaw", tag), "torso vs yaw (%s)" % tag))
        for idx, (key, label, (lo, hi)) in enumerate(FP.JOINTS):
            for s in range(STEPS):
                ang = lo + (hi - lo) * s / (STEPS - 1)
                vals = [0.0] * 5
                vals[idx] = ang
                bodies = _posed(neutral, tag, vals)
                for ka, kb, plabel in pairs:
                    sa, sb = bodies.get(ka), bodies.get(kb)
                    if sa is None or sb is None:
                        continue
                    vol = sa.common(sb).Volume
                    gap = None if vol > FP.MIN_VOL else sa.distToShape(sb)[0]
                    where = "%s %s %+.0f" % (tag, label, ang)
                    prev = worst.get(plabel)
                    # rank: any overlap beats any gap; else smallest gap wins
                    cur = (-(vol or 0), gap if gap is not None else -1)
                    if prev is None or cur < (-(prev[1] or 0),
                                              prev[0] if prev[0] is not None
                                              else -1):
                        worst[plabel] = (gap, vol if vol > FP.MIN_VOL else None,
                                         where)
            print("  swept %s %s" % (tag, label))

    print("\n=== ROM SWEEP INTERFERENCE (%d samples per joint) ===" % STEPS)
    bad = 0
    for label in sorted(worst, key=lambda k: (worst[k][1] is None,
                                              worst[k][0] if worst[k][0]
                                              is not None else -1)):
        gap, vol, where = worst[label]
        if vol:
            bad += 1
            print("  %-26s ** OVERLAP %9.1f mm3 ** worst at %s"
                  % (label, vol, where))
        else:
            flag = "  << under the %.1f mm buffer" % D.SWEEP_BUFFER \
                if gap < D.SWEEP_BUFFER else ""
            print("  %-26s min gap %6.2f mm  at %s%s"
                  % (label, gap, where, flag))
    print("\n%s" % ("ALL CLEAR -- no body passes through another over the full ROM"
                    if bad == 0 else "** %d PAIR(S) INTERFERE **" % bad))
    return bad == 0


# freecadcmd exec's this file, so __name__ is not reliably "__main__" -- call
# unconditionally and exit with the verdict so a shell can branch on it.
#
# FLUSH FIRST. freecadcmd tears the interpreter down on SystemExit without
# flushing Python's stdout, so exiting straight out of main() threw away the
# entire report and left only the C++ banner -- a check that printed nothing and
# said PASS. Found the hard way, 2026-07-30.
_ok = main()
sys.stdout.flush()
sys.stderr.flush()
sys.exit(0 if _ok else 1)
