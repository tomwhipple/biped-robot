"""PLAY the range of motion inside the FreeCAD 3D view -- one leg at a time.

Run:  open cad/step/bimo_v3yaw_articulated.FCStd in the GUI, then
      Macro -> Macros... -> freecad_rom_play.py -> Execute
      (or paste into the Python console: exec(open(...).read()))

User, 2026-07-30: "I want to see it move *within* CAD." cad/renders/joint_rom.mov
is a rendered video; this drives the real assembly in the real viewport, so you
can orbit, zoom and stop it mid-sweep.

Each joint runs neutral -> max -> min -> neutral while the OTHER leg holds
neutral as a reference, left leg first, then right -- the same sequence the
video uses, at the same ROM limits (dimensions.ROM).

WHY IT POSES BODIES AND NOT JOINTS
----------------------------------
A plain Revolute joint has no driving angle in FreeCAD 1.1: its `Angle` field
belongs to the Angle joint type and is INERT for a Revolute. Worse, WRITING it
drops every joint out of `Assembly.Joints` for the rest of the session (the
assembly then reports zero joints and solves trivially) -- you have to reopen
the file to recover. So this drives the rigid bodies' Placements straight from
the same kinematics freecad_pose.py / export_pose.py use. The joints stay
untouched and keep working for dragging afterwards.

That also means the SOLVER is not enforcing the limits here -- the kinematics
below already respect them, and check_assembly.py is what actually signs off
clearances. This is for watching, not for evidence.

PREFER cad/freecad_sim_setup.py NOW. FreeCAD's own Assembly -> Simulation gives
you a play button, a scrub slider and per-frame stepping, which this cannot.
What this still has over it: it needs no ACTIVATED assembly, so it works on an
FCStd straight out of a headless rebuild, before freecad_gui_ready.py has run.
"""
import os
import sys
import time

import FreeCAD as App

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import dimensions as D
import freecad_pose as FP

try:
    import FreeCADGui as Gui
except ImportError:                     # console mode: still poses, just blind
    Gui = None

STEPS = 8          # interpolation steps per transition
HOLD = 3           # frames parked at each extreme
FRAME_S = 0.045    # target seconds per frame


def _pose_one_leg(doc, tag, vals):
    """Place `tag`'s five bodies at `vals` = (yaw, roll, hip, knee, ankle);
    the other leg is parked at neutral so the moving one reads clearly."""
    for t, sign in (("L", 1), ("R", -1)):
        v = vals if t == tag else (0.0,) * 5
        for stem, plc in FP.stage_placements(sign * D.HIP_SEP / 2, *v).items():
            part = doc.getObject("%s_%s" % (stem, t))
            if part is not None:
                part.Placement = plc


def _smoothstep(a, b, t):
    e = t * t * (3 - 2 * t)
    return [x + (y - x) * e for x, y in zip(a, b)]


def play(doc=None, steps=STEPS, hold=HOLD, frame_s=FRAME_S, loops=1):
    doc = doc or FP._doc()
    if doc is None:
        raise RuntimeError("open cad/step/bimo_v3yaw_articulated.FCStd first")
    App.setActiveDocument(doc.Name)

    for _ in range(loops):
        for tag in ("L", "R"):
            for idx, (key, label, (lo, hi)) in enumerate(FP.JOINTS):
                cur = [0.0] * 5
                for tgt in (hi, lo, 0.0):
                    nxt = list(cur)
                    nxt[idx] = float(tgt)
                    for s in range(1, steps + 1):
                        v = _smoothstep(cur, nxt, s / steps)
                        _pose_one_leg(doc, tag, v)
                        _tick(doc, "%s %s  %+.0f deg" % (tag, label, v[idx]),
                              frame_s)
                    for _h in range(hold):
                        _tick(doc, "%s %s  %+.0f deg" % (tag, label, tgt),
                              frame_s)
                    cur = nxt
            # park this leg before handing over to the other one
            _pose_one_leg(doc, tag, (0.0,) * 5)
            _tick(doc, "", frame_s)
    print("ROM playback done -- both legs, 5 joints each")


def _tick(doc, caption, frame_s):
    doc.recompute()
    if Gui is not None:
        if caption:
            Gui.updateStatusBar(caption) if hasattr(Gui, "updateStatusBar") \
                else None
        Gui.updateGui()
    if frame_s:
        time.sleep(frame_s)


if __name__ == "__main__":
    play()
