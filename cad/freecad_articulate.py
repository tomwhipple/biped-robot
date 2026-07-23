# -*- coding: utf-8 -*-
"""Build a POSEABLE FreeCAD assembly of the v3yaw biped (10 DOF).

Run either way:
  * GUI:  Macro -> Macros... -> add this file -> Execute
          (or drag it into the Python console).  Needs FreeCAD >= 1.0 with the
          built-in **Assembly** workbench.
  * CLI:  /Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd \
              cad/freecad_articulate.py
          (Windows/Linux: freecadcmd cad/freecad_articulate.py)

What it makes
-------------
An `Assembly` document whose printed parts are grouped into the six rigid
bodies of one leg (torso + yaw carrier + roll yoke + thigh + shin + foot),
wired together by one **Revolute** joint per axis, each with the sim's
rotation limits baked in (EnableAngleMin/Max):

    hip yaw  +/-45     hip roll +/-25     hip pitch -110..+60
    knee     -95..+5   ankle    +/-40

The torso is grounded; both legs are built from the same part STEPs
(`cad/step/*.step`, re-exported by `export_step.py`) placed at the world
positions in `cad/dimensions.py`.  Servo bodies are intentionally omitted --
the printed skeleton is what you pose to check range of motion.

    -> cad/step/bimo_v3yaw_articulated.FCStd

How to POSE it
--------------
Open the .FCStd in the FreeCAD GUI, switch to the **Assembly** workbench, then
either (a) drag any part with the mouse -- the solver keeps every joint honest
and stops each axis at its limit -- or (b) double-click a joint in the tree and
type an angle.  Ground stays put; the chain follows.

Robustness notes
----------------
* STEP files are found by name (case-insensitive) under ``step/`` next to this
  macro -- no absolute paths, so it travels with the repo.
* Each joint's coordinate system is an ``App::LocalCoordinateSystem`` placed
  exactly on the real axis (from ``dimensions.py``), so nothing depends on
  fragile auto-generated ``EdgeNN`` names.
* Left/right legs are mirrored by lateral offset only (the design is
  translations, not mirrors -- same parts both sides).
* On open, FreeCAD may print ~20 yellow "joint ... has an invalid Reference"
  console warnings.  These are benign: they come from a PartDesign-oriented
  migration pass that doesn't apply to LCS-based joints -- the joints solve and
  pose correctly regardless (verified: neutral pose = standing, each axis
  articulates its chain within limits).
"""
import os
import sys

import FreeCAD as App
import Part
import JointObject  # from the Assembly workbench (Mod/Assembly)

# --------------------------------------------------------------------------- paths
try:
    HERE = os.path.dirname(os.path.abspath(__file__))
except NameError:                       # exec'd in the console: fall back to cwd
    HERE = os.path.abspath(".")
    if not os.path.isdir(os.path.join(HERE, "step")):
        HERE = os.path.dirname(App.ActiveDocument.FileName) if App.ActiveDocument else HERE
STEP = os.path.join(HERE, "step")
sys.path.insert(0, HERE)
import dimensions as D                  # pure-python constants, safe under FreeCAD

REVOLUTE = JointObject.JointTypes.index("Revolute")   # == 1

# JCS orientation: a Revolute joint spins about its coordinate system's LOCAL Z.
# Build a rotation that points local Z along each real axis.
ROT_Z = App.Rotation()                              # Z stays vertical  (hip yaw)
ROT_X = App.Rotation(App.Vector(0, 1, 0), 90)       # local Z -> world X (hip roll)
ROT_Y = App.Rotation(App.Vector(1, 0, 0), -90)      # local Z -> world Y (pitch/knee/ankle)

# world heights (mm) -- the same arithmetic export_assembly.py uses for v3yaw
DECK_TOP_Z = D.HIP_ROLL_Z + D.ROLL_BELOW_DECK_YAW   # 327.77, pelvis/tower local z=0
TOWER_TOP_Z = DECK_TOP_Z + D.TOWER_H


# --------------------------------------------------------------------------- helpers
def step_path(name):
    """Case-insensitive '<name>.step' lookup under step/."""
    want = name.lower() + ".step"
    for f in os.listdir(STEP):
        if f.lower() == want:
            return os.path.join(STEP, f)
    raise FileNotFoundError(
        "%s.step not found in %s -- run export_step.py first" % (name, STEP))


def read_shape(name):
    shp = Part.Shape()
    shp.read(step_path(name))
    return shp


def body(doc, asm, name, members):
    """One rigid body = an App::Part (identity placement) holding each printed
    solid at its world placement. `members` = [(step_name, App.Placement), ...].
    The whole App::Part is what the joint solver moves."""
    part = doc.addObject("App::Part", name)
    for step_name, plc in members:
        feat = doc.addObject("Part::Feature", "%s__%s" % (name, step_name))
        feat.Shape = read_shape(step_name)
        feat.Placement = plc
        part.addObject(feat)
    asm.addObject(part)
    return part


def jcs(doc, part, world_pos, rot, tag):
    """Local coordinate system on `part` at a world axis point/orientation.
    part.Placement is identity, so the LCS local placement == world placement."""
    lcs = doc.addObject("App::LocalCoordinateSystem", "%s_%s" % (part.Name, tag))
    part.addObject(lcs)
    lcs.Placement = App.Placement(world_pos, rot)
    return lcs


def revolute(doc, jg, name, a_part, a_lcs, b_part, b_lcs, amin, amax):
    """Revolute joint coinciding two LCS, with rotation limits (deg)."""
    j = jg.newObject("App::FeaturePython", name)
    JointObject.Joint(j, REVOLUTE)
    # Reference = [moving_part, ["<LCS name>.", ""]].  UtilsAssembly.getObject
    # prepends the part name, so "<LCS name>." resolves to the LCS itself; the
    # second (vertex) sub is empty, which makes findPlacement treat the whole
    # LCS frame as the joint coordinate system.  BOTH strings are required --
    # the file's migration pass reads ref[1][1] on reload.
    j.Reference1 = [a_part, ["%s." % a_lcs.Name, ""]]
    j.Reference2 = [b_part, ["%s." % b_lcs.Name, ""]]
    j.EnableAngleMin, j.AngleMin = True, float(amin)
    j.EnableAngleMax, j.AngleMax = True, float(amax)
    return j


# --------------------------------------------------------------------------- build
def build():
    doc = App.newDocument("bimo_v3yaw_articulated")
    asm = doc.addObject("Assembly::AssemblyObject", "Assembly")

    P = App.Placement
    V = App.Vector
    I = App.Rotation()

    # ---- torso (grounded): all the deck-up printed parts ----
    torso = body(doc, asm, "torso", [
        ("pelvis",      P(V(0, 0, DECK_TOP_Z), I)),
        ("tower",       P(V(0, 0, DECK_TOP_Z), I)),
        ("imu_carrier", P(V(0, 0, TOWER_TOP_Z), I)),
        ("gopro_base",  P(V(0, 0, TOWER_TOP_Z + D.IMU_CARRIER_T), I)),
    ])

    jg = doc.addObject("Assembly::JointGroup", "Joints")
    asm.addObject(jg)

    # ground the torso so the chain has a fixed base
    gj = jg.newObject("App::FeaturePython", "Ground_torso")
    JointObject.GroundedJoint(gj, torso)

    # ---- one leg, mirrored by lateral offset y ----
    def leg(tag, y):
        yaw = body(doc, asm, "yaw_%s" % tag,
                   [("yaw_carrier", P(V(0, y, D.HIP_YAW_Z), I))])
        roll = body(doc, asm, "roll_%s" % tag, [
            ("yoke_roll",  P(V(0, y, D.HIP_ROLL_Z), I)),
            ("yoke_pitch", P(V(0, y, D.HIP_PITCH_Z), I)),
        ])
        thigh = body(doc, asm, "thigh_%s" % tag,
                     [("leg_link", P(V(0, y, D.HIP_PITCH_Z), I))])
        shin = body(doc, asm, "shin_%s" % tag,
                    [("leg_link", P(V(0, y, D.KNEE_Z), I))])
        foot = body(doc, asm, "foot_%s" % tag,
                    [("foot", P(V(0, y, D.TPU_PROUD), I))])

        # joint coordinate systems, one pair per axis (same world axis on both
        # bodies -> pre-coincident, so the assembled/neutral pose is preserved)
        t_yaw = jcs(doc, torso, V(0, y, D.HIP_YAW_Z),   ROT_Z, "yaw_%s" % tag)
        y_yaw = jcs(doc, yaw,   V(0, y, D.HIP_YAW_Z),   ROT_Z, "yaw")
        y_rol = jcs(doc, yaw,   V(0, y, D.HIP_ROLL_Z),  ROT_X, "roll")
        r_rol = jcs(doc, roll,  V(0, y, D.HIP_ROLL_Z),  ROT_X, "roll")
        r_pit = jcs(doc, roll,  V(0, y, D.HIP_PITCH_Z), ROT_Y, "pitch")
        t_pit = jcs(doc, thigh, V(0, y, D.HIP_PITCH_Z), ROT_Y, "pitch")
        t_kne = jcs(doc, thigh, V(0, y, D.KNEE_Z),      ROT_Y, "knee")
        s_kne = jcs(doc, shin,  V(0, y, D.KNEE_Z),      ROT_Y, "knee")
        s_ank = jcs(doc, shin,  V(0, y, D.ANKLE_Z),     ROT_Y, "ankle")
        f_ank = jcs(doc, foot,  V(0, y, D.ANKLE_Z),     ROT_Y, "ankle")

        revolute(doc, jg, "hip_yaw_%s" % tag,   torso, t_yaw, yaw,   y_yaw, -D.YAW_SWEEP, D.YAW_SWEEP)
        revolute(doc, jg, "hip_roll_%s" % tag,  yaw,   y_rol, roll,  r_rol, -25, 25)
        revolute(doc, jg, "hip_pitch_%s" % tag, roll,  r_pit, thigh, t_pit, -110, 60)
        revolute(doc, jg, "knee_%s" % tag,      thigh, t_kne, shin,  s_kne, -95, 5)
        revolute(doc, jg, "ankle_%s" % tag,     shin,  s_ank, foot,  f_ank, -40, 40)

    leg("L", D.HIP_SEP / 2)
    leg("R", -D.HIP_SEP / 2)

    doc.recompute()
    try:
        asm.solve()
        print("solve: OK")
    except Exception as exc:                       # noqa: BLE001 -- report, don't crash
        print("solve warning:", repr(exc))

    out = os.path.join(STEP, "bimo_v3yaw_articulated.FCStd")
    doc.saveAs(out)
    n_joints = sum(1 for o in doc.Objects if o.Name.startswith(
        ("hip_", "knee_", "ankle_")))
    print("wrote %s  (%d revolute joints)" % (out, n_joints))
    return doc


# Runs top-to-bottom whether launched by freecadcmd or the GUI Macro menu.
build()
