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
(`cad/step/*.step`, re-exported by `parts.py`) placed at the world positions in
`cad/dimensions.py`.  Since 2026-07-28 every body also carries its MODELED
FASTENERS (`step/screws_*.step`, from `fasteners.py`) -- without them the file
poses a skeleton that can't collide the way the hardware does -- and its SERVO
MOCKS (`step/servo_*.step`, from `servos.py`), so the file shows the machine
rather than a printed-parts skeleton.

The servos ride the same bodies `dress.py` hangs them on, so they pose with the
chain.  They are context, NOT collision evidence: the disc faces touch the
cases by design, so `freecad_pose.py` skips any child named `*__servo_*` when
it intersects bodies.  Add a servo to a body here and it stays exempt.

    -> cad/step/bimo_v3yaw_articulated.FCStd

How to POSE it
--------------
Open the .FCStd in the FreeCAD GUI, switch to the **Assembly** workbench, then
either

  (a) DOUBLE-CLICK "Assembly" in the tree to ACTIVATE it, then DRAG a part with
      the mouse -- the solver keeps every joint honest and stops each axis at
      its ROM limit.  Ground stays put, the chain follows; Esc/undo puts it
      back.  Activating is not optional: with no active assembly the drag does
      nothing at all, and activation only works once freecad_gui_ready.py has
      run on the file (see below).
  (b) Assembly -> Simulation (needs the assembly active): give a joint a
      formula like `-1.2*time`, generate, and play it back -- FreeCAD's own
      motion feature, and the only built-in way to WATCH a joint sweep.
  (c) run `cad/freecad_pose.py` (Macro -> Macros...) for a slider panel: five
      sliders drive both legs to exact angles, and "Check collisions" paints
      any interference solid RED in the tree.  This is the repeatable way --
      dragging is for feel, sliders are for evidence.

Note that a plain Revolute joint has NO driving angle property in FreeCAD 1.1:
its `Angle` field exists but belongs to the Angle joint type and is INERT for a
Revolute -- which is why (c) sets the rigid bodies' placements from the same
kinematics `export_pose.py` uses rather than asking the solver for an angle.
Worse, WRITING that field on a Revolute joint drops every joint out of
`Assembly.Joints` for the rest of the session (the assembly then reports zero
joints and solves trivially); reopen the document to recover.

The file opens VISIBLE with a fitted isometric view: because console-mode
FreeCAD writes no GuiDocument.xml, this macro injects one after saving (a
ViewProvider per object -- printed parts + containers shown, the auto-generated
Origin datum planes hidden -- plus a saved camera).  If a future FreeCAD ever
still opens with hidden parts: select-all in the tree, press Space, View > Fit All.

!! AFTER EVERY HEADLESS REBUILD, RUN `cad/freecad_gui_ready.py` IN THE GUI !!
The injected ViewProviders carry only `Visibility`, so each joint's
ViewObject.Proxy is the placeholder integer 1 rather than a real
Assembly.JointObject.ViewProviderJoint.  The joints and the solver are fine
either way, but ACTIVATING the assembly (double-clicking it -- the prerequisite
for dragging a link and for the Simulation command) calls
`redrawJointPlacements` on those view providers and dies with
`AttributeError: 'int' object has no attribute 'redrawJointPlacements'`.
freecad_gui_ready.py attaches the real ones and re-saves; it needs FreeCADGui,
so it cannot be folded into this console-mode build.

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
import zipfile

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

# One build only: the pan-head variant was retired 2026-07-28 when the machine
# standardised on M2.5 flat-head self-tappers for every servo-case screw.
GRIP = "screws_grip"
DOC_NAME = "bimo_v3yaw_articulated"

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


def add_member(doc, part, step_name, plc):
    """Put one STEP solid into an existing rigid body at a world placement.
    The `<body>__<step>` name is what freecad_pose.py reads to tell printed
    parts and fasteners (collision evidence) from servo mocks (context)."""
    feat = doc.addObject("Part::Feature", "%s__%s" % (part.Name, step_name))
    feat.Shape = read_shape(step_name)
    feat.Placement = plc
    part.addObject(feat)
    return feat


def body(doc, asm, name, members):
    """One rigid body = an App::Part (identity placement) holding each printed
    solid at its world placement. `members` = [(step_name, App.Placement), ...].
    The whole App::Part is what the joint solver moves."""
    part = doc.addObject("App::Part", name)
    for step_name, plc in members:
        add_member(doc, part, step_name, plc)
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


# ------------------------------------------------------ GUI visibility injection
# freecadcmd (console mode) writes NO GuiDocument.xml, so the GUI opens the file
# with every object hidden and no fitted camera -- it "looks empty".  We inject a
# minimal-but-valid GuiDocument.xml (FreeCAD 1.1 schema) after saving: a
# ViewProvider per persisted object, geometry + containers visible, the auto-made
# Origin datum planes/axes/points hidden, plus a fitted isometric camera.

# Objects that must be visible; App::Part / Assembly / JointGroup are CONTAINERS
# whose visibility gates their children, so they must be on too.
_VISIBLE_TYPES = {
    "Part::Feature",
    "App::Part",
    "Assembly::AssemblyObject",
    "Assembly::JointGroup",
}


def _isometric_camera(center, dist=1300.0, height=560.0):
    """Coin OrthographicCamera settings string for a fitted isometric view."""
    n = App.Vector(1, -1, 1)
    n.normalize()                                    # camera offset from focal
    up = App.Vector(0, 0, 1)
    zc = n                                           # camera looks down its -Z
    yc = up - zc * up.dot(zc)
    yc.normalize()
    xc = yc.cross(zc)
    rot = App.Rotation(App.Matrix(
        xc.x, yc.x, zc.x, 0,
        xc.y, yc.y, zc.y, 0,
        xc.z, yc.z, zc.z, 0,
        0, 0, 0, 1))
    ax, ang = rot.Axis, rot.Angle
    pos = center + n * dist
    near, far = dist - 600.0, dist + 600.0
    body = (
        "OrthographicCamera {\n"
        "  viewportMapping ADJUST_CAMERA\n"
        "  position %.5f %.5f %.5f\n"
        "  orientation %.8f %.8f %.8f  %.7f\n"
        "  nearDistance %.5f\n"
        "  farDistance %.5f\n"
        "  aspectRatio 1\n"
        "  focalDistance %.5f\n"
        "  height %.5f\n\n}\n" % (
            pos.x, pos.y, pos.z, ax.x, ax.y, ax.z, ang,
            near, far, dist, height))
    return body.replace("&", "&amp;").replace("<", "&lt;").replace(
        ">", "&gt;").replace('"', "&quot;").replace("\n", "&#10;")


def _gui_document_xml(objects, camera_settings):
    """objects = [(name, typeid), ...]."""
    rows = []
    for name, typeid in objects:
        vis = "true" if typeid in _VISIBLE_TYPES else "false"
        rows.append(
            '        <ViewProvider name="%s" expanded="0">\n'
            '            <Properties Count="1" TransientCount="0">\n'
            '                <Property name="Visibility" type="App::PropertyBool" status="1">\n'
            '                    <Bool value="%s"/>\n'
            '                </Property>\n'
            '            </Properties>\n'
            '        </ViewProvider>' % (name, vis))
    return (
        "<?xml version='1.0' encoding='utf-8'?>\n"
        "<!-- FreeCAD GuiDocument, injected by freecad_articulate.py -->\n"
        '<Document SchemaVersion="1">\n'
        '    <ViewProviderData Count="%d">\n%s\n    </ViewProviderData>\n'
        '    <Camera settings="%s"/>\n'
        "</Document>\n" % (len(objects), "\n".join(rows), camera_settings))


def inject_gui_document(fcstd_path, doc):
    """Add/replace GuiDocument.xml inside the saved .FCStd zip (idempotent)."""
    objects = [(o.Name, o.TypeId) for o in doc.Objects]
    # centre the camera on the standing model (feet ~0, tower top ~400)
    cam = _isometric_camera(App.Vector(0.0, 0.0, 200.0))
    xml = _gui_document_xml(objects, cam)
    tmp = fcstd_path + ".tmp"
    with zipfile.ZipFile(fcstd_path, "r") as zin, \
            zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as zout:
        for item in zin.infolist():
            if item.filename == "GuiDocument.xml":      # drop old -> idempotent
                continue
            zout.writestr(item, zin.read(item.filename))
        zout.writestr("GuiDocument.xml", xml)
    os.replace(tmp, fcstd_path)
    n_vis = sum(1 for _, t in objects if t in _VISIBLE_TYPES)
    print("injected GuiDocument.xml: %d ViewProviders (%d visible)"
          % (len(objects), n_vis))


# --------------------------------------------------------------------------- build
def build():
    doc = App.newDocument(DOC_NAME)
    asm = doc.addObject("Assembly::AssemblyObject", "Assembly")

    P = App.Placement
    V = App.Vector
    I = App.Rotation()

    # ---- torso (grounded): all the deck-up printed parts + their screws ----
    torso = body(doc, asm, "torso", [
        ("pelvis",      P(V(0, 0, DECK_TOP_Z), I)),
        ("tower",       P(V(0, 0, DECK_TOP_Z), I)),
        ("imu_carrier", P(V(0, 0, TOWER_TOP_Z), I)),
        ("gopro_base",  P(V(0, 0, TOWER_TOP_Z + D.IMU_CARRIER_T), I)),
        ("screws_deck",       P(V(0, 0, DECK_TOP_Z), I)),
        ("screws_tower",      P(V(0, 0, DECK_TOP_Z), I)),
        ("screws_head_stack", P(V(0, 0, TOWER_TOP_Z), I)),
    ])

    jg = doc.addObject("Assembly::JointGroup", "Joints")
    asm.addObject(jg)

    # ground the torso so the chain has a fixed base
    gj = jg.newObject("App::FeaturePython", "Ground_torso")
    JointObject.GroundedJoint(gj, torso)

    # ---- one leg, mirrored by lateral offset y ----
    def leg(tag, y):
        # each body carries the fasteners that RIDE it (same grouping as
        # export_pose.py's stages): the horn-side discs move with the driven
        # link, the idler-side hardware stays with the frame that holds it.
        yaw = body(doc, asm, "yaw_%s" % tag, [
            ("yaw_carrier",        P(V(0, y, D.HIP_YAW_Z), I)),
            ("screws_yaw_carrier", P(V(0, y, D.HIP_YAW_Z), I)),
        ])
        roll = body(doc, asm, "roll_%s" % tag, [
            ("yoke_roll",     P(V(0, y, D.HIP_ROLL_Z), I)),
            ("screws_roll",   P(V(0, y, D.HIP_ROLL_Z), I)),
            ("yoke_pitch",    P(V(0, y, D.HIP_PITCH_Z), I)),
            ("screws_disc_y", P(V(0, y, D.HIP_PITCH_Z), I)),
        ])
        thigh = body(doc, asm, "thigh_%s" % tag, [
            ("leg_link",      P(V(0, y, D.HIP_PITCH_Z), I)),
            (GRIP,            P(V(0, y, D.HIP_PITCH_Z), I)),
            ("screws_disc_y", P(V(0, y, D.KNEE_Z), I)),
        ])
        shin = body(doc, asm, "shin_%s" % tag, [
            ("leg_link",      P(V(0, y, D.KNEE_Z), I)),
            (GRIP,            P(V(0, y, D.KNEE_Z), I)),
            ("screws_disc_y", P(V(0, y, D.ANKLE_Z), I)),
        ])
        foot = body(doc, asm, "foot_%s" % tag, [
            ("foot",        P(V(0, y, D.TPU_PROUD), I)),
            ("screws_foot", P(V(0, y, D.TPU_PROUD), I)),
        ])

        # servo mocks -- same body and same joint-local frame as dress.py, so
        # each case rides the link it is bolted to and poses with the chain.
        # The yaw servo is torso-fixed (its horn drives the carrier below it).
        add_member(doc, torso, "servo_yaw",   P(V(0, y, D.HIP_YAW_Z), I))
        add_member(doc, yaw,   "servo_roll",  P(V(0, y, D.HIP_ROLL_Z), I))
        add_member(doc, thigh, "servo_pitch", P(V(0, y, D.HIP_PITCH_Z), I))
        add_member(doc, shin,  "servo_pitch", P(V(0, y, D.KNEE_Z), I))
        add_member(doc, foot,  "servo_ankle", P(V(0, y, D.ANKLE_Z), I))

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

    out = os.path.join(STEP, DOC_NAME + ".FCStd")
    doc.saveAs(out)
    # console mode saves no GuiDocument.xml -> inject one so it opens VISIBLE
    inject_gui_document(out, doc)
    n_joints = sum(1 for o in doc.Objects if o.Name.startswith(
        ("hip_", "knee_", "ankle_")))
    print("wrote %s  (%d revolute joints)" % (out, n_joints))
    return doc


# Runs top-to-bottom whether launched by freecadcmd or the GUI Macro menu.
build()
