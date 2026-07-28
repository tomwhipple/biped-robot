# -*- coding: utf-8 -*-
"""Pose the articulated biped in FreeCAD -- sliders + collision highlight.

Open `cad/step/bimo_v3yaw_articulated.FCStd` (built by freecad_articulate.py),
then run this from Macro -> Macros... -> Execute. A small panel appears:

    hip yaw   -45 ....|.... +45          [ Check collisions ]
    hip roll  -25 ....|.... +25          [ Reset to neutral ]
    hip pitch -110 ...|.... +60          [x] mirror both legs
    knee       -95 ...|.... +5
    ankle     -40 ....|.... +40

Dragging a slider re-poses BOTH legs live (the sliders drive the rigid bodies
directly with the same kinematics `export_pose.py` uses -- a FreeCAD Revolute
joint has no driving angle property, so the solver can't be asked for "-60 deg
of hip"). Dragging a PART with the mouse still works and still obeys the joint
limits; the sliders are just the repeatable way to hit an exact pose.

"Check collisions" boolean-intersects every relatively-moving pair AT THE
CURRENT POSE (however you got there -- sliders or mouse) and, for each pair
that actually overlaps, drops a solid RED `COLLIDE_<a>_<b>` body into the tree
at the overlap. No red bodies == no interference. Gaps are printed to the
Report view (View -> Panels -> Report view), smallest first. The servo mocks in
the file are EXCLUDED from that math (see SERVO_MARK): they are there so the
tree looks like the machine, but the discs seat against their cases by design,
so scoring them would report a hard hit on every joint at every pose.

Headless self-test / batch use:
    BIMO_POSE="hip=-60,knee=-95" \
      /Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd cad/freecad_pose.py
    BIMO_PAN_HEADS=1 BIMO_POSE="hip=-60" freecadcmd cad/freecad_pose.py
        -> opens the matching .FCStd, poses it, prints the gap/collision table.

Reading the numbers: 0.30 mm carrier-vs-yoke is the DESIGNED bay-bore slip fit,
and 0.40 mm shin-vs-foot at ankle extremes is the locked fork/wall band. The
number that matters for the bench is yoke-vs-thigh: 0.70 mm with the flush flat
heads, a hard collision with the pan heads actually fitted (2026-07-28).
"""
import os
import sys

import FreeCAD as App
import Part

try:
    HERE = os.path.dirname(os.path.abspath(__file__))
except NameError:                       # exec'd from the console
    HERE = os.path.abspath(".")
    if not os.path.isdir(os.path.join(HERE, "step")):
        HERE = os.path.join(HERE, "cad")
sys.path.insert(0, HERE)
import dimensions as D

JOINTS = [                              # (key, label, ROM lo, ROM hi)
    ("yaw",   "hip yaw",   D.ROM["hip_yaw"]),
    ("roll",  "hip roll",  D.ROM["hip_roll"]),
    ("hip",   "hip pitch", D.ROM["hip_pitch"]),
    ("knee",  "knee",      D.ROM["knee"]),
    ("ankle", "ankle",     D.ROM["ankle"]),
]

# relatively-moving body pairs, by body-name stem (same set export_pose.py and
# check_assembly.py watch).
PAIRS = [("yaw", "roll"), ("roll", "thigh"), ("thigh", "shin"),
         ("shin", "foot"), ("roll", "shin"), ("thigh", "foot"),
         ("yaw", "thigh")]
# Servo mocks DO live in the file (freecad_articulate.py names them
# `<body>__servo_*`), but they are drawn-for-context only -- world_shape drops
# them so the printed-part clearances stay the numbers we sign off on.
SERVO_MARK = "__servo"
MIN_VOL = 0.5                           # mm3 -- below this is boolean noise
RED = (1.0, 0.05, 0.05)


# --------------------------------------------------------------- kinematics
def _rot(axis, ang, cx, cy, cz):
    """Placement rotating `ang` deg about `axis` through the point (cx,cy,cz)."""
    return App.Placement(App.Vector(0, 0, 0),
                         App.Rotation(axis, ang),
                         App.Vector(cx, cy, cz))


def stage_placements(y, yaw, roll, hip, knee, ankle):
    """World placement of each rigid body of the leg at lateral offset y."""
    X, Y, Z = App.Vector(1, 0, 0), App.Vector(0, 1, 0), App.Vector(0, 0, 1)
    Ty = _rot(Z, yaw, 0, y, D.HIP_YAW_Z)
    Tr = Ty.multiply(_rot(X, roll, 0, y, D.HIP_ROLL_Z))
    Th = Tr.multiply(_rot(Y, hip, 0, y, D.HIP_PITCH_Z))
    Tk = Th.multiply(_rot(Y, knee, 0, y, D.KNEE_Z))
    Ta = Tk.multiply(_rot(Y, ankle, 0, y, D.ANKLE_Z))
    return {"yaw": Ty, "roll": Tr, "thigh": Th, "shin": Tk, "foot": Ta}


def apply_pose(doc, angles, mirror=True):
    """Drive the rigid bodies. `angles` maps the JOINTS keys -> degrees.
    mirror=False poses the LEFT leg only (handy for seeing one side move)."""
    a = [angles.get(k, 0.0) for k, _, _ in JOINTS]
    for tag, sign in (("L", 1), ("R", -1)):
        vals = a if (mirror or tag == "L") else [0.0] * 5
        for stem, plc in stage_placements(sign * D.HIP_SEP / 2, *vals).items():
            part = doc.getObject("%s_%s" % (stem, tag))
            if part is not None:
                part.Placement = plc
    doc.recompute()


# ------------------------------------------------------------ collisions
def world_shape(part):
    """Compound of a rigid body's PRINTED+FASTENER solids at their CURRENT world
    placement. Servo mocks are skipped: the discs seat against the servo cases
    by design, so counting them would report a hard collision on every joint at
    every pose and drown the real hits."""
    shapes = []
    for obj in part.Group:
        if SERVO_MARK in obj.Name:                 # context geometry, not evidence
            continue
        shp = getattr(obj, "Shape", None)
        if shp is None or shp.isNull():
            continue
        s = shp.copy()
        s.Placement = obj.getGlobalPlacement()
        shapes.append(s)
    return Part.Compound(shapes)


def clear_collisions(doc):
    for obj in [o for o in doc.Objects if o.Name.startswith("COLLIDE_")]:
        doc.removeObject(obj.Name)


def check_collisions(doc, verbose=True):
    """Intersect every moving pair at the current pose. Returns
    [(label, volume_mm3_or_None, gap_mm_or_None), ...] sorted worst-first, and
    creates a red COLLIDE_* body for each real overlap."""
    clear_collisions(doc)
    bodies = {}
    for tag in ("L", "R"):
        for stem in ("yaw", "roll", "thigh", "shin", "foot"):
            part = doc.getObject("%s_%s" % (stem, tag))
            if part is not None:
                bodies[(stem, tag)] = world_shape(part)
    torso = doc.getObject("torso")
    if torso is not None:
        bodies[("torso", "")] = world_shape(torso)

    todo = []
    for tag in ("L", "R"):
        todo += [((a, tag), (b, tag), "%s vs %s (%s)" % (a, b, tag))
                 for a, b in PAIRS]
        todo.append((("torso", ""), ("yaw", tag), "torso vs yaw (%s)" % tag))
    todo.append((("yaw", "L"), ("yaw", "R"), "yaw L vs yaw R"))

    rows = []
    for ka, kb, label in todo:
        sa, sb = bodies.get(ka), bodies.get(kb)
        if sa is None or sb is None:
            continue
        common = sa.common(sb)
        vol = common.Volume
        if vol > MIN_VOL:
            name = "COLLIDE_%s" % label.replace(" ", "_").replace(
                "(", "").replace(")", "")
            feat = doc.addObject("Part::Feature", name)
            feat.Shape = common
            feat.Label = "COLLIDE %s" % label
            _paint(feat, RED)
            rows.append((label, vol, None))
        else:
            rows.append((label, None, sa.distToShape(sb)[0]))
    doc.recompute()

    rows.sort(key=lambda r: (-r[1] if r[1] else 1e6 + (r[2] or 0)))
    if verbose:
        hits = [r for r in rows if r[1]]
        App.Console.PrintMessage("\n--- pose check: %d collision(s) ---\n"
                                 % len(hits))
        for label, vol, gap in rows:
            if vol:
                App.Console.PrintMessage(
                    "  %-24s ** COLLISION %8.1f mm3 ** -> red body\n"
                    % (label, vol))
            else:
                App.Console.PrintMessage("  %-24s    gap %5.2f mm%s\n" % (
                    label, gap,
                    "   << under the %.1f mm buffer" % D.SWEEP_BUFFER
                    if gap < D.SWEEP_BUFFER else ""))
    return rows


def _paint(obj, rgb):
    """Colour an object if a GUI is running (no-op under freecadcmd)."""
    try:
        vo = obj.ViewObject
        if vo is None:
            return
        vo.ShapeColor = rgb
        vo.Transparency = 0
    except Exception:                                # noqa: BLE001
        pass


# ----------------------------------------------------------------- GUI panel
def show_panel(doc):
    from PySide import QtCore, QtWidgets
    import FreeCADGui as Gui

    class PosePanel(QtWidgets.QWidget):
        def __init__(self, doc):
            super(PosePanel, self).__init__()
            # Hold the NAME, not the document: closing and reopening the file
            # leaves a cached document object deleted, and every slider then
            # silently does nothing (the C++ object is gone). Re-resolve per
            # action instead -- see document().
            self.doc_name = doc.Name
            self.setWindowTitle("bimo pose")
            self.setWindowFlags(QtCore.Qt.Tool)
            grid = QtWidgets.QGridLayout(self)
            self.sliders, self.readouts = {}, {}
            for row, (key, label, (lo, hi)) in enumerate(JOINTS):
                sl = QtWidgets.QSlider(QtCore.Qt.Horizontal)
                sl.setMinimum(int(lo))
                sl.setMaximum(int(hi))
                sl.setValue(0)
                sl.setMinimumWidth(220)
                sl.valueChanged.connect(self.repose)
                out = QtWidgets.QLabel("0")
                out.setMinimumWidth(60)
                grid.addWidget(QtWidgets.QLabel(label), row, 0)
                grid.addWidget(QtWidgets.QLabel("%d" % lo), row, 1)
                grid.addWidget(sl, row, 2)
                grid.addWidget(QtWidgets.QLabel("%d" % hi), row, 3)
                grid.addWidget(out, row, 4)
                self.sliders[key], self.readouts[key] = sl, out

            self.mirror = QtWidgets.QCheckBox("mirror both legs")
            self.mirror.setChecked(True)
            self.mirror.stateChanged.connect(self.repose)
            grid.addWidget(self.mirror, len(JOINTS), 0, 1, 3)

            check = QtWidgets.QPushButton("Check collisions")
            check.clicked.connect(self.check)
            reset = QtWidgets.QPushButton("Reset to neutral")
            reset.clicked.connect(self.reset)
            grid.addWidget(check, len(JOINTS) + 1, 2, 1, 2)
            grid.addWidget(reset, len(JOINTS) + 1, 0, 1, 2)
            self.status = QtWidgets.QLabel(
                "drag a slider, then Check collisions")
            grid.addWidget(self.status, len(JOINTS) + 2, 0, 1, 5)

        def angles(self):
            return dict((k, float(s.value()))
                        for k, s in self.sliders.items())

        def document(self):
            """The live articulated document, re-resolved every time. Returns
            None (and says so) if the file was closed out from under us."""
            doc = App.listDocuments().get(self.doc_name)
            if doc is None:
                try:                           # reopened under another name?
                    doc = _doc()
                    self.doc_name = doc.Name
                except Exception:                        # noqa: BLE001
                    self.status.setText(
                        "no articulated document open -- reopen the .FCStd, "
                        "then reopen this panel")
                    return None
            return doc

        def repose(self):
            doc = self.document()
            if doc is None:
                return
            a = self.angles()
            for k, v in a.items():
                self.readouts[k].setText("%+d deg" % int(v))
            apply_pose(doc, a, mirror=self.mirror.isChecked())
            clear_collisions(doc)              # stale red bodies would lie
            self.status.setText("posed -- press Check collisions")
            try:
                Gui.updateGui()                # repaint mid-drag
            except Exception:                            # noqa: BLE001
                pass

        def check(self):
            doc = self.document()
            if doc is None:
                return
            rows = check_collisions(doc)
            hits = [r for r in rows if r[1]]
            if hits:
                self.status.setText(
                    "%d COLLISION(S): %s -- red in the tree"
                    % (len(hits), ", ".join(r[0] for r in hits[:2])))
            else:
                gaps = [r for r in rows if r[2] is not None]
                if gaps:
                    tight = min(gaps, key=lambda r: r[2])
                    self.status.setText("clear -- tightest %s at %.2f mm"
                                        % (tight[0], tight[2]))
                else:
                    self.status.setText("clear")

        def reset(self):
            for s in self.sliders.values():
                s.setValue(0)
            self.repose()

    panel = PosePanel(doc)
    panel.show()
    return panel


# --------------------------------------------------------------------- main
def _gui_up():
    try:
        import FreeCADGui as Gui
        return Gui.getMainWindow() is not None
    except Exception:                                # noqa: BLE001
        return False


def _doc():
    """The articulated document: the open one, else open it from step/."""
    for d in App.listDocuments().values():
        if d.getObject("Assembly") is not None and d.getObject("thigh_L"):
            return d
    name = "bimo_v3yaw_articulated"
    if os.environ.get("BIMO_PAN_HEADS", "") not in ("", "0", "false"):
        name += "_PANHEADS"
    path = os.path.join(HERE, "step", name + ".FCStd")
    if not os.path.isfile(path):
        raise RuntimeError("%s not found -- run freecad_articulate.py first"
                           % path)
    return App.openDocument(path)


def main():
    doc = _doc()
    spec = os.environ.get("BIMO_POSE", "")
    if spec:                                          # batch / headless
        angles = {}
        for item in spec.replace(";", ",").split(","):
            if not item.strip():
                continue
            k, v = item.split("=")
            angles[k.strip()] = float(v)
        unknown = set(angles) - set(k for k, _, _ in JOINTS)
        if unknown:
            raise SystemExit("unknown joint(s) %s -- use %s"
                             % (sorted(unknown),
                                [k for k, _, _ in JOINTS]))
        App.Console.PrintMessage("pose: %s\n" % angles)
        apply_pose(doc, angles)
        check_collisions(doc)
        return doc
    if _gui_up():
        return show_panel(doc)
    # freecadcmd DOES import FreeCADGui successfully but has no QApplication,
    # so probe for a real main window rather than trusting the import.
    App.Console.PrintMessage(
        "no GUI: set BIMO_POSE=\"hip=-60,knee=-95\" for a batch check\n")
    return doc


PANEL = main()
