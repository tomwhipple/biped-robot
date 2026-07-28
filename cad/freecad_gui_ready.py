"""Make a headless-built articulated .FCStd manipulable in the FreeCAD GUI.

WHY THIS EXISTS
---------------
`freecad_articulate.py` builds the assembly with `freecadcmd`, which creates no
view providers at all, and then injects a minimal GuiDocument.xml so the file at
least opens VISIBLE.  That injected XML gives every object a generic
ViewProvider carrying only `Visibility` -- so each joint's `ViewObject.Proxy` is
the placeholder integer `1` instead of an `Assembly.JointObject.ViewProviderJoint`.

The joints themselves are fine: the document reports all 10 revolute joints and
the solver owns the chain.  But ACTIVATING the assembly -- double-clicking
"Assembly" in the tree, which is the prerequisite for dragging parts and for the
Simulation command -- makes FreeCAD call `redrawJointPlacements` on every joint
view provider, and the placeholder raises

    AttributeError: 'int' object has no attribute 'redrawJointPlacements'

so you can never enter the assembly, and nothing is draggable.

WHAT THIS DOES
--------------
Attaches a real ViewProviderJoint (ViewProviderGroundedJoint for the ground) to
each joint and runs its `attach()` so the JCS scene nodes exist, then saves.
After this the file opens GUI-ready: double-click Assembly -> drag any link, or
Assembly -> Simulation to drive a joint with a formula.

Idempotent: joints that already have a real view provider are skipped.

RUN IT FROM THE GUI (it needs FreeCADGui -- freecadcmd cannot do this):
    open the .FCStd, then Macro -> Macros... -> freecad_gui_ready.py
Re-run it after every headless rebuild of the .FCStd.
"""
import FreeCAD as App

try:
    import FreeCADGui as Gui
except ImportError:                                   # pragma: no cover
    Gui = None
import JointObject


def _doc():
    """The articulated document: the active one if it is an assembly."""
    d = App.ActiveDocument
    if d is None or d.getObject("Assembly") is None:
        raise RuntimeError(
            "open an articulated .FCStd first (cad/step/bimo_v3yaw_*.FCStd)")
    return d


def attach_joint_view_providers(doc):
    """Give every joint a real Assembly view provider. Returns the names fixed."""
    jg = doc.getObject("Joints")
    if jg is None:
        raise RuntimeError("no JointGroup named 'Joints' in %s" % doc.Name)

    fixed = []
    for joint in jg.Group:
        vobj = joint.ViewObject
        if vobj is None:                              # headless: nothing to do
            continue
        proxy = getattr(vobj, "Proxy", None)
        if proxy is not None and hasattr(proxy, "redrawJointPlacements"):
            continue                                  # already a real one
        if isinstance(joint.Proxy, JointObject.GroundedJoint):
            JointObject.ViewProviderGroundedJoint(vobj)
        else:
            JointObject.ViewProviderJoint(vobj)
        # __init__ only sets Proxy; attach() builds the JCS scene nodes that
        # entering the assembly expects to find.
        proxy = vobj.Proxy
        if hasattr(proxy, "attach") and not hasattr(proxy, "switch_JCS1"):
            proxy.attach(vobj)
        fixed.append(joint.Name)
    return fixed


def main():
    doc = _doc()
    fixed = attach_joint_view_providers(doc)
    doc.recompute()
    if fixed:
        doc.save()
    App.Console.PrintMessage(
        "gui-ready: attached %d joint view provider(s)%s\n"
        % (len(fixed), " and saved" if fixed else " (already gui-ready)"))

    if Gui is not None:
        asm = doc.getObject("Assembly")
        Gui.ActiveDocument.setEdit(asm)               # == double-clicking it
        App.Console.PrintMessage(
            "assembly activated: %d drivable joint(s) -- drag a link, or "
            "Assembly -> Simulation\n" % len(asm.Joints))


main()
