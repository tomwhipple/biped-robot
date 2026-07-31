"""Build FreeCAD's OWN Assembly -> Simulation objects so the ROM plays natively.

In the GUI, with cad/step/bimo_v3yaw_articulated.FCStd open:

    1. Run cad/freecad_gui_ready.py FIRST if the FCStd has been rebuilt
       headlessly since you last did -- see that file. Without it, opening a
       simulation dies in redrawJointPlacements when it tries to activate the
       assembly.
    2. Macro -> Macros... -> freecad_sim_setup.py -> Execute
       (or in the Python console: exec(open("cad/freecad_sim_setup.py").read()))
    3. SAVE the document.

Afterwards, to watch a joint:
    Assembly tree -> Simulations -> double-click e.g. "ROM knee L"
    -> the task panel opens, computes the kinematics, and gives you a play
       button and a frame slider. Orbit and zoom while it runs. Close the panel
       with OK and the assembly returns to neutral.

Ten simulations, one per joint per leg, so you watch exactly one joint at a
time; the other leg and the rest of this one hold neutral as a reference.

WHY A MACRO AND NOT BAKED INTO THE FCStd
----------------------------------------
A Simulation is an App::FeaturePython whose tree behaviour -- claimChildren,
and the double-click that opens the player -- lives entirely in its GUI
ViewProvider. Built headlessly there is no ViewObject to attach one to, so the
objects would land in the file looking right and refuse to open. They have to
be created with a GUI attached. freecad_articulate.py also REBUILDS this FCStd
from scratch, which drops them; re-run this after any rebuild.

WHY EVERY SIMULATION DRIVES ALL TEN JOINTS
------------------------------------------
The assembly has ten revolute joints, so ten free DOF. A Simulation with one
Motion constrains exactly one of them and MbD is free to put the other nine
anywhere that solves -- drive knee_L alone and the foot lands 63 mm from where
the kinematics says, with the hip yawed out of plane. So each simulation here
carries ten Motions: the one under test sweeping, the other nine pinned at
"0*time". Verified against freecad_pose.stage_placements by _verify() below.

FORMULA
-------
MbD's parser takes sin, cos, pi and products; it does NOT take comparisons or
abs (those parse to a zero-frame simulation, silently). So no piecewise, which
rules out a literal neutral -> max -> min -> neutral ramp. Instead each joint
gets the smooth cycle

    phi(t) = a*sin(2*pi*t) + mid*(1 - cos(2*pi*t))          [RADIANS]

with mid = (lo+hi)/2, half = (hi-lo)/2, a = sqrt(half^2 - mid^2). That is a
sinusoid of amplitude `half` about `mid`, phase-shifted so phi(0) = phi(1) = 0.
It starts at NEUTRAL, rises to exactly `hi`, falls to exactly `lo`, and returns
-- the same sequence cad/animate_joint_rom.py films, hitting both ROM limits
exactly rather than approximately. (Needs |mid| <= half, true for all five
joints: the worst is hip_pitch at mid -25 vs half 85.)

Units are radians: a Motion of "0.7854*time" turns the knee exactly 45.000 deg,
and in the same direction our kinematics calls +45.

THIS IS FOR WATCHING, NOT FOR EVIDENCE. MbD does not do collision detection --
neither the joint limits nor part interference are enforced here. The swept
interference check is cad/freecad_rom_collide.py (exit 0/1); run it via
cad/run_checks.sh.
"""
import math
import os
import sys

import FreeCAD as App

try:
    HERE = os.path.dirname(os.path.abspath(__file__))
except NameError:                       # exec'd from the console
    HERE = os.path.abspath(".")
    if not os.path.isdir(os.path.join(HERE, "step")):
        HERE = os.path.join(HERE, "cad")
if HERE not in sys.path:
    sys.path.insert(0, HERE)

_ASM_MOD = "/Applications/FreeCAD.app/Contents/Resources/Mod/Assembly"
if os.path.isdir(_ASM_MOD) and _ASM_MOD not in sys.path:
    sys.path.insert(0, _ASM_MOD)

import dimensions as D
import freecad_pose as FP

if not App.GuiUp:
    # CommandCreateSimulation subclasses QtCore.QObject at module scope but only
    # imports the name under `if App.GuiUp`, so importing it headless dies on a
    # NameError. Binding it in builtins lets the class body resolve it -- we
    # only want the Simulation/Motion data classes, never the task panel.
    import builtins
    import PySide.QtCore
    builtins.QtCore = PySide.QtCore

import UtilsAssembly
from CommandCreateSimulation import Simulation, Motion

if App.GuiUp:
    from CommandCreateSimulation import ViewProviderSimulation, ViewProviderMotion

PREFIX = "ROM "                 # label prefix, so a rebuild is easy to spot
TIME_END = 1.0                  # one full out-and-back cycle
TIME_STEP = 0.01                # -> ~100 frames
FPS = 30

# our JOINTS key -> the joint Name freecad_articulate.py gives it
JOINT_NAME = {"yaw": "hip_yaw", "roll": "hip_roll", "hip": "hip_pitch",
              "knee": "knee", "ankle": "ankle"}


def _formula(lo, hi):
    """Radian formula sweeping [lo, hi] degrees, starting and ending at 0."""
    a, mid, _ = _shape(lo, hi)
    return "%.6f*sin(2*pi*time)+%.6f*(1-cos(2*pi*time))" % (a, mid)


def _shape(lo, hi):
    """(a, mid, phase) for phi(t) = a*sin(2pi t) + mid*(1 - cos(2pi t)).

    Rewritten, that is phi = mid + half*sin(2pi t - phase), so the peak sits at
    t = 1/4 + phase/2pi and the trough at t = 3/4 + phase/2pi -- NOT at 1/4 and
    3/4 unless the ROM is symmetric about neutral. The knee's peak is at
    t = 0.072.
    """
    mid = math.radians((lo + hi) / 2.0)
    half = math.radians((hi - lo) / 2.0)
    if abs(mid) > half:                 # amplitude smaller than its own offset
        raise ValueError("ROM %s..%s does not contain neutral" % (lo, hi))
    a = math.sqrt(half * half - mid * mid)
    return a, mid, math.atan2(mid, a)


def _assembly(doc):
    for o in doc.Objects:
        if o.isDerivedFrom("Assembly::AssemblyObject"):
            return o
    raise RuntimeError("no Assembly in %s" % doc.Name)


def _clear(doc, asm):
    """Drop any ROM_* simulations from a previous run, motions included."""
    grp = UtilsAssembly.getSimulationGroup(asm)
    for sim in list(grp.Group):
        if not sim.Label.startswith(PREFIX):
            continue
        for motion in list(sim.Group):
            doc.removeObject(motion.Name)
        doc.removeObject(sim.Name)


def build(doc=None):
    doc = doc or App.ActiveDocument
    asm = _assembly(doc)
    joints = {j.Name: j for j in UtilsAssembly.getJointGroup(asm).Group
              if getattr(j, "JointType", "") == "Revolute"}
    _clear(doc, asm)
    grp = UtilsAssembly.getSimulationGroup(asm)

    built = []
    for tag in ("L", "R"):
        for key, label, (lo, hi) in FP.JOINTS:
            driven = "%s_%s" % (JOINT_NAME[key], tag)
            if driven not in joints:
                print("  skip %s -- no such joint" % driven)
                continue

            sim = grp.newObject("App::FeaturePython", "Simulation")
            Simulation(sim)
            if App.GuiUp:
                ViewProviderSimulation(sim.ViewObject)
            sim.Label = "%s%s %s" % (PREFIX, label, tag)
            sim.aTimeStart = 0.0
            sim.bTimeEnd = TIME_END
            sim.cTimeStepOutput = TIME_STEP
            sim.jFramesPerSecond = FPS

            motions = []
            for name, joint in joints.items():
                # every other joint is PINNED -- see the module docstring
                formula = _formula(lo, hi) if name == driven else "0*time"
                m = asm.newObject("App::FeaturePython", "Motion")
                Motion(m, "Angular", joint, formula)
                if App.GuiUp:
                    ViewProviderMotion(m.ViewObject)
                motions.append(m)
            sim.Group = motions
            built.append((sim, driven, lo, hi))
            print("  built %-22s driving %-12s %+.0f..%+.0f deg"
                  % (sim.Label, driven, lo, hi))

    doc.recompute()
    return asm, built


def _verify(doc, asm, built, tol=0.15):
    """Play each simulation and confirm it really reaches the ROM limits.

    For each limit, walk every frame and find the one whose foot lands closest
    to where freecad_pose.stage_placements puts it at that angle. If the best
    frame is within `tol`, the simulation genuinely visits that limit -- which
    checks amplitude, sign, units AND the pinning of the other nine joints at
    once, since any of them drifting would move the foot off our number.

    Searching frames rather than computing which frame SHOULD hold the peak
    avoids depending on MbD's frame-to-time mapping, which is off by one from
    the obvious reading of cTimeStepOutput.
    """
    ok = True
    # generateSimulation solves from wherever the parts CURRENTLY are, so a sim
    # generated straight after another one starts from that one's last frame and
    # compounds. Verifying ten in a row without restoring drifted the foot 390 mm
    # by the fifth -- and identically on both legs, which is what gave it away.
    # The GUI's own panel does this same save/restore around a run.
    neutral = UtilsAssembly.saveAssemblyPartsPlacements(asm)
    for sim, driven, lo, hi in built:
        UtilsAssembly.restoreAssemblyPartsPlacements(asm, neutral)
        tag = driven[-1]
        key = [k for k, _l, _r in FP.JOINTS
               if JOINT_NAME[k] == driven[:-2]][0]
        foot = doc.getObject("foot_%s" % tag)
        asm.generateSimulation(sim)
        n = asm.numberOfFrames()
        if n < 4:
            print("  %-22s ** 0-frame simulation -- formula rejected **"
                  % sim.Label)
            ok = False
            continue
        idx = [k for k, _l, _r in FP.JOINTS].index(key)
        sign = 1 if tag == "L" else -1
        poses = []
        for f in range(n):
            asm.updateForFrame(f)
            poses.append(foot.Placement.Base)
        worst = 0.0
        for want in (hi, lo):
            vals = [0.0] * 5
            vals[idx] = want
            ref = FP.stage_placements(sign * D.HIP_SEP / 2, *vals)["foot"].Base
            worst = max(worst, min((p - ref).Length for p in poses))
        flag = "OK" if worst <= tol else "** MISMATCH **"
        print("  %-22s frames=%-4d reaches both limits to %7.4f mm  %s"
              % (sim.Label, n, worst, flag))
        ok &= worst <= tol
    UtilsAssembly.restoreAssemblyPartsPlacements(asm, neutral)
    return ok


def main():
    doc = App.ActiveDocument
    if doc is None:
        doc = App.openDocument(os.path.join(HERE, "step",
                                            "bimo_v3yaw_articulated.FCStd"))
    print("== building ROM simulations ==")
    asm, built = build(doc)
    print("== verifying against freecad_pose kinematics ==")
    ok = _verify(doc, asm, built)
    print("\n%s" % ("%d simulations ready -- SAVE THE DOCUMENT, then "
                    "double-click one under Assembly > Simulations"
                    % len(built) if ok else
                    "** simulations do NOT match the kinematics **"))
    return ok


_ok = main()
sys.stdout.flush()
if not App.GuiUp:
    # freecadcmd drops buffered stdout on SystemExit; flush above, then report.
    sys.exit(0 if _ok else 1)
