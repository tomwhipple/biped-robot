"""Export every printable part as STEP (exact solids for FreeCAD/Onshape).

Run:  .venv/bin/python cad/export_step.py   -> cad/step/*.step

STLs (cad/stl/) are for slicing; STEP is for inspection/editing -- FreeCAD opens
them as real BREP solids you can measure, section, and modify.
"""
import os
from build123d import export_step
import parts

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "step")
os.makedirs(OUT, exist_ok=True)

for name in ("pelvis", "yoke_roll", "yoke_pitch", "leg_link", "foot", "tower"):
    solid = getattr(parts, name)()
    path = os.path.join(OUT, f"{name}.step")
    export_step(solid, path)
    print(f"{name:12s} -> {path}")
