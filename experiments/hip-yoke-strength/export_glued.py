"""Export the glued hip yoke pieces and their union as STEP for FEM:
    cd <worktree> && python export_glued.py <tag>
writes build/experiments/hip-yoke-strength/fem/<tag>_union.step, _roll.step, _pitch.step"""
import os
import sys

WT = os.getcwd()
for p in (os.path.join(WT, "cad", "v6"), os.path.join(WT, "cad"), WT):
    sys.path.insert(0, p)
OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                       "build", "experiments", "hip-yoke-strength")
OUT = os.path.join(OUT_DIR, "fem")
os.makedirs(OUT, exist_ok=True)
from build123d import export_step, export_stl  # noqa: E402
import hip_yoke_glued as G  # noqa: E402

tag = sys.argv[1]
pieces = G.build()
u = pieces["hip_yoke_glued_roll"].fuse(pieces["hip_yoke_glued_pitch"]).clean()
print("union solids", len(u.solids()), "volume", round(u.volume, 1), "mass g", round(G.mass_g(u), 2))
for k, s in pieces.items():
    print(k, "mass g", round(G.mass_g(s), 2))
export_step(u, os.path.join(OUT, f"{tag}_union.step"))
export_stl(u, os.path.join(OUT, f"{tag}_union.stl"))
export_step(pieces["hip_yoke_glued_roll"], os.path.join(OUT, f"{tag}_roll.step"))
export_step(pieces["hip_yoke_glued_pitch"], os.path.join(OUT, f"{tag}_pitch.step"))
print("wrote", OUT)
