"""Where generated files go: build/<version>/<target>/, which git ignores.

    sys.path.insert(0, <repo root>)
    from paths import build_dir
    OUT = build_dir("v6", "step")       # <repo>/build/v6/step, created on first use

The versions are the designs: v6 (the robot), v5 (the 10-joint prototype),
bench-v2 and bench-v3 (the Plan B rigs). The targets are kinds of output:
stl, step, renders, movies, reports. `make` builds into the same folders, and
`make clean` deletes build/.

Some generated files are still committed, because the training box loads them
from git and runs no generator: the STLs below, the plant XMLs and their
meshes, the .npz references and the firmware headers.
"""
import os

ROOT = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(ROOT, "build")

V6_STL = os.path.join(ROOT, "cad", "v6", "stl")
V5_STL = os.path.join(ROOT, "cad", "stl")


def build_dir(version, target):
    path = os.path.join(BUILD, version, target)
    os.makedirs(path, exist_ok=True)
    return path
