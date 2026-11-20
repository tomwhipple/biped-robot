"""Where generated files go: build/<version>/<target>/, which git ignores.

    sys.path.insert(0, <repo root>)
    from paths import build_dir
    OUT = build_dir("v6", "step")       # <repo>/build/v6/step, created on first use

The versions are the designs: v6 (the robot), bench-v2 and bench-v3 (the
Plan B rigs). The targets are kinds of output:
stl, step, renders, movies, reports. `make` builds into the same folders, and
`make clean` deletes build/.

The plant the simulator and the trainer load is generated too, and so are the
printed parts' STLs (issue #144):

    V6_STL   build/v6/stl/   cad/v6/parts_v6.py's exports (`make cad`)
    V6_SIM   build/v6/sim/   the plant XMLs, the servo meshes they cite
                             (meshes/) and terrain_mosaic.npz (`make plant-data`)

Still committed, because they are recordings and not rebuildable: the get-up
references (sim/getup_ref_v6ar*.npz), and the firmware headers.
"""
import os

ROOT = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(ROOT, "build")

V6_STL = os.path.join(BUILD, "v6", "stl")
V6_SIM = os.path.join(BUILD, "v6", "sim")
V6_TERRAIN = os.path.join(V6_SIM, "terrain_mosaic.npz")
V6_SERVO_MESHES = os.path.join(V6_SIM, "meshes")


def v6_plant(name):
    """build/v6/sim/<basename of name>: where `make plant` writes a v6 plant and
    where every env and script resolves one."""
    return os.path.join(V6_SIM, os.path.basename(name))


def resolve_plant(xml):
    """A plant path as given when it exists; otherwise the basename in build/v6/sim.
    A run's config.json keeps the plant by name (or by the path it had on another
    host, or when plants lived in sim/), so every reader resolves through here."""
    if os.path.exists(xml):
        return xml
    return v6_plant(xml)


def build_dir(version, target):
    path = os.path.join(BUILD, version, target)
    os.makedirs(path, exist_ok=True)
    return path
