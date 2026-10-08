"""The printed v5 parts, loaded from git history.

v5b reuses the prototype's printed torso and legs, so their geometry is frozen:
it is whatever was printed, not whatever `cad/parts.py` says today. This module
pins the prototype's CAD modules to one commit (`git show`), writes them to a
gitignored cache, and imports them under their own names so they never mix
with the current `cad/` modules that the v6 parts import (`parts`,
`dimensions`, `check_assembly`).

    import v5_cad
    v5 = v5_cad.load()
    v5.parts.leg_link(), v5.D.HIP_SEP, v5.check_assembly.board_pcb_mock()
    v5_cad.v5body_xml()      # the v5 sim plant at the same commit, with its meshes
"""
from __future__ import annotations

import importlib
import os
import subprocess
import sys
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))

# the last main with the prototype's builders at their final state
V5_COMMIT = "357aacf563a86d52cf1ee27155b37f624d4d725d"
MODULES = ("dimensions", "parts", "fasteners", "check_assembly")
CACHE = os.path.join(HERE, ".v5cache", V5_COMMIT[:12])

# the v5 sim plant and the meshes it reads (meshdir="../cad/stl")
V5BODY = "sim/bimo_biped_v5body.xml"
V5BODY_MESHES = ("pelvis", "gopro_base", "yoke_roll", "yoke_pitch", "leg_link", "foot", "yaw_carrier")

_LOADED = None


def _git_file(repo_path, dest):
    """`git show V5_COMMIT:repo_path` to dest, once."""
    if os.path.exists(dest):
        return dest
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    blob = subprocess.run(["git", "-C", REPO, "show", f"{V5_COMMIT}:{repo_path}"],
                          check=True, capture_output=True).stdout
    with open(dest + ".tmp", "wb") as f:
        f.write(blob)
    os.replace(dest + ".tmp", dest)
    return dest


def _materialize():
    for name in MODULES:
        _git_file(f"cad/{name}.py", os.path.join(CACHE, f"{name}.py"))


def v5body_xml():
    """Path to the v5 plant XML at V5_COMMIT, its meshes beside it in the same
    relative layout, so it loads as it did then."""
    root = os.path.join(CACHE, "tree")
    for m in V5BODY_MESHES:
        _git_file(f"cad/stl/{m}.stl", os.path.join(root, "cad", "stl", f"{m}.stl"))
    return _git_file(V5BODY, os.path.join(root, V5BODY))


def load():
    """The pinned v5 modules: .D (dimensions), .parts, .fasteners,
    .check_assembly. Imported in isolation: whatever the
    caller already has under those module names is restored afterwards."""
    global _LOADED
    if _LOADED is not None:
        return _LOADED
    _materialize()
    saved = {n: sys.modules.pop(n) for n in MODULES if n in sys.modules}
    sys.path.insert(0, CACHE)
    try:
        mods = {n: importlib.import_module(n) for n in MODULES}
    finally:
        sys.path.remove(CACHE)
        for n in MODULES:
            sys.modules.pop(n, None)
        sys.modules.update(saved)
    _LOADED = SimpleNamespace(D=mods["dimensions"], **mods)
    return _LOADED
