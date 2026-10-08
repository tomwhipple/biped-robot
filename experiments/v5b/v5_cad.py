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

_LOADED = None


def _materialize():
    os.makedirs(CACHE, exist_ok=True)
    for name in MODULES:
        path = os.path.join(CACHE, f"{name}.py")
        if not os.path.exists(path):
            src = subprocess.run(["git", "-C", REPO, "show", f"{V5_COMMIT}:cad/{name}.py"],
                                 check=True, capture_output=True, text=True).stdout
            with open(path + ".tmp", "w") as f:
                f.write(src)
            os.replace(path + ".tmp", path)


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
