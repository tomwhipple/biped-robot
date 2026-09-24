"""Printability audit for the v7 torso parts, reusing cad/check_printability.py
verbatim (same thresholds, same algorithm) against cad/v6/stl/ instead of
cad/stl/. check_printability.py is written around ONE STL directory
(module-level `STL`) and ONE `ORIENT` table keyed by part name; this module
monkey-patches both at runtime rather than forking the file, so the v6 parts
are audited by the exact same code path as every v5 part.

    .venv/bin/python cad/v6/audit_torso.py                # pelvis_v7, head_shell, head_lid, camera_sled
    .venv/bin/python cad/v6/audit_torso.py pelvis_v7
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

import numpy as np  # noqa: E402
import check_printability as CP  # noqa: E402

CP.STL = os.path.join(HERE, "stl")

RX180 = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)   # deck-top-down
IDENT = np.eye(3)                                              # base-down

CP.ORIENT.update({
    "pelvis_v7": (RX180, "upside down: deck top on bed (v7 torso)"),
    "head": (IDENT, "base down: neck-horn plate on bed"),
    "head_shell": (IDENT, "base down: neck-horn plate on bed"),
    "head_lid": (IDENT, "flat, underside on bed, countersinks up"),
    "camera_sled": (np.array([[0, 0, 1.0], [0, 1.0, 0], [-1.0, 0, 0]]), "on its front face, bosses up"),
    "neck_collar": (IDENT, "flange down, flat, no supports"),
})

# NOTE (not waived): pelvis_v7's yaw-cell ceiling reports as an ISLAND
# (~46 x 25 mm, capping the servo cell so the battery layer can sit above
# it). This one is a genuine unsupported span with no workaround within the
# print: the STS3215 case fills the cell to within YAW_SEAT_GAP (0.3 mm) on
# every side, right up to where its idler face meets the ceiling, so there
# is no room anywhere in that footprint for a gusset, a taper, or an
# internal post without colliding with the servo itself. It needs slicer
# supports (same precedent as v5's yoke_roll/yoke_pitch) -- but it is NOT
# added to CP.SUPPORTED here, because that waiver is part-wide (every
# ISLAND/LEDGE/CEILING/BEAM on the part), and pelvis_v7 has other findings
# that are real, still-open design issues and should keep failing the gate
# until fixed. See the pelvis_v7.py module docstring and the CAD report for
# the full, unwaived finding list.


def run(names=None):
    names = names or ["pelvis_v7", "head_shell", "head_lid", "camera_sled"]
    bad = {}
    for name in names:
        path = os.path.join(CP.STL, CP.PRINT_STL.get(name, f"{name}.stl"))
        if not os.path.exists(path):
            print(f"{name}: ** no STL at {path} -- run the part's __main__ first **")
            continue
        f = CP.audit(name)
        if f:
            bad[name] = f
    print()
    if bad:
        print("SAG RISKS FOUND:")
        for name, fs in bad.items():
            for kind, a, desc in fs:
                print(f"  {name:12s} {kind:8s} {desc}")
        return False
    print("ALL V7 TORSO PARTS PRINT CLEAN")
    return True


if __name__ == "__main__":
    ok = run(sys.argv[1:] or None)
    sys.exit(0 if ok else 1)
