"""Printability audit for the torso parts, reusing cad/check_printability.py
verbatim (same thresholds, same algorithm) against cad/v6/stl/ instead of
cad/stl/. check_printability.py is written around ONE STL directory
(module-level `STL`) and ONE `ORIENT` table keyed by part name; this module
monkey-patches both at runtime rather than forking the file, so the v6 parts
are audited by the exact same code path as every v5 part.

    .venv/bin/python cad/v6/audit_torso.py                # pelvis_v8, torso_v8, the covers, yaw_bearing_housing, head parts
    .venv/bin/python cad/v6/audit_torso.py torso_v8
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

import numpy as np  # noqa: E402
import check_printability as CP  # noqa: E402

CP.STL = os.path.join(HERE, "stl")
# the torso prints with slicer support in its wide windows and its battery
# door's opening, inside cad/v6/print_settings.json's enforcer boxes and
# nowhere else; torso_v8.py audits it with those windows filled (no waiver)
CP.SUPPORTED["torso_v8"] = "support in its windows only (print_settings.json support_enforcers)"

RX180 = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)   # deck-top-down
IDENT = np.eye(3)                                              # base-down

CP.ORIENT.update({
    "pelvis_v8": (RX180, "upside down: ceiling on bed, cell walls rising"),
    "torso_v8": (RX180, "upside down: deck top on bed"),
    "front_cover_v8": (np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float), "outer face on bed"),
    "power_cover_v8": (np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]], float), "outer face on bed"),
    "battery_door_v8": (np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float), "outer face on bed"),
    "yaw_bearing_housing": (RX180, "upside down: the face against the pelvis on bed"),
    "head": (IDENT, "base down: neck-horn plate on bed"),
    "head_shell": (IDENT, "base down: neck-horn plate on bed"),
    "head_face": (IDENT, "face plate flat on bed, camera bosses up"),
    "neck_collar": (IDENT, "flange down, flat, no supports"),
})


def run(names=None):
    names = names or ["pelvis_v8", "torso_v8", "front_cover_v8", "power_cover_v8", "battery_door_v8",
                       "yaw_bearing_housing",
                       "head", "head_shell", "head_face"]
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
    print("ALL TORSO PARTS PRINT CLEAN")
    return True


if __name__ == "__main__":
    ok = run(sys.argv[1:] or None)
    sys.exit(0 if ok else 1)
