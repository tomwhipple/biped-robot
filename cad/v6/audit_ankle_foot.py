"""Printability audit for ankle_link / foot_L / foot_R, reusing
cad/check_printability.py's engine without touching that (v5) file: it sets
ORIENT/PRINT_STL for these three parts at runtime and points STL at
cad/v6/stl instead of cad/stl.

    .venv/bin/python cad/v6/audit_ankle_foot.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

import numpy as np  # noqa: E402
import check_printability as CP  # noqa: E402
import ankle_link as AL  # noqa: E402
import foot_v6 as F  # noqa: E402

CP.STL = os.path.join(HERE, "stl")

CP.ORIENT.update({
    "ankle_link": (CP.RY_XUP, "on its back like leg_link (RY_XUP): the rear "
                  "tine on the bed, the grip screws' teardrop roofs up"),
    "foot_L": (CP.IDENT, "sole down (v5 convention)"),
    "foot_R": (CP.IDENT, "sole down (v5 convention)"),
})

# ankle_link, on its back: the rear tine is on the bed and the box around the
# pitch servo's base starts at the web, 4.6 mm up (the ISLAND, supported from
# the plate). The front tine below the floor is a ledge over the roll servo's
# space and the front wall spans the box (the CEILING); those supports stand
# on the rear tine's inner face. The two THIN flags are the leg link's grip
# channel: the 0.97 mm horn plate behind the rib detent, and the sill under
# the detent's lower end, which the roll sweep slopes.
CP.SUPPORTED["ankle_link"] = (
    "supports on (the web starts 4.6 mm over the bed; the front wall and "
    "tine overhang the box and the roll servo's space)")


def main():
    names = sys.argv[1:] or ["ankle_link", "foot_L", "foot_R"]
    bad = {}
    for name in names:
        f = CP.audit(name)
        if f:
            bad[name] = f
    print()
    if bad:
        print("SAG RISKS FOUND:")
        for name, fs in bad.items():
            for kind, a, desc in fs:
                print(f"  {name:11s} {kind:8s} {desc}")
        sys.exit(1)
    print("ankle_link / foot_L / foot_R PRINT CLEAN (in their listed orientations)")


if __name__ == "__main__":
    main()
