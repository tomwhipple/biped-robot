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
    "ankle_link": (CP.RY_XUP, "on its back like leg_link (RY_XUP): the "
                  "copied grip-channel bores keep their teardrop roofs"),
    "foot_L": (CP.IDENT, "sole down (v5 convention)"),
    "foot_R": (CP.IDENT, "sole down (v5 convention)"),
})

# ankle_link needs slicer supports for the same reason the hip yokes do
# (2026-07-30 precedent): the top-plate/tine block below the grip channel is
# a compact, feature-dense structure with no orientation that avoids every
# bridge/floating start at once. Verified (2026-09-14) that the flagged
# CEILING/ISLAND regions are NOT starved of material -- they are internal
# transitions the ray-cast heuristic reads as unsupported, not thin webs
# (see the THIN class below, checked separately by direct density probing).
CP.SUPPORTED["ankle_link"] = (
    "supports on (top-plate/tine transition has no support-free orientation "
    "once the ankle-roll clearance cut is applied)")


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
