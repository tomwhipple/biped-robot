"""Standalone printability + interface audit runner for leg_link_v6, per
docs request: run cad/check_printability.py's audit on the STL, injecting
leg_link_v6's ORIENT/PRINT_STL entries into it AT RUNTIME (v5's own
check_printability.py is never edited -- its ORIENT/PRINT_STL/STL globals
are only monkey-patched in this process, same trick leg_link_v6.run_audits
uses internally).

    .venv/bin/python cad/v6/audit_leg_link.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

from build123d import export_stl  # noqa: E402
import leg_link_v6 as LL  # noqa: E402


def main():
    os.makedirs(LL.OUT_STL, exist_ok=True)
    p = LL.leg_link_v6()
    stl_path = os.path.join(LL.OUT_STL, "leg_link_v6.stl")
    export_stl(p, stl_path)
    ok = LL.run_audits(p, stl_path)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
