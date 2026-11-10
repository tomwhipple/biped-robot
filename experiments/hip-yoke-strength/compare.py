"""Worst interlayer stress per (case, measure) for several designs, side by side.
    python compare.py baseline v3 v4"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from interlayer import CASES, analyse  # noqa: E402

ROWS = ["roll  tension", "roll  shear", "pitch tension", "pitch shear", "glue  peel", "glue  shear"]
tags = sys.argv[1:]
print(f"{'case':6s} {'measure':14s} " + "".join(f"{t:>30s}" for t in tags))
for case in CASES:
    res = {t: analyse(t, case) for t in tags}
    for k in ROWS:
        cells = []
        for t in tags:
            loc, val = res[t][k][0]
            cells.append(f"{val:5.1f} ({loc[0]:5.1f},{loc[1]:5.1f},{loc[2]:5.1f})")
        print(f"{case:6s} {k:14s} " + "".join(f"{c:>30s}" for c in cells))
