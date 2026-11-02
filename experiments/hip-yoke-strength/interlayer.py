"""Interlayer stress of the glued hip yoke, each piece in its own print frame.
roll piece (z > -20): printed on edge, model -Y on the bed -> layers normal to Y:
    across-layer tension = s_yy, interlayer shear = |(s_xy, s_yz)|
pitch piece (z < -20): printed on its back, model -X on the bed -> layers normal to X:
    across-layer tension = s_xx, interlayer shear = |(s_xy, s_zx)|
glue plane (|z + 20| < 0.05): peel = s_zz, shear = |(s_zx, s_yz)|
Bolt-hole walls (load / restraint) are excluded. Hot spots are clustered 5 mm apart.
    python interlayer.py <tag> [<tag> ...]     (cases found under fem/)
"""
import os
import sys

import numpy as np

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                       "build", "experiments", "hip-yoke-strength")
HERE = os.path.join(OUT_DIR, "fem")
CASES = ("roll", "pitch", "yaw", "fz", "fx")


def spots(P, v, k=3, sep=5.0):
    out = []
    for i in np.argsort(-v):
        if len(out) >= k or v[i] <= 0:
            break
        if all(np.linalg.norm(P[i] - P[j]) > sep for j, _ in out):
            out.append((i, v[i]))
    return [(tuple(np.round(P[i], 1)), round(float(val), 1)) for i, val in out]


def analyse(tag, case):
    wd = os.path.join(HERE, tag if case == "roll" else f"{tag}_{case}")
    a = np.load(os.path.join(wd, "stress.npy"))
    P, S, far = a[:, :3], a[:, 3:9], a[:, 9] > 0.5
    sxx, syy, szz, sxy, syz, szx = S.T
    roll = far & (P[:, 2] > -19.95)
    pitch = far & (P[:, 2] < -20.05)
    glue = far & (np.abs(P[:, 2] + 20) < 0.05)
    res = {
        "roll  tension": spots(P[roll], syy[roll]),
        "roll  shear": spots(P[roll], np.hypot(sxy, syz)[roll]),
        "pitch tension": spots(P[pitch], sxx[pitch]),
        "pitch shear": spots(P[pitch], np.hypot(sxy, szx)[pitch]),
        "glue  peel": spots(P[glue], szz[glue], k=2),
        "glue  shear": spots(P[glue], np.hypot(szx, syz)[glue], k=2),
    }
    return res


if __name__ == "__main__":
    for case in CASES:
        print(f"\n=== {case}")
        for tag in sys.argv[1:]:
            try:
                r = analyse(tag, case)
            except FileNotFoundError:
                print(f"  {tag}: (not run)")
                continue
            print(f"  -- {tag}")
            for k, v in r.items():
                print(f"     {k:14s} " + "   ".join(f"{val:5.1f} @{loc}" for loc, val in v))
