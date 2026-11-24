"""At the moment a fall's joint difference (vs the same run/seed at gain 1.0) first reaches 5 deg:
the share of the squared difference in hip yaw / roll joints / sagittal joints, and the hip-pitch
antisymmetry (Lhp and Rhp of opposite sign = scissor).

  python modeshare.py TRDIR out.csv"""
import glob, os, re, sys
import numpy as np, pandas as pd
DT = 0.02
Y, R, P = [0, 6], [1, 5, 7, 11], [2, 3, 4, 8, 9, 10]
rows = []
for f in glob.glob(os.path.join(sys.argv[1], "*.npz")):
    m = re.match(r"(.*)_g([\d.]+)_s(\d+)\.npz", os.path.basename(f))
    run, g, s = m.group(1), float(m.group(2)), int(m.group(3))
    if g == 1.0:
        continue
    d = np.load(f)
    if not bool(d["fell"]):
        continue
    fb = os.path.join(sys.argv[1], f"{run}_g1.0_s{s}.npz")
    if not os.path.exists(fb):          # a group / gyro-scaled trace has no tagged x1.0 twin
        continue
    b = np.load(fb)
    n = min(len(d["QM"]), len(b["QM"]))
    dev = np.degrees(d["QM"][:n] - b["QM"][:n])
    k = np.where(np.abs(dev).max(1) > 5.0)[0]
    if not len(k):
        continue
    v = dev[k[0]]; e = (v ** 2).sum()
    rows.append(dict(run=run, g=g, s=s, yaw=(v[Y] ** 2).sum() / e, roll=(v[R] ** 2).sum() / e,
                     sag=(v[P] ** 2).sum() / e, scissor=float(np.sign(v[2]) != np.sign(v[8]) and min(abs(v[2]), abs(v[8])) > 0.5),
                     gz=float(np.sqrt((d["GY"][:k[0] + 1, 2] - b["GY"][:k[0] + 1, 2]) ** 2).mean()),
                     gx=float(np.sqrt((d["GY"][:k[0] + 1, 0] - b["GY"][:k[0] + 1, 0]) ** 2).mean())))
df = pd.DataFrame(rows)
df.to_csv(sys.argv[2], index=False)
