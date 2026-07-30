"""Generate the shared terrain mosaic both envs load (sim/terrain_mosaic.npz).

A single static heightfield of 1 m x 1 m tiles, each with a roughness
amplitude drawn from {0, 2, 5, 10, 15, 20} mm and a feature size drawn
from 0.10-0.20 m (carpet pile .. door-threshold scale). Per-episode
terrain variation comes from the SPAWN DRAW, not from regenerating the
field: each episode starts at a random (x, y) on the mosaic, so the
policy sees a different local roughness every episode while the model
stays static (no MJX model batching needed; the CPU referee loads the
same npz, so the two envs are bit-identical by construction).

Deterministic (seed 0) and versioned: regenerate ONLY with this script,
commit the npz alongside.

Grid: 300 rows (y, +/-3 m) x 600 cols (x, -1.5..10.5 m), 2 cm cells --
bumps resolved well under the 116 mm foot.

Run:  ../../.venv/bin/python gen_terrain_mosaic.py
"""
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "terrain_mosaic.npz")

NROW, NCOL = 300, 600            # y, x
RX, RY, CX = 6.0, 3.0, 4.5       # field spans x -1.5..10.5, y -3..3
CELL = 2 * RX / (NCOL - 1)       # ~2 cm
AMPS_MM = [0, 2, 5, 10, 15, 20]
TILE = 1.0                       # tile edge, m


def gauss_smooth(f, sigma_cells):
    n = max(int(3 * sigma_cells), 1)
    x = np.arange(-n, n + 1)
    k = np.exp(-0.5 * (x / max(sigma_cells, 1e-6)) ** 2)
    k /= k.sum()
    f = np.apply_along_axis(lambda r: np.convolve(r, k, mode="same"), 1, f)
    f = np.apply_along_axis(lambda c: np.convolve(c, k, mode="same"), 0, f)
    return f


rng = np.random.default_rng(0)
field = np.zeros((NROW, NCOL))
xs = (CX - RX) + np.arange(NCOL) * CELL
ys = -RY + np.arange(NROW) * (2 * RY / (NROW - 1))
for ty0 in np.arange(-RY, RY, TILE):
    for tx0 in np.arange(CX - RX, CX + RX, TILE):
        amp = rng.choice(AMPS_MM) * 1e-3
        if amp == 0.0:
            continue
        smooth = rng.uniform(0.10, 0.20)
        rmask = (ys >= ty0) & (ys < ty0 + TILE)
        cmask = (xs >= tx0) & (xs < tx0 + TILE)
        R, C = int(rmask.sum()), int(cmask.sum())
        pad = 40                       # convolve 'same' grows past the kernel
        tile = rng.uniform(0.0, 1.0, (R + 2 * pad, C + 2 * pad))
        tile = gauss_smooth(tile, smooth / CELL)
        r0 = (tile.shape[0] - R) // 2
        c0 = (tile.shape[1] - C) // 2
        tile = tile[r0:r0 + R, c0:c0 + C]
        tile -= tile.min()
        tile /= max(float(np.ptp(tile)), 1e-9)
        field[np.ix_(rmask, cmask)] = tile * amp

# one global smoothing pass so tile borders are steps of the same scale as
# the bumps themselves, not cliffs
field = gauss_smooth(field, 0.04 / CELL)
field = np.clip(field, 0.0, None)

np.savez(OUT, field=field.astype(np.float32),
         nrow=NROW, ncol=NCOL, rx=RX, ry=RY, cx=CX,
         max_amp=float(field.max()))
print(f"mosaic -> {OUT}  ({NROW}x{NCOL}, max height {field.max()*1000:.1f} mm,"
      f" mean {field.mean()*1000:.2f} mm)")
