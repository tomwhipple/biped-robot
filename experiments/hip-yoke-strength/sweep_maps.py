"""2D clearance maps around the roll horn arm: the swept neighbours (hip roll
for link 1, hip pitch for link 3), intersected with a region box per band and
projected onto a plane, as occupancy rasters. Writes sweep_maps.npz + a PNG.

    cd <worktree> && python sweep_maps.py
"""
import os
import sys
from concurrent.futures import ProcessPoolExecutor

WT = os.getcwd()
for p in (os.path.join(WT, "cad", "v6"), os.path.join(WT, "cad"), WT):
    sys.path.insert(0, p)
OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                       "build", "experiments", "hip-yoke-strength")
os.makedirs(OUT, exist_ok=True)
import numpy as np  # noqa: E402

RES = 0.25
# (name, region box x0,x1,y0,y1,z0,z1, projection axes (u, v) as indices into xyz)
MAPS = [
    ("front_arm_band y-10..12", (12, 32, -10, 12, -26, -2), (0, 2)),
    ("front_y12..17", (12, 32, 12, 17, -26, -2), (0, 2)),
    ("front_y17..23.45", (12, 32, 17, 23.45, -26, -2), (0, 2)),
    ("side_x-14..16", (-14, 16, 6, 26, -26, -2), (1, 2)),
    ("side_x16..21.45", (16, 21.45, 6, 26, -26, -2), (1, 2)),
    ("side_x21.45..27.45", (21.45, 27.45, 6, 26, -26, -2), (1, 2)),
]


def job(args):
    mname, box, (iu, iv), joint, nb, ang = args
    from build123d import Pos, Rot
    import probe_space as PS
    from common import shapes
    N = PS.neighbours()
    s = N[joint][nb]
    q = PS.posed(joint, s, ang, N["pz"])
    region = shapes.box(*box)
    try:
        inter = q & region
        if inter.volume < 1e-6:
            return mname, None
        verts, tris = inter.tessellate(0.05, 0.3)
    except Exception:  # noqa: BLE001
        return mname, None
    P = np.array([[v.X, v.Y, v.Z] for v in verts])
    return mname, (P[:, [iu, iv]], np.array(tris))


def raster(mname, box, iu, iv, parts):
    lo = (box[2 * iu], box[2 * iv]); hi = (box[2 * iu + 1], box[2 * iv + 1])
    nu = int(round((hi[0] - lo[0]) / RES)) + 1; nv = int(round((hi[1] - lo[1]) / RES)) + 1
    gu = lo[0] + np.arange(nu) * RES; gv = lo[1] + np.arange(nv) * RES
    occ = np.zeros((nv, nu), bool)
    for P, T in parts:
        for t in T:
            a, b, c = P[t[0]], P[t[1]], P[t[2]]
            u0, u1 = min(a[0], b[0], c[0]), max(a[0], b[0], c[0])
            v0, v1 = min(a[1], b[1], c[1]), max(a[1], b[1], c[1])
            i0, i1 = np.searchsorted(gu, u0), np.searchsorted(gu, u1, "right")
            j0, j1 = np.searchsorted(gv, v0), np.searchsorted(gv, v1, "right")
            if i1 <= i0 or j1 <= j0:
                continue
            UU, VV = np.meshgrid(gu[i0:i1], gv[j0:j1])
            d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
            if abs(d) < 1e-12:
                continue
            l1 = ((b[1] - c[1]) * (UU - c[0]) + (c[0] - b[0]) * (VV - c[1])) / d
            l2 = ((c[1] - a[1]) * (UU - c[0]) + (a[0] - c[0]) * (VV - c[1])) / d
            l3 = 1 - l1 - l2
            occ[j0:j1, i0:i1] |= (l1 >= -1e-9) & (l2 >= -1e-9) & (l3 >= -1e-9)
    return gu, gv, occ


if __name__ == "__main__":
    import probe_space as PS
    N = PS.neighbours()
    jobs = []
    for mname, box, axes in MAPS:
        for joint, key in (("roll", "hip_roll"), ("pitch", "hip_pitch")):
            lo, hi = N["rom"][key]
            for ang in np.linspace(lo, hi, 43):
                for nb in N[joint]:
                    jobs.append((mname, box, axes, joint, nb, float(ang)))
    parts = {m: [] for m, _, _ in MAPS}
    with ProcessPoolExecutor(max_workers=os.cpu_count() or 4) as ex:
        for mname, res in ex.map(job, jobs, chunksize=4):
            if res is not None:
                parts[mname].append(res)
    out = {}
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(2, 3, figsize=(18, 11))
    for ax, (mname, box, (iu, iv)) in zip(axs.flat, MAPS):
        gu, gv, occ = raster(mname, box, iu, iv, parts[mname])
        out[mname] = (gu, gv, occ)
        ax.imshow(occ, origin="lower", extent=(gu[0], gu[-1], gv[0], gv[-1]), cmap="Reds", alpha=0.8)
        ax.set_title(mname); ax.set_xlabel("xyz"[iu]); ax.set_ylabel("xyz"[iv]); ax.grid(True, alpha=0.4)
        ax.set_xticks(np.arange(np.ceil(gu[0]), gu[-1] + 0.1, 2)); ax.set_yticks(np.arange(np.ceil(gv[0]), gv[-1] + 0.1, 2))
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "sweep_maps.png"), dpi=80)
    np.savez(os.path.join(OUT, "sweep_maps.npz"), **{k.replace(" ", "_"): np.array(v[2]) for k, v in out.items()},
             **{k.replace(" ", "_") + "_u": v[0] for k, v in out.items()},
             **{k.replace(" ", "_") + "_v": v[1] for k, v in out.items()})
    print("wrote sweep_maps.png")
