"""Printability (filament-sag) audit: every part, in its PRINT orientation.

Motivated by the first foot print: the TPU-pad recess ceiling bridged 46 mm
and drooped badly in PETG, and nothing in the pipeline would have caught it --
check_assembly.py proves parts don't collide, not that they print. This closes
that gap: it loads each exported STL, rotates it into the orientation it is
actually printed in (per PARTS in parts.py / PRINT_LIST.md), and hunts for
geometry a PETG print will sag or fail on:

Down-facing facet clusters (steeper than 45 deg, above the first layer) are
classified by how their surroundings support them (perimeter ray test):

  CEILING  anchored all around (cavity roof / hole top) -> prints as a bridge;
           fails when the short span exceeds what PETG bridges (the
           foot-recess failure mode was a 46 mm ceiling)
  LEDGE    anchored on one side (rib / step underside) -> droops if it
           protrudes far; fails beyond ~1 mm of unsupported reach
  ISLAND   anchored nowhere below (floating start) -> always fails
  bore-top small horizontal-hole roof -> note: teardrop it
  CONTACT  first-layer area much smaller than the part footprint (part
           standing on pins/bosses)

Run:  .venv/bin/python cad/check_printability.py          # all parts
      .venv/bin/python cad/check_printability.py foot     # one part

Exit code 1 if any finding exceeds the FAIL thresholds, so it can gate like
check_assembly.py. Thresholds are PETG-tuned: short bridges (<= 8 mm) print
fine with 3 perimeters; bore tops <= 5 mm are noted (teardrop them) but only
fail if wider.
"""
import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
STL = os.path.join(HERE, "stl")

# print orientation: model -> print transform (rows = print axes in model coords)
RX180 = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)
RY_XUP = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]], float)   # model +X -> print +Z
IDENT = np.eye(3)

ORIENT = {
    # name: (rotation, note)  -- keep in sync with parts.PARTS
    "pelvis": (RX180, "upside down: deck top on bed"),
    "yoke_roll": (IDENT, "flange face on bed, arms up"),
    "yoke_pitch": (RX180, "flange face on bed (modeled flipped)"),
    "leg_link": (RY_XUP, "on its back: web face on bed"),
    "foot": (IDENT, "sole down"),
    "tower": (RX180, "upside down: top plate on bed"),
    "gopro_base": (IDENT, "base down, prongs up"),
}

# what actually goes to the slicer, where that differs from <name>.stl
PRINT_STL = {"leg_link": "leg_link_print.stl"}

COS45 = np.cos(np.radians(45.0))          # facet is a >45 deg overhang if
                                          # nz < -COS45 (straight down = -1)
Z_FIRST = 0.30                            # first-layer band above the bed
BRIDGE_OK = 8.0                           # PETG bridges this span fine (mm)
BORE_NOTE = 5.0                           # small bore tops: note, don't fail
LEDGE_OK = 1.2                            # unsupported ledge reach that's fine
ISLAND_AREA_OK = 3.0                      # mm^2 of floating facets we ignore
CONTACT_MIN_FRAC = 0.25                   # first layer >= this of footprint


def load_stl(path):
    """Binary (or ASCII) STL -> (N,3,3) float array of triangles."""
    with open(path, "rb") as f:
        head = f.read(84)
        if len(head) < 84 or head[:5] == b"solid" and b"facet" in f.read(200):
            return _load_ascii(path)
        n = struct.unpack("<I", head[80:84])[0]
        data = np.frombuffer(f.read(n * 50), dtype=np.uint8)
    if data.size != n * 50:
        return _load_ascii(path)
    tri = data.reshape(n, 50)[:, 12:48].copy().view("<f4").reshape(n, 3, 3)
    return tri.astype(np.float64)


def _load_ascii(path):
    pts = []
    with open(path) as f:
        for line in f:
            s = line.split()
            if s and s[0] == "vertex":
                pts.append([float(v) for v in s[1:4]])
    return np.array(pts).reshape(-1, 3, 3)


def clusters(idx, tris, tol=1e-3):
    """Group flagged triangles that share a vertex (union-find)."""
    parent = list(range(len(idx)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    key2tri = {}
    for i, t in enumerate(tris[idx]):
        for v in t:
            k = tuple(np.round(v / tol).astype(np.int64))
            j = key2tri.get(k)
            if j is None:
                key2tri[k] = i
            else:
                ra, rb = find(i), find(j)
                if ra != rb:
                    parent[ra] = rb
    groups = {}
    for i in range(len(idx)):
        groups.setdefault(find(i), []).append(idx[i])
    return list(groups.values())


def ray_down_dist(points, tris):
    """For each point, distance straight down to the nearest triangle below
    (np.inf if none). Brute-force Moller-Trumbore, vectorized over triangles."""
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    e1, e2 = v1 - v0, v2 - v0
    d = np.array([0.0, 0.0, -1.0])
    h = np.cross(d, e2)                          # (M,3)
    a = np.einsum("ij,ij->i", e1, h)
    ok0 = np.abs(a) > 1e-12
    out = np.full(len(points), np.inf)
    for pi, p in enumerate(points):
        s = p - v0
        u = np.einsum("ij,ij->i", s, h) / np.where(ok0, a, 1.0)
        q = np.cross(s, e1)
        v = q[:, 2] * -1.0 / np.where(ok0, a, 1.0)
        t = np.einsum("ij,ij->i", e2, q) / np.where(ok0, a, 1.0)
        hit = ok0 & (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9) & (t > 0.05)
        if hit.any():
            out[pi] = t[hit].min()
    return out


def audit(name, verbose=True):
    rot, note = ORIENT[name]
    fname = PRINT_STL.get(name, name + ".stl")
    tri = load_stl(os.path.join(STL, fname)) @ rot.T
    tri[:, :, 2] -= tri[:, :, 2].min()

    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    area2 = np.linalg.norm(n, axis=1)
    keep = area2 > 1e-9
    tri, n, area2 = tri[keep], n[keep], area2[keep]
    nz = n[:, 2] / area2
    area = area2 / 2
    zmin = tri[:, :, 2].min(axis=1)
    cen = tri.mean(axis=1)

    print(f"\n== {name}  [{note}]")
    findings = []

    # first-layer contact vs footprint
    contact = area[(nz < -0.9) & (zmin < Z_FIRST)].sum()
    span_x = tri[:, :, 0].max() - tri[:, :, 0].min()
    span_y = tri[:, :, 1].max() - tri[:, :, 1].min()
    foot_area = span_x * span_y
    frac = contact / foot_area
    print(f"  first-layer contact {contact:7.0f} mm2  "
          f"({100 * frac:4.1f}% of {span_x:.0f}x{span_y:.0f} bbox)")
    if frac < CONTACT_MIN_FRAC and contact < 600:
        findings.append(("CONTACT", contact,
                         f"part stands on {contact:.0f} mm2 first layer"))

    # down-facing facets above the first layer
    flagged = np.where((nz < -COS45) & (zmin > Z_FIRST))[0]
    for grp in clusters(flagged, tri):
        g = np.array(grp)
        ga = area[g].sum()
        lo = tri[g].reshape(-1, 3).min(axis=0)
        hi = tri[g].reshape(-1, 3).max(axis=0)
        sx, sy = hi[0] - lo[0], hi[1] - lo[1]
        span = min(sx, sy)                     # a bridge fails across its SHORT
        long_span = max(sx, sy)                # side only if both ends anchor
        # direct support under the cluster itself
        pts = cen[g][np.argsort(cen[g][:, 2])][:60] - [0, 0, 0.05]
        dist = ray_down_dist(pts, tri)
        supported = np.isfinite(dist).sum() > len(dist) * 0.5
        depth = np.median(dist[np.isfinite(dist)]) if np.isfinite(dist).any() else 0
        # perimeter ring: is the neighborhood anchored (ceiling) or not (ledge
        # / island)? sample a ring 1.5 mm outside the cluster bbox
        cx, cy = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2
        rx, ry = sx / 2 + 1.5, sy / 2 + 1.5
        ring = np.array([[cx + rx * np.cos(a), cy + ry * np.sin(a),
                          lo[2] - 0.05] for a in np.linspace(0, 2 * np.pi, 16,
                                                             endpoint=False)])
        fs = np.isfinite(ray_down_dist(ring, tri)).mean()
        z0 = lo[2]
        if supported:
            kind, bad = "CEILING", span > BRIDGE_OK
            extra = f"over {depth:.1f} mm cavity"
        elif fs >= 0.75:
            kind, bad = "CEILING", span > BRIDGE_OK
            extra = "roof of a through-void"
        elif fs >= 0.25:
            kind, bad = "LEDGE", span > LEDGE_OK
            extra = f"anchored one side ({100 * fs:.0f}% ring)"
        else:
            kind, bad = "ISLAND", ga > ISLAND_AREA_OK
            extra = "nothing below or around -> support"
        if kind == "CEILING" and long_span <= BORE_NOTE and ga < 40:
            kind, bad = "bore-top", False           # teardrop candidate
        desc = f"{ga:6.0f} mm2  {sx:5.1f} x {sy:5.1f} mm  at z {z0:5.1f}  {extra}"
        if bad:
            findings.append((kind, ga, desc))
        if verbose or bad:
            mark = "**" if bad else "  "
            print(f"  {mark}{kind:8s} {desc}")

    if not findings:
        print("  PASS")
    return findings


def main():
    names = sys.argv[1:] or list(ORIENT)
    bad = {}
    for name in names:
        f = audit(name)
        if f:
            bad[name] = f
    print()
    if bad:
        print("SAG RISKS FOUND:")
        for name, fs in bad.items():
            for kind, a, desc in fs:
                print(f"  {name:11s} {kind:8s} {desc}")
        sys.exit(1)
    print("ALL PARTS PRINT CLEAN (in their listed orientations)")


if __name__ == "__main__":
    main()
