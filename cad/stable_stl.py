"""Write an STL only when its geometry changed.

OCC tessellates the same solid a little differently on different machines
(macOS arm64 vs Linux x86_64): the facets arrive in another order, some quads
are split along the other diagonal, and some flat faces even get a different
number of vertices. So a rebuild on another machine rewrote committed STLs with
new bytes for the same solid, and `make cad` / `make plant` had to be run on
the machine that last built them.

export_stl_stable() exports to a temporary file beside the target and keeps the
existing file's bytes when the two are the same solid: the same area, volume,
centroid, second moments and bounding box -- what the plant takes from a mesh,
none of it dependent on triangulation -- to REL of each one's scale (TOL_MM for
positions). It then touches the kept file, so make sees it as fresh. A real
edit moves at least one of those and is written as before; a file it cannot
read is rewritten (the old behaviour), so a stale one is never kept.
"""
from __future__ import annotations

import os
import tempfile

import numpy as np

REL = 1e-7      # of each property's own scale; platform noise measured <= 2e-9 (2026-11-04)
TOL_MM = 1e-4   # centroid and bounding box


def _triangles(path):
    with open(path, "rb") as f:
        b = f.read()
    if len(b) < 84:
        return None
    n = int.from_bytes(b[80:84], "little")
    if n == 0 or len(b) != 84 + 50 * n:   # not binary STL (or truncated): can't judge
        return None
    rec = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    return np.frombuffer(b, dtype=rec, count=n, offset=84)["v"].astype(np.float64)


def mass_properties(t):
    """(area, volume, centroid, second moments about the centroid, bbox lo, bbox hi)
    of a closed triangle mesh: none of them depend on how its faces were triangulated."""
    a, b, c = t[:, 0], t[:, 1], t[:, 2]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1).sum()
    v = np.einsum("ij,ij->i", a, np.cross(b, c)) / 6.0       # signed tetrahedra from the origin
    vol = v.sum()
    s = a + b + c
    com = (v[:, None] * s).sum(0) / 4.0 / vol
    # int x x^T dV over each tetrahedron (0, a, b, c) = V/20 (a a^T + b b^T + c c^T + s s^T)
    outer = lambda u: u[:, :, None] * u[:, None, :]           # noqa: E731
    m2 = (v[:, None, None] * (outer(a) + outer(b) + outer(c) + outer(s))).sum(0) / 20.0
    m2 -= vol * np.outer(com, com)
    pts = t.reshape(-1, 3)
    return area, vol, com, m2, pts.min(0), pts.max(0)


def same_geometry(path_a, path_b) -> bool:
    """True when the two STLs are the same solid up to facet order and triangulation."""
    ta, tb = _triangles(path_a), _triangles(path_b)
    if ta is None or tb is None:
        return False
    A, B = mass_properties(ta), mass_properties(tb)
    close = lambda x, y: abs(x - y) <= REL * abs(x)           # noqa: E731
    return (close(A[0], B[0]) and close(A[1], B[1])
            and np.abs(A[2] - B[2]).max() <= TOL_MM
            and np.abs(A[3] - B[3]).max() <= REL * np.abs(A[3]).max()
            and np.abs(A[4] - B[4]).max() <= TOL_MM and np.abs(A[5] - B[5]).max() <= TOL_MM)


def replace_if_changed(new_path, path) -> bool:
    """Move new_path onto path unless path already holds the same geometry
    (then drop new_path and touch path). Returns True when path was rewritten."""
    if os.path.exists(path) and same_geometry(new_path, path):
        os.remove(new_path)
        os.utime(path, None)
        return False
    os.replace(new_path, path)
    return True


def export_stl_stable(shape, path, **kw) -> bool:
    """build123d.export_stl, but an unchanged part keeps its committed bytes."""
    from build123d import export_stl
    d = os.path.dirname(os.path.abspath(path))
    fd, tmp = tempfile.mkstemp(suffix=".stl", prefix=".tmp_", dir=d)
    os.close(fd)
    try:
        export_stl(shape, tmp, **kw)
        return replace_if_changed(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.remove(tmp)
