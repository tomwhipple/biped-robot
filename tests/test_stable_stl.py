"""cad/stable_stl.py: a rebuild of an unchanged part keeps the committed STL's
bytes whatever order or triangulation the exporter picked; a real change is written."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "cad"))
import stable_stl as S  # noqa: E402

# a 20 x 10 x 5 mm box, outward-wound, two triangles per face
_V = np.array([[0, 0, 0], [20, 0, 0], [20, 10, 0], [0, 10, 0],
               [0, 0, 5], [20, 0, 5], [20, 10, 5], [0, 10, 5]], float)
_QUADS = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]


def _box(flip=False, offset=(0, 0, 0), scale=1.0):
    tris = []
    for a, b, c, d in _QUADS:
        tris += [(a, b, d), (b, c, d)] if flip else [(a, b, c), (a, c, d)]
    return (_V * scale + offset)[np.array(tris)]


def _write(path, tris):
    rec = np.zeros(len(tris), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["v"] = tris
    with open(path, "wb") as f:
        f.write(b"\0" * 80 + len(tris).to_bytes(4, "little") + rec.tobytes())


@pytest.fixture
def committed(tmp_path):
    p = tmp_path / "part.stl"
    _write(p, _box())
    return p


def _rebuild(tmp_path, committed, tris):
    before = committed.read_bytes()
    new = tmp_path / "new.stl"
    _write(new, tris)
    rewritten = S.replace_if_changed(str(new), str(committed))
    assert not new.exists()
    return rewritten, committed.read_bytes() == before


def test_reordered_and_rotated_facets_keep_the_bytes(tmp_path, committed):
    t = _box()[np.random.default_rng(0).permutation(12)]
    t = np.roll(t, 1, axis=1)                          # each facet starts at another vertex
    assert _rebuild(tmp_path, committed, t) == (False, True)


def test_other_diagonals_keep_the_bytes(tmp_path, committed):
    assert _rebuild(tmp_path, committed, _box(flip=True)) == (False, True)


def test_float32_noise_keeps_the_bytes(tmp_path, committed):
    t = _box() + np.random.default_rng(1).normal(0, 1e-6, (12, 3, 3))
    assert _rebuild(tmp_path, committed, t) == (False, True)


@pytest.mark.parametrize("tris", [_box(offset=(0.01, 0, 0)), _box(scale=1.001), _box()[::-1][:, ::-1][:10]])
def test_a_real_change_is_written(tmp_path, committed, tris):
    assert _rebuild(tmp_path, committed, tris) == (True, False)


def test_an_unreadable_file_is_replaced(tmp_path):
    p = tmp_path / "part.stl"
    p.write_text("solid ascii\nendsolid\n")
    new = tmp_path / "new.stl"
    _write(new, _box())
    assert S.replace_if_changed(str(new), str(p)) is True
    assert S.same_geometry(str(p), str(p))
