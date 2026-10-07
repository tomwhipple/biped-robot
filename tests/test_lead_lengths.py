"""Pins for cad/v6/lead_lengths.py -- the issue-#77 per-hop lead table.

The numbers the table prints are geometric minima; these tests pin them to
the constants the CAD modules export (so the table cannot rot silently) and
re-check that the script runs. The ASSEMBLY crosscheck (built mocks,
build123d) is run in cad/run_checks.sh / pre-push, not here.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "cad"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "cad", "v6"))

import lead_lengths as LL  # noqa: E402


def test_table_shape():
    rows = LL.rows()
    # 16 leg-branch hops (8 per side) + 7 upper-branch hops (4 on A incl.
    # the neck, 3 on B):
    assert len(rows) == 23
    assert sum(1 for r in rows if r["b"] in ("R elbow", "L elbow")) == 2


def test_no_hop_exceeds_stock_but_the_arm():
    for r in LL.rows():
        need = r["span"] + r["loop"]
        if r["b"] in ("R elbow", "L elbow"):
            assert need > LL.STOCK_LEAD, "the arm hop is the known long one"
        else:
            assert need <= LL.STOCK_LEAD, f"{r['a']} -> {r['b']} needs {need:.0f} > 150"


def test_arm_hop_is_the_measured_span():
    for r in LL.rows():
        if r["b"] in ("R elbow", "L elbow"):
            assert r["span"] == LL.SHOULDER_ELBOW_SPAN
            # both folds ride this hop:
            assert r["loop"] > 25.0


def test_spans_stable():
    # pin a few spans to their current derived values (mm, +-1):
    want = {("hip pitch", "knee"): 110.0, ("knee", "ankle"): 110.0,
            ("port", "deck slot"): 102.0, ("deck slot", "battery layer"): 48.1,
            ("hip yaw", "hip roll"): 61.3, ("hip roll", "hip pitch"): 55.3}
    for r in LL.rows():
        if (r["a"], r["b"]) in want:
            assert abs(r["span"] - want[(r["a"], r["b"])]) < 1.0, \
                f"{r['a']}->{r['b']}: {r['span']}"
