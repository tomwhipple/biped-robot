"""The lift gate (gaitplay::LiftGate) in the dwell gait's SIL (experiments/roll-balance/sil_gait.run: the firmware's
track + trim + gate against the sim plant, the deployed code in the loop).

 1. nominal dwell gaits (T 4 and T 5, fwd / back / march): the gate never holds, and the run is the run without it
    (the same per-step reports, the same finish).
 2. a roll-rate kick in the double support before the first lift-off (the body rocks): the gate parks the clock, the
    step goes on after it settles (no timeout); and the same with a kick that leaves the body off level for good,
    where it times out and the track ends with both feet down -- no fall, where the run without the gate falls.
"""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
RB = os.path.join(ROOT, "experiments", "roll-balance")
sys.path.insert(0, RB)

KW = dict(model="dwell", seed=(5.2, 3.4), amp=(1.3, 1.3), gkw=dict(pelvis_x="stance", ankle_ff=(2.0, 2.0)))


def _run(T, direction, gate, **kw):
    try:
        import sil_gait as G
        return G.run(T, direction, "fit", 0.0, lift_gate=gate, **KW, **kw)
    except OSError:
        pytest.skip("libctrl_sil.so not built (make sil)")
    except AttributeError:
        pytest.skip("libctrl_sil.so predates the lift gate (make sil)")


@pytest.mark.slow
@pytest.mark.parametrize("T", [4.0, 5.0])
@pytest.mark.parametrize("direction", ["fwd", "back", "march"])
def test_nominal_gait_is_untouched(T, direction):
    a = _run(T, direction, False, n_steps=4)
    b = _run(T, direction, True, n_steps=4)
    assert b["gate"]["gates"] >= 4                                      # a gate frame at every lift-off
    assert b["gate"]["holds"] == 0 and b["gate"]["held_ms"] == 0 and b["gate"]["timeouts"] == 0
    assert a["gate"]["gates"] == 0
    assert not a["fell"] and not b["fell"] and a["done"] and b["done"]
    assert [(r["stance"], r["t"], r["lean_in"], r["lean_out"]) for r in a["reports"]] == \
           [(r["stance"], r["t"], r["lean_in"], r["lean_out"]) for r in b["reports"]]
    assert a["frames"] == b["frames"] and a["peak_tilt"] == b["peak_tilt"]


@pytest.mark.slow
def test_a_rocking_body_is_waited_for():
    b = _run(4.0, "fwd", True, n_steps=3, dist=(1.56, 1.58, 200.0, 1.0))
    g = b["gate"]
    assert g["holds"] >= 1 and g["timeouts"] == 0 and 150 <= g["held_ms"] < 3000
    assert not b["fell"] and b["done"] and len(b["reports"]) >= 3       # every step still happens


@pytest.mark.slow
def test_a_body_left_off_level_times_out_without_a_fall():
    kick = dict(n_steps=3, dist=(1.62, 1.64, 290.0, 1.0))
    a = _run(4.0, "fwd", False, **kick)
    b = _run(4.0, "fwd", True, **kick)
    assert a["fell"]                                                     # lifting on that body is a fall
    g = b["gate"]
    assert g["timeouts"] == 1 and g["held_ms"] == 3000
    assert not b["fell"] and b["done"] and len(b["reports"]) == 0       # no foot was lifted; both feet down
