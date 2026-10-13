"""Smoke-test the vendored ngspice end to end.

One small DC sweep through the real TI CSD17556Q5B subcircuit (the
rewritten-for-ngspice form in `vendor/`). If this fails, every other
circuit-level test in the suite is unreachable — better to fail cheaply here.
"""
from __future__ import annotations

import numpy as np
import pytest

from sim import ngsim
from sim.fetmodel import REWRITE_LIB, check_hashes


def test_vendored_hashes_are_pinned():
    # A swapped binary or a modified rewrite must fail loudly. If these
    # change, the change is reviewed, and the pins here are updated.
    statuses = check_hashes()
    for k, v in statuses.items():
        assert v == "ok", f"{k}: {v}"


@pytest.mark.spice
def test_ngspice_loads_the_ti_fet_model(ngspice):
    ng = ngspice
    net = f"""\
* TI CSD17556Q5B Vgs sweep, Vds=10V
.include {REWRITE_LIB}
Vds 1 0 10
Vgs 2 0 0
X1 1 2 0 CSD17556Q5B
.dc Vgs 1.5 3.0 0.25
.end
"""
    out = ng.run_netlist(net)
    # The scale vector is Vgs's sweep itself.
    sweep = out["v-sweep"] if "v-sweep" in out else out.get("vgs")
    # Find by name rather than guess spelling.
    names = set(out.keys())
    assert any(n.startswith("v") for n in names), f"no sweep vector: {names}"

    # Drain current: find the vector named after Vds's current. ngspice
    # names it `i(vds)` for some builds, `vds#branch` for others.
    candidates = [n for n in names if "vds" in n.lower() and n != "vds"]
    id_vec = None
    for c in candidates:
        v = out[c]
        if len(v) and np.all(np.isfinite(v)):
            id_vec = v
            break
    assert id_vec is not None, f"could not identify Id vector: {names}"

    # Below threshold (Vgs=1.5V) the part must conduct microamps at most;
    # by Vgs=3V with Vds=10V a healthy CSD17556Q5B is well into conduction.
    # ngspice reports current flowing INTO the positive terminal of Vds,
    # so on a sourcing supply this is negative; assert by magnitude.
    # Below threshold (Vgs=1.5V) the part conducts weak subthreshold current
    # (tens of mA at most at Vds=10V; the model's NFACTOR=1.7 means real
    # subthreshold current, not a hard cutoff). By Vgs=3V with Vds=10V a
    # healthy CSD17556Q5B is fully on with ~A of drain current. ngspice
    # reports current flowing INTO the positive terminal of Vds, so on a
    # sourcing supply it is negative; assert by magnitude.
    assert abs(id_vec[0]) < 0.05, (
        f"at Vgs=1.5 expected near-blocking, got {id_vec[0]} A"
    )
    assert abs(id_vec[-1]) > 0.5, (
        f"at Vgs=3.0 expected strong conduction, got {id_vec[-1]} A"
    )
    # monotonicity in between
    mag = np.abs(id_vec)
    diffs = np.diff(mag)
    assert np.all(diffs >= -1e-9), f"Id not monotone over sweep: {mag}"
