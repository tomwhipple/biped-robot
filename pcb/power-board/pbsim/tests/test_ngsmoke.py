"""Smoke-test the vendored ngspice end to end.

One small DC sweep through the real TI CSD17556Q5B subcircuit (the
rewritten-for-ngspice form in `vendor/`). If this fails, every other
circuit-level test in the suite is unreachable — better to fail cheaply here.
"""
from __future__ import annotations

import numpy as np
import pytest

from pbsim.fetmodel import REWRITE_LIB, check_hashes


def test_vendored_hashes_are_pinned():
    # A swapped binary or a modified rewrite must fail loudly. If these
    # change, the change is reviewed, and the pins in fetmodel.py are
    # updated.
    statuses = check_hashes()
    for k, v in statuses.items():
        assert v == "ok", f"{k}: {v}"


def test_rewrite_differs_from_ti_only_in_level_and_version():
    """TI's .PARAM expressions evaluated and substituted, the rewrite and
    TI's release agree token for token except LEVEL 7 -> 8 and VERSION
    3.2 -> 3.3 (vendor/README.md)."""
    from pbsim.rewrite_ti_model import INTENDED, rewrite_deltas
    assert rewrite_deltas() == INTENDED


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

    # Below threshold (Vgs=1.5V) the part conducts weak subthreshold current
    # (about 10 mA at Vds=10V; the model's NFACTOR=1.7 means real
    # subthreshold current, not a hard cutoff). By Vgs=3V with Vds=10V the
    # channel carries hundreds of amps (332 A; TI's Figure 3 is past 200 A
    # at 2.94 V — see test_ti_fet_model_tracks_ti_figure_3). ngspice
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


# TI CSD17556Q5B datasheet (SLPS392D) Figure 3, Transfer Characteristics,
# VDS = 5 V, TC = 25 °C: the gate voltage at which the typical curve
# crosses each drain current, read off the plot (±0.03 V reading error).
TI_FIG3_VGS_AT_ID = {20.0: 2.37, 100.0: 2.71, 200.0: 2.94}


@pytest.mark.spice
def test_ti_fet_model_tracks_ti_figure_3(ngspice):
    """The rewritten model reproduces TI's own transfer curve, so its
    absolute currents (hundreds of amps at Vgs = 3 V) are the device's,
    not an artefact of the LEVEL 7 -> 8 rewrite."""
    vgs = np.round(np.arange(2.0, 3.2001, 0.01), 2)
    out = ngspice.run_netlist(f"""\
* TI CSD17556Q5B transfer curve, Vds=5V (datasheet Figure 3)
.include {REWRITE_LIB}
Vds 1 0 5
Vgs 2 0 0
X1 1 2 0 CSD17556Q5B
.dc Vgs 2.0 3.2 0.01
.end
""")
    names = [n for n in out if "vds" in n.lower() and n != "vds"]
    assert names, f"could not identify Id vector: {set(out)}"
    id_mag = np.abs(out[names[0]])
    assert len(id_mag) == len(vgs)
    assert np.all(np.diff(id_mag) > 0)
    for i_d, vgs_ti in TI_FIG3_VGS_AT_ID.items():
        vgs_model = float(np.interp(i_d, id_mag, vgs))
        assert abs(vgs_model - vgs_ti) < 0.10, (
            f"model reaches {i_d:.0f} A at Vgs={vgs_model:.3f} V; "
            f"TI Figure 3 at {vgs_ti} V"
        )
