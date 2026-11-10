"""The power-lite board in ngspice (pcb/power-lite/sim/lite_sim.py, built from tools/parts.py).

Limits: TE V23134 (operate 7.2 V, release 1.6 V), onsemi PN2222A (VCEO 40 V, VEBO 6 V), ESP32 (VIL <= 0.25 VDD,
VIH >= 0.75 VDD at 3.3 V, ADC to ~3.1 V at 11 dB, a pin sourcing <= 20 mA), Littelfuse 0297015 (135 A^2s melting,
rev B record C13). The pack spans 9.0-12.6 V (3.0-4.2 V a cell).
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "pcb" / "power-lite" / "sim"))
import lite_sim as S  # noqa: E402

pytestmark = pytest.mark.skipif(S.NGSPICE is None, reason="ngspice not installed")
CELLS = (3.0, 3.7, 4.2)


@pytest.mark.parametrize("vcell", CELLS)
@pytest.mark.parametrize("estop", ("closed", "open"))
@pytest.mark.parametrize("servo_en", (3.3, 0.0, None))
def test_rail_logic_and_levels(vcell, estop, servo_en):
    r = S.op(vcell, estop, servo_en)
    pack = 3 * vcell
    on = estop == "closed" and servo_en == 3.3
    if on:
        assert r["servop"] >= 0.99 * pack
        assert r["vcoil"] >= 1.15 * 7.2, "coil below TE's operate voltage with margin"
        assert r["icoil"] >= S.OPERATE_A
        assert r["vce1"] <= 0.4, "Q1 not saturated"
        assert abs(r["igpio"]) <= 0.012, "SERVO_EN pin overloaded"
        assert 1.4 <= r["rail_v"] <= 3.1, "RAIL_V outside the ADC's useful range"
        assert 1e-3 <= r["iled2"] <= 20e-3
    else:
        assert r["servop"] <= 0.1, "servo rail on when it should be off"
        assert r["rail_v"] <= 0.05
    # ESTOP_N: low while the E-stop is released (coil supply up), high when it is pressed
    if estop == "closed":
        assert r["estop_n"] <= 0.25 * 3.3
    else:
        assert r["estop_n"] >= 0.75 * 3.3
    assert 1e-3 <= r["iled1"] <= 20e-3


@pytest.mark.parametrize("cause", ("estop", "servo_en"))
def test_release_is_clamped_and_prompt(cause):
    r = S.release(cause)
    assert r["trel"] <= 0.020, "relay slow to drop (coil inductance assumed 0.3 H)"
    assert r["vce1_max"] <= min(40.0, 3 * 3.7 + 1.5), "Q1 collector above VIN + a diode (or VCEO)"
    if cause == "estop":
        assert r["vhi_min"] >= -1.5, "COIL_HI driven far below ground: D1 is not clamping"
        assert r["vbe2_min"] >= -6.0, "Q2's base-emitter reverse past VEBO"


def test_getup_at_a_flat_pack_holds_the_relay():
    r = S.getup(3.0, 12.0)
    assert r["icoil_min"] >= 1.5 * S.RELEASE_A, "relay could drop during a get-up"
    assert r["logic_min"] >= 8.0 and r["servop_min"] >= 8.0


@pytest.mark.parametrize("servo_caps", ("200u", "2000u"))
def test_relay_closing_does_not_blow_f1(servo_caps):
    r = S.inrush(4.2, servo_caps)
    assert r["i2t_f1"] <= 0.1 * 135.0, "inrush near F1's melting I2t"
    assert r["tw100"] <= 320e-6, "inrush above 100 A for longer than a typical BMS short-circuit delay"
