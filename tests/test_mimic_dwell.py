"""walker_env.dwell_swing: the double-support dwell of the imitation reference (2026-11-19).

The JAX mirror (env_mjx._dwell_swing) is parity-gated in sim/mjx/parity_test.py (R22)."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))
from walker_env import dwell_swing  # noqa: E402


def test_feet_down_in_the_band_and_full_mid_swing():
    D = 0.3
    a = 0.5 * math.pi * D
    for ph in (0.0 + 1e-6, 0.9 * a, -0.9 * a, math.pi - 0.5 * a, -math.pi + 0.5 * a):
        l, r, _ = dwell_swing(ph, D)
        assert abs(l) < 1e-12 and abs(r) < 1e-12, ph
    l, r, _ = dwell_swing(math.pi / 2, D)
    assert abs(l - 1.0) < 1e-12 and r == 0.0
    l, r, _ = dwell_swing(-math.pi / 2, D)
    assert l == 0.0 and abs(r - 1.0) < 1e-12


def test_shift_crosses_in_the_band_and_holds_through_the_swing():
    D = 0.3
    a = 0.5 * math.pi * D
    assert dwell_swing(0.75 * a, D)[2] == 1.0          # over R before L lifts
    assert dwell_swing(-0.75 * a, D)[2] == -1.0        # over L before R lifts
    for ph in np.linspace(a, math.pi - a, 50):
        assert dwell_swing(ph, D)[2] == 1.0
    assert abs(dwell_swing(1e-9, D)[2]) < 1e-6


def test_swing_is_the_plain_sine_shape_compressed():
    """The swing keeps the sin shape over the middle 1 - D of the half cycle."""
    D = 0.3
    a = 0.5 * math.pi * D
    for f in (0.1, 0.25, 0.5, 0.75, 0.9):
        ph = a + f * (math.pi - 2 * a)
        assert abs(dwell_swing(ph, D)[0] - math.sin(math.pi * f)) < 1e-12
