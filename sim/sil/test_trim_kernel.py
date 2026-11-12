"""sim/trim_kernel.py (the stance trim on a gait clock, numpy / jax) against the firmware's own
supportFromPhase + TrimDriver + StanceTrim (libctrl_sil `sil_trim_*`), tick by tick, and numpy against jax.

The sequence exercises every path: a clock that marches and stops (lift on / off), both stance sides, a
roll that leans both ways, a lean past the abort limit, a learned T that saturates, and the fade.
"""
import ctypes
import math
import os

import numpy as np
import pytest

import trim_kernel as K

LIB = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                   "firmware", "host", "build", "libctrl_sil.so")


def _lib():
    if not os.path.exists(LIB):
        pytest.skip("libctrl_sil.so not built (make sil)")
    L = ctypes.CDLL(LIB)
    if not hasattr(L, "sil_trim_step"):
        pytest.skip("libctrl_sil.so predates sil_trim_* (make sil)")
    L.sil_trim_new.restype = ctypes.c_void_p
    L.sil_trim_new.argtypes = [ctypes.c_float] * 6
    L.sil_trim_free.argtypes = [ctypes.c_void_p]
    L.sil_trim_set_learned.argtypes = [ctypes.c_void_p, ctypes.c_char, ctypes.c_float]
    L.sil_trim_step.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_int, ctypes.c_float, ctypes.c_float,
                                ctypes.POINTER(ctypes.c_float)]
    return L


def _sequence(n=2400, seed=0):
    rng = np.random.default_rng(seed)
    dt = 0.02
    f = 0.6                                       # Hz: a half cycle (one step) of 0.83 s
    phase = np.zeros(n)
    lift = np.zeros(n, bool)
    roll = np.zeros(n)
    ph = 0.3
    for i in range(n):
        seg = (i // 400) % 3
        lift[i] = seg != 2                        # marches, then stands (the clock keeps running)
        ph += 2 * math.pi * f * dt
        phase[i] = ph
        # a lean toward the swing foot that grows over the run, plus noise; one big lean past the abort
        sw = math.sin(ph)
        roll[i] = math.radians(-6.0 * sw * (0.3 + i / n) + rng.normal(0, 0.4))
        st, prog, _ = K.support(ph, lift[i])
        if 1500 <= i < 1700 and st != 0 and 0.3 < prog < 0.9:     # mid raise: past the abort limit
            roll[i] = math.radians(25.0)
    return phase, lift, roll, dt


def test_kernel_matches_firmware():
    L = _lib()
    phase, lift, roll, dt = _sequence()
    h = L.sil_trim_new(0.25, 1.0, 15.0, 14.0, 0.6, 15.0)
    L.sil_trim_set_learned(h, b"L", math.radians(5.2))
    L.sil_trim_set_learned(h, b"R", math.radians(3.4))
    s = K.trim_init((), math.radians(5.2), math.radians(3.4))
    out = (ctypes.c_float * 6)()
    worst = 0.0
    states = set()
    for i in range(len(phase)):
        # the firmware takes float32 inputs: feed the kernel the same values
        ph32, r32 = float(np.float32(phase[i] % (2 * math.pi))), float(np.float32(roll[i]))
        L.sil_trim_step(h, ph32, int(lift[i]), r32, dt, out)
        s, hl, hr = K.trim_step(s, r32, ph32, bool(lift[i]), dt)
        states.add(int(out[5]))
        assert int(out[5]) == int(s["mode"]), (i, out[5], s["mode"])
        for a, b in ((out[0], hl), (out[1], hr), (out[3], s["T_L"]), (out[4], s["T_R"])):
            worst = max(worst, abs(a - float(b)))
    L.sil_trim_free(h)
    assert worst < 2e-4, worst                   # rad: float32 vs float64 over 2400 ticks
    assert {0, 5, 6, 7, 8} <= states             # every path was taken


def test_numpy_matches_jax():
    jnp = pytest.importorskip("jax.numpy")
    phase, lift, roll, dt = _sequence(seed=1)
    B = 4
    sn = K.trim_init((B,), 0.09, 0.06)
    sj = K.trim_init((B,), 0.09, 0.06, xp=jnp)
    for i in range(600):
        ph = np.full(B, phase[i]) + np.arange(B) * 0.7
        lf = np.full(B, lift[i])
        r = np.full(B, roll[i]) * (1 + 0.1 * np.arange(B))
        sn, hln, hrn = K.trim_step(sn, r, ph, lf, dt)
        sj, hlj, hrj = K.trim_step(sj, jnp.asarray(r, jnp.float32), jnp.asarray(ph, jnp.float32), jnp.asarray(lf), dt,
                                   xp=jnp)
        assert np.allclose(hln, np.asarray(hlj), atol=2e-4) and np.allclose(hrn, np.asarray(hrj), atol=2e-4), i
        assert np.array_equal(np.asarray(sn["mode"]), np.asarray(sj["mode"])), i
