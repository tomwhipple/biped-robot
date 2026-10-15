"""Parity gate (the critical-path acceptance): trace capture must leave every
referee number unchanged.

Drives eval_precision.run_one TWICE on an identical deterministic fake env
(rollouts identical by construction — the same _FakeEnv reset + scripted
act), once with `_trace_dir` set and once without, and asserts the results
are identical everywhere except the added `res["trace"]` path. A capture
that perturbed the rollout (extra obs reads, stepping, command mutation)
would diverge immediately here.
"""
import os
import sys
import tempfile

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim", "mjx"))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))

import eval_precision as E                          # noqa: E402
from gradebook import trace as gb_trace             # noqa: E402


class _Data:
    def __init__(self):
        self.qpos = np.zeros(7)
        self.qvel = np.zeros(6)
        self.geom_xpos = np.zeros((2, 3))


class _FakeEnv:
    """Minimal BimoWalkerEnv double: enough surface for Driver + a scripted
    balance-L scenario. Deterministic: the same seed always plays the same
    tape, so the ONLY variance between two run_one() calls can come from the
    trace hook itself."""

    def __init__(self):
        self.data = _Data()
        self.control_dt = 0.04
        self.max_steps = 10                 # short episode
        self.episode_seconds = self.max_steps * self.control_dt
        self._sole_gids = (0, 1)
        self._sole_z0 = 0.0
        self._ncmd = 0
        self.obs_frame = 0
        self.mark_xy = (0.0, 0.0)
        self.observation_space = type("o", (), {"shape": (4,)})()
        self.action_space = type("a", (), {"shape": (2,)})()
        self._step = 0

    def reset(self, seed=None):
        self._step = 0
        rng = np.random.default_rng(seed)         # same seed -> same qpos
        self.data.qpos[:] = np.concatenate(
            [np.zeros(3), rng.standard_normal(4) * 1e-6])
        return np.zeros(4), {}

    def set_command(self, *cmd):
        self._cmd = np.asarray(cmd, float)

    def _foot_contacts(self):
        return True, True

    def step(self, a):
        self._step += 1
        self.data.qpos[0] += 0.001            # tiny crawl: real state motion
        self.data.qpos[2] = 0.31
        term = self._step >= 10
        info = dict(power_w=10.0, height=0.31, up_z=0.999,
                    foot_err=0.005, foot_clear=0.033, foot_dx=0.001,
                    foot_dz=-0.001, cmd_lift=0.0, com_stance=0.02,
                    recovered=1.0)
        return np.zeros(4), 0.0, term, False, info


def _act(obs):
    return np.zeros(2)


def _drive(trace_dir=None, scen="balance_L"):
    env = _FakeEnv()
    secs, factory, _ = E._registry()[scen]
    saved = dict(E._trace_dir)
    E._trace_dir["dir"] = trace_dir
    E._trace_dir["scenario"] = scen
    saved_root = E.TRACE_RUN_ROOT
    if trace_dir:
        E.TRACE_RUN_ROOT = os.path.dirname(trace_dir.rstrip("/"))
    try:
        res = E.run_one(env, _act, factory, seed=7, record=False,
                        N=0.31, mass=5.0)
    finally:
        E._trace_dir["dir"] = saved["dir"]
        E._trace_dir["scenario"] = saved["scenario"]
        E.TRACE_RUN_ROOT = saved_root
    return res


def _nan_eq(a, b):
    """Deep dict/list/scalar equality with NaN == NaN."""
    if isinstance(a, dict) and isinstance(b, dict):
        if set(a) != set(b):
            return False
        return all(_nan_eq(a[k], b[k]) for k in a)
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(_nan_eq(x, y) for x, y in zip(a, b))
    if isinstance(a, float) and isinstance(b, float):
        if a != a and b != b:                # both NaN
            return True
    try:
        return bool(a == b)
    except Exception:
        return False


def test_trace_capture_parity():
    """Same rollout, trace off vs trace on: identical verdict/metrics; the
    trace-on result only GAINS a 'trace' key."""
    r_off = _drive(trace_dir=None)
    tmp = tempfile.mkdtemp()
    r_on = _drive(trace_dir=os.path.join(tmp, "traces"))

    dropable = {"trace", "frames"}        # frames: [] both (record=False)
    off = {k: v for k, v in r_off.items() if k not in dropable}
    on = {k: v for k, v in r_on.items() if k not in dropable}
    assert _nan_eq(off, on)
    # the trace file exists and carries the episode
    assert "trace" in r_on and r_on["trace"].endswith("balance_L_s7.npz")
    t = gb_trace.load(os.path.join(tmp, "traces",
                                   os.path.basename(r_on["trace"])))
    assert t["meta"]["scenario"] == "balance_L"
    assert t["meta"]["seed"] == 7
    assert t["meta"]["n_steps"] == 10
    assert t["up_z"].shape == (10,)
    assert t["cmd"].shape == (10, 7)
    # the verdict rode along (a grader reads success from the trace, it does
    # not re-derive it)
    assert t["meta"]["success"] == r_off["success"]


def test_trace_off_writes_nothing(tmp_path):
    """Default path (no --trace) must not create the traces dir at all."""
    r = _drive(trace_dir=None)
    assert "trace" not in r
    assert not os.path.exists(os.path.join(os.getcwd(), "traces"))
