"""Warp mempool hold (night fixer 2026-11-09).

robot_legs_prec28 died twice with CUDA OOM in Warp's convex_narrowphase
scratch allocation, each time minutes after ollama (Honcho's deriver) loaded
onto the GPU: Warp's mempool release threshold defaults to 0, so the memory
it frees between steps is free for ollama to take. hold_warp_mempool raises
the threshold for --mjx-impl warp runs. It must be a no-op for the JAX
backend and when turned off, and must never raise when no CUDA device exists.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim", "mjx"))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")


def test_jax_impl_is_noop():
    from train_mjx import hold_warp_mempool
    assert hold_warp_mempool("jax") is None


def test_opt_out(monkeypatch):
    from train_mjx import hold_warp_mempool
    monkeypatch.setenv("BIMO_WARP_MEMPOOL_HOLD", "0")
    assert hold_warp_mempool("warp") is None


def test_warp_without_cuda_does_not_raise():
    from train_mjx import hold_warp_mempool
    r = hold_warp_mempool("warp")   # CPU-only test env: None; on a GPU: bytes
    assert r is None or r > 0
