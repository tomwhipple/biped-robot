"""pytest wiring for tests/ (the pre-push hook's link-tests gate).

JAX is pinned to the CPU before any test module imports it, as sim/sil's
conftest does for the SIL suite. The gates are CPU checks, and on the
training box a test that let JAX pick the GPU opened a CUDA context beside
the night campaign's run (AGENTS.md, "GPU BOUNDARY"). Subprocesses the tests
spawn inherit it. Most test modules set the same default themselves; this
covers the ones that do not. Set JAX_PLATFORMS explicitly to override.
"""
import os

os.environ.setdefault("JAX_PLATFORMS", "cpu")
# torch too (test_mac_train): its trainer runs on the CPU, but torch.accelerator
# still opens a CUDA context, which OOMs whenever ollama holds the 12 GB card
# (2026-11-03: 11.6 GB, all three mac_train tests errored at setup). The gates
# are CPU checks; set CUDA_VISIBLE_DEVICES explicitly to override.
os.environ.setdefault("CUDA_VISIBLE_DEVICES", "")
