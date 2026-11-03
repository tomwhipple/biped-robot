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
