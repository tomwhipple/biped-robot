"""pytest wiring for the SIL suite.

JAX must be pinned to CPU before eval_precision imports it (the referee is a
CPU-plant convention, and a GPU init here would just be slow), and the `slow`
marker has to exist before collection so `-m "not slow"` works without a
pytest.ini outside sim/sil/.
"""
import os
import sys

os.environ.setdefault("JAX_PLATFORMS", "cpu")

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
for p in (HERE, SIM, os.path.join(SIM, "mjx")):
    if p not in sys.path:
        sys.path.insert(0, p)


def pytest_configure(config):
    config.addinivalue_line(
        "markers", "slow: closed-loop scenario runs (minutes, needs the lib)")
