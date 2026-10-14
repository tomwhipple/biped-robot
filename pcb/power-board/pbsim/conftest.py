"""pytest wiring for pcb/power-board/pbsim.

- Adds this package's parent to sys.path so tests can say
  ``from pbsim.pack import ...`` (when pytest is invoked as
  ``python -m pytest pcb/power-board/pbsim`` from the repo root).
- Registers custom markers.
- Exposes a session-scoped vendored ngspice for tests that need it, and a
  session-scoped parsed schematic so the 102-component parse happens once.
"""
from __future__ import annotations

import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

# Make `import pbsim.xxx` work when pytest is invoked as
# `python -m pytest pcb/power-board/pbsim` from the repo root *or* from this
# directory.
PARENT = os.path.dirname(HERE)               # pcb/power-board/
GRANDPARENT = os.path.dirname(PARENT)        # pcb/
if PARENT not in sys.path:
    sys.path.insert(0, PARENT)


def pytest_configure(config):
    config.addinivalue_line(
        "markers",
        "spice: needs the vendored libngspice.so (test skipped if missing)",
    )
    config.addinivalue_line(
        "markers",
        "slow: multi-second transient runs (kept out of CI default)",
    )


@pytest.fixture(scope="session")
def schematic():
    """The parsed schematic: components, labels, nets, wires."""
    from pbsim.schematicsnapshot import load
    return load()


@pytest.fixture()
def ngspice():
    """A fresh NgSpiceSession per test. Skips, with the reason, when this
    host has no loadable libngspice (the vendored one is linux-x86_64
    only) or a vendored artifact fails its hash pin."""
    from pbsim import ngsim
    from pbsim.fetmodel import check_hashes
    lib = ngsim.find_lib()
    if lib is None:
        pytest.skip("no libngspice for this host: the vendored one is "
                    "linux-x86_64 only (macOS: brew install libngspice, "
                    "or set PBSIM_NGSPICE)")
    hashes = check_hashes()
    if lib != str(ngsim.LIB_SO):
        hashes.pop("libngspice.so.0.0.9", None)
    bad = {k: v for k, v in hashes.items() if v != "ok"}
    if bad:
        pytest.skip(f"vendored ngspice artifact hash mismatch: {bad}")
    with ngsim.NgSpiceSession() as sess:
        yield sess
