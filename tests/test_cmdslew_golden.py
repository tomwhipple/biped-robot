"""The firmware's command slew (firmware/host/test_cmdslew.cpp) is pinned to sim/cmd_kernel.py (#266):
the literal golden table, rates, tick and target steps in the C++ test must equal the Python law's.
Drift on either side fails here (and the C++ test asserts the firmware matches its own table)."""
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))

import cmd_kernel  # noqa: E402

SRC = (ROOT / "firmware" / "host" / "test_cmdslew.cpp").read_text()
NUM = r"[-+]?(?:\d+\.?\d*(?:e[-+]?\d+)?)f"


def _floats(text):
    return [float(x[:-1]) for x in re.findall(NUM, text)]


def _block(start, end):
    m = re.search(re.escape(start) + r"(.*?)" + re.escape(end), SRC, re.S)
    assert m, f"{start!r} not found in test_cmdslew.cpp"
    return m.group(1)


def test_golden_table_matches_python():
    body = _block("// BEGIN GOLDEN TABLE", "// END GOLDEN TABLE")
    vals = np.asarray(_floats(body), np.float32)
    want = cmd_kernel.golden()
    assert vals.size == want.size
    np.testing.assert_array_equal(vals.reshape(want.shape), want)


def test_golden_constants_match_python():
    m = re.search(r"kGoldenRates\[3\]\s*=\s*\{(.*?)\};", SRC, re.S)
    assert m
    assert np.float32(_floats(m.group(1))).tolist() == np.float32(cmd_kernel.GOLDEN_RATES).tolist()
    m = re.search(r"kGoldenDt\s*=\s*(" + NUM + ")", SRC)
    assert m
    assert np.float32(_floats(m.group(1))[0]) == np.float32(cmd_kernel.GOLDEN_DT)
    m = re.search(r"kGoldenSteps\[\]\s*=\s*\{(.*?)\n\};", SRC, re.S)
    assert m
    steps = re.findall(r"\{\s*(\d+)\s*,\s*\{([^}]*)\}\s*\}", m.group(1))
    got = [(int(t), tuple(np.float32(_floats(v)).tolist())) for t, v in steps]
    want = [(t, tuple(np.float32(v).tolist())) for t, v in cmd_kernel.GOLDEN_STEPS]
    assert got == want
    m = re.search(r"kGoldenTicks\s*=\s*(\d+)", SRC)
    assert m and int(m.group(1)) == len(cmd_kernel.golden())


def test_the_limp_branch_resets_the_slew():
    """ctrl_task.cpp: every limp tick puts the slew back at the stand (torque can return without a
    re-arm: a link E-stop cleared by a disabled frame), and the (re)arm does too. The firmware's
    main loop is not host-built, so its wiring is pinned here, by its source."""
    src = (ROOT / "firmware" / "main" / "ctrl_task.cpp").read_text()
    m = re.search(r"\n        if \(limp\) \{\n(.*?)\n            continue;\n        \}\n", src, re.S)
    assert m, "the limp branch of ctrl_task.cpp moved: re-check that it resets g_cmd_slew"
    assert "g_cmd_slew.reset();" in m.group(1)
    arm = re.search(r"g_cmd_slew\.configure\(cmdslew::kDeployedRates, obs::kControlDt\);\s*"
                    r"g_cmd_slew\.reset\(\);", src)
    assert arm
