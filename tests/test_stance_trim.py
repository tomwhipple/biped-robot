"""The stance-hip trim's Python twin (sim/stance_trim.py) and the C++ host test's golden rows
(firmware/host/test_stance_trim.cpp) agree: the rows the twin prints are the rows the C++ test replays."""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))

import stance_trim as ST  # noqa: E402


def test_golden_rows_match_the_cpp_test():
    src = open(os.path.join(HERE, "..", "firmware", "host", "test_stance_trim.cpp")).read()
    rows = re.findall(r'\{"(\w+)", (-?[\d.]+)f, (-?[\d.]+)f, (\d+), (-?[\d.]+)f, (-?[\d.]+)f\}', src)
    gold = ST.golden()
    assert len(rows) == len(gold) > 40
    for (ev, roll, ramp, st, hike, t), g in zip(rows, gold):
        assert ev == g[0] and int(st) == g[3]
        assert abs(float(roll) - g[1]) < 1e-6 and abs(float(ramp) - g[2]) < 1e-6
        assert abs(float(hike) - g[4]) < 1e-6 and abs(float(t) - g[5]) < 1e-6


def test_signs_and_mirror():
    for side, lean in (("L", 0.05), ("R", -0.05)):
        tr = ST.StanceTrim()
        tr.begin(side, 0.0)
        tr.update(lean, 0.02, 1.0)
        assert tr.hike > 0 and tr.T[side] > 0        # leaning toward the swing foot: hike up
        assert (tr.hip_joint() > 0) == (side == "L")  # the joint frame mirrors
