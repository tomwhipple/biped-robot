"""The lift gate's Python twin (sim/lift_gate.py) and the C++ host test's golden rows
(firmware/host/test_lift_gate.cpp) agree: the rows the twin prints are the rows the C++ test replays."""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))

import lift_gate as LG  # noqa: E402


def test_golden_rows_match_the_cpp_test():
    src = open(os.path.join(HERE, "..", "firmware", "host", "test_lift_gate.cpp")).read()
    rows = re.findall(r"^\{(\d+), (\d+), (\d), (\d+), '(.)', (NAN|-?[\d.]+f?), (-?[\d.]+)f, (\d+), (\d), (\d), "
                      r"(-?[\d.]+)f, (\d)\},$", src, re.M)
    gold = LG.golden()
    assert len(rows) == len(gold) > 300
    for r, g in zip(rows, gold):
        assert [int(r[0]), int(r[1]), int(r[2]), int(r[3]), r[4]] == [g[0], g[1], g[2], g[3], g[4]]
        roll = float("nan") if r[5] == "NAN" else float(r[5].rstrip("f"))
        assert (roll != roll and g[5] != g[5]) or abs(roll - g[5]) < 1e-6
        assert abs(float(r[6]) - g[6]) < 1e-6
        assert [int(r[7]), int(r[8]), int(r[9])] == [g[7], g[8], g[9]]
        assert abs(float(r[10]) - g[10]) < 1e-6 and int(r[11]) == g[11]


def test_hold_release_and_timeout():
    g = LG.LiftGate()
    g.tick(0, 1000, False, 0, "", 0.0, 0.0)                    # the seed
    o = g.tick(100, 1100, True, 1100, "L", 0.1, 0.0)            # 5.7 deg off
    assert o.ev == LG.HOLD_START and o.holding and o.t_ms == 1100
    o = g.tick(200, 1200, False, 0, "", 0.0, 0.0)
    assert o.holding and o.t_ms == 1100                         # parked
    o = g.tick(360, 1360, False, 0, "", 0.0, 0.0)               # level and quiet for 160 ms
    assert o.ev == LG.HOLD_END and o.hold_ms == 260 and o.t_ms == 1100
    assert g.tick(380, 1380, False, 0, "", 0.0, 0.0).t_ms == 1120   # the clock goes on from the parked time
    g2 = LG.LiftGate()
    g2.tick(0, 1000, False, 0, "", 0.0, 0.0)
    g2.tick(100, 1100, True, 1100, "R", 0.1, 0.0)
    for w in range(120, 3100, 20):
        assert g2.tick(w, 1000 + w, False, 0, "", 0.1, 0.0).ev == LG.NONE
    assert g2.tick(3100, 4100, False, 0, "", 0.1, 0.0).ev == LG.TIMEOUT


def test_host_parses_the_gate_fields():
    sys.path.insert(0, os.path.join(HERE, "..", "experiments", "roll-balance"))
    import track_host as TH
    m = TH.TS_RE.search("TS 3 L in 2.1 out 0.4 e +0.3 T +5.0 hike +5.1 ok Tback G 420\r\n")
    assert m.group(10) == "420" and m.group(9) == " Tback"
    assert TH.TS_RE.search("TS 3 L in 2.1 out 0.4 e +0.3 T +5.0 hike +5.1 ok\r\n").group(10) is None     # an ungated step
    t = TH.GATE_TO_RE.search("TS 4 R GATE-TIMEOUT held 3000 ms dev -6.2 rate +3.0\r\n")
    assert t.groups() == ("4", "R", "3000", "-6.2", "+3.0")
    assert TH.TS_RE.search("TS 4 R GATE-TIMEOUT held 3000 ms dev -6.2 rate +3.0\r\n") is None


def test_the_dwell_gait_flags_one_frame_before_each_lift_off():
    sys.path.insert(0, os.path.join(HERE, "..", "experiments", "roll-balance"))
    import sil_gait as G
    sw, fr = G.gait(4.0, 4, first="L", model="dwell", lift_gate=True)
    flagged = [(t, h) for t, h, k in fr if k == "kf" and int(h[5], 16) & G.GATE]
    assert len(flagged) == len(sw.phases) == 4 + (len(sw.phases) - 4)
    for (t, h), p in zip(flagged, sw.phases):
        t_lo = 1e3 * (p["t0"] - sw.t_first)
        assert t <= t_lo + 1e-6 and t_lo - t < 100          # the last 100 ms frame at or before the lift-off
    _, plain = G.gait(4.0, 4, first="L", model="dwell")
    assert [h[:5] + h[6:] for _, h, _ in fr] == [h[:5] + h[6:] for _, h, _ in plain]   # nothing else differs
    assert not any(int(h[5], 16) & G.GATE for _, h, _ in plain)
