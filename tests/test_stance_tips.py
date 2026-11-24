"""experiments/roll-balance/stance_tips.py: the pure parts (TS parsing, single supports from the trim state, the
per-step rebound numbers, the summary). The flight-record side runs on real data only."""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "experiments", "roll-balance"))
import stance_tips as ST  # noqa: E402


def test_parse_ts_first_sighting_and_period():
    rows = [dict(t=10.0, text="TS 1 L in 3.4 out 4.2 e -1.9 T +6.0 hike +5.5 ok G 0\r\n"),
            dict(t=14.0, text="TS 1 L in 3.4 out 4.2 e -1.9 T +6.0 hike +5.5 ok G 0\r\n"
                              "TS 2 R in 1.6 out 5.3 e +4.4 T +3.2 hike +2.2 ok G 0"),
            dict(t=18.1, text="TS 3 L in 11.5 out 0.0 e +3.9 T +6.9 hike +7.9 ok Tback G 0"),
            dict(t=22.0, text="TS 4 R in 2.2 out 3.8 e +2.0 T +3.1 hike +2.7 ok")]
    ts, period = ST.parse_ts(rows)
    assert sorted(ts) == [1, 2, 3, 4]
    assert ts[1]["t"] == 10.0 and ts[1]["side"] == "L" and ts[1]["T"] == 6.0
    assert ts[3]["tback"] and ts[3]["lean_in"] == 11.5 and ts[3]["gate"] == 0
    assert ts[4]["gate"] is None and not ts[4]["tback"]
    assert abs(period - 4.0) < 1e-9


def test_chains_from_trim_state():
    s = [0, 0, 5, 5, 5, 6, 6, 7, 7, 0, 0, 5, 5, 6, 7, 5, 5, 6, 7, 7, 0]
    #          ^2        ^5    ^7          ^11  ^13 ^14 ^15 ^17^18
    assert ST.chains(np.array(s)) == [(2, 5, 7), (11, 13, 14), (15, 17, 18)]
    assert ST.chains(np.array([0, 0, 0])) == []


def test_step_metrics_signs_and_beyond_encoders():
    t = np.arange(0.0, 3.0, 0.02)
    # left stance (sign +1): the body rolls toward the swing foot by 3 deg at 0.6 s and back to -2 at 0.9 s; the
    # encoders (hip + ankle roll, joint frame) move +1 deg by 0.6 s, the other way (the trim hiking), so the roll
    # beyond them is 3 + 1 = 4 deg there (footoff_sag.step_traces: delta = imu - kin, kin = -s (enc - enc0))
    e = np.interp(t, [0, 0.6, 0.9, 3], [0, 3, -2, -2])
    enc = np.interp(t, [0, 0.6, 3], [10, 11, 11])
    g = np.gradient(e, t)
    m = ST.step_metrics(t, e, g, enc, 0.0, +1.0)
    assert abs(m["e_09"] + 2.0) < 1e-6 and abs(m["beyond_06"] - 4.0) < 1e-6
    assert m["gyro_06"] < 0                      # swinging back at 0.6 s
    # the same motion on a right stance is recorded with the opposite sign: the metrics come out the same
    mr = ST.step_metrics(t, -e, -g, -enc, 0.0, -1.0)
    for k in m:
        assert abs(m[k] - mr[k]) < 1e-6, k


def test_summarize_buckets():
    steps = [dict(period=4.0, side="L", k=1, tip=False, e_09=-2.0), dict(period=4.0, side="L", k=5, tip=True, e_09=4.0),
             dict(period=4.0, side="L", k=6, tip=False, e_09=-1.0), dict(period=3.5, side="L", k=6, tip=False, e_09=-3.0),
             dict(period=None, side="R", k=1, tip=False)]
    s = ST.summarize(steps)
    late = s[("normal", "L", "late")]
    assert late["n"] == 2 and late["tips"] == 1 and late["e_09"] == -1.0 and np.isnan(late["ld_st_ank"])
    assert s[("normal", "L", "early")]["n"] == 1 and s[("fast", "L", "late")]["tips"] == 0
    assert len(s) == 3


def test_window_mean():
    t = np.arange(0.0, 2.0, 0.02)
    y = np.where(t < 1.0, 10.0, 30.0)
    y[60] = np.nan
    assert ST.window_mean(t, y, 1.0, 0.3, 0.5) == 30.0
    assert ST.window_mean(t, y, 1.0, -0.25, -0.05) == 10.0
    assert np.isnan(ST.window_mean(t, y, 5.0, 0.3, 0.5))
