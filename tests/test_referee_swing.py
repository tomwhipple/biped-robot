"""The referee's swing tracker (eval_precision.SwingTracker): swings are the
sole's time out of contact, each scored by its peak height; chatter shorter
than SWING_MIN_TICKS is not a step."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim", "mjx"))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))


def test_swing_tracker_counts_real_swings_and_their_peaks():
    import eval_precision as E
    t = E.SwingTracker()
    stand = ((True, True), (0.0, 0.0))
    for _ in range(5):
        t.update(*stand)
    # left foot: a 10-tick swing peaking at 4 cm (a real lift)
    for k in range(10):
        t.update((False, True), (0.004 * (k + 1) if k < 10 else 0.0, 0.0))
    t.update(*stand)
    # right foot: a 6-tick shuffle peaking at 1 cm
    for k in range(6):
        t.update((True, False), (0.0, 0.01 if k == 3 else 0.005))
    t.update(*stand)
    # left foot: 2 ticks of contact chatter, not a step
    t.update((False, True), (0.002, 0.0)); t.update((False, True), (0.002, 0.0))
    t.update(*stand)
    s = t.summary()
    assert s["swing_n"] == 2
    assert s["swing_clear_frac"] == 0.5
    assert abs(s["swing_peak_med"] - 0.025) < 1e-9
    # ticks with a foot >= 3 cm up: the left swing's ticks 8, 9, 10 (3.2, 3.6, 4.0 cm)
    assert s["lift_frac"] == 3 / t.ticks


def test_swing_tracker_without_swings():
    import eval_precision as E
    t = E.SwingTracker()
    for _ in range(20):
        t.update((True, True), (0.0, 0.0))
    s = t.summary()
    assert s["swing_n"] == 0 and s["swing_clear_frac"] is None and s["lift_frac"] == 0.0
