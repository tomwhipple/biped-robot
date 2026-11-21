"""tools/pad_latency.py's two measurements on scripted SDL events (no pad):
the held-still verdict and the switch-off-to-disconnect time."""
import os
import sys
import time

import pytest

pytest.importorskip("pygame")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import pad_latency  # noqa: E402


class Scripted(pad_latency.Recorder):
    """Inputs every `period` s from instance 3; removed at `remove_after` s."""

    def __init__(self, path, period=None, remove_after=None):
        super().__init__(path)
        self.period, self.remove_after = period, remove_after
        self.t0 = time.monotonic()
        self.next = self.t0

    def poll(self):
        t = time.monotonic()
        out = []
        if self.remove_after is not None and t - self.t0 >= self.remove_after:
            self.row(t, "ctl_removed", 3)
            self.remove_after = None
            return [(t, "ctl_removed", 3)]
        if self.period is not None and t >= self.next:
            self.next = t + self.period
            self.row(t, "joy_axis", 3, "axis=6 value=0.01")
            out.append((t, "joy_axis", 3))
        return out


def test_a_streaming_pad_and_a_silent_one(tmp_path):
    r = Scripted(str(tmp_path / "a.csv"), period=0.01)
    out = pad_latency.still(r, 3, 0.4)
    assert out["streams"] and out["rate"] > 50 and out["max_gap_ms"] < 75
    r.close()
    r = Scripted(str(tmp_path / "b.csv"))
    out = pad_latency.still(r, 3, 0.3)
    assert not out["streams"] and out["events"] == 0
    r.close()
    lines = open(tmp_path / "a.csv").read().splitlines()
    assert lines[0].startswith("t_mono_s,t_utc,phase,event") and len(lines) > 20


def test_switch_off_times_the_disconnect(tmp_path):
    r = Scripted(str(tmp_path / "c.csv"), period=0.01, remove_after=0.3)
    r.t0 = time.monotonic()
    out = pad_latency.switch_off(r, 3, 2.0)
    r.close()
    assert out["removed"] is not None
    assert 0.0 <= out["after_last_input_ms"] < 50.0
