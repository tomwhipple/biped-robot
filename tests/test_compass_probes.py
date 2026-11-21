"""grade.py's compass probes (#263; experiments/policy/shaper-train): opt-in (the default probe
set is unchanged), eight headings straight and turning plus two heading changes mid-walk, and
compass_metrics grades a synthetic torso path in the body frame: a walk that holds its heading and
speed while turning holds; a wrong direction, a slow walk, a missing turn or a fall does not."""
import math
import os
import sys

import numpy as np
import pytest

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
for p in ("experiments/policy/shaper-train", "experiments/policy/stand-still", "experiments/policy/rca",
          "link", "sim"):
    sys.path.insert(0, os.path.join(ROOT, p))
os.environ.setdefault("JAX_PLATFORMS", "cpu")

import grade as G  # noqa: E402

DT = G.DT


def test_opt_in_and_the_set():
    assert not any(k.startswith("compass") for k in G.PROBES)        # default set untouched
    assert len(G.COMPASS_PROBES) == 8 * 2 + 2
    for k, d in G.COMPASS.items():
        c = G.COMPASS_PROBES[f"compass {k}"]["cmd_schedule"][1][1]
        assert math.hypot(c[0], c[1]) == pytest.approx(G.COMPASS_V)
        assert math.degrees(math.atan2(c[1], c[0])) == pytest.approx(d)
        assert c[2] == 0.0
        assert G.COMPASS_PROBES[f"compass {k} turn"]["cmd_schedule"][1][1][2] == G.COMPASS_W
    ch = G.COMPASS_PROBES["compass F>BR"]["cmd_schedule"]
    assert [ts for ts, _ in ch] == [0.0, 2.0, 5.0, 8.0] and ch[-1][1] == G.STAND_CMD


def _path(sched, run_s, scale=1.0, rot_deg=0.0, wz_scale=1.0, yaw0=0.3):
    """The torso's world path under an ideal follower of `sched` (body-frame velocity, scaled and
    rotated off the command), the tick convention of grade's episode loop, plus a gait sway."""
    n = int(round(run_s / DT))
    X, Y, YW = [], [], []
    x = y = 0.0
    yaw = yaw0
    times = [ts for ts, _ in sched]
    r = math.radians(rot_deg)
    for i in range(n):
        k = max(j for j, ts in enumerate(times) if int(round(ts / DT)) - 1 <= i)
        c = sched[k][1]
        yaw += wz_scale * c[2] * DT
        vbx = scale * (math.cos(r) * c[0] - math.sin(r) * c[1])
        vby = scale * (math.sin(r) * c[0] + math.cos(r) * c[1])
        x += (math.cos(yaw) * vbx - math.sin(yaw) * vby) * DT
        y += (math.sin(yaw) * vbx + math.cos(yaw) * vby) * DT
        sway = 0.01 * math.sin(2 * math.pi * 1.5 * i * DT)
        X.append(x - math.sin(yaw) * sway)
        Y.append(y + math.cos(yaw) * sway)
        YW.append(math.atan2(math.sin(yaw), math.cos(yaw)))
    return X, Y, YW


@pytest.mark.parametrize("name", ["compass FL turn", "compass BR", "compass B turn", "compass F>BR",
                                  "compass R"])
def test_an_ideal_follower_holds(name):
    p = G.COMPASS_PROBES[name]
    m = G.compass_metrics(*_path(p["cmd_schedule"], p["run_s"]), p["cmd_schedule"], False)
    assert m["compass_hold"], m
    assert m["compass_dir_err"] < 3.0 and abs(m["compass_speed_ratio"] - 1.0) < 0.1
    assert len(m["compass_segs"]) == (2 if ">" in name else 1)


def test_what_does_not_hold():
    p = G.COMPASS_PROBES["compass FL turn"]
    s, r = p["cmd_schedule"], p["run_s"]
    assert not G.compass_metrics(*_path(s, r, rot_deg=50.0), s, False)["compass_hold"]    # wrong way
    assert not G.compass_metrics(*_path(s, r, scale=0.3), s, False)["compass_hold"]      # too slow
    assert not G.compass_metrics(*_path(s, r, wz_scale=0.0), s, False)["compass_hold"]   # no turn
    assert not G.compass_metrics(*_path(s, r), s, True)["compass_hold"]                  # fell
    m = G.compass_metrics(*_path(s, r, rot_deg=50.0), s, False)
    assert m["compass_dir_err"] == pytest.approx(50.0, abs=3.0)
    # a heading change that the walker ignores (keeps walking forward) fails the second segment
    q = G.COMPASS_PROBES["compass F>L"]["cmd_schedule"]
    frozen = [q[0], q[1], (5.0, q[1][1]), q[3]]
    m = G.compass_metrics(*_path(frozen, 12.0), q, False)
    assert [sg["hold"] for sg in m["compass_segs"]] == [True, False] and not m["compass_hold"]


def test_compass_table():
    p = G.COMPASS_PROBES["compass L"]
    m = G.compass_metrics(*_path(p["cmd_schedule"], p["run_s"]), p["cmd_schedule"], False)
    eps = [dict(m, variant="v", probe="compass L", fell=False)]
    md = G.compass_table(eps, ["v"])
    assert "| v | 1/1" in md and "1/1 |" in md


def test_compass_settings_are_parsed_only_when_used(monkeypatch):
    """A malformed COMPASS_* does not break grade.py while the compass probes are off: the
    settings are parsed on first use, not at import."""
    import importlib
    monkeypatch.setenv("COMPASS_V", "not-a-number")
    try:
        g = importlib.reload(G)
        assert g.PROBES and not any(k.startswith("compass") for k in g.PROBES)
        with pytest.raises(ValueError):
            g.compass_cfg()
    finally:
        monkeypatch.delenv("COMPASS_V")
        G._COMPASS_CFG = None
        G._COMPASS_PROBES = None
    assert G.COMPASS_V == 0.15 and len(G.COMPASS_PROBES) == 8 * 2 + 2
