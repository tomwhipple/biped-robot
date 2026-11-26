"""track_host --id-wiggle (#259, #137): a small known multisine on one joint channel's goals, in the double supports
only, for identifying the body path's loop gain while walking (experiments/policy/x15-margin/wiggle_gain.py). Off (the
default) the frames are untouched; on, only the channel's joints move, only inside the windows (each one a double
support, opened at the step end frame, closed post_s before the lift gate frame), zero-mean, within the amplitude,
ramped in and out; the live run uploads exactly those frames and logs what reconstructs them."""
import json
import math
import os
import sys
import types

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "experiments", "roll-balance"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")

H = pytest.importorskip("track_host")

KW = dict(T_step=4.0, n_steps=6, direction="fwd", lift_gate=True, swing_tilt=3.0)


def _plan(**extra):
    kw = dict(KW, cmd=H.preset_cmd("fwd", 40.0), **extra)
    return H.plan(**kw)


def _ticks(hx):
    return [int(hx[6 + 3 * j: 9 + 3 * j], 16) for j in range(12)]


@pytest.fixture(scope="module")
def plans():
    spec = H.wiggle_spec(1.0, "roll", seed=1)
    return _plan(), _plan(id_wiggle=spec), spec


def test_off_leaves_the_plan_untouched(plans):
    p0, _, _ = plans
    p_none = _plan(id_wiggle=None)
    assert p_none["adds"] == p0["adds"] and p_none["frames"] == p0["frames"]
    assert "wiggle" not in p0 and "id_wiggle" not in p0["args"]


def test_on_moves_only_the_channel_inside_double_support_windows(plans):
    p0, p1, spec = plans
    sw = p1["sway"]
    wl = p1["wiggle"]
    win = wl["windows_ms"]
    assert len(win) == len(sw.phases) and wl["excited_s"] > 8.0
    ss = [(1e3 * (p["t0"] - sw.t_first), 1e3 * (p["t1"] - sw.t_first)) for p in sw.phases]
    gates = [tm for tm, hx, k in p1["frames"] if k == "kf" and int(hx[5], 16) & H.G.GATE]
    for k, (a, b) in enumerate(win):
        # inside the double support before lift-off k, after the previous touchdown, closed post_s before the gate
        assert a >= (ss[k - 1][1] if k else 0.0) and b <= gates[k] - 1e3 * spec["post_s"] + 1e-6
        assert not any(a < lo < b or a < td < b for lo, td in ss)
    jidx = {H.G.LEG.index(n) for n in spec["joints"]}
    moved = 0
    assert len(p0["frames"]) == len(p1["frames"])
    for (t0, h0, k0), (t1, h1, k1) in zip(p0["frames"], p1["frames"]):
        assert (t0, k0, h0[:6]) == (t1, k1, h1[:6])          # time, phase and flags as before
        if h0 == h1:
            continue
        moved += 1
        assert k0 == "kf" and any(a < t0 < b for a, b in win), t0    # never a rest frame, never outside a window
        q0, q1 = _ticks(h0), _ticks(h1)
        assert all(q0[j] == q1[j] for j in range(12) if j not in jidx)
        d = {q1[j] - q0[j] for j in jidx}
        assert len({abs(x) for x in d}) == 1                 # the same size on every joint of the channel
    assert moved == len(wl["frames"]) > 0


def test_zero_mean_capped_and_ramped(plans):
    _, p1, spec = plans
    wl = p1["wiggle"]
    cap = math.floor(spec["amp_deg"] * H.TICKS_PER_DEG)
    kf = [tm for tm, _, k in p1["frames"] if k == "kf"]
    off = {tm: 0 for tm in kf}
    off.update({tm: n for tm, n in wl["frames"]})
    t = np.array(kf, float)
    v = np.array([off[tm] for tm in kf], float)
    assert np.max(np.abs(v)) <= cap
    tt, play = H.catmull_play(t, v)                          # what the firmware plays at its 20 ms tick
    assert np.max(np.abs(play)) <= cap + 1e-9               # never past the amplitude, between frames either
    outside = np.ones(len(t), bool)
    for a, b in wl["windows_ms"]:
        inside = (t > a) & (t < b)
        outside &= ~inside
        assert int(v[inside].sum()) == 0                     # zero-mean: the window's ticks sum to 0 exactly
        assert np.max(np.abs(v[inside])) >= cap - 2          # and it uses the amplitude
        near = (np.abs(tt - a) <= 1e-6) | (np.abs(tt - b) <= 1e-6)
        assert np.all(play[near] == 0.0)                     # 0 at both ends of the window
    assert np.all(v[outside] == 0)                           # nothing outside the windows
    # continuous: per 20 ms tick the played offset moves at most what a full-amplitude 2.9 Hz sine would (no step)
    step = np.max(np.abs(np.diff(play)))
    assert step <= 2 * math.pi * max(spec["freqs_hz"]) * cap * 0.02 * 1.5
    # the ramps: within ramp_s of either end of a window the offset is under the envelope's share of the amplitude
    for a, b in wl["windows_ms"]:
        r = 1e3 * spec["ramp_s"]
        m = (tt > a) & (tt < a + 0.3 * r) | (tt < b) & (tt > b - 0.3 * r)
        assert np.max(np.abs(play[m]), initial=0.0) <= 0.5 * cap


def test_the_log_reconstructs_the_offsets(plans):
    p0, p1, spec = plans
    wl = p1["wiggle"]
    assert wl["seed"] == 1 and len(wl["phases_rad"]) == len(wl["freqs_hz"]) and wl["channel"] == "roll"
    by_t = dict((tm, n) for tm, n in wl["frames"])
    for (t0, h0, _), (_, h1, _) in zip(p0["frames"], p1["frames"]):
        q0, q1 = _ticks(h0), _ticks(h1)
        for name, sg in spec["joints"].items():
            j = H.G.LEG.index(name)
            assert q1[j] - q0[j] == sg * H.G.CAL[name][1] * by_t.get(t0, 0)
    # and from the spec alone (seed, lines, windows) the same ticks come out
    kf = [tm for tm, _, k in p1["frames"] if k == "kf"]
    n, _ = H.wiggle_ticks(kf, [tuple(w) for w in wl["windows_ms"]], H.wiggle_spec(1.0, "roll", seed=1))
    assert [[int(tm), int(x)] for tm, x in zip(kf, n) if x] == wl["frames"]
    json.dumps(wl)                                           # the summary row can carry it


def test_other_channels_and_seeds():
    s2 = H.wiggle_spec(0.5, "yaw", seed=7)
    p = _plan(id_wiggle=s2)
    assert p["wiggle"]["joints"] == {"L_hip_yaw": 1, "R_hip_yaw": 1}
    assert max(abs(n) for _, n in p["wiggle"]["frames"]) <= math.floor(0.5 * H.TICKS_PER_DEG)
    assert H.wiggle_spec(1.0, "roll", 1)["phases_rad"] != H.wiggle_spec(1.0, "roll", 2)["phases_rad"]


def test_refuses_a_large_amplitude_and_bad_input(capsys):
    for bad in ("1.5", "1.01", "0", "-0.5", "x", "1:shake"):
        with pytest.raises(ValueError):
            H.parse_wiggle(bad)
    with pytest.raises(ValueError):
        H.wiggle_spec(1.2)
    assert H.parse_wiggle("1") == (1.0, "roll") and H.parse_wiggle("0.5:pitch") == (0.5, "pitch")
    assert H.main(["--T", "4", "--dir", "fwd", "--steps", "4", "--id-wiggle", "1.2"]) == 2
    assert "REFUSED" in capsys.readouterr().out
    assert H.main(["--cmd", "stdin", "--id-wiggle", "1"]) == 2
    assert H.main(["--T", "4", "--model", "sway", "--id-wiggle", "1"]) == 2


def test_dry_run_prints_it_and_off_prints_nothing(capsys):
    assert H.main(["--T", "4", "--dir", "march", "--steps", "4"]) == 0
    assert "id wiggle" not in capsys.readouterr().out
    assert H.main(["--T", "4", "--dir", "march", "--steps", "4", "--id-wiggle", "1:roll"]) == 0
    out = capsys.readouterr().out
    assert "id wiggle: 1 deg on roll" in out and "PASS" in out


class _Robot:
    sw, sent = None, []

    def __init__(self, host=None, log=None, **_):
        self.cal = {n: (i + 1, H.G.CAL[n][0], H.G.CAL[n][1]) for i, n in enumerate(H.G.LEG)}
        self.head = 0
        self.c = types.SimpleNamespace(idle=lambda *_a, **_k: None)

    def cmd(self, line, idle=8.0):
        _Robot.sent.append(line)
        if line == "imu":                                    # the heading drifts 3 deg a read: the hold re-plans
            self.yaw = getattr(self, "yaw", 0.0) + math.radians(3.0)
            return f"up 0 0 1\r\nquat {math.cos(self.yaw / 2):.6f} 0.0 0.0 {math.sin(self.yaw / 2):.6f}\r\n"
        if line.startswith("track add"):
            return "track +4"
        if line == "track":
            self.head += 400
            n = sum(1 for p in _Robot.sw.phases if 1e3 * (p["t1"] - _Robot.sw.t_first) <= self.head)
            return f"track {'done' if n >= len(_Robot.sw.phases) else 'play'}: play t {self.head} ms\r\n" \
                   f"{n} single supports reported\r\n"
        if line == "track steps":
            n = sum(1 for p in _Robot.sw.phases if 1e3 * (p["t1"] - _Robot.sw.t_first) <= self.head)
            return "\r\n".join(f"TS {i + 1} {p['stance']} in 2.0 out 1.0 e +0.0 T +3.0 hike +3.0 ok"
                               for i, p in enumerate(_Robot.sw.phases[:n]))
        return "ok"

    def fallen(self):
        return False

    def close(self):
        pass


def test_live_uploads_the_wiggled_frames_and_logs_the_excitation(tmp_path, monkeypatch):
    fake = types.ModuleType("robot_io")
    fake.Robot = _Robot
    monkeypatch.setitem(sys.modules, "robot_io", fake)
    real_plan = H.plan

    last = {}

    def spy(*a, **k):
        p = real_plan(*a, **k)
        _Robot.sw = p["sway"]
        last["call"] = (a, dict(k))
        return p
    monkeypatch.setattr(H, "plan", spy)
    _Robot.sent = []
    log = tmp_path / "run.jsonl"
    rc = H.main(["--T", "4", "--dir", "fwd", "--steps", "6", "--stride-mm", "40", "--swing-tilt", "3", "--live", "--ahead", "1", "--poll", "0",
                 "--log", str(log), "--heading-hold", "on", "--id-wiggle", "1.0", "--id-seed", "3"])
    assert rc == 0
    rows = [json.loads(x) for x in open(log)]
    summ = rows[-1]
    assert summ["summary"] and summ["id_wiggle"]["seed"] == 3 and rows[0]["id_wiggle"] == summ["id_wiggle"]
    wl = summ["id_wiggle"]
    # the frames that went out, on the track's timeline (rest frames are not on it)
    t, sent = 0, {}
    for line in _Robot.sent:
        if line.startswith("track add"):
            for f in line.split()[2:]:
                t += int(f[:3], 16)
                if not int(f[5], 16) & H.G.REST:
                    sent[t] = f
    # the heading hold re-planned the steps not uploaded yet (their hip yaws) as the walk went; the last plan's args
    # reproduce every frame sent, and without the excitation they differ by exactly the logged offsets
    pa, kw = last["call"]
    assert len({round(x, 3) for x in kw["yaw_trim"]}) > 1         # the hold did re-plan
    wig = {tm: hx for tm, hx, k in real_plan(*pa, **kw)["frames"] if k == "kf"}
    plain = {tm: hx for tm, hx, k in real_plan(*pa, **{k: v for k, v in kw.items() if k != "id_wiggle"})["frames"]
             if k == "kf"}
    assert sent and all(sent[tm] == wig[tm] for tm in sent)
    by_t = dict((tm, n) for tm, n in wl["frames"])
    assert by_t and set(by_t) <= set(sent)
    cal = kw["cal"]
    for tm in sent:
        q0, q1 = _ticks(plain[tm]), _ticks(sent[tm])
        for name, sg in wl["joints"].items():
            j = H.G.LEG.index(name)
            assert q1[j] - q0[j] == sg * cal[name][1] * by_t.get(tm, 0)
