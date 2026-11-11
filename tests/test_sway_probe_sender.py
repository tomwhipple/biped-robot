"""experiments/sway-probe/sway_probe.py, the robot-side sender: dry run by default, one rung per `--go`,
frames the firmware's `pose` parser takes, every ending a `home`. A fake robot stands in for the Wi-Fi CLI: nothing
here opens a socket or touches hardware (the tests make a socket a failure)."""
import json
import math
import os
import re
import socket
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SWAY = os.path.join(HERE, "..", "experiments", "sway-probe")
sys.path.insert(0, SWAY)
sys.path.insert(0, os.path.join(HERE, "..", "tools"))
sys.path.insert(0, os.path.join(HERE, "..", "link"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import bus_cal  # noqa: E402
import sway_probe as SP  # noqa: E402
import sway_ref as SR  # noqa: E402
import wcliproto  # noqa: E402


@pytest.fixture(scope="module")
def ladder():
    ref = SR.SwayReference()
    return ref, ref.ladder()


@pytest.fixture(autouse=True)
def no_network(monkeypatch):
    def boom(*a, **k):
        raise AssertionError("the sender opened a socket")
    monkeypatch.setattr(socket, "socket", boom)


def fake_predictions(path, rungs, seg_ms=200, balance="off", up=24):
    band = lambda v: dict(min=v, p10=v, med=v, p90=v, max=v)
    out = dict(meta=dict(seg_ms=float(seg_ms), lead=rungs[0].lead, hz=rungs[0].hz, cycles=rungs[0].cycles, ramp_s=rungs[0].ramp_s,
                         balance=balance, chain="pose"), rungs=[])
    for r in rungs:
        side = dict(F_min_frac=band(0.0), F_min_N=band(0.0), clear_mm=band(6.0), unloaded=up, clears=up, n=up,
                    stance_F_max_frac=band(1.0), com_margin_mm=band(8.0))
        out["rungs"].append(dict(rung=r.meta(), nominal={}, band=dict(
            n=up, up=up, tilt_peak_deg=band(4.0), roll_peak_deg=band(4.0), roll_rate_peak_dps=band(50.0), gyro_ema_peak=band(0.5),
            flail_tilt_trips=0, flail_rate_trips=0, sides=dict(L=side, R=side))))
    path.write_text(json.dumps(out))
    return str(path)


# ---------------------------------------------------------------------------------------- frames
def test_frames_are_firmware_pose_lines_in_the_calibrated_ticks(ladder):
    ref, rungs = ladder
    cal = SP.illustrative_cal()
    segs = SP.plan(rungs[8], cal, 200)
    pat = re.compile(r"^pose( \d{1,4}){17} s200$")
    for s in segs:
        line = s.line()
        assert pat.match(line), line
        t = [int(x) for x in line.split()[1:18]]
        assert all(0 <= x <= 4095 for x in t)
        assert len(line) <= wcliproto.MAX_LINE
        assert wcliproto.secret_line(line) is None
        assert t == cal.pose_ticks(s.deg)                      # the bus_cal encoder, the same one the bench tools use
    # a keyframe round-trips through the calibration to within half a tick
    for nm in ref.names:
        j = cal.by_name(nm)
        k = segs[10]
        t = k.ticks[j.bus_index]
        assert abs(j.ticks_to_deg(t) - k.deg[nm]) <= 0.5 / bus_cal.TICKS_PER_DEG + 1e-9


def test_segments_are_the_keyframes_a_segment_ahead_each_at_least_200_ms(ladder):
    ref, rungs = ladder
    r = rungs[3]
    segs = SP.plan(r, SP.illustrative_cal(), 200)
    assert all(s.ms == 200 for s in segs)                      # the firmware refuses s<200
    assert [round(s.t_s, 2) for s in segs[:3]] == [0.0, 0.2, 0.4]
    k = 10                                                     # 200 ms = 10 keyframes
    assert np.allclose([segs[0].deg[n] for n in ref.names], np.degrees(r.q[k]))
    assert np.allclose([segs[-1].deg[n] for n in ref.names], np.degrees(r.q[-1]))      # ends on the standing pose
    assert len(segs) == math.ceil((len(r.t) - 1) / 10)
    with pytest.raises(ValueError):
        SP.plan(r, SP.illustrative_cal(), 100)
    with pytest.raises(ValueError):
        SP.plan(r, SP.illustrative_cal(), 250)


def test_balance_option_names_the_foot_that_is_not_swinging(ladder):
    ref, rungs = ladder
    segs = SP.plan(rungs[8], SP.illustrative_cal(), 200, balance="stance")
    r = rungs[8]
    for s in segs[8:-8]:
        k = int(round(s.t_s * 50)) + 5
        assert s.stance == ("R" if math.sin(r.phase[k]) > 0 else "L")
        assert s.line().endswith(f" s200 B{s.stance}")
    assert all(not s.stance for s in SP.plan(r, SP.illustrative_cal(), 200))


# ---------------------------------------------------------------------------------------- dry run
def test_dry_run_prints_the_plan_and_the_prediction_and_sends_nothing(ladder, tmp_path, capsys, monkeypatch):
    ref, rungs = ladder
    pred = fake_predictions(tmp_path / "p.json", rungs)
    monkeypatch.setattr(SP, "Robot", lambda *a, **k: (_ for _ in ()).throw(AssertionError("dry run built a robot")))
    assert SP.main(["--rung", "4", "--predictions", pred]) == 0
    out = capsys.readouterr().out
    assert "DRY RUN" in out and "rung 4" in out and "pose " in out and " s200" in out
    assert "sim prediction for this rung" in out and "stays up 24/24" in out and "clearance 6.0 / 6.0 / 6.0 mm" in out
    assert "stop criteria: tilt > 6.0 deg" in out            # 1.5 x 4.0 deg band, at the floor
    assert "--go" in out


def test_no_rung_lists_the_ladder_and_runs_nothing(ladder, tmp_path, capsys):
    pred = fake_predictions(tmp_path / "p.json", ladder[1])
    assert SP.main(["--predictions", pred]) == 0
    out = capsys.readouterr().out
    assert out.count("rung ") >= 12 and "Dry run: nothing was sent" in out


def test_a_plan_the_sim_did_not_predict_says_so(ladder, tmp_path, capsys):
    pred = fake_predictions(tmp_path / "p.json", ladder[1], seg_ms=300)
    assert SP.main(["--rung", "2", "--predictions", pred]) == 0
    assert "NO SIM PREDICTION" in capsys.readouterr().out


def test_go_needs_an_explicit_rung(tmp_path):
    with pytest.raises(SystemExit):
        SP.main(["--go"])
    with pytest.raises(SystemExit):
        SP.main(["--rung", "13", "--go"])


def test_go_refuses_without_a_prediction_for_the_plan(ladder, tmp_path, capsys):
    pred = fake_predictions(tmp_path / "p.json", ladder[1], seg_ms=300)
    called = []
    rc = SP.main(["--rung", "2", "--go", "--predictions", pred], robot_factory=lambda: called.append(1), input_fn=lambda s: "GO 2")
    assert rc == 2 and not called


def test_go_needs_the_typed_confirmation_for_that_rung(ladder, tmp_path):
    pred = fake_predictions(tmp_path / "p.json", ladder[1])
    called = []
    for typed in ("", "go 2", "GO 3", "yes"):
        rc = SP.main(["--rung", "2", "--go", "--predictions", pred, "--out", str(tmp_path)],
                     robot_factory=lambda: called.append(1), input_fn=lambda s, t=typed: t)
        assert rc == 2
    assert not called


# ---------------------------------------------------------------------------------------- the run (fake robot)
CAL_TEXT = "\n".join(f"  {j.name:12s} id {j.servo_id:2d}  zero {j.zero_steps}  dir {'+' if j.direction > 0 else '-'}{abs(j.direction)}  "
                     f"{'fitted' if j.fitted else 'NOT FITTED'}" for j in SP.illustrative_cal().joints) + "\nloaded from NVS at boot\n"


class FakeRobot:
    def __init__(self, tilts=None, rec="rec ON -> 10.0.0.5:4220  wifi CONNECTED", cal=CAL_TEXT, pose_reply="pose: streaming",
                 home_ok=True, raise_at=None):
        self.cmds = []
        self.tilts = list(tilts or [])
        self.rec, self.cal, self.pose_reply, self.home_ok, self.raise_at = rec, cal, pose_reply, home_ok, raise_at

    def run(self, line, timeout=0):
        self.cmds.append(line)
        w = line.split()[0]
        if self.raise_at is not None and w == "pose" and sum(c.startswith("pose") for c in self.cmds) > self.raise_at:
            raise KeyboardInterrupt
        if w == "rec":
            return self.rec
        if w == "cal":
            return self.cal
        if w in ("bench", "torque"):
            return f"{w}: ok"
        if w == "pose":
            return self.pose_reply
        if w == "imu":
            tilt = self.tilts.pop(0) if self.tilts else 1.0
            up = (0.0, -math.sin(math.radians(tilt)), math.cos(math.radians(tilt)))
            return f"driver x\nup       {up[0]: .4f} {up[1]: .4f} {up[2]: .4f}   (body z)\ngyro     0.0100 0.0200 0.0300 rad/s\n"
        if w == "home":
            if not self.home_ok:
                raise RuntimeError("home: timeout")
            return "home readback: all 12 joints at their zeros -- torque released"
        if w == "release":
            return "release all: ok"
        raise AssertionError(line)


def run(robot, ladder, rung_i=2, collector=lambda run, ip: (True, "up"), stops=None):
    ref, rungs = ladder
    cal = SP.illustrative_cal()
    rung = rungs[rung_i]
    segs = SP.plan(rung, cal, 200)
    log = []
    res = SP.run_rung(robot, rung, cal, segs, stops or SP.stops_for(None), log.append, "10.0.0.5", 200,
                      collector_check=collector, clock=iter(np.arange(0, 1e6, 1e-3)).__next__, sleep=lambda s: None)
    return res, segs, log


def test_a_clean_run_ends_with_home_and_nothing_after_it(ladder):
    r = FakeRobot()
    (status, why), segs, log = run(r, ladder)
    assert status == "done", why
    assert r.cmds[-1] == "home" and r.cmds.count("home") == 1 and "release" not in r.cmds
    assert sum(c.startswith("pose") for c in r.cmds) == len(segs) + 1          # the zero stand + the segments
    # order: bench, torque, cal show, zero stand, upright check, then pose/imu pairs
    assert r.cmds[:4] == ["bench", "torque", "cal show", SP.zero_line(SP.illustrative_cal())]
    assert r.cmds[4] == "imu" and r.cmds[5].startswith("pose") and r.cmds[6] == "imu"


def test_a_tilt_past_the_stop_ends_the_rung_with_home(ladder):
    r = FakeRobot(tilts=[0.5] + [1.0] * 5 + [9.0])           # the upright check, five good segments, then 9 deg
    (status, why), segs, log = run(r, ladder)
    assert status == "stopped" and "tilt 9.0" in why
    assert r.cmds[-1] == "home" and r.cmds.count("home") == 1
    assert sum(c.startswith("pose") for c in r.cmds) == 1 + 6              # no pose after the stop


def test_a_refused_pose_ends_with_home(ladder):
    r = FakeRobot(pose_reply="pose: usage")
    (status, why), segs, log = run(r, ladder)
    assert status == "refused" and r.cmds[-1] == "home"


def test_not_upright_at_the_start_never_plays_the_rung(ladder):
    r = FakeRobot(tilts=[8.0])
    (status, why), segs, log = run(r, ladder)
    assert status == "refused" and "tilt 8.0" in why
    assert sum(c.startswith("pose") for c in r.cmds) == 1 and r.cmds[-1] == "home"


def test_ctrl_c_ends_with_home(ladder):
    r = FakeRobot(raise_at=4)
    (status, why), segs, log = run(r, ladder)
    assert status == "stopped" and why == "Ctrl-C" and r.cmds[-1] == "home"


def test_home_failing_falls_back_to_release(ladder):
    r = FakeRobot(home_ok=False, tilts=[0.5, 20.0])
    (status, why), segs, log = run(r, ladder)
    assert status == "stopped" and r.cmds[-2:] == ["home", "release"]


def test_an_unfitted_leg_joint_refuses_before_any_pose(ladder):
    bad = CAL_TEXT.replace("L_ankle_roll id 12  zero 2048  dir +1  fitted", "L_ankle_roll id 12  zero 2048  dir +1  NOT FITTED")
    r = FakeRobot(cal=bad)
    with pytest.raises(SystemExit):
        run(r, ladder)
    assert not any(c.startswith("pose") for c in r.cmds)


def test_no_collector_means_no_motion_command_at_all(ladder):
    r = FakeRobot()
    (status, why), segs, log = run(r, ladder, collector=lambda run, ip: (False, "flight collector is not running here"))
    assert status == "refused" and r.cmds == []


# ---------------------------------------------------------------------------------------- the collector check
def test_collector_check_needs_local_collector_and_robot_recording_to_this_host():
    up = lambda: (True, "running: pid 1\nfoo.frec: 5 pkts")
    ok, why = SP.check_collector(lambda l: "rec ON -> 10.0.0.5:4220  wifi CONNECTED", "10.0.0.5", up)
    assert ok
    assert not SP.check_collector(lambda l: "rec ON -> 10.0.0.5:4220", "10.0.0.5", lambda: (False, "not running"))[0]
    assert not SP.check_collector(lambda l: "rec off -> 10.0.0.5:4220", "10.0.0.5", up)[0]
    assert "not to this host" in SP.check_collector(lambda l: "rec ON -> 10.0.0.9:4220", "10.0.0.5", up)[1]
    assert not SP.check_collector(lambda l: "rec ON -> 10.0.0.5:4999", "10.0.0.5", up)[0]
    assert not SP.check_collector(lambda l: "garbage", "10.0.0.5", up)[0]


def test_imu_status_parse():
    t = "up       0.0000 -0.0872  0.9962   (body z)\ngyro     0.1 0.2 0.2 rad/s\n"
    tilt, g = SP.parse_imu(t)
    assert abs(tilt - 5.0) < 0.01 and abs(g - 0.3) < 1e-9
    assert SP.parse_imu("nothing") is None


def test_stop_criteria_scale_with_the_sim_band_inside_the_cap():
    mk = lambda v, w=10.0: dict(band=dict(tilt_peak_deg=dict(max=v), roll_rate_peak_dps=dict(max=w)))
    assert SP.stops_for(mk(2.0))["tilt_deg"] == 6.0
    assert SP.stops_for(mk(6.0))["tilt_deg"] == 9.0
    assert SP.stops_for(mk(40.0))["tilt_deg"] == 12.0
    # the rate stop is 1.5x the band's worst roll rate, between 1 and 3 rad/s
    assert SP.stops_for(mk(2.0, 10.0))["rate_rad_s"] == 1.0
    assert SP.stops_for(mk(2.0, 76.0))["rate_rad_s"] == pytest.approx(1.5 * math.radians(76.0), abs=0.01)
    assert SP.stops_for(mk(2.0, 900.0))["rate_rad_s"] == 3.0
    assert SP.stops_for(None) == dict(tilt_deg=6.0, rate_rad_s=1.0)


def test_go_refuses_a_rung_the_sim_band_fell_on(ladder, tmp_path, capsys):
    pred = fake_predictions(tmp_path / "p.json", ladder[1], up=17)       # 17 of 24 stay up
    pred_d = json.loads((tmp_path / "p.json").read_text())
    for R in pred_d["rungs"]:
        R["band"]["n"] = 24
    (tmp_path / "p.json").write_text(json.dumps(pred_d))
    called = []
    rc = SP.main(["--rung", "2", "--go", "--predictions", pred, "--out", str(tmp_path)],
                 robot_factory=lambda: called.append(1), input_fn=lambda s: "GO 2")
    assert rc == 2 and not called
    assert "not safe across the band" in capsys.readouterr().err


def test_the_committed_predictions_cover_the_ladder_and_the_lead_variants(ladder):
    ref, _ = ladder
    for fn, lead in (("predictions_refit.json", SR.LEAD), ("predictions.json", SR.LEAD), ("predictions_lead0.4.json", 0.4), ("predictions_lead1.2.json", 1.2)):
        pred = SP.load_predictions(os.path.join(SWAY, fn))
        rungs = ref.ladder(lead=lead)
        assert len(rungs) == 12
        for r in rungs:
            R, why = SP.prediction_for(pred, r, 200)
            assert R is not None, (fn, r.label, why)
            assert R["band"]["n"] == 24 and set(R["band"]["sides"]) == {"L", "R"}
            st = SP.stops_for(R)
            assert 6.0 <= st["tilt_deg"] <= 12.0 and 1.0 <= st["rate_rad_s"] <= 3.0
    # a different clock, cycle count or segment length is not this prediction
    assert os.path.samefile(SP.DEFAULT_PREDICTIONS, os.path.join(SWAY, "predictions_refit.json"))      # the sender defaults to the refit plant
    pred = SP.load_predictions(SP.DEFAULT_PREDICTIONS)
    assert pred["meta"]["plant"] == "refit"
    assert SP.prediction_for(pred, ref.rung(1, "sway", 0.02), 200)[0] is not None
    assert SP.prediction_for(pred, ref.rung(1, "sway", 0.02, hz=0.9), 200)[0] is None
    assert SP.prediction_for(pred, ref.rung(1, "sway", 0.02), 300)[0] is None
    assert SP.prediction_for(pred, ref.rung(1, "sway", 0.02), 200, "stance")[0] is None
