"""Offline tests for tools/gain_bench.py -- the Plan B bench (issue #73).

Covers:
  * the reply parsers against the firmware's own printf formats, read out of
    firmware/main/cli.cpp (a wording change there fails here, not on the bench);
  * the rehearsal servo (FakeBoard) prints exactly what the firmware prints;
  * the stiffness fit (slope with an intercept; quantization floor on the
    error), the hold and step statistics, and the verdict rules of the issue;
  * whole `stiffness` and `hold` sessions against FakeBoard with a scripted
    operator: one go per motion, gains restored at the end, the P x4 ratio
    recovered, a limit cycle caught;
  * the stance shortfall read from a squat_bench --balance CSV.

Nothing here touches hardware.
"""

from __future__ import annotations

import csv
import json
import math
import os
import re
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import gain_bench as gb  # noqa: E402
from bus_cal import POLICY_ORDER  # noqa: E402

CLI_CPP = open(os.path.join(ROOT, "firmware", "main", "cli.cpp")).read()


def c_format(fmt: str, *args) -> str:
    """Render a C printf format with Python's %: same field widths."""
    return re.sub(r"%(\d*)l?([du])", r"%\1d", fmt) % args


# -- the firmware's formats ------------------------------------------------------

POS_FMT = ('"id %ld  pos %5ld  spd %5ld  load %4ld  %4.1f V  %u C  "', '"err 0x%02X\\r\\n"')
GAINS_READ_FMT = '"id %ld  P %u  D %u  I %u   expected %s"'
GAINS_OK_FMT = '"id %ld: P %u D %u, verified after commit -- safe to "'
REG_FMT = '"id %ld reg %ld = %u (0x%02X)\\r\\n"'
MOVE_FMT = '"move id %ld -> %ld ticks, %ld ms, spd %ld, acc %ld: %s\\r\\n"'
SCAN_FMT = '"  id %3d  pos %5ld  %4.1f V  err 0x%02X  %s%s\\r\\n"'


@pytest.mark.parametrize("fragment", [*POS_FMT, GAINS_READ_FMT, GAINS_OK_FMT, REG_FMT, MOVE_FMT,
                                      SCAN_FMT, '"%d servo(s)\\r\\n"', "clamped to",
                                      '"%s %s: %s\\r\\n", on ? "torque" : "release"',
                                      'out("? (try `help`)\\r\\n")',
                                      '"busy: the control loop owns the bus'])
def test_the_parsed_formats_are_the_firmwares(fragment):
    assert fragment in CLI_CPP, f"cli.cpp no longer prints {fragment}: update gain_bench's parsers"


def test_parse_pos_reads_the_firmware_line():
    fmt = POS_FMT[0].strip('"') + POS_FMT[1].strip('"').replace("\\r\\n", "\r\n")
    line = c_format(fmt.replace("%4.1f", "%.1f"), 30, 2011, -40, 517, 12.1, 31, 0x20)
    r = gb.parse_pos("pos 30\r\n" + line, 30)
    assert r == {"pos": 2011, "spd": -40, "load": 517, "volt": 12.1, "temp": 31, "err": 0x20}
    assert gb.parse_pos(line, 31) is None
    assert gb.parse_pos("id 30: timeout\r\n", 30) is None


GAINS_WRITE_FMTS = [
    ('"id %ld: writing P %ld D %ld to EEPROM (torque must be OFF; "', False),   # progress
    ('"id %ld: P %u D %u, verified after commit -- safe to "', True),
    ('"REFUSED: id %ld has torque ON. `release %ld` first -- "', True),
    ('"REFUSED: id %ld did not answer the torque read -- "', True),
    ('"id %ld: WROTE BUT READ BACK P %u D %u -- the EEPROM did "', True),
    ('"id %ld: FAILED (%s) -- EEPROM re-lock attempted; "', True),
    ('"gains: id must be 0-253 (one servo; never broadcast)\\r\\n"', True),
    ('"gains: P must be 1-254 and D 0-254 (factory 32/32)\\r\\n"', True),
]


@pytest.mark.parametrize("fmt,ends", GAINS_WRITE_FMTS)
def test_only_a_verdict_ends_the_gain_write(fmt, ends):
    """The progress line comes at once and the verdict ~2 s later: a pattern
    that stops on the progress line reads no verdict (bench, 2026-09-30)."""
    assert fmt in CLI_CPP
    line = re.sub(r"%\w+", "30", fmt.strip('"'))
    line = line.replace("safe to ", "safe to power down")
    assert bool(re.search(gb.GAINS_WRITE_UNTIL, line)) == ends, line


def test_the_rehearsal_servo_delivers_late_like_the_tether():
    fake = gb.FakeBoard(sid=30)
    first = fake.cmd("gains 30 64 64", until=r"must be")          # the old, wrong pattern
    assert "writing P 64" in first and "verified" not in first
    assert "verified after commit" in fake.cmd("release 30", until=r"release \S+: \S+\r?\n")
    ok = fake.cmd("gains 30 96 96", until=gb.GAINS_WRITE_UNTIL)
    assert gb.parse_gains_write(ok, 30) == (96, 96)


def test_parse_gains_and_reg():
    line = c_format(GAINS_READ_FMT.strip('"'), 30, 128, 64, 0, "(no row in servo_gains.h)")
    assert gb.parse_gains_read(line, 30) == (128, 64, 0)
    ok = c_format(GAINS_OK_FMT.strip('"'), 30, 128, 64)
    assert gb.parse_gains_write(ok, 30) == (128, 64)
    assert gb.parse_gains_write("REFUSED: id 30 has torque ON.", 30) is None
    reg = c_format(REG_FMT.strip('"').replace("\\r\\n", ""), 30, 36, 80, 80)
    assert gb.parse_reg(reg, 30, 36) == 80
    assert gb.parse_reg(reg, 30, 35) is None


def test_parse_scan():
    fmt = SCAN_FMT.strip('"').replace("\\r\\n", "\r\n")
    text = ("scanning IDs 0-253 ...\r\n"
            + c_format(fmt, 1, 3532, 12.1, 0, "R_hip_roll", "")
            + c_format(fmt, 30, 2048, 12.0, 0x20, "-", "")
            + "2 servo(s)\r\n")
    got = gb.parse_scan(text)
    assert [(g["id"], g["pos"], g["joint"], g["err"]) for g in got] == \
        [(1, 3532, "R_hip_roll", 0), (30, 2048, "-", 0x20)]
    assert gb.parse_pos(text, 30) is None                  # not mistaken for a `pos` reply


def test_bus_map_ids_are_refused_for_motion(tmp_path, monkeypatch):
    monkeypatch.setattr("builtins.input", Script())
    assert gb.main(["--rehearse", "--session", str(tmp_path), "stiffness", "1"]) == 2
    assert "clamps" in open(tmp_path / "session.log").read()
    assert gb.main(["--rehearse", "--session", str(tmp_path), "read", "1"]) == 0


def test_a_clamped_move_releases():
    class Clamped(gb.FakeBoard):
        def cmd(self, line, until="", timeout=0.0):
            out = super().cmd(line, until, timeout)
            if line.startswith("move"):
                out = "id 30: 2048 is outside x's range [0..10], clamped to 10\r\n" + out
            return out
    fake = Clamped(sid=30)
    op = gb.Operator(ask=lambda p: "go" if "type go" in p else "", say=lambda s: None)
    with pytest.raises(gb.Abort, match="pin move"):
        gb.run_stiffness(fake, op, 30, [(32, 32)], 0.1, [0.5], 2, 0.0)
    assert not fake.torque


def test_the_rehearsal_servo_prints_what_the_firmware_prints():
    fake = gb.FakeBoard(sid=30)
    fmt = POS_FMT[0].strip('"') + POS_FMT[1].strip('"').replace("\\r\\n", "\r\n")
    want = c_format(fmt.replace("%4.1f", "%4.1f"), 30, 2048, 0, 0, 12.1, 31, 0)
    assert fake.cmd("pos 30") == want
    assert gb.parse_scan(fake.cmd("scan"))[0]["id"] == 30
    assert fake.cmd("gains 30").startswith(
        c_format(GAINS_READ_FMT.strip('"'), 30, 32, 32, 0, "(no row in servo_gains.h)"))
    assert c_format(REG_FMT.strip('"').replace("\\r\\n", "\r\n"), 30, 36, 80, 80) == \
        fake.cmd("reg 30 36 1")
    fake.cmd("torque 30")
    assert fake.cmd("move 30 2050 0 100").startswith("move id 30 -> 2050 ticks")
    assert "REFUSED" in fake.cmd("gains 30 64 64")          # torque on: refused, like the firmware
    fake.cmd("release 30")
    assert "verified after commit" in fake.cmd("gains 30 64 64")


# -- analysis ------------------------------------------------------------------

def test_fit_recovers_k_and_ignores_a_friction_offset():
    k = 20.0                                        # N*m/rad
    pts = [(t, (t - 0.2) / k * gb.TICKS_PER_RAD) for t in (0.5, 1.0, 1.5)]
    fit = gb.fit_stiffness(pts)
    assert fit["k"] == pytest.approx(k)
    assert fit["intercept_ticks"] == pytest.approx(-0.2 / k * gb.TICKS_PER_RAD)
    # a perfect fit still carries the +-0.5 tick quantization floor
    assert fit["k_rel_se"] > 0
    assert math.isnan(gb.fit_stiffness(pts[:1])["k"])


def test_ladder():
    assert gb.parse_ladder("32,64,96,128", 1.0) == [(32, 32), (64, 64), (96, 96), (128, 128)]
    assert gb.parse_ladder("128:32, 96", 0.5) == [(128, 32), (96, 48)]
    for bad in ("0", "255", "128:300", ""):
        with pytest.raises(ValueError):
            gb.parse_ladder(bad, 1.0)


def _samples(pos, spd=0, err=0, dt=0.02):
    return [{"t": k * dt, "pos": p, "spd": spd, "load": 10, "volt": 12.0, "temp": 30,
             "err": err} for k, p in enumerate(pos)]


def test_window_and_step_stats():
    w = gb.window_stats(_samples([2048, 2049, 2048, 2047]))
    assert (w["range"], w["n"], w["pos_min"], w["pos_max"]) == (2, 4, 2047, 2048 + 1)
    # a step 2048 -> 2071 that overshoots to 2075 and settles at 2070
    pos = [2048, 2060, 2075, 2072, 2070] + [2070] * 60
    s = gb.step_stats(_samples(pos), 2048, 2071)
    assert s["overshoot"] == pytest.approx(5)
    assert s["residual_range"] == 0
    assert s["offset"] == pytest.approx(-1)
    assert s["settle_s"] == pytest.approx(0.08)


def test_verdicts():
    assert gb.verdict(4.1, True, 0) == "PASS"
    assert gb.verdict(3.9, True, 0, ratio_se=0.2).startswith("MARGINAL")
    assert gb.verdict(3.7, True, 0, ratio_se=0.2).startswith("CONDITIONAL")
    assert gb.verdict(3.4, True, 0).startswith("CONDITIONAL")
    assert gb.verdict(2.5, True, 0) == "not stiff enough"
    assert gb.verdict(4.5, False, 0) == "FAIL (not quiet)"
    assert gb.verdict(4.5, True, 0x20).startswith("FAIL (protection: overload")
    assert gb.verdict(4.5, True, 0x01) == "PASS"         # voltage is the supply, not the gain
    assert gb.verdict(4.5, None, 0) == "stiff enough; hold not run"
    assert gb.stance_verdict(0.25) == "PASS"
    assert gb.stance_verdict(0.35).endswith("3x route only")
    assert gb.stance_verdict(1.2) == "FAIL"


# -- whole sessions against the rehearsal servo ---------------------------------

class Script:
    """A scripted operator: `go` to every motion, enter to every wait."""

    def __init__(self):
        self.prompts = []

    def __call__(self, prompt):
        self.prompts.append(prompt)
        return "go" if "type go" in prompt else ""

    def gos(self):
        return [p for p in self.prompts if "type go" in p]


def test_stiffness_session_recovers_p_times_4(tmp_path, monkeypatch):
    script = Script()
    monkeypatch.setattr("builtins.input", script)
    rc = gb.main(["--rehearse", "--session", str(tmp_path), "stiffness", "30",
                  "--samples", "3"])
    assert rc == 0
    res = json.load(open(tmp_path / "stiffness.json"))
    assert [(r["p"], r["d"]) for r in res["rows"]] == [(32, 32), (64, 64), (96, 96),
                                                       (128, 128), (160, 160)]
    assert res["start_gains"] == [32, 32, 0]
    # one go per motion: the torque-on/pin at each P, nothing else moves
    assert len(script.gos()) == 5
    summ = gb.summarise(res, None)
    by_p = {j["p"]: j for j in summ["joined"]}
    assert by_p[32]["k"] == pytest.approx(17.0, rel=0.05)     # the fake's P = 32 stiffness
    assert by_p[128]["ratio"] == pytest.approx(4.0, rel=0.1)
    assert by_p[64]["ratio"] == pytest.approx(2.0, rel=0.1)
    # exactly 4x stiffer reads 3.90 +- 0.17 through whole ticks: on the line
    assert by_p[128]["verdict"].startswith("MARGINAL")
    assert by_p[160]["verdict"] == "stiff enough; hold not run"
    log = open(tmp_path / "session.log").read()
    writes = re.findall(r">>> gains 30 (\d+) (\d+)\n", log)
    assert writes[-2:] == [("160", "160"), ("32", "32")]         # the ladder, then restored
    md = gb.render_report(str(tmp_path))
    assert "## Stiffness vs P" in md and "| 128 | 128 |" in md


def test_hold_session_catches_a_limit_cycle(tmp_path):
    script = Script()
    op = gb.Operator(ask=script, say=lambda s: None)
    fake = gb.FakeBoard(sid=30, buzz_p=128)
    res = gb.run_hold(fake, op, 30, [(32, 32), (128, 128)], ["down"], quiet_s=0.03,
                      step_ticks=23, step_s=0.05, settle_s=0.0, inertia="0.25 kg at 0.10 m")
    rows = [gb.hold_row(r) for r in res["rows"]]
    assert [r["is_quiet"] for r in rows] == [True, False]
    assert rows[1]["quiet"]["range"] == 6
    # per P: torque-on/pin, step out, step back
    assert len(script.gos()) == 6
    assert not fake.torque                                   # released after every P


def test_no_go_stops_before_the_motion(tmp_path):
    fake = gb.FakeBoard(sid=30)
    op = gb.Operator(ask=lambda p: "no" if "type go" in p else "", say=lambda s: None)
    with pytest.raises(gb.Abort):
        gb.run_stiffness(fake, op, 30, [(64, 64)], 0.1, [0.5], 2, 0.0)
    assert not fake.torque


def test_no_terminal_aborts_and_restores(tmp_path, monkeypatch):
    def eof(prompt):
        raise EOFError
    monkeypatch.setattr("builtins.input", eof)
    rc = gb.main(["--rehearse", "--session", str(tmp_path), "stiffness", "30",
                  "--ladder", "32,128", "--samples", "2"])
    assert rc == 2
    log = open(tmp_path / "session.log").read()
    assert "no operator on stdin" in log
    assert re.findall(r">>> gains 30 (\d+) (\d+)\n", log)[-1] == ("32", "32")


def test_a_jump_at_torque_on_releases(tmp_path):
    class Jumpy(gb.FakeBoard):
        def cmd(self, line, until="", timeout=0.0):
            out = super().cmd(line, until, timeout)
            if line.startswith("torque"):
                self.goal = self.rest + 200                   # a stale goal register
            return out
    fake = Jumpy(sid=30)
    op = gb.Operator(ask=lambda p: "go" if "type go" in p else "", say=lambda s: None)
    with pytest.raises(gb.Abort, match="jumped"):
        gb.run_stiffness(fake, op, 30, [(32, 32)], 0.1, [0.5], 2, 0.0)
    assert not fake.torque


# -- step 1 ----------------------------------------------------------------------

def _squat_csv(path, l_roll_deg, l_roll_load):
    names = list(POLICY_ORDER)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["rep", "phase", "hip_cmd", "knee_cmd", "ankle_cmd", "t", "tilt_deg"]
                   + [f"{n}_deg" for n in names] + [f"{n}_load" for n in names])
        for rep, phase in enumerate(("lean+lift", "lean", "zero"), 1):
            ang = [0.0] * len(names)
            load = [5] * len(names)
            if phase != "zero":
                ang[names.index("L_hip_roll")] = l_roll_deg
                load[names.index("L_hip_roll")] = l_roll_load
            w.writerow([rep, phase, 0, 0, 0, rep * 3.0, 1.5] + ang + load)


def test_stance_shortfall_from_squat_bench_csv(tmp_path):
    p = tmp_path / "stance.csv"
    _squat_csv(p, -6.8, 120)                     # the 2026-09-13 stock result
    r = gb.stance_from_csv(str(p), "L", 8.0)
    assert r["target"] == -8.0
    assert r["short"] == pytest.approx(1.2)
    assert r["load"] == 120
    assert [ph["phase"] for ph in r["phases"]] == ["lean+lift", "lean"]
    assert gb.stance_verdict(r["short"]) == "FAIL"
    _squat_csv(p, -7.8, 130)
    assert gb.stance_verdict(gb.stance_from_csv(str(p), "L", 8.0)["short"]) == "PASS"


def test_roll_ids_are_the_bus_maps():
    assert gb.roll_id("L") == 5 and gb.roll_id("R") == 1


# -- a bus-map ID on the bench (--allow-bus-id) ------------------------------------

def test_the_envelope_is_the_firmwares():
    lo, hi = gb.mech_envelope_deg()
    assert len(lo) == len(hi) == 17
    assert (lo[10], hi[10]) == (-25.0, 25.0)                 # id 11, R_ankle_roll
    joint = gb.BusJoint("R_ankle_roll", 10, 11, "A", 2048, 1, False)
    assert gb.envelope_ticks(joint, -25.0, 25.0) == (1764, 2332)
    flipped = gb.BusJoint("R_ankle_roll", 10, 11, "A", 2048, -1, False)
    assert gb.envelope_ticks(flipped, -25.0, 25.0) == (1764, 2332)
    near_top = gb.BusJoint("R_ankle_roll", 10, 11, "A", 4000, 1, False)
    assert gb.envelope_ticks(near_top, -25.0, 25.0) == (3716, 4095)


def test_room_refuses_the_wrap_and_the_clamp():
    env = (1764, 2332, "R_ankle_roll")
    gb.check_room(11, 2048, 2048 - 120, 2048 + 23 + 120, env)
    with pytest.raises(gb.Abort, match="wrap"):
        gb.check_room(11, 4094, 4094 - 120, 4094 + 120, None)   # the bench servo as found
    with pytest.raises(gb.Abort, match="envelope"):
        gb.check_room(11, 2250, 2250 - 120, 2250 + 120, env)


def test_park_turns_the_bare_horn_to_mid_band():
    fake = gb.FakeBoard(sid=11, pos=4094)
    op = gb.Operator(ask=lambda p: "go" if "type go" in p else "", say=lambda s: None)
    env = gb.bus_envelope(fake, 11)
    assert env[:2] == (1764, 2332)
    res = gb.run_park(fake, op, 11, None, env)
    assert (res["start"], res["target"], res["end"]) == (4094, 2048, 2048)
    assert not fake.torque
    with pytest.raises(gb.Abort, match="envelope"):
        gb.run_park(fake, op, 11, 3000, env)


def test_a_goal_on_the_wrap_is_refused_before_any_motion():
    script = Script()
    fake = gb.FakeBoard(sid=11, pos=4094)
    with pytest.raises(gb.Abort, match="wrap"):
        gb.run_stiffness(fake, gb.Operator(ask=script, say=lambda s: None), 11, [(32, 32)],
                         0.1, [0.5], 2, 0.0, env=gb.bus_envelope(fake, 11))
    assert script.gos() == [] and not fake.torque


def test_allow_bus_id(tmp_path, monkeypatch):
    monkeypatch.setattr("builtins.input", Script())
    args = ["--rehearse", "--session", str(tmp_path), "stiffness", "11", "--ladder", "32",
            "--samples", "2"]
    assert gb.main(args) == 2                                  # refused without the flag
    assert gb.main(args + ["--allow-bus-id"]) == 0


# -- a gain write whose ack is lost (#101) -------------------------------------------

def test_a_landed_write_with_a_lost_ack_is_accepted(monkeypatch):
    monkeypatch.setattr(gb, "EEPROM_COMMIT_S", 0.0)
    fake = gb.FakeBoard(sid=11, ack_loss=1)
    assert gb.write_gains(fake, 11, 64, 64) == "landed, ack lost"
    assert (fake.p, fake.d) == (64, 64)
    assert gb.write_gains(fake, 11, 32, 32) == "ok"


def test_an_unlocked_eeprom_is_not_accepted_the_retry_relocks(monkeypatch):
    monkeypatch.setattr(gb, "EEPROM_COMMIT_S", 0.0)
    fake = gb.FakeBoard(sid=11, ack_loss=1)
    fake.lock = 0                                  # the re-lock never landed either
    assert gb.write_gains(fake, 11, 64, 64) == "ok"   # same-value retry: verified, locked
    assert fake.lock == 1


def test_a_write_that_never_lands_is_retried_once_then_refused(monkeypatch):
    monkeypatch.setattr(gb, "EEPROM_COMMIT_S", 0.0)

    class Dead(gb.FakeBoard):
        def _reply(self, line):
            if line.startswith("gains") and len(line.split()) == 4:
                self.writes += 1
                return "id 11: FAILED (write-failed) -- EEPROM re-lock attempted\r\n"
            return super()._reply(line)
    fake = Dead(sid=11)
    fake.writes = 0
    with pytest.raises(gb.Abort, match="not verified"):
        gb.write_gains(fake, 11, 64, 64)
    assert fake.writes == 2 and (fake.p, fake.d) == (32, 32)


def test_the_bench_session_runs_through_lost_acks(tmp_path, monkeypatch):
    monkeypatch.setattr(gb, "EEPROM_COMMIT_S", 0.0)
    script = Script()
    op = gb.Operator(ask=script, say=lambda s: None)
    fake = gb.FakeBoard(sid=30, ack_loss=2)
    res = gb.run_stiffness(fake, op, 30, [(32, 32), (64, 64), (128, 128)], 0.1, [0.5, 1.0],
                           2, 0.0)
    assert [r["gain_write"] for r in res["rows"]] == ["ok", "landed, ack lost",
                                                      "landed, ack lost"]
