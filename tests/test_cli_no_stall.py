"""#297 (2026-11-25): no per-step CLI poll may stall the bench loop. The track player, `pose` streams and `balance`
run in cli::poseTick on the housekeeping task (firmware/main/link_task.cpp houseTask), the same task that executes
every CLI line, tether or Wi-Fi. A command that blocks there blocks the walking loop for as long as it runs: the old
plain `imu` read the IMU 51 times to time the read, ~115 ms at every step (docs/design-v6/
2026-11-25-weight-shift-snap-sim.md). cli.cpp only builds under ESP-IDF, so these are source-level guards, plus the
host parsers against the reply the firmware now prints."""
import os
import re
import sys

import pytest

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
MAIN = os.path.join(ROOT, "firmware", "main")
CLI = open(os.path.join(MAIN, "cli.cpp")).read()
WCLI = open(os.path.join(MAIN, "wifi_cli.cpp")).read()
HOUSE = open(os.path.join(MAIN, "link_task.cpp")).read()


def _body(src, signature):
    """The text of the function whose definition starts with `signature`, to its closing brace at column 0."""
    i = src.index(signature)
    j = src.index("\n}\n", i)
    return src[i:j]


IMU = _body(CLI, "void cmdImu(Sink out, int argc, char** argv) {")
STATUS = IMU[IMU.index("// Default: status."):]
STATUS_CODE = re.sub(r"//[^\n]*", "", STATUS)          # the comments may tell the history


def test_the_premise_cli_lines_and_the_bench_loop_share_one_task():
    house = _body(HOUSE, "void houseTask(void*) {")
    assert "wcliExecute(line.text, line.seq)" in house and "cli::execute(line.text, &tetherSay)" in house
    assert "cli::poseTick(&houseSay)" in house
    pose = _body(CLI, "void poseTick(Sink out) {")
    assert "trackTick(now, el, out)" in pose and "balTick(now, out)" in pose


def test_plain_imu_reads_nothing_from_the_part_but_its_id():
    assert "peekImu(" in STATUS_CODE                 # the sampler's latest sample
    for slow in ("dev->read(", "readRaw(", "calibrateBias(", "kN", "vTaskDelay("):
        assert slow not in STATUS_CODE, f"plain `imu` does {slow} again: it runs on the bench loop's task (#297)"


def test_imu_diagnostics_that_block_are_refused_while_the_bench_loop_plays():
    gate = IMU.index("benchLoopBusy(out, argv[1])")
    cond = IMU[IMU.index("const bool raw_many"):gate]
    for sub in ("time", "raw", "ae", "bias"):
        assert f'!strcmp(argv[1], "{sub}")' in cond, sub
    assert "atoi(argv[2]) > 1" in cond                   # `imu raw 1` (one read, board_bridge polls it) stays allowed
    for sub in ("time", "raw", "ae"):
        handler = IMU.index(f'if (argc >= 2 && !strcmp(argv[1], "{sub}"))')
        assert gate < handler, f"`imu {sub}` runs before the bench-loop gate"
    assert gate < IMU.index('if (argc >= 2 && !strcmp(argv[1], "bias")) {')
    timing = IMU[IMU.index('if (argc >= 2 && !strcmp(argv[1], "time"))'):]
    assert "dev->read(tmp)" in timing[:timing.index("return;")]          # the timing loop lives in `imu time` only
    busy = _body(CLI, "static bool benchLoopBusy(Sink out, const char* what) {")
    assert "s_traj_active" in busy and "s_track_on" in busy and "s_trace_on" in busy


def test_track_steps_formats_first_and_prints_once():
    trk = _body(CLI, "void cmdTrack(Sink out, int argc, char** argv) {")
    steps = trk[trk.index('if (!strcmp(sub, "steps")) {'):]
    steps = steps[:steps.index("        return;\n    }")]
    assert "say(" not in steps and steps.count("out(") == 1


def test_wifi_cli_sends_outside_its_lock():
    task = _body(WCLI, "void wcliTask(void*) {")
    send = WCLI[WCLI.index("    void send(const wcli::Peer& to"):WCLI.index("    bool submit(")]
    assert "lwip_sendto" not in send                       # send() only queues under g_mu
    assert task.index("unlock();", task.index("g_srv.pump(g_tx, 8);")) < task.index("g_tx.flushOut();")


# ---- the host parsers against the new reply (same line formats as before)

STATUS_REPLY = (
    "driver   QMI8658C\r\n"
    "who_am_i 0x05 (expect 0x05)   rev 0x7C   addr 0x6B\r\n"
    "up        0.0123 -0.0456  0.9988   (body z, yaw stripped = sim framezaxis at yaw 0)\r\n"
    "gyro      0.0010 -0.0020  0.0030 rad/s\r\n"
    "bias      0.00100  0.00200 -0.00300 rad/s   (from NVS)\r\n"
    "mount     0.7071  0.0000  0.7071  0.0000\r\n"
    "levelled yes   accel-reject streak 0\r\n"
    "quat      0.9990  0.0100  0.0200  0.0349\r\n"
    "projgrav -0.0123  0.0456  0.9988   (world +z in BODY frame)\r\n"
    "sample   2 ms old (the sampler's, 4 ms period); `imu time` times the read\r\n")


def test_the_reply_still_has_every_line_the_host_tools_parse():
    for fmt in ('"up       % .4f % .4f % .4f   (body z', '"gyro     % .4f % .4f % .4f rad/s', '"quat     % .4f',
                '"projgrav % .4f % .4f % .4f', '"levelled %s   accel-reject streak %d'):
        assert fmt in STATUS, fmt
    sys.path.insert(0, os.path.join(ROOT, "experiments", "roll-balance"))
    os.environ.setdefault("JAX_PLATFORMS", "cpu")
    H = pytest.importorskip("track_host")
    assert H.yaw_of_imu(STATUS_REPLY) == pytest.approx(4.0, abs=0.1)
    num = r"[-+]?\d*\.?\d+"
    assert re.search(rf"^up\s+({num})\s+({num})\s+({num})", STATUS_REPLY, re.M)        # experiments/stepper/robot_io
    assert re.search(rf"^gyro\s+({num})\s+({num})\s+({num})", STATUS_REPLY, re.M)
