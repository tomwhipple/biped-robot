"""Flight recorder: the Python codec, the collector and the reader.

Run:  .venv/bin/python -m pytest tests/test_flight_recorder.py -q

The C++ side of the codec is pinned to tools/flightrec.py by
firmware/host/test_flightrec.cpp (byte vectors); here the collector is driven
with synthetic packets -- holes, duplicates, a late packet, a reboot, an
unsynced clock -- both directly and over a real UDP socket on loopback.
"""
import json
import os
import socket
import subprocess
import sys
import time

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import flightrec as F  # noqa: E402
import flight_collector as C  # noqa: E402
import flight_reader as R  # noqa: E402

T0 = 1793382251_000000          # 2026-10-30T17:44:11Z, us
BOOT = 0xA1B2C3D4


def tick(seq, k, source=F.SRC_CTRL, boot=BOOT, utc=True, drop_ring=0, drop_net=0,
         pos=None):
    """A 12-joint tick at 50 Hz: tick k, boot clock 20 ms * k."""
    t_boot = 5_000_000 + 20_000 * k
    h = dict(boot_id=boot, seq=seq, t_utc_us=(T0 + 20_000 * k) if utc else 0,
             t_boot_us=t_boot, drop_ring=drop_ring, drop_net=drop_net)
    joints = [dict(bus=b, flags=F.JF_FB_OK | F.JF_NOM_VALID | F.JF_GOAL_VALID | F.JF_DRIVEN,
                   nominal=2048 + 10 * b, goal=2048 + 10 * b + 2,
                   pos=(pos if pos is not None else 2048 + 10 * b + 5),
                   load=-100 + b, temp_c=30, voltage_dv=121) for b in range(12)]
    return F.encode_tick(h, dict(
        source=source, flags=F.TF_IMU_VALID | F.TF_FB_READ, tick=k,
        fb_off_us=1500, fb_read_us=3300, roll=0.01 * k, pitch=-0.02,
        quat=(1.0, 0.0, 0.0, 0.0), joints=joints,
        bal=dict(state=1, side=ord("L"), u=0.05, err=-0.01)))


def event(seq, text, code=F.EV_CMD, k=0, boot=BOOT):
    h = dict(boot_id=boot, seq=seq, t_utc_us=T0 + 20_000 * k, t_boot_us=5_000_000 + 20_000 * k)
    return F.encode_event(h, dict(code=code, text=text))


def hello(seq, boot=BOOT, k=0):
    h = dict(boot_id=boot, seq=seq, t_utc_us=T0 + 20_000 * k, t_boot_us=5_000_000 + 20_000 * k)
    return F.encode_hello(h, dict(
        fw="test", utc_synced=1, enabled=1, idle_hz=10, cal_ok=1,
        id=[52, 6, 7, 8, 51, 2, 3, 4, 10, 9, 11, 12, 13, 14, 15, 16, 17],
        fitted=[1] * 12 + [0] * 5, dir=[1, -1] * 8 + [1], zero=[2048] * 17))


# -- codec --------------------------------------------------------------------
def test_tick_roundtrip_every_field():
    p = F.decode(tick(9, 3, source=F.SRC_BENCH_POSE, drop_ring=2, drop_net=5))
    assert p["type"] == F.T_TICK and p["seq"] == 9 and p["source"] == F.SRC_BENCH_POSE
    assert (p["drop_ring"], p["drop_net"]) == (2, 5)
    assert p["njoint"] == 12 and p["joints"][4]["goal"] == 2048 + 42
    assert p["joints"][11]["load"] == -89
    assert p["bal"]["side"] == ord("L") and p["bal"]["state"] == 1
    assert abs(p["bal"]["u"] - 0.05) < 1e-7
    assert p["ctrl"]["cmd"] == (0.0,) * 7
    assert len(tick(0, 0)) == F.HEADER_LEN + F.TICK_FIXED_LEN + 12 * F.JOINT_LEN


def test_sizes_match_the_firmware():
    # record.h: kHeaderLen 36, kTickFixedLen 164, kJointLen 20, max 576/slot
    assert F.HEADER_LEN + F.TICK_FIXED_LEN + 17 * F.JOINT_LEN <= 576
    assert len(hello(0)) == 36 + 116 + 5 * 17


def test_event_text_trimmed_and_capped():
    p = F.decode(event(1, "balance: landed\r\n", code=F.EV_OUT))
    assert p["text"] == "balance: landed" and p["code"] == F.EV_OUT
    assert len(F.decode(event(1, "x" * 500))["text"]) == F.MAX_TEXT


def test_decode_rejects_garbage():
    with pytest.raises(F.DecodeError):
        F.decode(b"hello")
    good = tick(1, 1)
    with pytest.raises(F.DecodeError):
        F.decode(good[:-1])
    with pytest.raises(F.DecodeError):
        F.decode(b"XX" + good[2:])


def test_vectors_header_is_current():
    """firmware/host/vectors/flightrec_vectors.h is what the generator writes."""
    path = os.path.join(ROOT, "firmware", "host", "vectors", "flightrec_vectors.h")
    before = open(path).read()
    subprocess.run([sys.executable, os.path.join(ROOT, "tools", "gen_flightrec_vectors.py")],
                   check=True, capture_output=True)
    assert open(path).read() == before, "regenerate: make vectors"


# -- collector ------------------------------------------------------------------
def test_seq_tracker_holes_dups_late():
    tr = C.SeqTracker()
    for s in (10, 11, 14, 12, 12, 17):
        tr.add(s)
    assert sorted(tr.missing) == [13, 15, 16]
    assert tr.late == 1 and tr.duplicates == 1 and tr.received == 5
    assert tr.gaps() == [[13, 13], [15, 16]]


def test_collector_session_file_and_summary(tmp_path):
    logs = []
    col = C.Collector(str(tmp_path), label="t", log=logs.append)
    rx = T0 + 3000
    seqs = [2, 3, 4, 5, 7, 8, 6, 10, 11]        # 9 lost, 6 late
    col.handle(hello(0), rx)
    col.handle(event(1, "pose 2082 1717 s3000 BL"), rx)
    for i, s in enumerate(seqs):
        col.handle(tick(s, s, drop_ring=1 if s >= 10 else 0), rx + 20_000 * i)
    col.handle(b"garbage", rx + 1)
    summ = col.rotate("test")
    path = os.path.join(str(tmp_path), summ["file"])
    assert summ["file"].startswith("flight_20261030T174411Z_t") and os.path.exists(path)
    b = summ["boots"][f"{BOOT:08x}"]
    assert b["gaps"] == [[9, 9]] and b["missing"] == 1
    assert b["late"] == 1 and b["robot_drop_ring"] == 1
    assert b["lost_on_network"] == 0           # the robot owned up to the one hole
    assert summ["undecodable"] == 1
    assert summ["by_type"] == {"tick": 9, "event": 1, "hello": 1}
    assert summ["tick_rates"]["ctrl"]["median_hz"] == 50.0
    on_disk = json.load(open(path[:-5] + ".summary.json"))
    assert on_disk["packets"] == 12


def test_collector_splits_on_reboot_and_silence(tmp_path):
    col = C.Collector(str(tmp_path), split_gap_s=5.0, log=lambda m: None)
    col.handle(tick(0, 0), T0)
    col.handle(tick(1, 1), T0 + 20_000)
    col.handle(tick(0, 0, boot=0x1234), T0 + 40_000 + 1_000_000)     # rebooted
    col.handle(tick(1, 1, boot=0x1234), T0 + 10_000_000)             # 9 s quiet
    col.rotate("end")
    assert len(col.closed) == 3
    assert len({c["file"] for c in col.closed}) == 3


def test_collector_over_udp(tmp_path):
    """The real service: a socket, SIGTERM, files on disk."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    proc = subprocess.Popen([sys.executable, os.path.join(ROOT, "tools", "flight_collector.py"),
                             "--port", str(port), "--bind", "127.0.0.1", "--out", str(tmp_path),
                             "--status-every", "0"], stdout=subprocess.PIPE, text=True)
    try:
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        deadline = time.time() + 10
        while time.time() < deadline and not list(tmp_path.glob("*.frec")):
            tx.sendto(hello(0), ("127.0.0.1", port))
            time.sleep(0.1)
        for k in range(1, 51):
            if k != 25:                              # one lost on purpose
                tx.sendto(tick(k, k), ("127.0.0.1", port))
        tx.sendto(event(51, "balance ABORT (tip)", code=F.EV_OUT, k=50), ("127.0.0.1", port))
        time.sleep(0.5)
    finally:
        proc.terminate()
        out, _ = proc.communicate(timeout=10)
    assert "session closed" in out
    summ = json.load(open(next(tmp_path.glob("*.summary.json"))))
    b = summ["boots"][f"{BOOT:08x}"]
    assert [25, 25] in b["gaps"]
    assert summ["by_type"]["tick"] == 49 and summ["by_type"]["event"] == 1


# -- reader -------------------------------------------------------------------------
def make_session(tmp_path, utc=True):
    col = C.Collector(str(tmp_path), log=lambda m: None)
    rx0 = T0 + 4000
    col.handle(hello(0), rx0)
    col.handle(event(1, "pose 2082 1717 2685 s3000 BL", k=1), rx0 + 20_000)
    for k in range(2, 102):
        col.handle(tick(k, k, utc=utc, pos=2100), rx0 + 20_000 * k)
    col.handle(event(102, "balance: landed", code=F.EV_OUT, k=101), rx0 + 20_000 * 102)
    summ = col.rotate("end")
    return os.path.join(str(tmp_path), summ["file"])


def test_reader_tables_and_angles(tmp_path):
    s = R.load(make_session(tmp_path))
    assert len(s.ticks) == 100 and len(s.events) == 2 and len(s.hellos) == 1
    rows = R.tick_rows(s)
    r = rows[0]
    assert r["source"] == "ctrl" and r["utc_src"] == "robot"
    # R_hip_pitch is bus 1, dir -1 in the hello: (2100 - 2048) * -2pi/4096
    assert r["R_hip_pitch.pos_rad"] == pytest.approx(-52 * 2 * 3.141592653589793 / 4096)
    assert r["R_hip_roll.err_rad"] == pytest.approx((2100 - 2050) * 2 * 3.141592653589793 / 4096)
    assert r["t_fb_utc"] == pytest.approx(r["t_utc"] + 0.0015)
    ev = R.event_rows(s)
    assert [e["kind"] for e in ev] == ["cmd", "out"]
    summ = R.summary(s)
    assert summ["sources"]["ctrl"]["median_hz"] == 50.0
    assert summ["sources"]["ctrl"]["feedback_answered_pct"]["L_ankle_roll"] == 100.0


def test_reader_unsynced_clock_falls_back(tmp_path):
    s = R.load(make_session(tmp_path, utc=False))
    t = s.ticks[0]
    assert t["utc_src"] == "boot+ntp"          # hello/events were synced
    assert t["utc_us"] == T0 + 20_000 * 2


def test_align_video_and_serial(tmp_path):
    path = make_session(tmp_path)
    s = R.load(path)
    rows = R.tick_rows(s)
    # 25 fps camera starting 0.3 s into the record, 40 frames, one before it
    pts = tmp_path / "cam_sony.pts"
    first_ms = (T0 + 20_000 * 2) // 1000 + 300
    pts.write_text("# timecode format v2\n" + "\n".join(
        str(first_ms + 40 * i) for i in range(-30, 40)) + "\n")
    al = R.align_video(rows, R.read_pts(str(pts)))
    assert len(al) == 70
    assert not al[0]["in_record"] and al[-1]["in_record"]
    t_first, t_last = rows[0]["t_fb_utc"], rows[-1]["t_fb_utc"]
    inside = [a for a in al if t_first <= a["frame_utc"] <= t_last]
    assert len(inside) > 30
    assert all(abs(a["tick_dt_ms"]) <= 10.0 for a in inside)   # 50 Hz: <= half a tick
    log = tmp_path / "2026-10-30" / "step_serial.log"
    log.parent.mkdir()
    log.write_text("17:44:11.000Z # port opened\n17:44:11.050Z > pose 1 2 3\n"
                   "17:44:11.100Z   pose -> 12 joints\nnoise line\n")
    ser = R.read_serial_log(str(log))
    assert [e["kind"] for e in ser] == ["note", "cmd", "out"]
    assert ser[1]["t_utc"] == pytest.approx(T0 / 1e6 + 0.05)
    assert R.main(["align", path, "--pts", str(pts), "--serial", str(log)]) == 0
    tl = (tmp_path / (os.path.basename(path)[:-5] + "_timeline.csv")).read_text()
    assert "pose 1 2 3" in tl and "balance: landed" in tl


def test_csv_and_plot(tmp_path):
    path = make_session(tmp_path)
    assert R.main(["csv", path]) == 0
    head = open(path[:-5] + "_ticks.csv").readline()
    assert "L_knee.pos_rad" in head and "bal_u" in head and "accel_x" in head
    pytest.importorskip("matplotlib")
    out = str(tmp_path / "p.png")
    assert R.main(["plot", path, "--joints", "R_knee,L_knee", "--out", out]) == 0
    assert os.path.getsize(out) > 10_000
