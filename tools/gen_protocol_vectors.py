#!/usr/bin/env python3
"""Freeze link/protocol.py's output into golden vectors for the C++ port.

Run:  .venv/bin/python tools/gen_protocol_vectors.py

link/protocol.py is the reference implementation (docs/control-channel.md
"Porting to firmware"); firmware/components/linkproto/ is a transcription of
it. A transcription needs a diff, so this walks the Python encoder, decoder
and Watchdog over a fixed script and writes what it produced into a C header
that firmware/host/test_protocol.cpp asserts against. Regenerate and re-run
the host tests whenever protocol.py changes -- a silent divergence between the
laptop's sender and the robot's decoder is exactly the bug this closes.

Every case value is rounded through numpy.float32 first: the firmware's API
takes float, Python's arithmetic is double, and (double)0.0005f is not 0.0005.
Freezing the float32 value is what makes the two sides comparable at all.

Deliberately included edge cases:
  * crc16_ccitt(b"123456789") == 0x29B1, the standard CCITT-FALSE check value
  * 0.0625 m/s, i.e. an exact 62.5 milli-unit TIE that is also exactly
    representable in float32 -- Python's round() is round-half-to-EVEN and a C
    port reaching for lround() (half-away-from-zero) gets 63 instead of 62
  * saturation past +-32.767, sequence wraparound, every single-bit flip class
  * a Watchdog script covering LIVE/STAND/RELAX/ESTOP, reordering, duplicates,
    a sender restart, and the "a stale frame must not pet the watchdog" rule
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "link"))

import protocol as P  # noqa: E402

OUT = os.path.join(ROOT, "firmware", "host", "vectors", "protocol_vectors.h")

# -- the command frames we freeze -------------------------------------------
CMD_CASES = [
    # (seq, vx, wz, flags)
    (0, 0.0, 0.0, 0),
    (1, 0.5, 0.0, P.FLAG_ENABLE),
    (7, 0.85, -0.4, P.FLAG_ENABLE),
    (3, 0.7, -0.2, P.FLAG_ENABLE),
    (42, 0.6667, 0.3333, P.FLAG_ENABLE),
    (99, -1.0, 1.0, P.FLAG_ENABLE | P.FLAG_ESTOP),
    (2 ** 32 - 1, 99.0, -99.0, 0xFF),          # int16 saturation both ways
    (5, 0.0625, -0.0625, P.FLAG_ENABLE),       # exact 62.5 tie -> 62 (even)
    (6, 0.1875, -0.1875, P.FLAG_ENABLE),       # exact 187.5 tie -> 188 (even)
    (123456, 0.3, 1.0, P.FLAG_ENABLE),
]

# extended command frames (2026-08-31): the 7-wide ext_cmd vector on the
# wire. Includes the short-frame rule (extras at defaults -> 14 B), the
# full-length layout, milli ties and saturation on the new channels.
CMD_EXT_CASES = [
    # (seq, vx, wz, flags, vy, crouch, lift, foot_dx, foot_dz)
    (10, 0.0, 0.0, P.FLAG_ENABLE, 0.0, 1.0, 0.0, 0.0, 0.0),   # -> short frame
    (11, 0.4, 0.0, P.FLAG_ENABLE, 0.0, 1.0, -1.0, 0.03, 0.0),
    (12, 0.0, 0.0, P.FLAG_ENABLE, 0.0, 0.8, 1.0, -0.02, 0.045),
    (13, 0.0, -0.2, P.FLAG_ENABLE, 0.15, 0.7, 0.0, 0.0, 0.0),
    (14, 0.0, 0.0, P.FLAG_ENABLE, 0.0, 1.0, 0.0, 0.0625, -0.0625),  # milli tie
    (15, 0.0, 0.0, 0xFF, 99.0, -99.0, 99.0, 99.0, -99.0),     # saturation
]

# Extended telemetry (mirror mode): the classic body plus 10 joint angles.
# Frozen as BYTES like everything else here, so the C++ port is diffed against
# numbers rather than against whatever the Python happened to do that day.
TLM_EXT_CASES = [
    # (seq_echo, state, vbat_v, up_z, vx_est, wz_est, err, late, joints)
    (1, P.LinkState.LIVE, 12.0, 1.0, 0.0, 0.0, 0x00, 0,
     (0.0,) * 10),                                   # a clean zero stand
    (2, P.LinkState.LIVE, 12.0, 0.99, 0.4, 0.0, 0x00, 2,
     (0.05, -0.12, 0.63, -1.21, 0.58,
      -0.05, 0.12, 0.61, -1.19, 0.57)),              # mid-stride, near-mirrored
    (3, P.LinkState.STAND, 11.0, 0.95, 0.0, 0.0, 0x03, 9,
     (0.0625, -0.0625, 0.0005, -0.0005, 0.0015,      # milli ties, both signs
      -0.0015, 0.5, -0.5, 1.5707963, -1.5707963)),
    (4, P.LinkState.FALLEN, 10.5, 0.1, 0.0, 0.0, 0xFF, 100,
     (99.0, -99.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),   # saturation
]

TLM_CASES = [
    # (seq_echo, state, vbat_v, up_z, vx_est, wz_est, servo_err, late_pct)
    (0, P.LinkState.LIVE, 11.4, 1.0, 0.0, 0.0, 0x00, 0),
    (99, P.LinkState.STAND, 11.4, 0.98, 0.55, -0.1, 0b100, 3),
    (7, P.LinkState.RELAX, 7.35, -0.2, -0.75, 0.4, 0xFF, 100),
    (2 ** 32 - 1, P.LinkState.ESTOP, 0.0, 0.0, 0.0, 0.0, 0x81, 255),
    (11, P.LinkState.LIVE, 12.6005, 0.9995, 0.0625, -0.1875, 0x02, 7),
    # The robot-latched under-voltage states. Included so the C++ port's
    # decodeTelemetry range check is diffed against the real top-of-enum
    # rather than against whichever value it was written for.
    (500, P.LinkState.VLAND, 9.9, 0.97, 0.0, 0.0, 0x00, 4),
    (501, P.LinkState.VSAFE, 9.8, 0.31, 0.0, 0.0, 0x00, 4),
    # Robot-latched fall: torso down, torque off. up_z is what tripped it.
    (502, P.LinkState.FALLEN, 11.2, 0.12, 0.0, 0.0, 0x00, 4),
    # Benched: what the WiFi beacon says before anyone arms the loop. The
    # zeros are real -- ctrl never published, so the snapshot is boot state.
    (0, P.LinkState.BENCH, 0.0, 0.0, 0.0, 0.0, 0x00, 0),
    # ... and benched WITH a diagnostic in seq_echo (2026-08-30). These are
    # the frames the incident of that day would have produced: a robot that
    # ignored six seconds of ARM frames, now saying why. Frozen here so the
    # firmware's packDiag() and the Python pack_diag() cannot drift apart in
    # the one field whose meaning depends on the state byte.
    (P.pack_diag(False, False, P.ArmResult.REFUSED_NO_CAL),
     P.LinkState.BENCH, 11.4, 0.0, 0.0, 0.0, 0x00, 0),
    (P.pack_diag(False, True, P.ArmResult.NONE),
     P.LinkState.BENCH, 11.4, 0.0, 0.0, 0.0, 0x00, 0),
]

# -- bench diagnostics -------------------------------------------------------
# Every packing, plus a value from a hypothetical NEWER firmware (0xF0) that
# this client must not mis-report as an older reason.
DIAG_CASES = [
    (False, False, P.ArmResult.NONE),
    (False, True, P.ArmResult.NONE),
    (False, False, P.ArmResult.REFUSED_NO_CAL),
    (False, True, P.ArmResult.REFUSED_NO_CAL),
    (True, True, P.ArmResult.ACCEPTED),
    (False, True, P.ArmResult.ACCEPTED),
    (False, True, P.ArmResult.DISARMED_FALL),
]
DIAG_RAW = [0x00, 0x02, 0x21, 0x13, 0xF0, 0xF3]   # decode-only, incl. unknown

# -- arm latch script --------------------------------------------------------
# flags per frame, in arrival order; the expected verdict is computed by the
# Python reference. Covers: first frame never counts (both levels), held
# levels, an ARM-ignorant sender, and duplicates.
ARM_SCRIPT = [
    0, 0, P.FLAG_ARM, P.FLAG_ARM | P.FLAG_ENABLE, P.FLAG_ARM | P.FLAG_ENABLE,
    P.FLAG_ENABLE, 0, P.FLAG_ARM, P.FLAG_ARM | P.FLAG_ESTOP, 0, 0,
    P.FLAG_ARM,
]
ARM_SCRIPT_HELD = [P.FLAG_ARM, P.FLAG_ARM, 0, P.FLAG_ARM]   # boots under ARM=1

# -- watchdog script --------------------------------------------------------
# ("rx", seq, vx, wz, flags, now_ms) | ("state", now_ms) | ("cmd", now_ms)
S, R = P.STALE_MS, P.RELAX_MS
WD_SCRIPT = [
    ("state", 0.0),                                   # never heard anything
    ("cmd", 0.0),
    ("rx", 1, 0.8, 0.0, P.FLAG_ENABLE, 0.0),
    ("state", 10.0), ("cmd", 10.0),
    ("state", S - 1.0), ("state", S + 1.0), ("cmd", S + 1.0),
    ("rx", 2, 0.5, 0.2, P.FLAG_ENABLE, S + 2.0),      # a late packet revives it
    ("state", S + 3.0), ("cmd", S + 3.0),
    ("rx", 1, 0.9, 0.0, P.FLAG_ENABLE, S + 4.0),      # reordered -> dropped
    ("cmd", S + 5.0),
    ("rx", 2, 0.9, 0.0, P.FLAG_ENABLE, S + 6.0),      # duplicate -> dropped
    ("state", R + 1.0),                               # ... and did not pet it
    ("rx", 3, 9.0, 9.0, P.FLAG_ENABLE, R + 2.0),      # clamped on the way in
    ("cmd", R + 3.0),
    ("rx", 4, 0.8, 0.0, 0, R + 4.0),                  # disabled: live but zero
    ("state", R + 5.0), ("cmd", R + 5.0),
    ("rx", 5, 0.15, 0.5, P.FLAG_ENABLE, R + 6.0),     # untrained band -> stand
    ("cmd", R + 7.0),
    ("rx", 6, 0.8, 0.0, P.FLAG_ESTOP, R + 8.0),       # e-stop latches
    ("state", R + 9.0), ("cmd", R + 9.0),
    ("rx", 7, 0.8, 0.0, P.FLAG_ENABLE, R + 10.0),     # no instant re-arm
    ("state", R + 11.0),
    ("rx", 8, 0.0, 0.0, 0, R + 12.0),                 # neutral clears it
    ("state", R + 13.0),
    ("rx", 9, 0.8, -0.3, P.FLAG_ENABLE, R + 14.0),
    ("cmd", R + 15.0),
    ("rx", 50000, 0.4, 0.0, P.FLAG_ENABLE, R + 16.0),
    ("rx", 0, 0.5, 0.0, P.FLAG_ENABLE, R + 17.0),     # sender restart resyncs
    ("cmd", R + 18.0),
]

_STATE_C = {P.LinkState.LIVE: "kLive", P.LinkState.STAND: "kStand",
            P.LinkState.RELAX: "kRelax", P.LinkState.ESTOP: "kEstop"}


def f32(x):
    """The value the firmware's float parameter will actually hold."""
    return float(np.float32(x))


def carr(b):
    return "{" + ", ".join("0x%02X" % x for x in b) + "}"


def cstr(s):
    """A C string literal body. The reason lines are plain ASCII prose, but
    escaping is one line and a stray quote would be a silent syntax error."""
    return s.replace("\\", "\\\\").replace('"', '\\"')


def fl(x):
    """A float literal that is always valid C: %g drops the '.' on integers,
    and `0f` / `-1f` are syntax errors."""
    s = "%.9g" % float(x)
    if "." not in s and "e" not in s and "E" not in s and "n" not in s:
        s += ".0"
    return s + "f"


def main():
    L = []
    w = L.append
    w("// GENERATED by tools/gen_protocol_vectors.py -- do not edit.")
    w("// Source of truth: link/protocol.py (the reference implementation).")
    w("#pragma once")
    w("#include <stddef.h>")
    w("#include <stdint.h>")
    w("")
    w("namespace protocol_vectors {")
    w("")

    # -- CRC ---------------------------------------------------------------
    w("struct CrcCase { const char* text; size_t len; uint16_t crc; };")
    w("inline const CrcCase kCrc[] = {")
    for s in (b"123456789", b"", b"BM\x01\x01", b"\x00" * 12, bytes(range(18))):
        w('    {"%s", %d, 0x%04X},' % (
            "".join("\\x%02x" % c for c in s), len(s), P.crc16_ccitt(s)))
    w("};")
    w("inline constexpr size_t kNumCrc = sizeof kCrc / sizeof kCrc[0];")
    w("")

    # -- command frames ----------------------------------------------------
    w("struct CmdCase {")
    w("    uint32_t seq; float vx; float wz; uint8_t flags;")
    w("    uint8_t wire[%d];" % P.CMD_LEN)
    w("    float dec_vx; float dec_wz;   // what decode_command() gives back")
    w("};")
    w("inline const CmdCase kCmd[] = {")
    for seq, vx, wz, flags in CMD_CASES:
        vx, wz = f32(vx), f32(wz)
        wire = P.encode_command(seq, vx, wz, flags)
        d = P.decode_command(wire)
        w("    {%du, %s, %s, 0x%02X, %s, %s, %s}," % (
            seq & 0xFFFFFFFF, fl(vx), fl(wz), flags & 0xFF, carr(wire),
            fl(d.vx), fl(d.wz)))
    w("};")
    w("inline constexpr size_t kNumCmd = sizeof kCmd / sizeof kCmd[0];")
    w("")

    # -- extended command frames -------------------------------------------
    w("struct CmdExtCase {")
    w("    uint32_t seq; float vx; float wz; uint8_t flags;")
    w("    float vy; float crouch; float lift; float fdx; float fdz;")
    w("    uint8_t wire[%d]; size_t wire_len;   // short-frame rule: defaults"
      % P.CMD_LEN_EXT)
    w("    float dec[7];   // decode: vx, wz, vy, crouch, lift, fdx, fdz")
    w("    float clamped[5];  // clamp_ext_to_envelope(vy, crouch, lift,")
    w("                       // fdx, fdz) of the DECODED frame")
    w("};")
    w("inline const CmdExtCase kCmdExt[] = {")
    for seq, vx, wz, flags, vy, crouch, lift, fdx, fdz in CMD_EXT_CASES:
        vx, wz, vy, crouch = f32(vx), f32(wz), f32(vy), f32(crouch)
        lift, fdx, fdz = f32(lift), f32(fdx), f32(fdz)
        wire = P.encode_command(seq, vx, wz, flags, vy=vy, crouch=crouch,
                                lift=lift, foot_dx=fdx, foot_dz=fdz)
        d = P.decode_command(wire)
        cl = P.clamp_ext_to_envelope(d)
        pad = bytes(wire) + b"\x00" * (P.CMD_LEN_EXT - len(wire))
        w("    {%du, %s, %s, 0x%02X, %s, %s, %s, %s, %s," % (
            seq & 0xFFFFFFFF, fl(vx), fl(wz), flags & 0xFF, fl(vy),
            fl(crouch), fl(lift), fl(fdx), fl(fdz)))
        w("     %s, %d," % (carr(pad), len(wire)))
        w("     {%s}," % ", ".join(fl(v) for v in (
            d.vx, d.wz, d.vy, d.crouch, d.lift, d.foot_dx, d.foot_dz)))
        w("     {%s}}," % ", ".join(fl(v) for v in cl))
    w("};")
    w("inline constexpr size_t kNumCmdExt = "
      "sizeof kCmdExt / sizeof kCmdExt[0];")
    w("")

    # -- malformed command frames ------------------------------------------
    good = P.encode_command(1, 0.5, 0.0, P.FLAG_ENABLE)
    bad = [
        ("kBadLength", good[:-1]),
        ("kBadLength", good + b"\x00"),
        ("kBadMagic", b"XX" + good[2:]),
        ("kBadVersion", good[:2] + b"\x09" + good[3:]),
        ("kBadCrc", good[:8] + b"\xff\xff" + good[10:]),
    ]
    w("struct BadCase { uint8_t wire[%d]; size_t len; const char* err; };"
      % (P.CMD_LEN + 1))
    w("inline const BadCase kBadCmd[] = {")
    for err, b in bad:
        pad = bytes(b) + b"\x00" * (P.CMD_LEN + 1 - len(b))
        w('    {%s, %d, "%s"},' % (carr(pad), len(b), err))
    w("};")
    w("inline constexpr size_t kNumBadCmd = sizeof kBadCmd / sizeof kBadCmd[0];")
    w("")

    # -- telemetry frames --------------------------------------------------
    w("struct TlmCase {")
    w("    uint32_t seq; uint8_t state; float vbat; float up_z;")
    w("    float vx; float wz; uint8_t servo_err; uint8_t late;")
    w("    uint8_t wire[%d];" % P.TLM_LEN)
    w("    uint8_t diag;   // Telemetry.diag: seq_echo's low byte, BENCH only")
    w("};")
    w("inline const TlmCase kTlm[] = {")
    for seq, st, vb, up, vx, wz, err, late in TLM_CASES:
        vb, up, vx, wz = f32(vb), f32(up), f32(vx), f32(wz)
        t = P.Telemetry(seq_echo=seq, state=st, vbat_v=vb, up_z=up,
                        vx_est=vx, wz_est=wz, servo_err=err,
                        loop_late_pct=late)
        wire = P.encode_telemetry(t)
        w("    {%du, %d, %s, %s, %s, %s, 0x%02X, %d, %s, 0x%02X}," % (
            seq & 0xFFFFFFFF, list(P.LinkState).index(st), fl(vb), fl(up),
            fl(vx), fl(wz), err, late, carr(wire), t.diag))
    w("};")
    w("inline constexpr size_t kNumTlm = sizeof kTlm / sizeof kTlm[0];")
    w("")

    # -- extended telemetry (mirror mode) ----------------------------------
    w("struct TlmExtCase {")
    w("    uint32_t seq; uint8_t state; float vbat; float up_z;")
    w("    float vx; float wz; uint8_t servo_err; uint8_t late;")
    w("    float joints[%d];" % P.NUM_JOINTS)
    w("    uint8_t wire[%d];" % P.TLM_LEN_EXT)
    w("};")
    w("inline const TlmExtCase kTlmExt[] = {")
    for seq, st, vb, up, vx, wz, err, late, joints in TLM_EXT_CASES:
        vb, up, vx, wz = f32(vb), f32(up), f32(vx), f32(wz)
        joints = tuple(f32(q) for q in joints)
        t = P.Telemetry(seq_echo=seq, state=st, vbat_v=vb, up_z=up,
                        vx_est=vx, wz_est=wz, servo_err=err,
                        loop_late_pct=late, joints=joints)
        wire = P.encode_telemetry(t)
        assert len(wire) == P.TLM_LEN_EXT, len(wire)
        w("    {%du, %d, %s, %s, %s, %s, 0x%02X, %d, {%s}, %s}," % (
            seq & 0xFFFFFFFF, list(P.LinkState).index(st), fl(vb), fl(up),
            fl(vx), fl(wz), err, late, ", ".join(fl(q) for q in joints),
            carr(wire)))
    w("};")
    w("inline constexpr size_t kNumTlmExt = sizeof kTlmExt / sizeof kTlmExt[0];")
    w("")

    # -- bench diagnostics -------------------------------------------------
    # The reason STRINGS are frozen too. They are what the operator reads on
    # a silent bench, and two copies of a sentence drift the moment one side
    # is reworded; this makes that a failing test.
    w("struct DiagCase {")
    w("    int run; int cal_ok; uint8_t result; uint8_t packed;")
    w("    const char* reason;")
    w("};")
    w("inline const DiagCase kDiag[] = {")
    for run, cal_ok, res in DIAG_CASES:
        packed = P.pack_diag(run, cal_ok, res)
        w('    {%d, %d, %d, 0x%02X, "%s"},' % (
            int(run), int(cal_ok), res.value, packed,
            cstr(P.diag_reason(packed))))
    w("};")
    w("inline constexpr size_t kNumDiag = sizeof kDiag / sizeof kDiag[0];")
    w("")
    w("// Decode-only, including a value from a NEWER firmware: an unknown")
    w("// ArmResult must produce its own message, never an older reason.")
    w("struct DiagRawCase { uint8_t diag; int known; const char* reason; };")
    w("inline const DiagRawCase kDiagRaw[] = {")
    for d in DIAG_RAW:
        r = P.diag_arm_result(d)
        w('    {0x%02X, %d, "%s"},' % (d, int(r is not None),
                                       cstr(P.diag_reason(d))))
    w("};")
    w("inline constexpr size_t kNumDiagRaw = "
      "sizeof kDiagRaw / sizeof kDiagRaw[0];")
    w("")

    # -- watchdog ----------------------------------------------------------
    w("// op: 0 = accept(pkt, now) -> `accepted`;  1 = state(now) -> `state`;")
    w("//     2 = command(now) -> (vx, wz)")
    w("struct WdStep {")
    w("    int op; uint32_t seq; float vx; float wz; uint8_t flags;")
    w("    float now_ms; int accepted; const char* state;")
    w("    float out_vx; float out_wz; uint32_t rejected;")
    w("};")
    w("inline const WdStep kWatchdog[] = {")
    dog = P.Watchdog()
    for step in WD_SCRIPT:
        if step[0] == "rx":
            _, seq, vx, wz, flags, now = step
            vx, wz = f32(vx), f32(wz)
            ok = dog.accept(P.Command(seq=seq, vx=vx, wz=wz, flags=flags), now)
            w('    {0, %du, %s, %s, 0x%02X, %s, %d, "", 0.0f, 0.0f, %d},'
              % (seq, fl(vx), fl(wz), flags, fl(now), int(ok), dog.rejected))
        elif step[0] == "state":
            now = step[1]
            st = dog.state(now)
            w('    {1, 0u, 0.0f, 0.0f, 0, %s, 0, "%s", 0.0f, 0.0f, %d},'
              % (fl(now), _STATE_C[st], dog.rejected))
        else:
            now = step[1]
            v, ww = dog.command(now)
            w('    {2, 0u, 0.0f, 0.0f, 0, %s, 0, "", %s, %s, %d},'
              % (fl(now), fl(v), fl(ww), dog.rejected))
    w("};")
    w("inline constexpr size_t kNumWatchdog = "
      "sizeof kWatchdog / sizeof kWatchdog[0];")
    w("")

    # -- arm latch ---------------------------------------------------------
    w("// verdict: -1 = no request, 0 = disarm (bench), 1 = arm (run)")
    w("struct ArmStep { uint8_t flags; int verdict; };")
    for name, script in (("kArm", ARM_SCRIPT), ("kArmHeld", ARM_SCRIPT_HELD)):
        w("inline const ArmStep %s[] = {" % name)
        latch = P.ArmLatch()
        for flags in script:
            v = latch.update(flags)
            w("    {0x%02X, %d}," % (flags, -1 if v is None else int(v)))
        w("};")
        w("inline constexpr size_t kNum%s = sizeof %s / sizeof %s[0];"
          % (name[1:], name, name))
    w("")

    # -- envelope ----------------------------------------------------------
    w("struct ClampCase { float vx; float wz; float out_vx; float out_wz; };")
    w("inline const ClampCase kClamp[] = {")
    for vx, wz in [(0.0, 0.0), (0.01, 0.0), (0.1, 0.0), (0.29, 0.0),
                   (0.3, 0.0), (0.0, 0.7), (9.0, 9.0), (-9.0, -9.0),
                   (-0.29, 0.5), (-0.5, -1.5), (1.0, 1.0)]:
        vx, wz = f32(vx), f32(wz)
        ov, ow = P.clamp_to_envelope(vx, wz)
        w("    {%s, %s, %s, %s}," % (fl(vx), fl(wz), fl(ov), fl(ow)))
    w("};")
    w("inline constexpr size_t kNumClamp = sizeof kClamp / sizeof kClamp[0];")
    w("")
    w("}  // namespace protocol_vectors")
    w("")

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as f:
        f.write("\n".join(L))
    print(f"wrote {OUT} "
          f"({len(CMD_CASES)} cmd, {len(TLM_CASES)}+{len(TLM_EXT_CASES)} tlm, "
          f"{len(WD_SCRIPT)} watchdog steps)")


if __name__ == "__main__":
    main()
