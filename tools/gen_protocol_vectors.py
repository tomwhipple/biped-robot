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
]

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
    w("};")
    w("inline const TlmCase kTlm[] = {")
    for seq, st, vb, up, vx, wz, err, late in TLM_CASES:
        vb, up, vx, wz = f32(vb), f32(up), f32(vx), f32(wz)
        t = P.Telemetry(seq_echo=seq, state=st, vbat_v=vb, up_z=up,
                        vx_est=vx, wz_est=wz, servo_err=err,
                        loop_late_pct=late)
        wire = P.encode_telemetry(t)
        w("    {%du, %d, %s, %s, %s, %s, 0x%02X, %d, %s}," % (
            seq & 0xFFFFFFFF, list(P.LinkState).index(st), fl(vb), fl(up),
            fl(vx), fl(wz), err, late, carr(wire)))
    w("};")
    w("inline constexpr size_t kNumTlm = sizeof kTlm / sizeof kTlm[0];")
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
          f"({len(CMD_CASES)} cmd, {len(TLM_CASES)} tlm, "
          f"{len(WD_SCRIPT)} watchdog steps)")


if __name__ == "__main__":
    main()
