#!/usr/bin/env python3
"""Freeze tools/flightrec.py's encoder into golden vectors for the C++ port.

Run:  .venv/bin/python tools/gen_flightrec_vectors.py     (or `make vectors`)

The flight recorder's wire format is written twice: in C++ on the robot
(firmware/components/flightrec) and in Python on mira (tools/flightrec.py,
which the collector and every analysis decode with). This writes, per case,
a C++ function that builds the record field by field AND the bytes Python
encoded for the same values, into firmware/host/vectors/flightrec_vectors.h;
firmware/host/test_flightrec.cpp encodes the former and asserts it equals the
latter. Any field moved, resized or reordered on either side fails there.

Every float is a dyadic rational (k / 2^n), exact in float32, so the two
languages cannot disagree about rounding.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import flightrec as F  # noqa: E402

OUT = os.path.join(ROOT, "firmware", "host", "vectors", "flightrec_vectors.h")

# A real robot clock reading (2026-10-30T17:44:11.25Z) so every byte of the
# u64 is non-zero and the byte order is diffed against a value that shows it.
T_UTC = 1793382251250000


def tick_case(n_joints, source, with_bal):
    h = dict(boot_id=0xA1B2C3D4, seq=0x01020304 + n_joints, t_utc_us=T_UTC,
             t_boot_us=0x0000000123456789, drop_ring=3, drop_net=0x00010002)
    joints = []
    for k in range(n_joints):
        joints.append(dict(
            bus=k, flags=(k * 7) & 0x1F, status=(0x20 if k == 3 else 0),
            temp_c=25 + k, voltage_dv=120 - k, moving=k & 1,
            nominal=2048 + 37 * k - 300, goal=2048 + 41 * k - 310,
            goal_speed=15 + 100 * k, pos=-(k + 1) if k == 5 else 2000 + 13 * k,
            speed=(-1) ** k * 25 * k, load=-500 + 77 * k, current=-3 + 2 * k))
    bal = {}
    if with_bal:
        bal = dict(state=1, side=ord("L"), ankle_bus=11, hip_bus=4,
                   ref=-0.0625, err=0.03125, integral=-0.5, u=0.125,
                   rate=-1.75, roll=-0.09375, ankle_rad=0.0625,
                   hip_rad=0.15625, ankle_ticks=-41, hip_ticks=102,
                   clamps=70000)
    t = dict(source=source, flags=0xA5, link_state=6, tick=987654,
             work_us=8123, fb_off_us=-250, fb_read_us=3375, imu_age_us=-1,
             accel=(0.5, -9.75, 0.25), gyro=(0.015625, -0.25, 2.5),
             quat=(0.875, -0.125, 0.25, -0.0625), up=(0.0, -0.5, 0.75),
             roll=-0.28125, pitch=0.046875, bal=bal,
             ctrl=dict(cmd=(0.25, -0.125, 0.5, 1.0, -1.0, 0.03125, 0.0),
                       phase=0.375, vbat_mv=12100, batt=1, home_stage=3),
             joints=joints)
    return h, t


def event_case(code, text, a, b):
    h = dict(boot_id=0xA1B2C3D4, seq=77, t_utc_us=T_UTC + 20000,
             t_boot_us=55555555, drop_ring=0, drop_net=1)
    return h, dict(code=code, src=0, text=text, a=a, b=b)


def hello_case():
    h = dict(boot_id=0xA1B2C3D4, seq=5, t_utc_us=0, t_boot_us=1000000,
             drop_ring=0, drop_net=0)
    m = dict(fw="v6-flight-test", utc_synced=1, enabled=1, idle_hz=10, cal_ok=1)
    for i, k in enumerate(F.HELLO_COUNTERS):
        m[k] = 1000 * (i + 1) + i
    m["id"] = [52, 6, 7, 8, 51, 2, 3, 4, 10, 9, 11, 12, 13, 14, 15, 16, 17]
    m["fitted"] = [1] * 12 + [0] * 5
    m["dir"] = [1, -1] * 8 + [1]
    m["zero"] = [2048 + 11 * b - 90 for b in range(F.MAX_JOINTS)]
    return h, m


def cbytes(b: bytes) -> str:
    return ", ".join(f"0x{x:02X}" for x in b)


def cfloat(x: float) -> str:
    return repr(float(x)) + "f"


def emit_header(h, ptype):
    return [f"    h.type = flightrec::Type::{ptype};",
            f"    h.boot_id = 0x{h['boot_id']:08X}u;",
            f"    h.seq = {h['seq']}u;",
            f"    h.t_utc_us = {h['t_utc_us']}ull;",
            f"    h.t_boot_us = {h['t_boot_us']}ull;",
            f"    h.drop_ring = {h['drop_ring']}u;",
            f"    h.drop_net = {h['drop_net']}u;"]


def emit_tick(name, h, t, wire):
    src = {F.SRC_CTRL: "kCtrl", F.SRC_BENCH_POSE: "kBenchPose",
           F.SRC_BENCH_IDLE: "kBenchIdle"}[t["source"]]
    L = [f"inline void {name}(flightrec::Header& h, flightrec::Tick& t) {{"]
    L += emit_header(h, "kTick")
    L += [f"    t.source = flightrec::Source::{src};",
          f"    t.flags = {t['flags']};", f"    t.link_state = {t['link_state']};",
          f"    t.njoint = {len(t['joints'])};", f"    t.tick = {t['tick']}u;",
          f"    t.work_us = {t['work_us']}u;", f"    t.fb_off_us = {t['fb_off_us']};",
          f"    t.fb_read_us = {t['fb_read_us']}u;",
          f"    t.imu_age_us = {t['imu_age_us']};"]
    for arr in ("accel", "gyro", "quat", "up"):
        for i, v in enumerate(t[arr]):
            L.append(f"    t.{arr}[{i}] = {cfloat(v)};")
    L += [f"    t.roll = {cfloat(t['roll'])};", f"    t.pitch = {cfloat(t['pitch'])};"]
    for k, v in t["bal"].items():
        L.append(f"    t.bal.{k} = {cfloat(v) if isinstance(v, float) else v};")
    for i, v in enumerate(t["ctrl"]["cmd"]):
        L.append(f"    t.ctrl.cmd[{i}] = {cfloat(v)};")
    c = t["ctrl"]
    L += [f"    t.ctrl.phase = {cfloat(c['phase'])};",
          f"    t.ctrl.vbat_mv = {c['vbat_mv']};", f"    t.ctrl.batt = {c['batt']};",
          f"    t.ctrl.home_stage = {c['home_stage']};"]
    for k, j in enumerate(t["joints"]):
        L.append("    t.joint[%d] = {%s};" % (k, ", ".join(
            f"static_cast<{'uint8_t' if i < 6 else ('uint16_t' if f == 'goal_speed' else 'int16_t')}>({j[f]})"
            for i, f in enumerate(F.JOINT_FIELDS))))
    L.append("}")
    L.append(f"inline const uint8_t {name}_wire[{len(wire)}] = {{{cbytes(wire)}}};")
    return L


def emit_event(name, h, e, wire):
    code = {v: k for k, v in vars(F).items() if k.startswith("EV_")}[e["code"]]
    cname = "k" + "".join(p.capitalize() for p in code[3:].lower().split("_"))
    text = e["text"].encode()
    L = [f"inline void {name}(flightrec::Header& h, flightrec::Event& e) {{"]
    L += emit_header(h, "kEvent")
    L += [f"    e.code = flightrec::Ev::{cname};", f"    e.src = {e['src']};",
          f"    e.a = {e['a']};", f"    e.b = {e['b']};",
          '    flightrec::setText(e, "%s");' % "".join(f"\\x{x:02x}" for x in text),
          "}"]
    L.append(f"inline const uint8_t {name}_wire[{len(wire)}] = {{{cbytes(wire)}}};")
    return L


def emit_hello(name, h, m, wire):
    L = [f"inline void {name}(flightrec::Header& h, flightrec::Hello& m) {{"]
    L += emit_header(h, "kHello")
    L.append(f'    const char fw[] = "{m["fw"]}";')
    L.append("    for (size_t i = 0; i < sizeof fw; ++i) m.fw[i] = fw[i];")
    for k in ("utc_synced", "enabled", "idle_hz", "cal_ok"):
        L.append(f"    m.{k} = {m[k]};")
    for k in F.HELLO_COUNTERS:
        L.append(f"    m.{k} = {m[k]}u;")
    for b in range(F.MAX_JOINTS):
        L.append(f"    m.id[{b}] = {m['id'][b]}; m.fitted[{b}] = {m['fitted'][b]}; "
                 f"m.dir[{b}] = {m['dir'][b]}; m.zero[{b}] = {m['zero'][b]};")
    L.append("}")
    L.append(f"inline const uint8_t {name}_wire[{len(wire)}] = {{{cbytes(wire)}}};")
    return L


def main():
    out = ["// GENERATED by tools/gen_flightrec_vectors.py -- do not edit.",
           "// Source of truth: tools/flightrec.py (what the collector decodes with).",
           "#pragma once", "#include <stddef.h>", "#include <stdint.h>", "",
           '#include "flightrec/record.h"', "", "namespace flightrec_vectors {", ""]
    cases = [("tick12_bal", 12, F.SRC_BENCH_POSE, True),
             ("tick17_ctrl", 17, F.SRC_CTRL, False),
             ("tick0_idle", 0, F.SRC_BENCH_IDLE, False)]
    for name, n, src, bal in cases:
        h, t = tick_case(n, src, bal)
        wire = F.encode_tick(h, t)
        assert F.decode(wire)["joints"] == [{**j} for j in t["joints"]]
        out += emit_tick(name, h, t, wire) + [""]
    for name, args in (("ev_cmd", (F.EV_CMD, "pose 2082 1717 2685 1765 s3000 BL", 0, 0)),
                       ("ev_bal", (F.EV_BAL_STATE, "", 4, 1)),
                       ("ev_out", (F.EV_OUT, "balance ABORT (tip): roll err -12.41 deg\r\n", -7, 300))):
        h, e = event_case(*args)
        out += emit_event(name, h, e, F.encode_event(h, e)) + [""]
    h, m = hello_case()
    out += emit_hello("hello", h, m, F.encode_hello(h, m)) + [""]
    out.append("}  // namespace flightrec_vectors")
    text = "\n".join(out) + "\n"
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as f:
        f.write(text)
    print(f"wrote {os.path.relpath(OUT, ROOT)}")


if __name__ == "__main__":
    main()
