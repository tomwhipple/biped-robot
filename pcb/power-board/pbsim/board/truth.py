"""The exhaustive steady-state truth table: every combination of the board's
discrete inputs, each simulated on the whole board, each output compared with
what the design says it should be.

Inputs (and their levels):

- board state before the observation: never switched on, or switched on by a
  50 ms POWER tap at 0.05 s;
- E-STOP contact A (pins 1–2) closed/open, contact B (pins 3–4) closed/open
  (both open = J11 unplugged or a broken lead);
- SERVO_EN driven low, driven high, or high-impedance (ESP32 not driving);
- POWER held from 1.2 s to the end, or released;
- the ESP32 on USB, or not;
- the General Driver on J3 with SW1 on, with SW1 off, or J3 unplugged;
- the pack: XT60 and balance lead both in, XT60 only, balance only, neither.

Expected outputs, from design record §5, §7 and harness.md:

- PACK+ / LOGIC_OUT (J3): up iff the BQ76922 is powered (XT60 in), the
  board was switched on (tap, or the POWER hold), and the balance lead is in
  (with it out the BQ76922 cannot see the cells and must not close Q2);
- +3V3_HOST: up iff the General Driver is fed (PACK+ up, J3 in, SW1 on) or
  USB powers the ESP32;
- SERVO+ (J4–J7) / RAIL_V: up iff PACK+ is up, contact A is closed, and a
  powered ESP32 drives SERVO_EN high; RAIL_V = SERVO+ × R39/(R38+R39);
- PWR_INT_N: low iff POWER is held, high otherwise (when +3V3_HOST is up);
- ESTOP_N: high iff contact B is open (when +3V3_HOST is up);
- SERVO_FLT_N: high (no fault) when +3V3_HOST is up;
- SERVO_IMON: (sense voltage) × 0.9 × R36/R25 for the rail's current;
- no signal back-powers an unpowered ESP32 (+3V3_HOST < 0.3 V when unfed);
- no absolute-maximum rating is exceeded for longer than 20 µs.
"""
from __future__ import annotations

import itertools
from dataclasses import dataclass

from .board import Board, Esp32, Scenario, steps
from .stress import describe, floating

T_STOP = 2.4
OBS = (2.30, 2.40)

STATES = ("off", "on")
CONTACT = ("closed", "open")
SERVO_EN = ("low", "high", "hiz")
POWER_HELD = (False, True)
USB = (False, True)
GD = ("on", "sw1_off", "absent")
PACK = ("both", "xt60_only", "balance_only", "none")


@dataclass(frozen=True)
class Case:
    state: str
    estop_a: str
    estop_b: str
    servo_en: str
    power_held: bool
    usb: bool
    gd: str
    pack: str

    @property
    def name(self) -> str:
        return (f"{self.state}-A{self.estop_a[0]}-B{self.estop_b[0]}-"
                f"en_{self.servo_en}-pwr{'H' if self.power_held else 'R'}-"
                f"usb{int(self.usb)}-gd_{self.gd}-pack_{self.pack}")


def all_cases() -> list[Case]:
    return [Case(*c) for c in itertools.product(
        STATES, CONTACT, CONTACT, SERVO_EN, POWER_HELD, USB, GD, PACK)]


def scenario(c: Case) -> Scenario:
    btn = [(0.0, 0.0)]
    if c.state == "on":
        btn += [(0.05, 1.0), (0.10, 0.0)]
    if c.power_held:
        btn += [(1.2, 1.0)]
    xt = 1.0 if c.pack in ("both", "xt60_only") else 0.0
    bal = 1.0 if c.pack in ("both", "balance_only") else 0.0
    en = {"low": steps((0, 0)), "high": steps((0, 1)), "hiz": steps((0, 0))}
    hiz = steps((0, 1)) if c.servo_en == "hiz" else []
    return Scenario(
        name=c.name, t_stop=T_STOP, tmax=50e-6,
        xt60=steps((0, xt)), balance=steps((0, bal)),
        power_btn=btn,
        estop_a=steps((0, 1.0 if c.estop_a == "closed" else 0.0)),
        estop_b=steps((0, 1.0 if c.estop_b == "closed" else 0.0)),
        usb=steps((0, 1.0 if c.usb else 0.0)),
        gd_present=c.gd != "absent", gd_sw1=c.gd == "on",
        esp=Esp32(servo_en=en[c.servo_en], servo_en_hiz=hiz),
        record_every=4)


def expected(c: Case) -> dict:
    powered = c.pack in ("both", "xt60_only")
    switched_on = c.state == "on" or c.power_held
    pack_on = powered and switched_on and c.pack == "both"
    rail3 = (pack_on and c.gd == "on") or c.usb
    servo = pack_on and c.estop_a == "closed" and c.servo_en == "high" and rail3
    return {
        "pack_on": pack_on,
        "v3v3_on": rail3,
        "servo_on": servo,
        "pwr_int_n": (None if not rail3 else (not c.power_held)),
        "estop_n": (None if not rail3 else (c.estop_b == "open")),
        "flt_n": (None if not rail3 else True),
    }


def measure(r, b: Board) -> dict:
    w = lambda k: float(r.window(k, *OBS).mean())  # noqa: E731
    return {k: w(k) for k in ("pack", "logic", "servo", "railv", "pwr_int_n",
                              "estop_n", "flt_n", "imon", "v3v3", "en", "inp",
                              "ts2", "ld", "rst", "q2_vgs", "q3_vgs", "bst",
                              "u1_mode")}


def judge(c: Case, m: dict, b: Board) -> list[str]:
    """Failed checks (empty = the case passes)."""
    e = expected(c)
    bad = []
    vpack = 3 * 3.8

    def on(x, name, want):
        if want and x < 0.9 * vpack:
            bad.append(f"{name} should be up, is {x:.2f} V")
        if not want and x > 0.5:
            bad.append(f"{name} should be off, is {x:.2f} V")

    on(m["pack"], "PACK+", e["pack_on"])
    on(m["logic"], "LOGIC_OUT (J3)", e["pack_on"])
    on(m["servo"], "SERVO+", e["servo_on"])
    ratio = 10e3 / 57e3
    if abs(m["railv"] - m["servo"] * ratio) > 0.05:
        bad.append(f"RAIL_V {m['railv']:.3f} ≠ SERVO+ × 10/57")
    if e["v3v3_on"] and m["v3v3"] < 3.2:
        bad.append(f"+3V3_HOST should be up, is {m['v3v3']:.2f}")
    if not e["v3v3_on"] and m["v3v3"] > 0.3:
        bad.append(f"+3V3_HOST back-powered to {m['v3v3']:.2f} V")
    vil, vih = 0.25 * 3.3, 0.75 * 3.3        # ESP32 input thresholds
    for key, name in (("pwr_int_n", "PWR_INT_N"), ("estop_n", "ESTOP_N"),
                      ("flt_n", "SERVO_FLT_N")):
        want = e[key]
        if want is None:
            if m[key] > 0.3:
                bad.append(f"{name} {m[key]:.2f} V into an unpowered ESP32")
            continue
        if want and m[key] < vih:
            bad.append(f"{name} should read high, is {m[key]:.2f} V")
        if not want and m[key] > vil:
            bad.append(f"{name} should read low, is {m[key]:.2f} V")
    for v in judged_violations(b, xt60_out=c.pack not in ("both", "xt60_only")):
        bad.append(describe(v))
    return bad


def judged_violations(b: Board, xt60_out: bool) -> list[dict]:
    return [v for v in b.mon.violations(min_s=20e-6)
            if not floating(v, xt60_out)]


def run_case(c: Case) -> dict:
    b = Board(scenario(c))
    r = b.run()
    m = measure(r, b)
    bad = judge(c, m, b)
    out = c.pack not in ("both", "xt60_only")
    return {"case": c.__dict__, "name": c.name, "expected": expected(c),
            "measured": m, "fail": bad,
            "floating": [v for v in b.mon.violations() if floating(v, out)],
            "transients": [v for v in b.mon.violations()
                           if v["outside_s"] < 20e-6 and not floating(v, out)],
            "events": [(t, e) for t, e in r.events][:60]}
