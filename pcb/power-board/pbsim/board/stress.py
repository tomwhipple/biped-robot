"""Absolute-maximum ratings, checked at every accepted time point.

Each check is a voltage the netlist defines (a pin to its reference, a FET's
gate to source, a capacitor's two ends) with the limits its datasheet gives.
A run records each check's extremes; a scenario passes only if none is
exceeded. Sources: BQ76922 DS 6.1, TPS4811-Q1 TPS 6.1, CSD17556Q5B §5.1
(VDS 30 V, VGS ±20 V), Vishay 2N7002K (VDS 60 V, VGS ±20 V), the ESP32
datasheet (GPIO −0.3 V to 3.6 V; above VDD, the input clamp's current, 1 mA),
and each capacitor's rated voltage
from its MPN.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Callable

from .netlist import Netlist, node_name, si


ESD_IS, ESD_VT = 1e-15, 0.02585      # the ESP32 input clamp as board.py models it (d_esd)


@dataclass
class Check:
    name: str
    f: Callable[[Callable[[str], float]], float]
    lo: float
    hi: float
    source: str


def _cap_rating(value: str, mpn: str) -> float | None:
    m = re.search(r"(\d+(?:\.\d+)?)V", value)
    if m:
        return float(m.group(1))
    m = re.match(r"CC0[46]03[A-Z]{2}[A-Z0-9]{3}([5-90])", mpn.split()[-1])
    if m:
        return {"5": 6.3, "6": 10, "7": 16, "8": 25, "9": 50, "0": 100}[m.group(1)]
    if mpn.endswith("CL31B106KLHNNNE"):
        return 25.0
    return None


def checks(nl: Netlist) -> list[Check]:
    n = {net: node_name(net) for net in nl.nets}
    u1 = {int(k): n[v] for k, v in nl.parts["U1"].pins.items()}
    u2 = {int(k): n[v] for k, v in nl.parts["U2"].pins.items()}
    C: list[Check] = []

    def pin(node):
        return lambda g: g(node)

    def diff(a, b):
        return lambda g: g(a) - g(b)

    vss = u1[10]
    bq = "BQ76922 DS 6.1"
    for p, lo, hi, nm in ((1, -0.3, 60, "BAT"), (29, -0.3, 60, "PACK"),
                          (28, -0.3, 60, "LD"), (30, -0.3, 60, "DSG"),
                          (31, -0.3, 60, "CHG"), (21, -0.3, 6, "RST_SHUT"),
                          (17, -0.3, 6, "SCL"), (18, -0.3, 6, "SDA"),
                          (14, -0.3, 6, "TS2 (wake pin)"), (9, -0.3, 6, "VC0"),
                          (15, -0.3, 2, "REG18"), (11, -0.3, 2.1, "SRP"),
                          (12, -0.3, 2.1, "SRN"), (13, -0.3, 2.1, "TS1"),
                          (2, -0.3, 60, "VC5"), (8, -0.3, 60, "VC1"),
                          (7, -0.3, 60, "VC2")):
        C.append(Check(f"U1 {nm}", diff(u1[p], vss), lo, hi, bq))
    # 3S: VC3 and VC4 are tied to VC2 (pin 7), so the top cell is VC5−VC2
    for a, b, nm in ((8, 9, "VC1−VC0"), (7, 8, "VC2−VC1"), (2, 7, "VC5−VC2")):
        C.append(Check(f"U1 {nm}", diff(u1[a], u1[b]), -0.3, 60, bq))
    C.append(Check("U1 CP1−BAT", diff(u1[32], u1[1]), -0.3, 15, bq))
    tp = "TPS4811-Q1 TPS 6.1"
    for p, lo, hi, nm in ((20, -1, 100, "VS"), (18, -1, 100, "CS+"),
                          (17, -1, 100, "CS−"), (19, -1, 100, "ISCP"),
                          (13, -30, 100, "SRC"), (9, -0.3, 5.5, "TMR"),
                          (8, -0.3, 5.5, "IWRN"), (10, -0.3, 5.5, "DIODE"),
                          (1, -1, 20, "EN/UVLO"), (3, -1, 20, "INP"),
                          (2, -1, 20, "INP_G"), (4, -1, 20, "FLT_T"),
                          (5, -1, 20, "FLT_I"), (7, -1, 7.5, "IMON"),
                          (12, -30, 112, "BST"), (15, -30, 112, "PU"),
                          (14, -30, 112, "PD")):
        C.append(Check(f"U2 {nm}", pin(u2[p]), lo, hi, tp))
    for p, nm in ((12, "BST"), (15, "PU"), (14, "PD"), (11, "G")):
        C.append(Check(f"U2 {nm}−SRC", diff(u2[p], u2[13]), -0.3, 16, tp))
    for p, nm in ((20, "VS"), (18, "CS+"), (17, "CS−")):
        C.append(Check(f"U2 {nm}−SRC", diff(u2[p], u2[13]), -60, 100, tp))
    C.append(Check("U2 CS+−CS−", diff(u2[18], u2[17]), -0.3, 0.3, tp))
    C.append(Check("U2 EN/UVLO (rec. ≤ 15 V)", pin(u2[1]), 0 - 1, 15, "TPS 6.2"))
    for ref in ("Q1", "Q2", "Q3"):
        pins = nl.parts[ref].pins
        s = n[pins["1"]]
        C.append(Check(f"{ref} VGS", diff(n[pins["4"]], s), -20, 20,
                       "CSD17556Q5B §5.1"))
        C.append(Check(f"{ref} VDS", diff(n[pins["5"]], s), -30, 30,
                       "CSD17556Q5B §5.1 (negative = body diode)"))
    pins = nl.parts["Q4"].pins
    C.append(Check("Q4 VGS", diff(n[pins["1"]], n[pins["2"]]), -20, 20,
                   "Vishay 2N7002K"))
    C.append(Check("Q4 VDS", diff(n[pins["3"]], n[pins["2"]]), -60, 60,
                   "Vishay 2N7002K"))
    j9 = nl.parts["J9"].pins
    j12 = nl.parts["J12"].pins
    rail = n[j9["1"]]
    # Above the ESP32's VDD the question is the current its input clamp
    # conducts, as for RAIL_V below: when +3V3_HOST collapses, a pin held up by
    # its pull-up and the lead's capacitance lags it (with rev B's R10 of
    # 100 kΩ, PWR_INT_N by about 0.6 V for 8–21 µs). The current is the model's
    # clamp (board.py d_esd) at that voltage; 1 mA taken as the limit.
    for pn, nm in (("1", "KILL"), ("2", "SERVO_EN"), ("4", "PWR_INT_N"),
                   ("6", "ESTOP_N")):
        node = n[j12[pn]]
        C.append(Check(f"ESP32 {nm} clamp current above VDD (A)",
                       lambda g, a=node, b=rail: ESD_IS * math.expm1(
                           min(max(g(a) - max(g(b), 0.0), 0.0), 1.0) / ESD_VT),
                       -1.0, 1e-3, "1 mA injection taken as the limit"))
        C.append(Check(f"ESP32 {nm}", pin(node), -0.3, 3.6, "ESP32 datasheet"))
    # RAIL_V comes from SERVO+ through R38, so above the ESP32's VDD the real
    # question is the clamp current it injects (amps, 1 mA taken as the limit:
    # Espressif publishes none)
    r38, r39 = si(nl.parts["R38"].value), si(nl.parts["R39"].value)
    srv, rv = n["SERVO+"], n[j12["5"]]
    C.append(Check("ESP32 GPIO34 clamp current from RAIL_V (A)",
                   lambda g: max(0.0, (g(srv) - g(rv)) / r38 - g(rv) / r39)
                   if g(rv) > max(g(rail), 0.0) + 0.3 else 0.0,
                   -1.0, 1e-3, "1 mA injection taken as the limit"))
    C.append(Check("ESP32 RAIL_V (powered)", lambda g: g(rv) if g(rail) > 3.0
                   else 0.0, -0.3, 3.6, "ESP32 datasheet"))
    for ref, part in nl.parts.items():
        if not ref.startswith("C"):
            continue
        r = _cap_rating(part.value, part.mpn)
        if r is None:
            continue
        a, b = n[part.pins["1"]], n[part.pins["2"]]
        lo = -r if ref != "C21" else -1.0     # electrolytic: polarity
        C.append(Check(f"{ref} ({part.value})", diff(a, b), lo, r,
                       f"{part.mpn} rating"))
    for ref in ("D8", "D9"):
        pins = nl.parts[ref].pins
        C.append(Check(f"{ref} below VRWM", diff(n[pins["1"]], n[pins["2"]]),
                       -1.5, 15.0, "SMBJ15A VRWM 15 V"))
    return C


def floating(v: dict, xt60_out: bool) -> bool:
    """With the XT60 out, U1's VSS has no DC path to the cells, and its cell
    inputs settle wherever the solver puts the floating island: VC0 − VSS is
    −0.13 V on ngspice 47 and −1.45 V on ngspice 42, at nanoamps. No model of
    the IC's internals exists to say where silicon would sit, so these checks
    are reported as floating, not judged; TI's answer is the PCB tie of
    simulation record F-9."""
    return xt60_out and v["check"].startswith("U1 VC")


def describe(v: dict) -> str:
    """A violation in words: the measured range against the limits."""
    lo, hi = v["limit"]
    return (f"abs max: {v['check']} ranged {v['lo']:.2f} to {v['hi']:.2f} "
            f"against limits {lo:g} to {hi:g}, outside for "
            f"{v['outside_s'] * 1e3:.3f} ms")


class Monitor:
    """Extremes of every check, and how long each spent outside its limits
    (a microsecond capacitive kick and a sustained overstress are different
    findings)."""

    def __init__(self, nl: Netlist):
        self.checks = checks(nl)
        k = len(self.checks)
        self.lo = [float("inf")] * k
        self.hi = [float("-inf")] * k
        self.t_lo = [0.0] * k
        self.t_hi = [0.0] * k
        self.out_s = [0.0] * k
        self._t = 0.0

    def step(self, t: float, v: dict[str, float]):
        g = lambda k: v.get(k, 0.0)  # noqa: E731
        dt = max(t - self._t, 0.0)
        self._t = t
        for i, c in enumerate(self.checks):
            x = c.f(g)
            if x < self.lo[i]:
                self.lo[i], self.t_lo[i] = x, t
            if x > self.hi[i]:
                self.hi[i], self.t_hi[i] = x, t
            if x < c.lo - 1e-6 or x > c.hi + 1e-6:
                self.out_s[i] += dt

    def violations(self, min_s: float = 0.0) -> list[dict]:
        out = []
        for i, c in enumerate(self.checks):
            if (self.lo[i] < c.lo - 1e-6 or self.hi[i] > c.hi + 1e-6) and \
                    self.out_s[i] >= min_s:
                out.append({"check": c.name, "lo": self.lo[i], "hi": self.hi[i],
                            "limit": (c.lo, c.hi), "t_hi": self.t_hi[i],
                            "t_lo": self.t_lo[i], "outside_s": self.out_s[i],
                            "source": c.source})
        return out

    def table(self) -> list[dict]:
        return [{"check": c.name, "lo": self.lo[i], "hi": self.hi[i],
                 "limit": (c.lo, c.hi), "source": c.source}
                for i, c in enumerate(self.checks)]
