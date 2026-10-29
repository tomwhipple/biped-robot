"""The whole board in ngspice, built from the schematic's netlist, plus the
world around it (the pack, the buttons, the ESP32's pins, the General
Driver's input, the four servo branches), driven by one ``Scenario``.

Every part on the schematic becomes a SPICE element from its value and MPN
(``devices.py``); U1 and U2 are the pin models in ``ics.py``. Nothing about
the board's connections is typed by hand: it comes from ``netlist.load()``.
The world attaches to connector pins (J1–J12), so it follows the connectors
wherever the schematic routes them.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Callable

import numpy as np

from . import devices
from .engine import Engine
from .ics import (BqLimits, BqOtp, Bq76922, Ctl, TpsLimits, Tps48111)
from .netlist import Netlist, load, node_name, si
from .stress import Monitor

# ---------------------------------------------------------------------------
# scenario description
# ---------------------------------------------------------------------------

Sched = list[tuple[float, float]]   # (time, value) steps; value holds until next


def steps(*pairs) -> Sched:
    return [(float(t), float(v)) for t, v in pairs]


@dataclass
class Esp32:
    """The ESP32's side of J12. ``kill``/``servo_en`` are scheduled output
    levels (1 = drive high, 0 = drive low, None = high-impedance) applied
    once the firmware is up; ``firmware`` may override them per step."""
    kill: Sched = field(default_factory=lambda: [(0.0, 0.0)])
    servo_en: Sched = field(default_factory=lambda: [(0.0, 0.0)])
    kill_hiz: Sched = field(default_factory=list)       # times outputs float
    servo_en_hiz: Sched = field(default_factory=list)
    boot_s: float = 0.30          # outputs high-impedance until boot + this
    firmware: Callable | None = None


@dataclass
class Scenario:
    name: str
    t_stop: float
    tmax: float = 50e-6
    temp_c: float = 25.0                 # board and pack temperature
    leak: str = "typ"                    # "typ" or "max" leakage corner
    cells: tuple[Sched, Sched, Sched] = (steps((0, 3.8)),) * 3
    r_cell: float = 5e-3                 # per cell, ohms
    r_lead: float = 2e-3                 # each discharge-lead conductor
    l_lead: float = 100e-9
    xt60: Sched = field(default_factory=lambda: steps((0, 1)))
    balance: Sched = field(default_factory=lambda: steps((0, 1)))
    balance_map: tuple[int, int, int, int] = (0, 1, 2, 3)  # J2 pin k ← tap
    power_btn: Sched = field(default_factory=lambda: steps((0, 0)))
    estop_a: Sched = field(default_factory=lambda: steps((0, 1)))  # 1 = closed
    estop_b: Sched = field(default_factory=lambda: steps((0, 1)))
    usb: Sched = field(default_factory=lambda: steps((0, 0)))  # ESP32 on USB
    gd_present: bool = True              # General Driver on J3
    gd_sw1: bool = True
    gd_load_w: float = 9.6               # design record §3 upper model
    esp: Esp32 = field(default_factory=Esp32)
    branch_a: dict[int, Sched] = field(default_factory=dict)  # J4..J7 amps
    branch_c: dict[int, float] = field(default_factory=lambda: {
        4: 706e-6, 5: 353e-6, 6: 706e-6, 7: 235e-6})
    branch_n: dict[int, int] = field(default_factory=lambda: {
        4: 6, 5: 3, 6: 6, 7: 2})
    shorts: dict[str, Sched] = field(default_factory=dict)  # place -> 0/1
    otp: BqOtp = field(default_factory=BqOtp)
    bq: BqLimits = field(default_factory=BqLimits)
    tps: TpsLimits = field(default_factory=TpsLimits)
    cap_derate: dict[str, float] = field(default_factory=dict)
    diode_sense: bool = False
    q5_temp_c: float | None = None
    rt1_temp_c: float | None = None
    record_every: int = 1
    # (t0, t1, max step): windows simulated with a short step (shorts, trips)
    fine: list[tuple[float, float, float]] = field(default_factory=list)
    # design what-ifs: part value overrides, D2 swapped for a silicon pair,
    # and extra SPICE lines built from the board (callables of the Board)
    patch: dict[str, str] = field(default_factory=dict)
    d2_silicon: bool = False
    extra: list[Callable] = field(default_factory=list)
    # servo electronics modelled as this resistance per servo below 5 V
    servo_lowv_ohms: float | None = None
    r_cells: tuple[float, float, float] | None = None   # overrides r_cell
    fine_on_dsg: float = 0.0    # fine-step window after every Q2 turn-on
    gd_iq_off_a: float = 0.0    # General Driver input current below UVLO


SHORT_PLACES = {
    # place: (node lookup, series R, series L)
    "PACK+": ("PACK+", 10e-3, 100e-9),
    "SERVO+": ("SERVO+", 10e-3, 100e-9),
    "J3": ("gd_vin", 10e-3, 400e-9),      # end of the logic lead
    "J4": ("br4", 10e-3, 100e-9),
    "J5": ("br5", 10e-3, 100e-9),
    "J6": ("br6", 10e-3, 100e-9),
    "J7": ("br7", 10e-3, 100e-9),
}

FUSE = {  # cold resistance (record C13), melting I²t (A²s), rating (A)
    "F1": (4.58e-3, 135.0, 15.0),
    "F2": (33.75e-3, 9.4, 3.0),
}


def pwl(s: Sched, tr: float = 1e-6, scale: float = 1.0) -> str:
    pts = []
    last = None
    for t, v in s:
        v *= scale
        if last is None:
            pts.append((0.0, v if t <= 0 else 0.0))
            if t > 0:
                pts.append((t, 0.0))
                pts.append((t + tr, v))
        else:
            pts.append((t, last))
            pts.append((t + tr, v))
        last = v
    pts.sort(key=lambda p: p[0])
    clean = []
    for t, v in pts:
        if clean and t <= clean[-1][0]:
            t = clean[-1][0] + 1e-12
        clean.append((t, v))
    return "pwl(" + " ".join(f"{t:.9g} {v:.9g}" for t, v in clean) + ")"


def sched_at(s: Sched, t: float, default: float | None = 0.0):
    v = default
    for ts, val in s:
        if ts <= t + 1e-15:
            v = val
    return v


# ---------------------------------------------------------------------------
# the board
# ---------------------------------------------------------------------------

class Board:
    """Builds the deck for a scenario and acts as the engine's controller."""

    def __init__(self, sc: Scenario, nl: Netlist | None = None):
        self.sc = sc
        self.nl = nl or load()
        self.n = {net: node_name(net) for net in self.nl.nets}
        u1 = self.nl.parts["U1"]
        u2 = self.nl.parts["U2"]
        self.u1 = Bq76922({int(k): self.n[v] for k, v in u1.pins.items()},
                          sc.otp, sc.bq, si(self.nl.parts["R1"].value))
        self.u2 = Tps48111({int(k): self.n[v] for k, v in u2.pins.items()},
                           sc.tps, diode_sense=sc.diode_sense,
                           r_set=si(self.nl.parts["R25"].value))
        self.ctl: dict[str, Ctl] = {
            "v_gd_buck": Ctl(0.0, 1e-3), "v_gd_3v3": Ctl(0.0, 0.5e-3),
            "v_esp_kill_en": Ctl(0.0), "v_esp_kill_lvl": Ctl(0.0),
            "v_esp_sen_en": Ctl(0.0), "v_esp_sen_lvl": Ctl(0.0),
            "v_f1": Ctl(1.0, 10e-6), "v_f2": Ctl(1.0, 10e-6)}
        self.buck_on = False
        self.rail3v3_since: float | None = None
        self.fuse_i2t = {"F1": 0.0, "F2": 0.0}
        self.fuse_open: dict[str, float] = {}
        self.events: list[tuple[float, str]] = []
        self.rec: dict[str, list[float]] = {}
        self.vecs: list[str] = []
        self.sched_times = sorted({t for s in self._all_scheds() for t, _ in s
                                   if t > 0} | {sc.esp.boot_s}
                                  | {w[0] for w in sc.fine})
        self._step_n = 0
        self.last_t = 0.0
        self.mon = Monitor(self.nl)

    # -- helpers ------------------------------------------------------------
    def node(self, ref: str, pin) -> str:
        return self.n[self.nl.net(ref, pin)]

    def net(self, name: str) -> str:
        return self.n[name]

    def _all_scheds(self):
        sc = self.sc
        yield from sc.cells
        yield sc.xt60
        yield sc.balance
        yield sc.power_btn
        yield sc.estop_a
        yield sc.estop_b
        yield sc.usb
        yield sc.esp.kill
        yield sc.esp.servo_en
        yield sc.esp.kill_hiz
        yield sc.esp.servo_en_hiz
        yield from sc.branch_a.values()
        yield from sc.shorts.values()

    # -- deck -----------------------------------------------------------------
    def deck(self) -> str:
        sc = self.sc
        L = [f"* power board rev A, scenario {sc.name}",
             f".options rshunt=1e12 temp={sc.temp_c} tnom=27 reltol=1e-3 "
             "abstol=1e-10 vntol=1e-6 itl4=60 method=gear",
             devices.model_block(),
             ".model d_esd d (is=1e-15 n=1 rs=10 bv=100 cjo=2p)"]
        L += self._parts()
        L += self.u1.spice()
        L += self.u2.spice()
        L += self._world()
        for f in sc.extra:
            L += f(self)
        for name in self.ctl:
            L.append(f"{name} x_{name[2:]} 0 dc 0 external")
        self.vecs = self._save_list()
        L.append(".save " + " ".join(self.vecs))
        t_end = sc.t_stop * 1.001 + 1e-5      # see Engine.run(t_valid=)
        L.append(f".tran {min(sc.tmax, 1e-5):.3g} {t_end:.9g} 0 {sc.tmax:.3g}")
        L.append(".end")
        return "\n".join(L) + "\n"

    def _parts(self) -> list[str]:
        sc, L = self.sc, []
        am = {"Q2": "q2_s", "Q3": "q3_s"}     # ammeters on the FET sources
        for ref, part in sorted(self.nl.parts.items()):
            kind = ref.rstrip("0123456789")
            pins = {k: self.n[v] for k, v in part.pins.items()}
            if ref in sc.patch:
                part = replace(part, value=sc.patch[ref])
            if kind in ("J", "TP", "U"):
                continue
            if ref == "D2" and sc.d2_silicon:
                # what-if: a silicon common-cathode pair with 1N4148W's data
                L += devices.diode_lines("D2a", pins["1"], pins["3"], "1N4148W",
                                         sc.temp_c, sc.leak)
                L += devices.diode_lines("D2b", pins["2"], pins["3"], "1N4148W",
                                         sc.temp_c, sc.leak)
                continue
            if kind == "R":       # a 0 R jumper as 0.1 mΩ: ngspice takes no zero resistor
                L.append(f"r{ref} {pins['1']} {pins['2']} {max(si(part.value), 1e-4):.6g}")
            elif kind == "RT":
                t = sc.rt1_temp_c if sc.rt1_temp_c is not None else sc.temp_c
                L.append(f"r{ref} {pins['1']} {pins['2']} "
                         f"{devices.ntc_ohms(t):.6g}")
            elif kind == "C":
                c = si(part.value) * sc.cap_derate.get(ref, 1.0)
                if ref == "C21":   # aluminium electrolytic: ESR
                    L.append(f"c{ref} {pins['1']} c21_esr {c:.6g}")
                    L.append(f"r{ref}_esr c21_esr {pins['2']} 0.025")
                else:
                    L.append(f"c{ref} {pins['1']} {pins['2']} {c:.6g}")
            elif kind == "F":
                r = FUSE[ref][0]
                ctl = f"x_{ref.lower()}"
                L.append(f"b{ref} {pins['1']} {pins['2']} i=v({pins['1']},"
                         f"{pins['2']})*v({ctl})/{r}")
                L.append(f"r{ref}_open {pins['1']} {pins['2']} 1e9")
            elif kind == "D":
                mpn = {"BAT54C": "BAT54C", "BAV70": "BAV70", "PMEG4002EJ": "PMEG4002EJ",
                       "16V": "MMSZ5246B", "15V": "MMSZ5245B", "1N4148W": "1N4148W",
                       "SMBJ15A": "SMBJ15A"}[part.value]
                if mpn in ("BAT54C", "BAV70"):   # KiCad symbols: 1, 2 anodes; 3 cathode
                    L += devices.diode_lines(f"{ref}a", pins["1"], pins["3"],
                                             mpn, sc.temp_c, sc.leak)
                    L += devices.diode_lines(f"{ref}b", pins["2"], pins["3"],
                                             mpn, sc.temp_c, sc.leak)
                else:
                    a = pins["2"]
                    k = pins["1"]
                    L += devices.diode_lines(ref, a, k, mpn, sc.temp_c, sc.leak)
            elif kind == "Q":
                if part.value == "CSD17556Q5B":
                    s = pins["1"]
                    assert pins["2"] == s == pins["3"], ref
                    if ref in am:
                        L.append(f"v_am_{ref.lower()} {am[ref]} {s} dc 0")
                        s = am[ref]
                    L += devices.fet_lines(ref, pins["5"], pins["4"], s,
                                           sc.temp_c, sc.leak)
                elif part.value == "2N7002":
                    g, s, d = pins["1"], pins["2"], pins["3"]
                    L.append(f"m{ref} {d} {g} {s} {s} n_2n7002k w=1 l=1")
                    L.append(f"d{ref}_body {s} {d} d_2n7002k_body")
                elif part.value == "MMBT3904":
                    b, e, c = pins["1"], pins["2"], pins["3"]
                    tq = sc.q5_temp_c if sc.q5_temp_c is not None else sc.temp_c
                    L.append(f"q{ref} {c} {b} {e} q_mmbt3904 temp={tq}")
                else:
                    raise ValueError(f"no model for {ref} {part.value}")
            else:
                raise ValueError(f"no model for {ref} ({part.value})")
        return L

    def _world(self) -> list[str]:
        sc, nd = self.sc, self.node
        L = ["* ---- world: pack, leads, buttons, ESP32, General Driver, servos"]
        # pack: three cells with internal resistance; taps tp0..tp3
        L += ["r_pk_ref tp0 0 1e9"]
        for k in range(3):
            L.append(f"v_cell{k+1} tp{k}_c{k+1} tp{k} {pwl(sc.cells[k], 1e-4)}")
            rc = sc.r_cells[k] if sc.r_cells else sc.r_cell
            L.append(f"r_cell{k+1} tp{k}_c{k+1} tp{k+1} {rc:.6g}")
        # discharge lead and XT60 (J1.1 BAT−, J1.2 BAT+)
        L.append(f"v_xt60 x_xt60 0 {pwl(sc.xt60, 1e-6)}")
        for tap, pin, nm in ((0, 1, "n"), (3, 2, "p")):
            L.append(f"r_lead_{nm} tp{tap} lead_{nm}a {sc.r_lead:.6g}")
            L.append(f"l_lead_{nm} lead_{nm}a lead_{nm} {sc.l_lead:.6g}")
            L.append(f"b_xt60_{nm} lead_{nm} {nd('J1', pin)} i=v(lead_{nm},"
                     f"{nd('J1', pin)})*(1e-12+v(x_xt60)*1000)")
        # balance lead: J2 pin k+1 ← tap balance_map[k], 50 mΩ of thin wire
        L.append(f"v_bal x_bal 0 {pwl(sc.balance, 1e-6)}")
        for k, tap in enumerate(sc.balance_map):
            L.append(f"b_bal{k+1} tp{tap} {nd('J2', k + 1)} i=v(tp{tap},"
                     f"{nd('J2', k + 1)})*(1e-12+v(x_bal)*20)")
        # POWER (J10) and E-STOP (J11: A 1–2, B 3–4); 1 = contact closed
        for nm, s, a, b in (("pwr", sc.power_btn, ("J10", 1), ("J10", 2)),
                            ("esa", sc.estop_a, ("J11", 1), ("J11", 2)),
                            ("esb", sc.estop_b, ("J11", 3), ("J11", 4))):
            L.append(f"v_{nm} x_{nm} 0 {pwl(s, 1e-6)}")
            L.append(f"b_{nm} {nd(*a)} {nd(*b)} i=v({nd(*a)},{nd(*b)})*"
                     f"(1e-12+v(x_{nm})*20)")
        # ESP32 / General Driver 3.3 V rail on J9.1, the ESP32's pins on J12
        r3 = nd("J9", 1)
        L.append(f"b_gd3v3 {r3} 0 i=v(x_gd_3v3)*(v({r3})-3.3)/0.2")
        L.append(f"r_gd3v3_load {r3} 0 1k")
        L.append(f"v_usb x_usb 0 {pwl(sc.usb)}")
        for nm, pin in (("kill", 1), ("sen", 2)):
            n = nd("J12", pin)
            L.append(f"b_esp_{nm} {n} 0 i=v(x_esp_{nm}_en)*(v({n})-"
                     f"3.3*v(x_esp_{nm}_lvl))/30")
        for pin in (1, 2, 4, 5, 6):
            n = nd("J12", pin)
            L.append(f"d_esp_hi{pin} {n} {r3} d_esd")
            L.append(f"d_esp_lo{pin} 0 {n} d_esd")
        # harness capacitance: ~0.3 m of lead on every signal and button line
        # (≈ 30 pF) plus the ESP32 pin (≈ 5 pF)
        for ref, pins in (("J12", (1, 2, 4, 5, 6, 7, 8)), ("J9", (3, 4)),
                          ("J10", (1,)), ("J11", (1, 2, 3))):
            for pin in pins:
                L.append(f"c_hc_{ref}_{pin} {nd(ref, pin)} 0 35p")
        for pin in (3, 4):     # SDA, SCL: the General Driver's pull-ups
            n = nd("J9", pin)
            L.append(f"r_i2c_pu{pin} {n} {r3} 4.7k")
            L.append(f"d_esp_i2c{pin} {n} {r3} d_esd")
        # General Driver input on J3 (H1 → AO4407 → SW1 → R11 → MP8759)
        if sc.gd_present:
            j3 = nd("J3", 1)
            # 0.3 m of 22 AWG pair (record C13: 16 mΩ, ≈0.3 µH) + H1, the
            # AO4407, SW1 and R11 (≈24 mΩ)
            rin = 0.04 if sc.gd_sw1 else 1e9
            L.append(f"l_gd_in {j3} gd_l 0.3u")
            L.append(f"r_gd_in gd_l gd_vin {rin}")
            L.append("c_gd_in gd_vin gd_c 44u")
            L.append("r_gd_cesr gd_c 0 5m")
            # constant-power buck; draws nothing below its UVLO whatever the
            # control says, plus the below-UVLO quiescent current (unknown:
            # the MP8759 datasheet is not in the repository)
            L.append(f"b_gd_buck gd_vin 0 i=v(x_gd_buck)*{sc.gd_load_w / 0.9:.4g}/"
                     "max(v(gd_vin),3.5)*(0.5+0.5*tanh((v(gd_vin)-3.7)/0.1))")
            if sc.gd_iq_off_a:
                L.append(f"b_gd_iq gd_vin 0 i={sc.gd_iq_off_a:.4g}*"
                         "tanh(max(v(gd_vin),0)/0.3)")
        # servo branches: lead, bulk capacitance, electronics, motor current
        for j in (4, 5, 6, 7):
            vp = nd(f"J{j}", 2)
            if sc.branch_n.get(j, 0) == 0:     # nothing plugged into this header
                L.append(f"r_br{j} br{j} 0 1e9")
                continue
            L.append(f"r_br{j} {vp} br{j}a 0.03")
            L.append(f"l_br{j} br{j}a br{j} 200n")
            L.append(f"c_br{j} br{j} br{j}c {sc.branch_c[j]:.6g}")
            L.append(f"r_br{j}c br{j}c 0 0.05")
            L.append(f"b_br{j}e br{j} 0 i={0.03 * sc.branch_n[j]:.4g}*"
                     + f"(0.5+0.5*tanh((v(br{j})-5)/0.5))")
            if sc.servo_lowv_ohms:
                L.append(f"r_br{j}lowv br{j} 0 "
                         f"{sc.servo_lowv_ohms / sc.branch_n[j]:.6g}")
            s = sc.branch_a.get(j, steps((0, 0)))
            L.append(f"v_br{j}i x_br{j}i 0 {pwl(s, 50e-6)}")
            L.append(f"b_br{j}m br{j} 0 i=v(x_br{j}i)*tanh(max(v(br{j}),0)/2)")
            L.append(f"r_br{j}data {nd(f'J{j}', 1)} 0 100k")
        # deliberate shorts
        for place, s in sc.shorts.items():
            look, r, l = SHORT_PLACES[place]
            node = self.net(look) if look in self.nl.nets else look
            tag = place.replace("+", "p").lower()
            L.append(f"v_sh_{tag} x_sh_{tag} 0 {pwl(s, 0.2e-6)}")
            L.append(f"l_sh_{tag} {node} sh_{tag} {l:.6g}")
            L.append(f"b_sh_{tag} sh_{tag} 0 i=v(sh_{tag})*(1e-9+v(x_sh_{tag})/{r})")
        return L

    def _save_list(self) -> list[str]:
        want = {self.net(n) for n in self.nl.nets
                if not n.startswith("unconnected")}
        for ic in (self.u1, self.u2):
            want |= set(ic.p.values())
        for ref in ("Q1", "Q2", "Q3", "Q4"):
            for pin in self.nl.parts[ref].pins:
                want.add(self.node(ref, pin))
        want |= {self.node("J3", 1), "u2_iscp_i", "tp0", "tp1", "tp2", "tp3"}
        want |= {self.node("F1", 2), self.node("F2", 2)}
        want |= {f"br{j}" for j in (4, 5, 6, 7)}
        if self.sc.gd_present:
            want.add("gd_vin")
        want.discard("0")
        return sorted(want) + ["v_am_q2#branch", "v_am_q3#branch"]

    # -- controller interface ------------------------------------------------
    def source(self, name: str, t: float) -> float:
        if name.startswith("v_u1_"):
            return self.u1.source(name, t)
        if name.startswith("v_u2_"):
            return self.u2.source(name, t)
        return self.ctl[name].at(t)

    def next_event(self, t: float) -> float:
        te = min(self.u1.next_event(t), self.u2.next_event(t))
        for c in self.ctl.values():
            if c.t0 + c.tr > t:
                te = min(te, c.t0 if c.t0 > t else c.t0 + c.tr)
        for ts in self.sched_times:
            if ts > t + 1e-12:
                te = min(te, ts)
                break
        for t0, t1, dt in self.sc.fine:
            if t0 - 1e-12 <= t < t1:
                te = min(te, t + dt)
        return te

    def step(self, t: float, v: dict[str, float]):
        dt = t - self.last_t
        self.last_t = t
        self.v = v
        was = self.u1.ctl["dsg"].v1 > 0.5
        self.u1.step(t, v)
        if self.sc.fine_on_dsg and not was and self.u1.ctl["dsg"].v1 > 0.5:
            self.sc.fine.append((t, t + self.sc.fine_on_dsg, 20e-9))
        self.u2.step(t, v)
        self._general_driver(t, v)
        self._esp32(t, v)
        self._fuses(t, v, dt)
        self.mon.step(t, v)
        self._step_n += 1
        if self._step_n % self.sc.record_every == 0 or t >= self.sc.t_stop:
            self._record(t, v)

    def _general_driver(self, t, v):
        sc = self.sc
        vin = v.get("gd_vin", 0.0) if sc.gd_present else 0.0
        if self.buck_on and vin < 3.9:
            self.buck_on = False
            self.events.append((t, "GD buck off (UVLO)"))
        elif not self.buck_on and vin > 4.25:
            self.buck_on = True
            self.events.append((t, "GD buck on"))
        self.ctl["v_gd_buck"].set(t, 1.0 if self.buck_on else 0.0)
        usb = sched_at(sc.usb, t) > 0.5
        rail = self.buck_on or usb
        self.ctl["v_gd_3v3"].set(t, 1.0 if rail else 0.0)
        if rail and self.rail3v3_since is None:
            self.rail3v3_since = t
            self.events.append((t, "+3V3_HOST up"))
        elif not rail and self.rail3v3_since is not None:
            self.rail3v3_since = None
            self.events.append((t, "+3V3_HOST down"))

    def _esp32(self, t, v):
        e = self.sc.esp
        up = (self.rail3v3_since is not None
              and t - self.rail3v3_since >= e.boot_s)
        lv = {}
        for nm, sch, hz in (("kill", e.kill, e.kill_hiz),
                            ("sen", e.servo_en, e.servo_en_hiz)):
            lvl = sched_at(sch, t)
            hiz = sched_at(hz, t, 0.0) > 0.5
            lv[nm] = (lvl, hiz)
        if e.firmware is not None and up:
            lv = e.firmware(t, v, self, lv)
        for nm in ("kill", "sen"):
            lvl, hiz = lv[nm]
            drive = up and not hiz and lvl is not None
            self.ctl[f"v_esp_{nm}_en"].set(t, 1.0 if drive else 0.0)
            self.ctl[f"v_esp_{nm}_lvl"].set(t, float(lvl or 0.0))

    def _fuses(self, t, v, dt):
        for ref in ("F1", "F2"):
            if ref in self.fuse_open:
                continue
            r, melt, rating = FUSE[ref]
            a = self.node(ref, 1)
            b = self.node(ref, 2)
            i = (v.get(a, 0.0) - v.get(b, 0.0)) / r if a != "0" else 0.0
            # heating above what the fuse carries indefinitely (110 % of
            # its rating); short pulses reduce to plain I²t
            self.fuse_i2t[ref] = max(0.0, self.fuse_i2t[ref]
                                     + (i * i - (1.1 * rating) ** 2) * dt)
            if self.fuse_i2t[ref] > melt:
                self.fuse_open[ref] = t
                self.ctl[f"v_{ref.lower()}"].set(t, 0.0)
                self.events.append((t, f"{ref} melted (I²t {melt} A²s)"))

    # -- recording ------------------------------------------------------------
    PROBES = {
        "pack": ("PACK+",), "servo": ("SERVO+",), "railv": ("/RAIL_V",),
        "pwr_int_n": ("/PWR_INT_N",), "estop_n": ("/ESTOP_N",),
        "flt_n": ("/SERVO_FLT_N",), "imon": ("/SERVO_IMON",),
        "v3v3": ("+3V3_HOST",), "kill": ("/KILL",), "servo_en": ("/SERVO_EN",),
    }

    def _record(self, t, v):
        r = self.rec
        g = v.get
        n = self.net
        u1p, u2p = self.u1.p, self.u2.p
        vals = {k: g(n(p[0]), 0.0) for k, p in self.PROBES.items()}
        vss = g(u1p[10], 0.0)
        vals.update({
            "t": t,
            "logic": g(self.node("J3", 1), 0.0),
            "gd_vin": g("gd_vin", 0.0),
            "bat": g(n("BAT+"), 0.0) - vss,
            "ts2": g(u1p[14], 0.0) - vss,
            "rst": g(u1p[21], 0.0) - vss,
            "ld": g(u1p[28], 0.0) - vss,
            "en": g(u2p[1], 0.0), "inp": g(u2p[3], 0.0),
            "bst": g(u2p[12], 0.0) - g(u2p[13], 0.0),
            "tmr": g(u2p[9], 0.0),
            "q2_vgs": g(self.node("Q2", 4), 0.0) - g(self.node("Q2", 1), 0.0),
            "q1_vgs": g(self.node("Q1", 4), 0.0) - g(self.node("Q1", 1), 0.0),
            "q3_vgs": g(self.node("Q3", 4), 0.0) - g(self.node("Q3", 1), 0.0),
            "q2_vds": g(self.node("Q2", 5), 0.0) - g(self.node("Q2", 1), 0.0),
            "q3_vds": g(self.node("Q3", 5), 0.0) - g(self.node("Q3", 1), 0.0),
            "i_q2": g("v_am_q2#branch", 0.0),
            "i_q3": g("v_am_q3#branch", 0.0),
            "i_pack": self.u1.i_pack,
            "u1_mode": ["OFF", "SHUTDOWN", "BOOT", "NORMAL", "SOFTSHUT"].index(
                self.u1.mode),
            "u1_dsg": float(self.u1.dsg_on), "u1_chg": float(self.u1.chg_on),
            "u2_gate": float(self.u2.gate),
            "u2_latch": {None: 0, "OC": 1, "SC": 2, "OT": 3}[self.u2.latch],
            "u2_active": float(self.u2.active),
            "esp_up": float(self.rail3v3_since is not None),
        })
        for k, val in vals.items():
            r.setdefault(k, []).append(val)

    # -- run -------------------------------------------------------------------
    def run(self) -> "Result":
        out = Engine.get().run(self.deck(), self, t_valid=self.sc.t_stop)
        events = sorted(self.events + self.u1.events + self.u2.events)
        return Result(self.sc, {k: np.asarray(v) for k, v in self.rec.items()},
                      events, out, self)


@dataclass
class Result:
    sc: Scenario
    w: dict[str, np.ndarray]
    events: list[tuple[float, str]]
    log: str
    board: Board

    def at(self, key: str, t: float) -> float:
        tt = self.w["t"]
        i = int(np.searchsorted(tt, t, side="right")) - 1
        return float(self.w[key][max(i, 0)])

    def window(self, key: str, t0: float, t1: float) -> np.ndarray:
        tt = self.w["t"]
        m = (tt >= t0) & (tt <= t1)
        return self.w[key][m]

    def first(self, key: str, pred, t0: float = 0.0) -> float | None:
        tt = self.w["t"]
        x = self.w[key]
        m = (tt >= t0) & pred(x)
        idx = np.nonzero(m)[0]
        return float(tt[idx[0]]) if len(idx) else None

    def event_times(self, text: str) -> list[float]:
        return [t for t, e in self.events if text in e]
