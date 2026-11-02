"""SPICE models for the board's discrete parts, from their datasheets.

Each diode is a forward model plus an explicit reverse-leakage current whose
size at the run's temperature comes from the datasheet (a table or a
max-at-25 °C figure with a stated doubling rule). Leakage is what decides the
off-state questions — whether a cold board stays off, whether a hot one wakes
itself — so it is modelled from the documents rather than from SPICE's
saturation current, which for a Schottky does not match both the forward and
the reverse curve.

``corner`` selects typical or worst-case leakage. Where a datasheet gives
only a maximum, typical is taken as a tenth of it and the choice is stated
in ``LEAK``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from pbsim.fetmodel import REWRITE_LIB


@dataclass(frozen=True)
class Leak:
    """Reverse leakage at a reference reverse voltage, by temperature (°C)."""
    table: tuple[tuple[float, float], ...]   # (T, amps) typical
    max_over_typ: float
    source: str

    def at(self, t_c: float, corner: str) -> float:
        pts = self.table
        if t_c <= pts[0][0]:
            (t0, i0), (t1, i1) = pts[0], pts[1]
        elif t_c >= pts[-1][0]:
            (t0, i0), (t1, i1) = pts[-2], pts[-1]
        else:
            for (t0, i0), (t1, i1) in zip(pts, pts[1:]):
                if t0 <= t_c <= t1:
                    break
        f = (t_c - t0) / (t1 - t0)
        i = math.exp(math.log(i0) + f * (math.log(i1) - math.log(i0)))
        return i * (self.max_over_typ if corner == "max" else 1.0)


def _doubling(i25: float, per_c: float = 10.0) -> tuple[tuple[float, float], ...]:
    return tuple((t, i25 * 2 ** ((t - 25.0) / per_c)) for t in (-40, 25, 85, 125))


LEAK = {
    # Diotec BAT54C, "Reverse characteristics (typical values)", read at
    # VR ≈ 5 V. The only maximum is 2 µA at 25 V / 25 °C, and leakage rises
    # with VR, so 2 µA also bounds a part at 5 V: max = 80 × the 5 V typical
    # (2 µA / 25 nA), the same treatment as the other diodes' maxima.
    "BAT54C": Leak(((-25, 3e-9), (25, 2.5e-8), (75, 7e-7), (125, 4.5e-5),
                    (150, 1.8e-4)), 80.0, "Diotec BAT54C p.2 table and chart"),
    # Nexperia PMEG4002EJ: 0.2 µA typ / 0.3 µA max at 10 V, 25 °C (Table 7);
    # silicon-Schottky doubling every 10 °C assumed above that.
    "PMEG4002EJ": Leak(_doubling(2e-7), 1.5, "Nexperia PMEG4002EJ Table 7"),
    # Diodes 1N4148W: 25 nA max at 20 V / 25 °C, 30 µA max at 25 V / 150 °C;
    # typical a tenth of max.
    "1N4148W": Leak(((25, 2.5e-9), (150, 3e-6)), 10.0, "Diodes 1N4148W"),
    # Diodes BAV70 (D2 from rev B): 25 nA max at 20 V / 25 °C, 30 µA max at 25 V /
    # 150 °C, the same limits as the 1N4148W; typical a tenth of max.
    "BAV70": Leak(((25, 2.5e-9), (150, 3e-6)), 10.0, "Diodes BAV70"),
    # Diodes MMSZ5246B: 0.1 µA max at 12 V / 25 °C; typical a tenth; ×2/10 °C.
    "MMSZ5246B": Leak(_doubling(1e-8), 10.0, "Diodes MMSZ5246B table"),
    # Diodes MMSZ5245B (D10, 15 V): 0.1 µA max at 11 V / 25 °C; as MMSZ5246B.
    "MMSZ5245B": Leak(_doubling(1e-8), 10.0, "Diodes MMSZ5245B table"),
    # Bourns SMBJ15A: 1 µA max at VRWM 15 V / 25 °C; typical a tenth; ×2/10 °C.
    "SMBJ15A": Leak(_doubling(1e-7), 10.0, "Bourns SMBJ15A table"),
    # TI CSD17556Q5B IDSS: 1 µA max at 24 V / 25 °C; no typical, no curve:
    # typical a tenth, ×2/10 °C (an assumption, stated in the report).
    "CSD17556Q5B": Leak(_doubling(1e-7), 10.0, "TI CSD17556Q5B §5.5 IDSS"),
}

# Reverse voltage at which each table is read; leakage is scaled as
# tanh(VR / 0.1 V) below it (a junction's leakage saturates within ~0.1 V)
# and held flat above it.
DIODE_MODELS = {
    # Schottky 40 V 0.2 A: VF 0.25 V @ 1 mA, 0.32 V @ 10 mA, 0.44 V @ 100 mA.
    "PMEG4002EJ": ".model d_pmeg4002ej d (is=8e-8 n=1.05 rs=0.7 bv=40 ibv=10u "
                  "cjo=14p eg=0.69 xti=2)",
    # Schottky 30 V: VF < 240 mV @ 0.1 mA, < 320 mV @ 1 mA, < 400 mV @ 10 mA.
    "BAT54C": ".model d_bat54c d (is=2.5e-8 n=1.0 rs=2 bv=30 ibv=10u cjo=10p "
              "eg=0.69 xti=2)",
    "1N4148W": ".model d_1n4148w d (is=2.52n rs=0.568 n=1.752 bv=100 "
               "ibv=100u cjo=2p m=0.333 vj=0.5 tt=20n)",
    # BAV70: VF ≤ 715 mV @ 1 mA, 855 mV @ 10 mA, 1 V @ 50 mA (the 1N4148W's limits)
    "BAV70": ".model d_bav70 d (is=2.52n rs=0.568 n=1.752 bv=75 "
             "ibv=2.5u cjo=2p m=0.333 vj=0.5 tt=6n)",
    # 16 V zener: VZ 15.2–16.8 V @ IZT 7.8 mA, ZZT 17 Ω.
    "MMSZ5246B": ".model d_mmsz5246b d (is=1e-14 n=1 rs=1 bv=16 ibv=7.8m "
                 "nbv=1.2 cjo=60p)",
    # 15 V zener: VZ 14.25–15.75 V @ IZT 8.5 mA, ZZT 16 Ω.
    "MMSZ5245B": ".model d_mmsz5245b d (is=1e-14 n=1 rs=1 bv=15 ibv=8.5m "
                 "nbv=1.2 cjo=60p)",
    # TVS: VBR 16.7–18.5 V @ 1 mA, VC 24.4 V @ 24.6 A (10/1000 µs).
    "SMBJ15A": ".model d_smbj15a d (is=1e-12 n=1.2 rs=0.28 bv=17.6 ibv=1m "
               "cjo=1.5n)",
}

OTHER_MODELS = """
* Vishay 2N7002K: VGS(th) 1–2.5 V, about 2 Ω at VGS 10 V (level-1 fit)
.model n_2n7002k nmos (level=1 vto=1.8 kp=0.3 lambda=0.02 rd=0.3 rs=0.3
+ cgso=20p cgdo=6p cbd=15p is=1e-14)
.model d_2n7002k_body d (is=1e-12 n=1.2 rs=0.5 bv=60)
* MMBT3904 (2N3904 Gummel-Poon, ON Semiconductor's published values)
.model q_mmbt3904 npn (is=6.734f xti=3 eg=1.11 vaf=74.03 bf=416.4 ne=1.259
+ ise=6.734f ikf=66.78m xtb=1.5 br=.7371 nc=2 isc=0 ikr=0 rc=1 cjc=3.638p
+ mjc=.3085 vjc=.75 fc=.5 cje=4.493p mje=.2593 vje=.75 tr=239.5n tf=301.2p
+ itf=.4 vtf=4 xtf=2 rb=10)
"""


def fet_include() -> str:
    return f".include {REWRITE_LIB}"


def ntc_ohms(t_c: float, r25: float = 10e3, beta: float = 3380.0) -> float:
    """Murata NCP18XH103F03RB: 10 kΩ ±1 %, B25/50 = 3380 K ±1 %."""
    return r25 * math.exp(beta * (1.0 / (t_c + 273.15) - 1.0 / 298.15))


def _model_params(mpn: str) -> dict[str, float]:
    out = {"is": 1e-14, "n": 1.0, "eg": 1.11, "xti": 3.0}
    for tok in DIODE_MODELS[mpn].replace("(", " ").replace(")", " ").split():
        if "=" in tok:
            k, v = tok.split("=")
            if k in out:
                out[k] = float(v.rstrip("munpfk")) * {
                    "m": 1e-3, "u": 1e-6, "n": 1e-9, "p": 1e-12, "f": 1e-15,
                    "k": 1e3}.get(v[-1], 1.0)
    return out


def model_is(mpn: str, t_c: float) -> float:
    """ngspice's diode saturation current at t_c (diotemp.c, TNOM 27 °C)."""
    p = _model_params(mpn)
    t, tn = t_c + 273.15, 300.15
    vt = 8.617333e-5 * t
    return p["is"] * math.exp((t / tn - 1.0) * p["eg"] / (p["n"] * vt)
                              + p["xti"] / p["n"] * math.log(t / tn))


def diode_lines(name: str, anode: str, cathode: str, mpn: str, t_c: float,
                corner: str) -> list[str]:
    """The diode, with its total reverse current set to the datasheet's: the
    model's own saturation current plus a correction, cathode→anode."""
    m = DIODE_MODELS[mpn].split()[1]
    extra = LEAK[mpn].at(t_c, corner) - model_is(mpn, t_c)
    return [f"d{name} {anode} {cathode} {m}",
            f"b{name}_lk {cathode} {anode} i={extra:.4e}*tanh(max(v({cathode},"
            f"{anode}),0)/0.1)"]


def fet_lines(name: str, d: str, g: str, s: str, t_c: float,
              corner: str) -> list[str]:
    leak = LEAK["CSD17556Q5B"].at(t_c, corner)
    return [f"x{name} {d} {g} {s} CSD17556Q5B",
            f"b{name}_lk {d} {s} i={leak:.4e}*tanh(v({d},{s})/1.0)"]


def model_block() -> str:
    return "\n".join([fet_include(), *DIODE_MODELS.values(), OTHER_MODELS])
