"""ngspice simulation of the power-lite board, built from tools/parts.py (the board's spec).

The board's parts are taken from parts.py net for net; around them sit models of what the board connects
to. Values marked ASSUMED are not measured: they bound the results and are checked at bring-up.
- the pack: 3 cells, 15 mOhm, 2 x 150 nH of lead (ASSUMED, as rev B's pbsim "15 mOhm pack");
- the BMS: a closed switch of 2.5 mOhm (ShillehTek board: 8 x P75N02LDG, 5 mOhm each, 4 + 4 in series);
  its trip points are not modelled (undocumented; the bench check measures them);
- the main switch 3 mOhm; F1 4.58 mOhm (rev B record C13), F2 32 mOhm (ASSUMED);
- K1: TE V23134 coil 90 Ohm with its 470 Ohm suppression resistor, L = 0.3 H (ASSUMED, not on TE's sheet);
  the contact closes at the operate current (7.2 V / 90 Ohm) and opens at the release current (1.6 V / 90 Ohm),
  1.5 mOhm closed (15 mV at 10 A typ., TE);
- the ESP32: SERVO_EN from a 3.3 V pin through 30 Ohm; IO4's internal pull-up 45 kOhm to 3.3 V;
- the General Driver on J3: 11 Ohm (1 A) with 100 uF (ASSUMED);
- the servos: a current source with 200 uF of input capacitance in all (ASSUMED).
"""
import importlib.util
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
# loaded by path: in a full pytest run "parts" is already cad/parts.py
_spec = importlib.util.spec_from_file_location("power_lite_parts", HERE.parent / "tools" / "parts.py")
PD = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(PD)

NGSPICE = shutil.which("ngspice")
OPERATE_A, RELEASE_A = 7.2 / 90.0, 1.6 / 90.0
MODELS = """
.model D1N4007 D(IS=7.03n N=1.81 RS=34.1m BV=1000 IBV=50n CJO=10p TT=4.32u)
.model DTVS D(IS=1p N=1 RS=10m BV=15.2 IBV=1m)
.model DLEDR D(IS=1e-20 N=1.8 RS=5)
.model DLEDG D(IS=1e-21 N=1.8 RS=5)
.model QPN2222A NPN(IS=14.34f XTI=3 EG=1.11 VAF=74.03 BF=255.9 NE=1.307 ISE=14.34f IKF=.2847 XTB=1.5
+ BR=6.092 NC=2 ISC=0 IKR=0 RC=1 CJC=7.306p MJC=.3416 VJC=.75 FC=.5 CJE=22.01p MJE=.377 VJE=.75
+ TR=46.91n TF=411.1p ITF=.6 VTF=1.7 XTF=3 RB=10)
.model KCONTACT csw(it={it} ih={ih} ron=1.5m roff=1e9)
.model ESTOPSW sw(vt=0.5 vh=0.1 ron=50m roff=1e9)
""".format(it=(OPERATE_A + RELEASE_A) / 2, ih=(OPERATE_A - RELEASE_A) / 2)


def n(net):
    return "0" if net == "GND" else net.lower()


def board_lines():
    """The board's own parts, from parts.py (connectors and wire pads are where the harness attaches)."""
    out = []
    for ref, d in PD.parts.items():
        p = d["nets"]
        if d["lib_id"] == "Device:R":
            v = d["value"].replace("k", "k").replace("M", "Meg")
            out.append(f"R{ref} {n(p['1'])} {n(p['2'])} {v}")
        elif ref == "C1":
            out.append(f"CC1 {n(p['1'])} c1esr 1000u IC=0\nRC1ESR c1esr 0 30m")
        elif ref == "C2":
            out.append(f"CC2 {n(p['1'])} {n(p['2'])} 100n")
        elif ref == "D1":
            out.append(f"DD1 {n(p['2'])} {n(p['1'])} D1N4007")
        elif ref in ("D2", "D3"):
            out.append(f"D{ref} {n(p['2'])} {n(p['1'])} DTVS")
        elif ref in ("LED1", "LED2"):
            out.append(f"D{ref} {n(p['2'])} {n(p['1'])} {'DLEDG' if ref == 'LED1' else 'DLEDR'}")
        elif ref in ("Q1", "Q2"):
            out.append(f"Q{ref} {n(p['3'])} {n(p['2'])} {n(p['1'])} QPN2222A")
        elif ref == "F1":
            out.append(f"RF1 {n(p['1'])} {n(p['2'])} 4.58m")
        elif ref == "F2":
            out.append(f"RF2 {n(p['1'])} {n(p['2'])} 32m")
        elif ref == "K1":
            out.append(f"LK1 {n(p['A1'])} k1a 0.3 IC=0\nRK1 k1a k1b 90\nVK1SENSE k1b {n(p['A2'])} 0\n"
                       f"RK1SUPP {n(p['A1'])} {n(p['A2'])} 470\n"
                       f"VK1CONTACT {n(p['13'])} k1c 0\nWK1 k1c {n(p['14'])} VK1SENSE KCONTACT")
    return out


def deck(vcell=3.7, estop="closed", servo_en=3.3, i_servo=0.0, extra="", analysis=".op", meas=""):
    """estop: 'closed', 'open', or a PWL for its control (1 = closed); servo_en: volts, None (pin floating),
    or a PWL string; i_servo: amps or a PWL string for the servos' draw."""
    L = ["* power-lite", MODELS]
    # pack, leads, BMS, main switch (outside the board)
    L += [f"VC1 c1 bn {vcell}", f"VC2 c2 c1 {vcell}", f"VC3 c3 c2 {vcell}", "RPACK c3 c3r 15m",
          "LPOS c3r batp 150n", "LNEG bn batn 150n", "VBMS batn bmsx 0", "RBMS bmsx 0 2.5m",
          "RSW batp vin 3m"]
    # E-stop contact (J11: LOGIC to COIL_HI)
    ctl = {"closed": "DC 1", "open": "DC 0"}.get(estop, estop)
    L += [f"VESTOPCTL estopctl 0 {ctl}", "SESTOP logic coil_hi estopctl 0 ESTOPSW"]
    # ESP32 and the General Driver
    if servo_en is not None:
        src = f"DC {servo_en}" if isinstance(servo_en, (int, float)) else servo_en
        L += [f"VEN en_src 0 {src}", "REN en_src servo_en 30"]
    L += ["V3V3 esp3v3 0 3.3", "RPU esp3v3 estop_n 45k", "RGD logic 0 11", "CGD logic 0 100u IC=0"]
    # servos on SERVO+
    src = f"DC {i_servo}" if isinstance(i_servo, (int, float)) else i_servo
    L += [f"ISRV servop 0 {src}", "CSRV servop 0 200u IC=0", "RSRVLEAK servop 0 100k"]
    L += board_lines()
    L += [extra, analysis, meas, ".end"]
    return "\n".join(x for x in L if x)


def run(text, values=()):
    """Run ngspice in batch; returns {name: float} from .meas results and from `print` of `values`."""
    assert NGSPICE, "ngspice not found"
    ctrl = ""
    if values:
        ctrl = ".control\nrun\nprint " + " ".join(values) + "\n.endc\n"
    with tempfile.TemporaryDirectory() as td:
        f = Path(td) / "deck.cir"
        f.write_text(text.replace("\n.end", "\n" + ctrl + ".end"))
        r = subprocess.run([NGSPICE, "-b", str(f)], capture_output=True, text=True, timeout=600)
    out = r.stdout + r.stderr
    res = {}
    for m in re.finditer(r"^\s*([\w()\[\]#.,+\-/*]+)\s*=\s*([-+0-9.eE]+)", out, re.M):
        try:
            res[m.group(1).lower()] = float(m.group(2))
        except ValueError:
            pass
    if "error" in out.lower() and not res:
        raise RuntimeError(out[-2000:])
    return res


OP_VALUES = ("v(coil_hi)", "v(coil_lo)", "i(vk1sense)", "i(ven)", "v(estop_n)", "v(rail_v)", "v(servop)",
             "v(logic)", "v(led1_a)", "v(led2_a)", "v(q2_b)")


def op(vcell, estop, servo_en, i_servo=0.0):
    """Steady state: a transient run long enough for the relay to settle to the state its coil current
    decides (the contact's hysteresis has no DC answer), read at its end."""
    r = run(deck(vcell, estop, servo_en, i_servo, analysis=".tran 1m 300m uic",
                 meas="\n".join(f".meas tran {k} FIND {v} AT=299m" for k, v in (
                     ("vhi", "v(coil_hi)"), ("vce1", "v(coil_lo)"), ("icoil", "i(vk1sense)"),
                     ("igpio", "i(ven)"), ("estop_n", "v(estop_n)"), ("rail_v", "v(rail_v)"),
                     ("servop", "v(servop)"), ("logic", "v(logic)"), ("vled1", "v(led1_a)"),
                     ("vled2", "v(led2_a)"), ("vbe2", "v(q2_b)"), ("vbe1", "v(q1_b)")))))
    if "vhi" in r and "vce1" in r:
        r["vcoil"] = r["vhi"] - r["vce1"]
    if "logic" in r and "vled1" in r:
        r["iled1"] = (r["logic"] - r["vled1"]) / 4.7e3
    if "servop" in r and "vled2" in r:
        r["iled2"] = (r["servop"] - r["vled2"]) / 4.7e3
    return r


I2T = "BI2T i2t 0 V=i(vk1contact)*i(vk1contact)"


def inrush(vcell=4.2, csrv="200u"):
    """The relay closing onto the discharged servo rail: SERVO_EN rises at 1 ms."""
    t = deck(vcell, "closed", "PWL(0 0 1m 0 1.001m 3.3)", 0.0, extra=I2T, analysis=".tran 1u 60m 0 1u uic",
             meas="\n".join([".meas tran ipack MIN i(vbms)", ".meas tran icontact MAX i(vk1contact)",
                             ".meas tran i2t_f1 INTEG v(i2t) FROM=1m TO=60m",
                             ".meas tran tw100 TRIG i(vk1contact) VAL=100 RISE=1 TARG i(vk1contact) VAL=100 FALL=1",
                             ".meas tran tw50 TRIG i(vk1contact) VAL=50 RISE=1 TARG i(vk1contact) VAL=50 FALL=1",
                             ".meas tran tclose WHEN i(vk1sense)=" + f"{OPERATE_A}" + " RISE=1"]))
    t = t.replace("CSRV servop 0 200u", f"CSRV servop 0 {csrv}")
    r = run(t)
    if "ipack" in r:
        r["ipack"] = -r["ipack"]   # discharge current flows into VBMS's - node
    return r


def release(cause, vcell=3.7):
    """The rail on from 0, then at 100 ms either the E-stop opens or the ESP32 drops SERVO_EN."""
    if cause == "estop":
        t = deck(vcell, "PWL(0 1 100m 1 100.01m 0)", 3.3, 1.0, analysis=".tran 10u 200m uic",
                 meas="\n".join([
                     ".meas tran trel TRIG v(estopctl) VAL=0.5 FALL=1 TARG i(vk1sense) VAL=" + f"{RELEASE_A}" + " FALL=1",
                     ".meas tran vhi_min MIN v(coil_hi) FROM=100m TO=200m",
                     ".meas tran vbe2_min MIN v(q2_b) FROM=100m TO=200m",
                     ".meas tran vce1_max MAX v(coil_lo) FROM=100m TO=200m",
                     ".meas tran vsw_max MAX v(logic,coil_hi) FROM=100m TO=200m"]))
    else:
        t = deck(vcell, "closed", "PWL(0 3.3 100m 3.3 100.01m 0)", 1.0, analysis=".tran 10u 200m uic",
                 meas="\n".join([
                     ".meas tran trel TRIG v(en_src) VAL=1.65 FALL=1 TARG i(vk1sense) VAL=" + f"{RELEASE_A}" + " FALL=1",
                     ".meas tran vce1_max MAX v(coil_lo) FROM=100m TO=200m"]))
    return run(t)


def getup(vcell=3.0, amps=12.0):
    """A 12 A get-up for 2 s at a nearly flat pack: the rail and the coil must hold."""
    pwl = f"PWL(0 0 200m 0 201m {amps} 2.2 {amps} 2.201 0)"
    return run(deck(vcell, "closed", 3.3, pwl, analysis=".tran 1m 2.4 uic",
                    meas="\n".join([".meas tran logic_min MIN v(logic) FROM=200m TO=2.2",
                                    ".meas tran servop_min MIN v(servop) FROM=200m TO=2.2",
                                    ".meas tran icoil_min MIN i(vk1sense) FROM=200m TO=2.2"])))


if __name__ == "__main__":
    for vc in (3.0, 3.7, 4.2):
        for es in ("closed", "open"):
            for en in (3.3, 0.0, None):
                v = op(vc, es, en)
                print(f"pack {3 * vc:4.1f} V  E-stop {es:6s}  SERVO_EN {str(en):4s}: "
                      f"coil {v.get('vcoil', float('nan')):5.2f} V  SERVO+ {v.get('servop', float('nan')):5.2f} V  "
                      f"ESTOP_N {v.get('estop_n', float('nan')):4.2f} V  RAIL_V {v.get('rail_v', float('nan')):4.2f} V")
