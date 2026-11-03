"""A readable, wired circuit diagram of the power-lite board (the KiCad schematic labels every pin
instead of drawing wires). Same parts and nets as tools/parts.py.
usage: uv run --with schemdraw --with matplotlib python tools/draw_circuit.py <out.png>"""
import sys

import schemdraw
import schemdraw.elements as elm

schemdraw.use("matplotlib")
OUT = sys.argv[1]
TOP, LOW = 8.0, 3.0          # the servo rail row and the logic / coil row

with schemdraw.Drawing(show=False) as d:
    d.config(unit=2.4, fontsize=10)

    # ---- pack, BMS (cuts the negative lead: its P- is the system ground), main switch
    elm.Battery().at((0, 3.6)).up().length(3.0).label("3S LiPo\n9.0–12.6 V\nJ1 XT60", loc="bottom", ofst=0.3)
    elm.Line().at((0, 6.6)).up().length(TOP - 6.6)
    elm.Line().at((0, 3.6)).down().length(2.6)
    elm.RBox().at((0, 1.0)).right().length(3.0).label("BMS 3S (ShillehTek CF-3SJH25A-C)\nB- in (W1), P- out (W2)", loc="bottom")
    elm.Line().at((3.0, 1.0)).right().length(1.6)
    elm.Ground().at((4.6, 1.0)).label("P- = system GND", loc="right", fontsize=9)
    elm.Label().at((0.3, 2.0)).label("J2 balance lead → BMS taps W3–W5", loc="right", fontsize=8)
    elm.Line().at((0, TOP)).right().length(1.0)
    elm.Switch().at((1.0, TOP)).right().length(2.4).label("main switch\n57000-01 (W6/W7)", loc="top")
    elm.Line().at((3.4, TOP)).right().length(0.6)
    elm.Dot().at((4.0, TOP)).label("VIN", loc="bottom", ofst=(0.35, -0.1))
    elm.Line().at((4.0, TOP)).right().length(1.0)
    elm.Dot().at((5.0, TOP))
    elm.Zener().at((5.0, TOP)).down().length(2.2).reverse().label("D3\n1.5KE16A", loc="top")
    elm.Ground().at((5.0, TOP - 2.2))

    # ---- servo branch: F1 -> relay contact -> SERVO+ -> bulk, TVS, RAIL_V, LED, servos
    elm.Line().at((5.0, TOP)).right().length(0.8)
    elm.Fuse().at((5.8, TOP)).right().length(2.2).label("F1 15 A", loc="top")
    elm.Switch().at((8.0, TOP)).right().length(2.4).label("K1 contact\n30 → 87", loc="top")
    elm.Dot().at((10.4, TOP)).label("SERVO+", loc="bottom", ofst=(0.5, -0.1))
    elm.Line().at((10.4, TOP)).right().length(11.0)
    for x, kind, lab in ((12.0, "cap", "C1\n1000 µF 25 V"), (14.2, "tvs", "D2\n1.5KE16A")):
        elm.Dot().at((x, TOP))
        if kind == "cap":
            elm.Capacitor(polar=True).at((x, TOP)).down().length(2.2).label(lab, loc="top")
        else:
            elm.Zener().at((x, TOP)).down().length(2.2).reverse().label(lab, loc="top")
        elm.Ground().at((x, TOP - 2.2))
    elm.Dot().at((16.4, TOP))
    elm.Resistor().at((16.4, TOP)).down().length(1.8).label("R3 47k", loc="bottom")
    elm.Dot().at((16.4, TOP - 1.8))
    elm.Line().at((16.4, TOP - 1.8)).right().length(0.5)
    elm.Label().at((16.9, TOP - 1.8)).label("RAIL_V → J12.5\n(GPIO34)", loc="right", fontsize=8)
    elm.Resistor().at((16.4, TOP - 1.8)).down().length(1.8).label("R4 10k", loc="bottom")
    elm.Ground().at((16.4, TOP - 3.6))
    elm.Dot().at((20.0, TOP))
    elm.Resistor().at((20.0, TOP)).down().length(1.4).label("R8 4.7k", loc="top")
    elm.LED().at((20.0, TOP - 1.4)).down().length(1.4).label("LED2\nrail on", loc="top")
    elm.Ground().at((20.0, TOP - 2.8))
    elm.Label().at((22.0, TOP + 0.9)).label("→ J4–J7 servo branches\n   (1 data, 2 SERVO+, 3 GND)\n   data from J8 ← General Driver H5",
                                       loc="right", fontsize=9)

    # ---- logic branch: F2 -> General Driver and the E-stop / coil circuit
    elm.Line().at((4.0, TOP)).down().length(TOP - LOW)
    elm.Fuse().at((4.0, LOW)).right().length(2.2).label("F2 3 A", loc="top")
    elm.Dot().at((6.2, LOW)).label("LOGIC", loc="bottom", ofst=(0, -0.1))
    elm.Line().at((6.2, LOW)).up().length(1.2)
    elm.Label().at((6.2, LOW + 1.4)).label("→ J3: General Driver H1\n(ESP32, IMU, Pi 5 V)", loc="top", fontsize=9)
    elm.Line().at((6.2, LOW)).right().length(1.4)
    elm.Dot().at((7.6, LOW))
    elm.Resistor().at((7.6, LOW)).down().length(1.4).label("R7 4.7k", loc="top")
    elm.LED().at((7.6, LOW - 1.4)).down().length(1.4).label("LED1\nlogic on", loc="top")
    elm.Ground().at((7.6, LOW - 2.8))
    elm.Line().at((7.6, LOW)).right().length(1.2)
    elm.Button(nc=True).at((8.8, LOW)).right().length(2.4).label("E-STOP (NC, latching)\nJ11", loc="top")
    elm.Dot().at((11.2, LOW))
    elm.Label().at((12.3, LOW + 0.35)).label("COIL_HI", fontsize=9)
    elm.Line().at((11.2, LOW)).right().length(2.2)
    # ESTOP_N divider
    elm.Resistor().at((11.2, LOW)).down().length(1.5).label("R5 47k", loc="top")
    elm.Dot().at((11.2, LOW - 1.5)).label("ESTOP_N → J12.6 (IO4)", loc="left", fontsize=8)
    elm.Resistor().at((11.2, LOW - 1.5)).down().length(1.5).label("R6 10k", loc="top")
    elm.Ground().at((11.2, LOW - 3.0))
    # coil, flyback diode, Q1
    elm.Dot().at((13.4, LOW))
    elm.Inductor2(loops=3).at((13.4, LOW)).down().length(2.2).label("K1 coil\n90 Ω, 0.13 A", loc="top")
    elm.Dot().at((13.4, LOW - 2.2)).label("COIL_LO", loc="left", fontsize=8)
    elm.Line().at((13.4, LOW)).right().length(1.4)
    elm.Diode().at((14.8, LOW - 2.2)).up().length(2.2).label("D1\n1N4007", loc="bottom")
    elm.Line().at((13.4, LOW - 2.2)).right().length(1.4)
    q = elm.BjtNpn(circle=True).at((13.4, LOW - 4.2)).anchor("collector").label("Q1\nPN2222A", loc="right")
    elm.Line().at((13.4, LOW - 2.2)).to(q.collector)
    elm.Ground().at(q.emitter)
    bx, by = q.base[0] - 0.6, q.base[1]
    elm.Line().at(q.base).left().length(0.6)
    elm.Dot().at((bx, by))
    elm.Resistor().at((bx, by)).left().length(2.2).label("R1 470", loc="top")
    elm.Label().at((bx - 2.4, by)).label("SERVO_EN ← J12.2\n(IO16)", loc="left", fontsize=8)
    elm.Resistor().at((bx, by)).down().length(1.4).label("R2 10k", loc="bottom")
    elm.Ground().at((bx, by - 1.4))
    # the coil works the contact
    elm.Line(ls="--", color="gray").at((13.0, LOW - 0.35)).to((9.2, TOP - 0.3))

    elm.Label().at((0, -6.6)).label(
        "Power lite rev A (pcb/power-lite). The E-stop opens the coil circuit, so K1 drops SERVO+ without firmware; "
        "R2 holds Q1 off until the ESP32 drives SERVO_EN. J12.3 is GND.", loc="right", fontsize=9)
    d.save(OUT, dpi=170)
print("written", OUT)
