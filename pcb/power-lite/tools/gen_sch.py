"""Generate pcb/power-lite/power-lite.kicad_sch from tools/parts.py (the spec).

Every pin gets a short stub and a net label, so the drawing carries the spec's connections and
nothing is wired by hand. usage: python3 tools/gen_sch.py   (run from pcb/power-lite)
"""
import math
import sys
import uuid
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import parts as PD  # noqa: E402
import sexpr as S  # noqa: E402
from sexpr import Q  # noqa: E402

LIB = Path("/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols")
OUT = HERE.parent / f"{PD.PROJECT}.kicad_sch"
NS = uuid.UUID("3c8e1f52-7d0b-4a8e-9f61-5a2b9c0d1e7f")
ROOT = str(uuid.uuid5(NS, "root"))
G = 2.54

# ref: (x, y, rotation) on an A3 sheet, mm, y down
PLACE = {
    "J1": (40.64, 50.8, 0), "J2": (40.64, 81.28, 0),
    "W1": (88.9, 45.72, 0), "W2": (88.9, 55.88, 0), "W3": (88.9, 66.04, 0), "W4": (88.9, 76.2, 0),
    "W5": (88.9, 86.36, 0), "W6": (132.08, 45.72, 0), "W7": (132.08, 55.88, 0), "D3": (152.4, 76.2, 0),
    "F1": (177.8, 50.8, 90), "F2": (177.8, 81.28, 90),
    "K1": (231.14, 60.96, 0), "D1": (213.36, 60.96, 90), "J11": (195.58, 111.76, 0),
    "Q1": (238.76, 111.76, 0), "R1": (220.98, 121.92, 90), "R2": (238.76, 129.54, 0),
    "C1": (271.78, 55.88, 0), "D2": (287.02, 55.88, 0),
    "J4": (335.28, 43.18, 0), "J5": (335.28, 60.96, 0), "J6": (335.28, 78.74, 0), "J7": (335.28, 96.52, 0),
    "J8": (335.28, 116.84, 0),
    "J3": (195.58, 152.4, 0), "R7": (218.44, 152.4, 0), "LED1": (218.44, 167.64, 90),
    "R3": (264.16, 152.4, 0), "R4": (264.16, 167.64, 0), "R5": (287.02, 152.4, 0), "R6": (287.02, 167.64, 0),
    "R8": (241.3, 152.4, 0), "LED2": (241.3, 167.64, 90), "J12": (335.28, 160.02, 0),
}

_cache = {}


def libsym(lib_id):
    lib, name = lib_id.split(":")
    if lib not in _cache:
        _cache[lib] = S.lib_symbols(LIB / f"{lib}.kicad_sym")
    return S.flatten(_cache[lib], name)


def uid(*a):
    return Q(str(uuid.uuid5(NS, "/".join(map(str, a)))))


def font(size=1.27, hide=False, justify=None):
    e = ["effects", ["font", ["size", size, size]]]
    if justify:
        e.append(["justify"] + justify.split())
    if hide:
        e.append(["hide", "yes"])
    return e


def r3(v):
    return round(v + 0.0, 3)


def xform(px, py, rot):
    """library offset (y up) -> sheet offset (y down) for a symbol rotated by rot."""
    sx, sy = px, -py
    t = math.radians(rot)
    return sx * math.cos(t) + sy * math.sin(t), -sx * math.sin(t) + sy * math.cos(t)


items, labels, ncs = [], [], []
used = {}
for ref, d in PD.parts.items():
    x, y, rot = PLACE[ref]
    sym = libsym(d["lib_id"])
    used[d["lib_id"]] = sym
    pins = S.pins(sym)
    for num, (px, py, ang, _t) in pins.items():
        X, Y = xform(px, py, rot)
        p = (r3(x + X), r3(y + Y))
        # the pin points from its end toward the body; the stub goes the other way
        bx, by = xform(px + math.cos(math.radians(ang)), py + math.sin(math.radians(ang)), rot)
        dx, dy = (x + X) - (x + bx), (y + Y) - (y + by)
        d_ = "R" if dx > 0.5 else "L" if dx < -0.5 else "D" if dy > 0.5 else "U"
        net = d["nets"].get(num)
        if net is None:
            ncs.append(p)
            continue
        end = (r3(p[0] + G * (dx > 0.5) - G * (dx < -0.5)), r3(p[1] + G * (dy > 0.5) - G * (dy < -0.5)))
        items.append(["wire", ["pts", ["xy", p[0], p[1]], ["xy", end[0], end[1]]],
                      ["stroke", ["width", 0], ["type", "default"]], ["uuid", uid("w", ref, num)]])
        ang_, just = {"R": (0, "left bottom"), "L": (180, "right bottom"), "U": (90, "left bottom"),
                      "D": (270, "right bottom")}[d_]
        labels.append((net, end, ang_, just))
    props = {pp[1]: pp for pp in S.find_all(sym, "property")}

    def field(name, dy):
        a = S.find(props[name], "at")
        X, Y = xform(float(a[1]), float(a[2]), rot)
        return [r3(x + X), r3(y + Y + dy), 0]

    # the W parts are solder pads for wires, not parts to buy
    s = ["symbol", ["lib_id", Q(d["lib_id"])], ["at", x, y, rot], ["unit", 1], ["exclude_from_sim", "no"],
         ["in_bom", "no" if ref.startswith("W") else "yes"], ["on_board", "yes"], ["dnp", "no"],
         ["uuid", uid("sym", ref)],
         ["property", Q("Reference"), Q(ref), ["at"] + field("Reference", 0), font()],
         ["property", Q("Value"), Q(d["value"]), ["at"] + field("Value", 0), font()],
         ["property", Q("Footprint"), Q(d["fp"]), ["at", x, y, 0], font(hide=True)],
         ["property", Q("Datasheet"), Q(d["datasheet"]), ["at", x, y, 0], font(hide=True)],
         ["property", Q("Description"), Q(d["desc"]), ["at", x, y, 0], font(hide=True)]]
    if d["mpn"]:
        s.append(["property", Q("MPN"), Q(d["mpn"]), ["at", x, y, 0], font(hide=True)])
    for num in sorted(pins, key=lambda k: (len(k), k)):
        s.append(["pin", Q(num), ["uuid", uid("pin", ref, num)]])
    s.append(["instances", ["project", Q(PD.PROJECT), ["path", Q("/" + ROOT), ["reference", Q(ref)], ["unit", 1]]]])
    items.append(s)

for i, (net, p, ang, just) in enumerate(labels):
    items.append(["label", Q(net), ["at", p[0], p[1], ang], ["fields_autoplaced", "yes"], font(justify=just),
                  ["uuid", uid("l", i, net)]])
for p in ncs:
    items.append(["no_connect", ["at", p[0], p[1]], ["uuid", uid("nc", p)]])
NOTE = ("Power lite: 3S pack -> ShillehTek BMS (switches the negative lead; W1-W5) -> main switch (W6/W7) -> "
        "F1 15 A servo rail through K1, F2 3 A logic feed.\nK1 drops when the E-stop (J11, NC) opens or the ESP32 "
        "releases SERVO_EN. Design: docs/design-v6/2026-11-02-power-lite-options.md")
items.append(["text", Q(NOTE), ["exclude_from_sim", "no"], ["at", 30.48, 200.66, 0],
              font(1.27, justify="left bottom"), ["uuid", uid("note")]])

lib_symbols = ["lib_symbols"]
for lib_id, sym in sorted(used.items()):
    s = list(sym)
    s[1] = Q(lib_id)
    lib_symbols.append(s)
sch = ["kicad_sch", ["version", 20250114], ["generator", Q("eeschema")], ["generator_version", Q("9.0")],
       ["uuid", Q(ROOT)], ["paper", Q("A3")],
       ["title_block", ["title", Q("Power lite")], ["date", Q("2026-11-02")], ["rev", Q("A")],
        ["company", Q("biped-robot")],
        ["comment", 1, Q("Design: docs/design-v6/2026-11-02-power-lite-options.md")],
        ["comment", 2, Q("Generated by pcb/power-lite/tools/gen_sch.py from tools/parts.py")]],
       lib_symbols] + items + [["sheet_instances", ["path", Q("/"), ["page", Q("1")]]], ["embedded_fonts", "no"]]
OUT.write_text(S.dump(sch) + "\n", encoding="utf-8")
print(f"{len(PD.parts)} symbols, {len(labels)} labels, {len(ncs)} no-connects -> {OUT.name}")
