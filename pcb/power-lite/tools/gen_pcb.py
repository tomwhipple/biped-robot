"""Build pcb/power-lite/power-lite.kicad_pcb from the schematic netlist (run with KiCad's python).

Placement, outline, rules and the high-current pours are set here; Freerouting routes the rest
(tools/route.py), then tools/post_pcb.py imports the session and fills the pours.
usage: gen_pcb.py <netlist>   (from pcb/power-lite)
"""
import sys
from pathlib import Path

import pcbnew

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import sexpr as S  # noqa: E402

PROJ = HERE.parent
BOARD = str(PROJ / "power-lite.kicad_pcb")
FPROOT = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints/"
W, H = 97.0, 64.5
OX, OY = 100.0, 60.0
MM = pcbnew.FromMM


def P(x, y):
    return pcbnew.VECTOR2I(MM(OX + x), MM(OY + y))


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


# ref: (x, y, rotation) in board mm, y down, rotation CCW as KiCad
PLACE = {
    # pack, BMS and switch pads (left)
    # J1 as rev B's board: the mating face (the footprint's straight edge) 0.25 mm past the left edge,
    # the pins 16.1 mm in; pin 2 (B+) above pin 1 (B-)
    "J1": (16.1, 19.0, 90), "W6": (23.0, 9.0, 0), "W7": (31.0, 9.0, 0), "W1": (23.0, 19.0, 0),
    "W2": (23.0, 26.0, 0), "J2": (4.0, 43.5, 90), "W5": (11.0, 34.5, 0), "W4": (11.0, 38.5, 0),
    "W3": (11.0, 42.5, 0), "D3": (22.0, 42.0, 0),
    # fuses, relay, servo rail (top band)
    "F1": (30.0, 17.8, 0), "F2": (30.0, 33.0, 0), "K1": (68.0, 22.0, 90),
    "D2": (76.0, 6.0, 270), "C1": (76.0, 31.0, 0),
    "J4": (88.0, 9.0, 90), "J5": (88.0, 20.0, 90), "J6": (88.0, 31.0, 90), "J7": (88.0, 42.0, 90),
    "J8": (88.0, 53.0, 90),
    # coil drive and telemetry (bottom band)
    "D1": (46.0, 40.0, 0), "Q1": (57.0, 45.5, 0), "R1": (66.0, 42.0, 270), "R2": (69.5, 42.0, 270),
    "R3": (74.0, 44.0, 270), "R4": (77.5, 44.0, 270), "R8": (81.0, 44.0, 270),
    "R5": (49.5, 47.0, 270), "R6": (53.0, 47.0, 270), "R7": (44.0, 47.0, 270),
    "Q2": (57.5, 51.5, 0), "C2": (72.0, 57.5, 0),
    "LED1": (34.0, 50.0, 0), "LED2": (82.0, 60.0, 0),
    "J3": (28.0, 59.5, 0), "J11": (40.0, 61.0, 0), "J12": (52.0, 61.0, 0),
}
HOLES = [(3.2, 3.2), (W - 3.2, 3.2), (3.2, H - 3.2), (W - 3.2, H - 3.2)]
# silkscreen references that the footprints' defaults would put on a neighbour
REF_AT = {"K1": (61.0, 22.0), "D3": (29.6, 46.4), "J2": (6.0, 31.4), "Q2": (60.0, 55.2), "C2": (74.5, 60.6),
          "W3": (14.0, 42.5), "W4": (14.0, 38.5), "W5": (14.0, 34.5)}

# top-layer pours for the high-current nets: (net, polygon)
F_ZONES = [
    ("BATP", rect(1.0, 6.4, 26.4, 15.0)),
    ("BATN", rect(8.0, 15.8, 26.4, 23.0)),
    ("VIN", [(28.4, 6.0), (34.6, 6.0), (34.6, 44.8), (20.0, 44.8), (20.0, 39.4), (28.4, 39.4)]),
    ("SRV_F", rect(38.4, 15.4, 56.4, 25.4)),
    ("SERVOP", rect(64.4, 1.4, 85.6, 38.6)),
]
# (layer, width, net, points): the SERVO+ stubs from the pour to each servo header's pin 2
TRACKS = [(pcbnew.F_Cu, 1.5, "SERVOP", [(85.0, y - 2.5), (88.0, y - 2.5)]) for y in (9.0, 20.0, 31.0, 42.0)]
# the balance header's ends: B+ (J2.4) straight up into the BATP pour; B- (J2.1) on the bottom to J1's B- pad
# the servo data bus on the bottom, OUTSIDE the header column, so the headers' GND pins (pin 3) meet the
# plane directly and the servo return current does not detour around a cut
TRACKS += [(pcbnew.B_Cu, 0.5, "SERVO_DATA", [(88.0, 53.0), (89.9, 53.0), (89.9, 9.0), (88.0, 9.0)])]
TRACKS += [(pcbnew.B_Cu, 0.5, "SERVO_DATA", [(88.0, y), (89.9, y)]) for y in (20.0, 31.0, 42.0)]
# the two transistors' emitters tied to J12's GND pin on the top, so their ground does not depend on
# whichever patch of the bottom plane the signal traces leave around them
TRACKS += [(pcbnew.F_Cu, 0.6, "GND", [(57.0, 45.5), (57.5, 51.5), (56.0, 61.0)])]
# (J2.4 up the left side, past the BATN pour, into the BATP pour; W5 across to it)
TRACKS += [(pcbnew.F_Cu, 1.0, "BATP", [(4.0, 36.0), (4.0, 14.4)]),
           (pcbnew.F_Cu, 0.6, "BATP", [(11.0, 34.5), (4.0, 34.5)]),
           (pcbnew.B_Cu, 0.8, "BATN", [(4.0, 43.5), (6.6, 43.5), (6.6, 19.0), (16.1, 19.0)])]
POWER_NETS = ("BATP", "BATN", "VIN", "SRV_F", "SERVOP", "GND")
WIDE_NETS = ("LOGIC", "COIL_HI", "COIL_LO")


def load_netlist(path):
    net = S.parse(open(path).read())[0]
    comps = {}
    for comp in S.find_all(S.find(net, "components"), "comp"):
        ref = S.find(comp, "ref")[1]
        fields = {S.find(f, "name")[1]: (f[2] if len(f) > 2 else "")
                  for f in S.find_all(S.find(comp, "fields") or [], "field")}
        comps[ref] = dict(value=S.find(comp, "value")[1], fp=S.find(comp, "footprint")[1],
                          uuid=S.find(comp, "tstamps")[1], pins={},
                          datasheet=(S.find(comp, "datasheet") or ["", "", "~"])[1],
                          desc=(S.find(comp, "description") or ["", ""])[1], mpn=fields.get("MPN", ""))
    for n in S.find_all(S.find(net, "nets"), "net"):
        name = S.find(n, "name")[1]   # as the schematic names it, e.g. "/SERVOP"
        for node in S.find_all(n, "node"):
            ref, pin = S.find(node, "ref")[1], S.find(node, "pin")[1]
            if ref in comps:
                comps[ref]["pins"][pin] = name
    return comps


MODEL3D = "${KIPRJMOD}/power-lite.3dshapes/"


WIRES = {"W1": "wire_heavy_black", "W2": "wire_heavy_black", "W6": "wire_heavy_red", "W7": "wire_heavy_red",
         "W3": "wire_tap_blue", "W4": "wire_tap_green", "W5": "wire_tap_red"}


def set_3d_models(ref, fp):
    """The 3D viewer's picture: the stock LED model is red, this KiCad has no Mini-SPOX model, the
    holders' blade fuses are not footprints, and the W pads take wires (bodies from tools/gen_3d.py).
    Models() hands back copies, so a model is replaced by clearing the list and adding a new one."""
    def only(filename):
        ms = fp.Models()
        ms.clear()
        m = pcbnew.FP_3DMODEL()
        m.m_Filename = filename
        ms.push_back(m)

    if ref == "LED1":
        only("${KICAD10_3DMODEL_DIR}/LED_THT.3dshapes/LED_D3.0mm_Green.step")
    elif ref in ("J4", "J5", "J6", "J7", "J8"):
        only(MODEL3D + "Molex_SPOX_5267-03A_body.step")
    elif ref in WIRES:
        only(MODEL3D + WIRES[ref] + ".step")
    elif ref in ("F1", "F2"):
        m = pcbnew.FP_3DMODEL()
        m.m_Filename = MODEL3D + ("MINI_fuse_15A.step" if ref == "F1" else "MINI_fuse_3A.step")
        fp.Models().push_back(m)


def check_courtyards(board):
    boxes = []
    for fp in board.GetFootprints():
        c = fp.GetCourtyard(pcbnew.F_CrtYd)
        if c.OutlineCount():
            boxes.append((fp.GetReference(), c.BBox()))
    bad = 0
    for i, (a, ba) in enumerate(boxes):
        for b, bb in boxes[i + 1:]:
            if ba.Intersects(bb):
                print(f"courtyard overlap: {a} {b}")
                bad += 1
    edge = pcbnew.BOX2I(P(0, 0), pcbnew.VECTOR2I(MM(W), MM(H)))
    for ref, bx in boxes:
        if not edge.Contains(bx) and ref != "J1":
            print(f"courtyard past the edge: {ref}")
            bad += 1
    print(f"courtyard check: {bad} problems")


def main():
    comps = load_netlist(sys.argv[1])
    missing = sorted(set(comps) - set(PLACE))
    assert not missing, f"unplaced: {missing}"
    board = pcbnew.NewBoard(BOARD)
    board.SetCopperLayerCount(2)
    ds = board.GetDesignSettings()
    ds.m_MinClearance = MM(0.2)
    ds.m_TrackMinWidth = MM(0.2)
    ds.m_ViasMinSize = MM(0.8)
    ds.m_ViasMinAnnularWidth = MM(0.15)   # PCBWay's minimum ring
    ds.m_MinThroughDrill = MM(0.4)
    ds.m_CopperEdgeClearance = MM(0.5)
    ds.m_HoleClearance = MM(0.25)
    ds.m_HoleToHoleMin = MM(0.25)
    ns = ds.m_NetSettings
    dflt = ns.GetDefaultNetclass()
    dflt.SetTrackWidth(MM(0.4))
    dflt.SetClearance(MM(0.25))
    dflt.SetViaDiameter(MM(0.8))
    dflt.SetViaDrill(MM(0.4))
    for name, width in (("Power", 1.5), ("Wide", 1.0)):
        nc = pcbnew.NETCLASS(name)
        nc.SetTrackWidth(MM(width))
        nc.SetClearance(MM(0.25))   # one clearance everywhere: the router and DRC then agree
        nc.SetViaDiameter(MM(1.2))
        nc.SetViaDrill(MM(0.6))
        ns.SetNetclass(name, nc)
    for n in POWER_NETS:
        ns.SetNetclassPatternAssignment("/" + n, "Power")
    for n in WIDE_NETS:
        ns.SetNetclassPatternAssignment("/" + n, "Wide")

    nets = {}

    def net(name):
        if name not in nets:
            ni = pcbnew.NETINFO_ITEM(board, name)
            board.Add(ni)
            nets[name] = ni
        return nets[name]

    for ref, c in sorted(comps.items()):
        lib, name = c["fp"].split(":")
        path = str(PROJ / "power-lite.pretty") if lib == "power-lite" else FPROOT + lib + ".pretty"
        fp = pcbnew.FootprintLoad(path, name)
        assert fp is not None, c["fp"]
        fp.SetFPID(pcbnew.LIB_ID(lib, name))
        fp.SetReference(ref)
        fp.SetValue(c["value"])
        fp.SetPath(pcbnew.KIID_PATH("/" + c["uuid"]))
        fp.SetSheetname("/")
        fp.SetSheetfile("power-lite.kicad_sch")
        fp.SetField("Datasheet", c["datasheet"])
        fp.SetField("Description", c["desc"])
        if c["mpn"]:
            fp.SetField("MPN", c["mpn"])
        for f in ("Datasheet", "Description", "MPN"):
            if fp.HasField(f):
                fp.GetField(f).SetVisible(False)
        set_3d_models(ref, fp)
        board.Add(fp)
        x, y, rot = PLACE[ref]
        fp.SetPosition(P(x, y))
        fp.SetOrientationDegrees(rot)
        rf = fp.Reference()
        if ref.startswith("R"):          # axial, rotated 270: the reference inside the body, along it
            rf.SetPosition(P(x, y + 5.08))
            rf.SetTextAngleDegrees(90)
            rf.SetTextSize(pcbnew.VECTOR2I(MM(0.8), MM(0.8)))
            rf.SetTextThickness(MM(0.12))
        elif ref in ("J4", "J5", "J6", "J7", "J8"):   # beside each servo header, level with pin 1
            rf.SetPosition(P(x + 3.6, y))
            rf.SetTextAngleDegrees(90)
        elif ref in REF_AT:
            rf.SetPosition(P(*REF_AT[ref]))
            rf.SetTextAngleDegrees(0)
        elif ref in ("W1", "W2", "W6", "W7"):   # the silk labels "BMS B-", "BMS P-" and "SW" name these
            rf.SetVisible(False)
        for pad in fp.Pads():
            n = c["pins"].get(pad.GetNumber())
            if n:
                pad.SetNet(net(n))

    for i, (x, y) in enumerate(HOLES, 1):
        h = pcbnew.FootprintLoad(FPROOT + "MountingHole.pretty", "MountingHole_2.7mm_M2.5")
        h.SetFPID(pcbnew.LIB_ID("MountingHole", "MountingHole_2.7mm_M2.5"))
        h.SetReference(f"H{i}")
        h.Reference().SetVisible(False)
        h.SetValue("M2.5")
        h.SetAttributes(h.GetAttributes() | pcbnew.FP_BOARD_ONLY | pcbnew.FP_EXCLUDE_FROM_BOM
                        | pcbnew.FP_EXCLUDE_FROM_POS_FILES)
        board.Add(h)
        h.SetPosition(P(x, y))

    for (x0, y0), (x1, y1) in (((0, 0), (W, 0)), ((W, 0), (W, H)), ((W, H), (0, H)), ((0, H), (0, 0))):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(P(x0, y0))
        s.SetEnd(P(x1, y1))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(MM(0.1))
        board.Add(s)

    def zone(layer, netname, pts, prio=0, conn=pcbnew.ZONE_CONNECTION_FULL, name=""):
        z = pcbnew.ZONE(board)
        z.SetLayer(layer)
        z.SetNet(net(netname))
        o = z.Outline()
        o.NewOutline()
        for x, y in pts:
            o.Append(MM(OX + x), MM(OY + y))
        z.SetAssignedPriority(prio)
        z.SetPadConnection(conn)
        z.SetLocalClearance(MM(0.4))
        z.SetMinThickness(MM(0.3))
        z.SetZoneName(name)
        board.Add(z)

    # the bottom GND plane is added after routing (post_pcb.py), so the router can use the bottom layer
    for n, pts in F_ZONES:
        zone(pcbnew.F_Cu, "/" + n, pts, prio=1, name=f"{n} pour")
    for layer, width, n, pts in TRACKS:
        for a, b in zip(pts, pts[1:]):
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(P(*a))
            t.SetEnd(P(*b))
            t.SetWidth(MM(width))
            t.SetLayer(layer)
            t.SetNet(net("/" + n))
            t.SetLocked(True)
            board.Add(t)

    board.BuildConnectivity()
    check_courtyards(board)
    pcbnew.SaveBoard(BOARD, board)
    print("footprints", len(comps), "nets", len(nets))


if __name__ == "__main__":
    main()
