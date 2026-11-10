"""Print one footprint's pads and its F.Fab / F.SilkS / F.CrtYd geometry in board mm (origin at the board's
top-left corner). usage: <kicad python> tools/fp_probe.py <board> <ref>"""
import sys

import pcbnew

OX, OY = 100.0, 60.0
b = pcbnew.LoadBoard(sys.argv[1])
fp = b.FindFootprintByReference(sys.argv[2])


def mm(p):
    return round(pcbnew.ToMM(p.x) - OX, 2), round(pcbnew.ToMM(p.y) - OY, 2)


print("origin", mm(fp.GetPosition()), "rot", fp.GetOrientationDegrees())
for pad in fp.Pads():
    print("pad", pad.GetNumber() or "-", mm(pad.GetPosition()), pad.GetNetname())
for g in fp.GraphicalItems():
    layer = b.GetLayerName(g.GetLayer())
    if layer in ("F.Fab", "F.SilkS", "F.Courtyard", "F.CrtYd") and hasattr(g, "GetStart"):
        print(layer, g.GetShapeStr() if hasattr(g, "GetShapeStr") else "", mm(g.GetStart()), mm(g.GetEnd()))
bb = fp.GetCourtyard(pcbnew.F_CrtYd).BBox()
print("courtyard x", round(pcbnew.ToMM(bb.GetX()) - OX, 2), "..", round(pcbnew.ToMM(bb.GetRight()) - OX, 2),
      "y", round(pcbnew.ToMM(bb.GetY()) - OY, 2), "..", round(pcbnew.ToMM(bb.GetBottom()) - OY, 2))
