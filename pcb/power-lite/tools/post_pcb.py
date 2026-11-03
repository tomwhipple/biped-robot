"""After Freerouting (run with KiCad's python): import the session, add the silkscreen labels, fill the
pours and save. usage: post_pcb.py <board> <session.ses>"""
import sys

import pcbnew

MM = pcbnew.FromMM
OX, OY = 100.0, 60.0
BOARD, SES = sys.argv[1], sys.argv[2]
LABELS = [  # text, x, y, size, back side
    ("BIPED POWER LITE rev A", 48.0, 2.2, 1.0, False),
    ("PACK", 2.0, 24.6, 1.0, False), ("BAL", 1.2, 31.4, 1.0, False),
    ("BMS B-", 22.0, 3.6, 0.8, False), ("BMS P-", 28.6, 3.6, 0.8, False), ("SW", 20.4, 26.6, 0.8, False),
    ("15A", 34.0, 13.4, 1.0, False), ("3A", 34.0, 29.0, 1.0, False),
    # every part is on top, so the connector legends go on the back, in the pad-free areas;
    # mirrored text runs left from its anchor as seen from the front
    ("J2 BAL 1=B- 4=B+", 30.0, 48.0, 0.8, True), ("W1 BMS B-  W2 BMS P-", 30.0, 50.0, 0.8, True),
    ("W3 B3  W4 B2  W5 B1+", 30.0, 52.0, 0.8, True), ("W6 W7 MAIN SWITCH", 30.0, 54.0, 0.8, True),
    ("J3 TO GD H1 (1=+)", 24.0, 56.0, 0.8, True), ("J11 E-STOP NC", 24.0, 57.8, 0.8, True),
    ("J4 R LEG  J5 R UPPER  J6 L LEG", 63.0, 3.0, 0.8, True),
    ("J7 L UPPER  J8 DATA IN (D V G)", 63.0, 5.0, 0.8, True),
    ("F1 15A SERVO  F2 3A LOGIC", 63.0, 7.0, 0.8, True),
    ("J12 2 SRV_EN", 64.0, 50.0, 0.8, True), ("3 GND 5 RAIL_V", 64.0, 52.0, 0.8, True),
    ("6 ESTOP_N", 64.0, 54.0, 0.8, True),
]

W, H = 97.0, 64.5
SOLID_GND = {("J3", "2"), ("J11", "2"), ("J12", "3")}   # small connector pads: a thermal would starve

board = pcbnew.LoadBoard(BOARD)
assert pcbnew.ImportSpecctraSES(board, SES), "SES import failed"
gnd = board.FindNet("/GND")
z = pcbnew.ZONE(board)
z.SetLayer(pcbnew.B_Cu)
z.SetNet(gnd)
o = z.Outline()
o.NewOutline()
for x, y in ((0.5, 0.5), (W - 0.5, 0.5), (W - 0.5, H - 0.5), (0.5, H - 0.5)):
    o.Append(MM(OX + x), MM(OY + y))
z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
z.SetLocalClearance(MM(0.4))
z.SetMinThickness(MM(0.3))
z.SetZoneName("GND plane")
board.Add(z)
for fp in board.GetFootprints():
    for pad in fp.Pads():
        if (fp.GetReference(), pad.GetNumber()) in SOLID_GND:
            pad.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
for text, x, y, size, back in LABELS:
    t = pcbnew.PCB_TEXT(board)
    t.SetText(text)
    t.SetPosition(pcbnew.VECTOR2I(MM(OX + x), MM(OY + y)))
    t.SetLayer(pcbnew.B_SilkS if back else pcbnew.F_SilkS)
    t.SetTextSize(pcbnew.VECTOR2I(MM(size), MM(size)))
    t.SetTextThickness(MM(0.15))
    if back:
        t.SetMirrored(True)   # read from the back; mirrored text is right-aligned to its anchor
        t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_LEFT)
    else:
        t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_LEFT)
    board.Add(t)
filler = pcbnew.ZONE_FILLER(board)
filler.Fill(board.Zones())
board.BuildConnectivity()
pcbnew.SaveBoard(BOARD, board)
print("session imported, labels added, pours filled")
