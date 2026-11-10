"""Export the router's view of the board (run with KiCad's python): pours shrunk so routes end inside the
real fill, and GND taken out of the netlist, since every through-hole GND pad lands on the bottom plane.
usage: export_dsn.py <board> <out.dsn>"""
import re
import sys

import pcbnew

SHRINK = 0.45

b = pcbnew.LoadBoard(sys.argv[1])
for z in b.Zones():
    z.Outline().Deflate(pcbnew.FromMM(SHRINK), pcbnew.CORNER_STRATEGY_CHAMFER_ALL_CORNERS, pcbnew.FromMM(0.01))
assert pcbnew.ExportSpecctraDSN(b, sys.argv[2])
text = open(sys.argv[2]).read()
m = re.search(r'\(net "?/GND"?\s*\(pins ([^)]*)\)', text)
text = text[:m.start(1)] + text[m.end(1):]
# the router rounds to its own grid: 10 um more clearance than KiCad checks, so its tracks pass DRC
text = re.sub(r"\(clearance (\d+(?:\.\d+)?)", lambda mm: f"(clearance {float(mm.group(1)) + 10:g}", text)
open(sys.argv[2], "w").write(text)
print("DSN written; GND left to the bottom plane")
