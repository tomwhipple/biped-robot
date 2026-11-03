"""For the assembly figures (run with KiCad's python): save a copy of the board with the 3D models of every
footprint outside <keep> removed, and write each footprint's courtyard and pads as JSON.
usage: asm_strip.py <board> <out board> <out json> [ref ...]"""
import json
import sys

import pcbnew

OX, OY = 100.0, 60.0
board, out, out_json, keep = sys.argv[1], sys.argv[2], sys.argv[3], set(sys.argv[4:])
b = pcbnew.LoadBoard(board)
geom = {}
for fp in b.GetFootprints():
    ref = fp.GetReference()
    if ref not in keep:
        fp.Models().clear()
    else:   # the blade fuses go in at bring-up, not during assembly
        kept = [m for m in fp.Models() if "MINI_fuse" not in m.m_Filename]
        fp.Models().clear()
        for m in kept:
            fp.Models().push_back(m)
    cy = fp.GetCourtyard(pcbnew.F_CrtYd).BBox()
    geom[ref] = {
        "crtyd": [pcbnew.ToMM(cy.GetX()) - OX, pcbnew.ToMM(cy.GetY()) - OY,
                  pcbnew.ToMM(cy.GetRight()) - OX, pcbnew.ToMM(cy.GetBottom()) - OY],
        "pads": {pd.GetNumber(): [pcbnew.ToMM(pd.GetPosition().x) - OX, pcbnew.ToMM(pd.GetPosition().y) - OY]
                 for pd in fp.Pads() if pd.GetNumber()},
        "value": fp.GetValue(),
    }
pcbnew.SaveBoard(out, b)
with open(out_json, "w") as f:
    json.dump(geom, f)
