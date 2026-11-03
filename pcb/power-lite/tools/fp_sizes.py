"""Print each footprint's courtyard box (relative to its origin, unrotated) for placement planning.
Run with KiCad's python: <kicad python> tools/fp_sizes.py   (from pcb/power-lite)"""
import sys
from pathlib import Path

import pcbnew

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import parts as PD  # noqa: E402

FPROOT = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints/"
seen = {}
for ref, d in PD.parts.items():
    lib, name = d["fp"].split(":")
    if name in seen:
        print(f"{ref:5s} = {seen[name]}")
        continue
    path = str(HERE.parent / "power-lite.pretty") if lib == "power-lite" else FPROOT + lib + ".pretty"
    fp = pcbnew.FootprintLoad(path, name)
    bb = fp.GetCourtyard(pcbnew.F_CrtYd).BBox() if fp.GetCourtyard(pcbnew.F_CrtYd).OutlineCount() else fp.GetBoundingBox()
    x0, y0 = pcbnew.ToMM(bb.GetX()), pcbnew.ToMM(bb.GetY())
    w, h = pcbnew.ToMM(bb.GetWidth()), pcbnew.ToMM(bb.GetHeight())
    pads = [(p.GetNumber(), round(pcbnew.ToMM(p.GetPosition().x), 2), round(pcbnew.ToMM(p.GetPosition().y), 2))
            for p in fp.Pads()]
    seen[name] = ref
    print(f"{ref:5s} {name}: box x {x0:.2f}..{x0 + w:.2f}, y {y0:.2f}..{y0 + h:.2f} ({w:.1f} x {h:.1f}); pads {pads[:4]}")
