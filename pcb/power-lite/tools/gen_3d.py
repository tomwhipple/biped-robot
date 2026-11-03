"""Simple 3D bodies for the two custom footprints, so KiCad's 3D viewer shows a populated board.

Outlines from the makers' drawings, in KiCad model coordinates (x right, y UP = -footprint y, z up from the
board top): the Amass XT60PW-M body (18.2 x 15.5 x 8.4 mm, datasheet V1.2) and the TE Power Relay F7 cover
(26.2 x 26.2 x 25.2 mm max, TE V23134). usage: python3 tools/gen_3d.py   (from pcb/power-lite)
"""
from pathlib import Path

from build123d import Box, Color, Cylinder, Location, Pos, export_step

OUT = Path(__file__).resolve().parent.parent / "power-lite.3dshapes"
OUT.mkdir(exist_ok=True)


def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


# XT60PW-M: the footprint's F.Fab outline is x -4.15..11.35, y (footprint) -16.35..1.85; the mating face is the
# straight edge at footprint y -16.35 (model y +16.35); two gold pins stand in the face
xt = box(-4.15, 11.35, -1.85, 16.35, 0.0, 8.4)
xt.color = Color(0.95, 0.75, 0.1)
pins = [Location((x, 16.35, 4.2), (90, 0, 0)) * Cylinder(1.75, 1.0) for x in (0.0, 7.2)]
for p in pins:
    p.color = Color(0.85, 0.65, 0.2)
xt_parts = [xt] + pins
from build123d import Compound  # noqa: E402

export_step(Compound(xt_parts), OUT / "AMASS_XT60PW-M_body.step")

# TE F7 relay: cover 26.2 x 26.2, 87 row 2.7 from the lower edge -> footprint y -23.5..2.7 (model y -2.7..23.5),
# 25.2 tall, on 1.5 mm standoffs
rl = box(-13.1, 13.1, -2.7, 23.5, 1.5, 25.2 + 1.5)
rl.color = Color(0.12, 0.12, 0.12)
export_step(rl, OUT / "TE_PowerRelay_F7_body.step")
print("written:", sorted(p.name for p in OUT.glob("*.step")))
