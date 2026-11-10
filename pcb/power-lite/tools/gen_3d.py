"""Simple 3D bodies for parts whose models this KiCad install lacks, so its 3D viewer shows the board populated.

KiCad model coordinates: x right, y UP (= -footprint y), z up from the board's top.
- Amass XT60PW-M: 18.2 x 15.5 x 8.4 mm (Amass datasheet V1.2), on the footprint's F.Fab outline.
- TE Power Relay F7 cover: 26.2 x 26.2 x 25.2 mm max (TE V23134).
- Molex 22-03-5035 (Mini-SPOX 5267-03A): the footprint's 9.9 x 4.9 mm outline; the 5 mm height is
  approximate (no Molex drawing in the repo).
- Littelfuse MINI blade fuses in the Keystone 3568 holders (whose stock model tops out at 7.37 mm): a body
  10.9 x 3.8 mm above the holder, in the standard colours, violet 3 A and light blue 15 A.
usage: python3 tools/gen_3d.py   (from pcb/power-lite)
"""
from pathlib import Path

from build123d import Box, Color, Cylinder, Pos, export_step

OUT = Path(__file__).resolve().parent.parent / "power-lite.3dshapes"
OUT.mkdir(exist_ok=True)


def box(x0, x1, y0, y1, z0, z1, color):
    b = Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)
    b.color = Color(*color)
    return b


BODIES = {   # file: (x0, x1, y0, y1, z0, z1, rgb); one solid each, so the colour survives the export
    "AMASS_XT60PW-M_body.step": (-4.15, 11.35, -1.85, 16.35, 0.0, 8.4, (0.95, 0.75, 0.10)),
    "TE_PowerRelay_F7_body.step": (-13.1, 13.1, -2.7, 23.5, 1.5, 26.7, (0.12, 0.12, 0.12)),
    "Molex_SPOX_5267-03A_body.step": (-2.45, 7.45, -1.8, 3.1, 0.0, 5.0, (0.93, 0.93, 0.90)),
    "MINI_fuse_3A.step": (-0.49, 10.41, -3.6, 0.2, 7.4, 15.4, (0.55, 0.30, 0.80)),
    "MINI_fuse_15A.step": (-0.49, 10.41, -3.6, 0.2, 7.4, 15.4, (0.45, 0.72, 0.95)),
}
for name, (x0, x1, y0, y1, z0, z1, rgb) in BODIES.items():
    export_step(box(x0, x1, y0, y1, z0, z1, rgb), OUT / name)

# wire stubs on the W pads (BMS and main-switch leads): 14 AWG power leads, thin cell-tap leads
WIRES = {"wire_heavy_black": (1.45, (0.08, 0.08, 0.08)), "wire_heavy_red": (1.45, (0.80, 0.10, 0.10)),
         "wire_tap_blue": (0.6, (0.15, 0.35, 0.85)), "wire_tap_green": (0.6, (0.15, 0.65, 0.25)),
         "wire_tap_red": (0.6, (0.80, 0.10, 0.10))}
for name, (r, rgb) in WIRES.items():
    w = Pos(0, 0, 6.0) * Cylinder(r, 12.0)
    w.color = Color(*rgb)
    export_step(w, OUT / f"{name}.step")
print("written:", sorted(BODIES) + sorted(WIRES))
