"""Figure: the driver board's SERVICE EDGE and the cuts that free it.

Run:  .venv/bin/python cad/render_service_access.py
      -> cad/renders/service_access.png

What it shows, on the +y side of the v6 torso (BR_SVC_* in dimensions.py):

  grey    pelvis, with the WINDOW / RELIEF / EAR NOTCH cut into the +y cheek
  dark    the General Driver PCB standing in its recess, INCLUDING the two
          USB-C shells that stand BOARD_GD_SHELL_PROUD off the service edge
  orange  what is mated to that edge in service -- the USB-C flashing lead and
          the XH battery pigtail, as their plug bodies

The point of the figure is that the orange bodies pass THROUGH printed plastic
that used to be solid: before 2026-08-06 the cheek stood 0.41 mm inside the
shells and squarely in front of both plugs.
"""
import os
import shutil

from build123d import Pos

import check_assembly as CA
import dimensions as D
import parts
from export_assembly import piece
from render_electronics_steps import render

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "renders")
TMP = os.path.join(HERE, "renders", "_svc")
os.makedirs(OUT, exist_ok=True)

PRINT = (0.80, 0.82, 0.86, 1)
PCB = (0.09, 0.28, 0.16, 1)
COMP = (0.28, 0.29, 0.32, 1)
ORANGE = (0.93, 0.46, 0.10, 1)


def main():
    ch = [
        piece("pelvis", PRINT, parts.pelvis()),
        piece("board_pcb", PCB, Pos(0, 0, 0) * CA.board_pcb_mock()),
        piece("board_comp", COMP, Pos(0, 0, 0) * CA.board_comp_mock()),
        piece("service_plugs", ORANGE, Pos(0, 0, 0) * CA.board_plugs_mock()),
    ]
    # Azimuth 270 = camera on +y looking toward -y (az 0 faces the board's aft
    # face), so the service side is the 250..300 band. EVERY VIEW LOOKS UP
    # (positive elevation = camera below): the deck EAR reaches out to
    # y = DECK_L/2 while the cheek stops at BR_CHEEK_Y_OUT, so it overhangs the
    # window by 15 mm and hides the whole fix from anywhere above the deck
    # plane. ~25 deg is the shallowest angle that clears it -- which is also
    # what a hand holding a USB cable has to reach past, so the camera angle
    # here is the honest one rather than a flattering one.
    lookat = (-0.038, 0.018, -0.012)
    render(ch, os.path.join(OUT, "service_access.png"),
           [(258, 24), (275, 32), (296, 20)], lookat=lookat, dist=0.200, px=760)
    shutil.rmtree(TMP, ignore_errors=True)


if __name__ == "__main__":
    main()
