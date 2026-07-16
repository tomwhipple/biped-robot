"""Electronics-mounting figures for docs/assembly.md section 9.

Run:  .venv/bin/python cad/render_electronics_steps.py
      -> docs/assembly/step08a_board.png   (driver board + 4x M2.5, bench frame)
      -> docs/assembly/step08b_imu.png     (BNO085 tape-mount on the top plate)

step08a is drawn in BENCH frame: the tower sits upside down on its top plate
(exactly how it comes off the printer), the board drops in components-UP
(= face-DOWN once the tower is righted), and the four M2.5 x 8 self-tappers
(ORANGE) drive down through the corner holes into the standoffs.

step08b is the robot frame: the BNO085 (ORANGE) foam-tapes onto the top
plate at the REAR (-y), long axis on y, 3 mm proud of the rear edge -- the
only clear 25.4 x 17.8 footprint on the printed tower (the interior strips
are all <13 mm; measured 2026-07-16). Its cable drops over the open rear
end straight to the board's GPIO headers.
"""
import os
import shutil
import numpy as np
import mujoco
import imageio.v2 as imageio
from build123d import (Box, Compound, Cylinder, Pos, Rot, export_stl)

import dimensions as D
import parts
from export_assembly import piece

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "docs", "assembly")
TMP = os.path.join(HERE, "renders", "_elec")
os.makedirs(OUT, exist_ok=True)
os.makedirs(TMP, exist_ok=True)

ORANGE = (0.93, 0.46, 0.10, 1)
PRINT = (0.80, 0.82, 0.86, 1)
PCB = (0.10, 0.11, 0.13, 1)
OLED = (0.15, 0.35, 0.75, 1)
PART = (0.30, 0.31, 0.34, 1)
TAPE = (0.35, 0.35, 0.35, 1)

ZT0 = D.TOWER_H - D.TOWER_TOP_T                  # 40, plate underside
BX, BY = D.BOARD_HOLES[1] / 2, D.BOARD_HOLES[0] / 2   # standoffs (11.5, 29)


def board_pieces(dz=0.0):
    """Waveshare driver-board mock, tower-local ROBOT frame, mounted pose
    (face-DOWN: components below the PCB), shifted dz along z."""
    zt = ZT0 - D.BOARD_STANDOFF + dz             # board top face, 34
    zb = zt - 1.6                                # pcb underside, 32.4
    ch = [piece("board_pcb", PCB, Pos(0, 0, zt - 0.8) * Box(30, 65, 1.6))]
    # components hang DOWN from the pcb underside
    ch.append(piece("board_oled", OLED, Pos(3, 0, zb - 2) * Box(12, 26, 4)))
    ch.append(piece("board_esp32", PART, Pos(-8, -20, zb - 1.5) * Box(9, 11, 3)))
    ch.append(piece("board_usbc", PART, Pos(-10, 26, zb - 1.6) * Box(9, 7, 3.2)))
    for sy in (24, -24):                         # 3-pin bus ports
        ch.append(piece("board_port", (0.9, 0.9, 0.82, 1),
                        Pos(8, sy, zb - 2.5) * Box(8, 10, 5)))
    ch.append(piece("board_dc", PART, Pos(10, -29, zb - 3) * Box(9, 9, 6)))
    return ch


def screw_m25(x, y, dz=0.0):
    """M2.5 x 8 self-tap, robot frame, pointing UP into a standoff (drives
    from below the board), shifted dz along z."""
    zh = ZT0 - D.BOARD_STANDOFF - 1.6 + dz       # head seat = pcb underside
    s = Pos(x, y, zh + 3) * Cylinder(1.25, 6)    # shaft up into the standoff
    s += Pos(x, y, zh - 1) * Cylinder(2.3, 2)    # pan head below
    return s


def imu_pieces(dz=0.0):
    """BNO085 breakout mock at its tape pad: top plate REAR, long axis on y,
    x -8.9..8.9, y -51..-25.6 (3 mm proud of the rear edge), robot frame."""
    zp = D.TOWER_H + 1.0 + dz                    # pcb bottom (on 1 mm tape)
    cy = -38.3                                   # pad center y
    ch = [piece("imu_pcb", ORANGE, Pos(0, cy, zp + 0.8) * Box(17.8, 25.4, 1.6))]
    ch.append(piece("imu_chip", PART, Pos(0, cy, zp + 1.6 + 0.6) * Box(5, 4.5, 1.2)))
    for e in (1, -1):                            # JST-SH jacks on the short ends
        ch.append(piece("imu_jack", PART,
                        Pos(0, cy + e * 10.5, zp + 1.6 + 1.4) * Box(7, 4, 2.8)))
    return ch


def render(children, path, views, lookat, dist, px=880):
    items = []
    for i, ch in enumerate(children):
        stl = os.path.join(TMP, f"e{i}.stl")
        export_stl(ch, stl)
        items.append((f"e{i}", stl, tuple(ch.color)[:3]))
    assets = "\n".join(f'<mesh name="{n}" file="{s}" scale="0.001 0.001 0.001"/>'
                       for n, s, _ in items)
    geoms = "\n".join(f'<geom type="mesh" mesh="{n}" contype="0" '
                      f'conaffinity="0" rgba="{c[0]} {c[1]} {c[2]} 1"/>'
                      for n, _, c in items)
    xml = f"""<mujoco>
      <visual><headlight ambient="0.5 0.5 0.52" diffuse="0.45 0.45 0.45"/>
        <global offwidth="{px}" offheight="{px}"/>
        <quality shadowsize="2048"/></visual>
      <asset><texture name="sky" type="skybox" builtin="gradient"
        rgb1="0.93 0.94 0.96" rgb2="0.80 0.83 0.88" width="256" height="256"/>
        {assets}</asset>
      <worldbody><light pos="0.4 -0.6 1.0" dir="-0.35 0.5 -0.75" castshadow="true"/>
        {geoms}</worldbody>
    </mujoco>"""
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, height=px, width=px)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.lookat[:] = lookat
    cam.distance = dist
    frames = []
    for az, el in views:
        cam.azimuth, cam.elevation = az, el
        r.update_scene(d, cam)
        frames.append(r.render().copy())
    imageio.imwrite(path, np.concatenate(frames, axis=1))
    r.close()
    print(f"wrote {path}")


def main():
    # -- step08a: board into the inverted tower (bench frame), M2.5 exploded
    # (build a fresh tower per figure: Location * shape can relocate the
    # underlying solid, so sharing one across figures leaks the flip)
    flip = Pos(0, 0, D.TOWER_H) * Rot(180, 0, 0)  # plate lands on the bench
    ch = [piece("tower", PRINT, flip * parts.tower())]
    ch += [piece(p.label, tuple(p.color), flip * p) for p in board_pieces(dz=-42)]
    for sx in (BX, -BX):
        for sy in (BY, -BY):
            ch.append(piece("screw", ORANGE, flip * screw_m25(sx, sy, dz=-68)))
    render(ch, os.path.join(OUT, "step08a_board.png"),
           [(140, -25), (90, -75)], lookat=(0, 0, 0.045), dist=0.30)

    # -- step08b: IMU tape pad on the top plate rear (robot frame)
    ch = [piece("tower", PRINT, parts.tower())]
    ch += board_pieces()                          # board already mounted
    ch.append(piece("tape", TAPE,
                    Pos(0, -38.3, D.TOWER_H + 0.5) * Box(16, 20, 1)))
    ch += imu_pieces(dz=26)
    render(ch, os.path.join(OUT, "step08b_imu.png"),
           [(215, -20), (270, -65)], lookat=(0, -0.02, 0.048), dist=0.30)
    shutil.rmtree(TMP, ignore_errors=True)


if __name__ == "__main__":
    main()
