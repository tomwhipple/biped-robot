# General Driver for Robots: manufacturer documents

These are Waveshare's documents for the robot's controller board, mirrored
verbatim, plus the datasheets for two parts on it. Waveshare publishes **no user
manual** for this board. The connector table, the FAQ and the demo code exist only
on the wiki. Its stable permalink is
<https://www.waveshare.com/w/index.php?title=General_Driver_for_Robots&oldid=103968>,
retrieved 2026-08-03.

| file | what it is | source |
|---|---|---|
| [General_Driver_for_Robots-schematic.pdf](General_Driver_for_Robots-schematic.pdf) | the full schematic, one A3 sheet. It is the authority for every net and GPIO | [files.waveshare.com](https://files.waveshare.com/upload/3/37/General_Driver_for_Robots.pdf) |
| [General_Driver_for_Robots-dimensions.pdf](General_Driver_for_Robots-dimensions.pdf) | dimension drawing: outline **65.01 × 56.01 mm**, holes on a 58 × 49 mm grid (58 along the 65 edge) | Waveshare |
| [General_Driver_for_Robots-connector-diagram.jpg](General_Driver_for_Robots-connector-diagram.jpg) | the official annotated photo, with every connector numbered 1–27, front and back | Waveshare wiki |
| [JST-XH-connector-datasheet.pdf](JST-XH-connector-datasheet.pdf) | JST XH series, the family of the power inlet: 3 A AC/DC per contact at AWG #22, conductors AWG #30–#22 | [jst-mfg.com](https://www.jst-mfg.com/product/pdf/eng/eXH.pdf) |
| [General_Driver_for_Robots_STEP.zip](General_Driver_for_Robots_STEP.zip) | Waveshare's 3D model, one STEP (AP214, Autodesk export, 2023-01-13): the PCB outline with its holes and 145 placed parts by reference designator. `cad/v6/gd_plate.py` reads it | [files.waveshare.com](https://files.waveshare.com/upload/8/8e/General_Driver_for_Robots_STEP.zip), listed on the wiki, retrieved 2026-10-24 |
| [GENERAL-DRIVER-FOR-ROBOTS-STR-DXF.zip](GENERAL-DRIVER-FOR-ROBOTS-STR-DXF.zip) | the dimension drawing as DXF | [files.waveshare.com](https://files.waveshare.com/upload/5/50/GENERAL-DRIVER-FOR-ROBOTS-STR-DXF.zip), retrieved 2026-10-24 |
| [QMI8658C-datasheet.pdf](QMI8658C-datasheet.pdf) | QST QMI8658C 6-axis IMU, the robot's IMU at 0x6B. Rev 0.6, 2021-01-13, stamped "advance information" | QST, via Waveshare |

Where the documents disagree:

- **Dimensions.** The wiki lists the board as "65 x 65mm"; the drawing dimensions 56.01 across the holes' edge plus an 8.99 × 52.51 tab past it that carries both 2 × 20 headers. The STEP's outline agrees with the drawing: 65.0 × 65.01 overall, the holes Ø3.0 on 49 × 58.
- **What the STEP leaves out.** The XH power inlet H1 is not in it. The two 2 × 20 sockets are drawn as pins only (9.0 mm under the PCB). Its PCB is 0.41 mm thick; the real board's thickness is not measured.
- **Input voltage.**
  - The wiki says "DC 7-13V" and that the XH port "directly powers the serial bus servo and motor".
  - The rev 1.2 silkscreen says "DC 9-12.6V", which would exclude 2S.
  - This is unresolved. A 3S pack is inside both ranges.
- **Bus current.** The wiki FAQ limits bus-servo current to "5A for long-term operation", which is "up to 5 servos (ensuring that these 5 servos are not stalled simultaneously)". The robot runs 17. That is open; see [wiring.md](../../wiring.md#current-the-open-constraint).
- **QMI8658C output registers.** The datasheet's Motion-on-Demand text cites output registers 0x25–0x3D, while its dQ/dV register map puts them at 0x49–0x56. Trust the silicon, not the sheet (`imu ae` on the CLI).

The connector → net → GPIO map derived from the schematic is in
[sensor-expansion.md §1](../../sensor-expansion.md#1-the-boards-connectors). What the
INA219 measures is in [wiring.md](../../wiring.md#what-the-ina219-measures).
