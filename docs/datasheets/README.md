# Datasheets

These are manufacturer documents for the parts in the build. They are mirrored
here so that CAD work and bring-up do not depend on a live listing. The owner's
policy: only source parts that have real manufacturer documentation, meaning
connector graphics and specs. A rendered web snapshot or a self-authored "manual"
does not count.

## Feetech STS3215 (ST-3215-C018): every joint

The files are in [`st3215/`](st3215/). The part to buy is the **12 V class,
ST-3215-C018** (30 kg·cm).

- The 7.4 V STS3215 shares the case but cannot run on 3S.
- Resellers (Waveshare, RCmall, Stemedu …) sell the same Feetech part, so buy by voltage class.

| file | what it is | source |
|---|---|---|
| `ST-3215-C018-datasheet-12V-30kg.pdf` | **The authoritative spec.** Ed. A/0, 2023-07-20, 8 pp. Chinese with English headings. §5 electrical, §6 mechanical, §7 control and protections, §9 outline, §10 performance curves | [feetechrc.com](https://www.feetechrc.com/525603.html) |
| `ST-3215-C018-outline-and-performance.png` | rendered quick-look of the outline and performance pages | rendered locally |
| `ST3215-outline-drawing.pdf` / `.dxf` / `.png` | standalone dimensioned drawing, 3 views plus mounting holes. The title block says "SCS215", the same case family; every dimension matches the C018 outline | Waveshare [ST3215-2D.zip](https://files.waveshare.com/upload/0/08/ST3215-2D.zip) |
| `ST3215-Servo-User-Manual-EN.pdf` | Waveshare's ST3215 user manual. It is mostly about Waveshare's Servo Driver board and its web UI, and has **no register map** | [Waveshare wiki](https://www.waveshare.com/wiki/ST3215_Servo) |
| `STS3215-datasheet-7.4V-19kg-EN-machine-translated.pdf` | The **7.4 V** part, machine-translated. Wrong voltage class. Never read torque or current from it | [Core Electronics](https://core-electronics.com.au/attachments/uploads/sts3215-smart-servo-datasheet-translated.pdf) |
| `../../cad/vendor/ST3215.step` | 3D solid, bounding box 45.22 × 37.8 × 24.72 mm | Waveshare [ST3215-3D.zip](https://files.waveshare.com/upload/5/59/ST3215-3D.zip) |

The C018's key numbers:

- 12 V rated, 4–14 V input range (protection trips outside that range).
- 30 kg·cm stall (2.94 N·m) at 2.7 A.
- 180 mA running with no load; 30 mA idle.
- 0.222 s/60° with no load.
- Backlash ≤ 0.5°; gear ratio 1:345.
- 55 ± 1 g.
- 25T output spline, OD 5.9; M3×6 horn screw.
- 5264-3P connector (1 GND, 2 V+, 3 signal) on a 150 mm lead.
- Bus at 38.4 kbps–1 Mbps; factory default 1 Mbps, ID 1.

The Waveshare wiki gives the supply as 6–12.6 V.

The drawing's dimensions, in mm:

- Case: 45.22 × 24.72; 35 overall across the output axis.
- Horn face to idler-disc face: **37.25**.
- Output axis: 10.11 from the output end, centred across the width.
- Horn and idler discs: Ø19.2, on a Ø14 4 × M3 bolt circle.
- Case bosses: ±10.25 across the width, in rows 8.30 and 29.00 behind the output axis.

`cad/dimensions.py` states the same numbers. The drawing is the receipt for them.

## Waveshare General Driver for Robots: the controller

The files are in [`general-driver/`](general-driver/), which has its own
[index](general-driver/README.md). The set is:

- the schematic;
- the dimension drawing;
- the annotated connector photo;
- the JST XH datasheet;
- the QMI8658C IMU datasheet.

## Not mirrored yet

| document | source | why |
|---|---|---|
| Feetech ST3215 memory table v3.7 (register map, defaults) | [xls](https://files.waveshare.com/upload/2/27/ST3215%20memory%20register%20map-EN.xls), cited in `firmware/components/scsbus/include/scsbus/registers.h` | the register reference behind [servo-map.md §4](../servo-map.md#4-registers) |
| Feetech communication protocol manual | [pdf](https://files.waveshare.com/upload/2/27/Communication_Protocol_User_Manual-EN(191218-0923).pdf) | packet format; the source of the `scsbus` golden packets |

## To download

These parts are in the build, but their manufacturer documents are not in the
repo yet:

| part | manufacturer documents |
|---|---|
| Raspberry Pi 4 Model B | [product brief](https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-product-brief.pdf), [datasheet](https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-datasheet.pdf), [mechanical drawing](https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-mechanical-drawing.pdf) |
| Raspberry Pi Camera Module 3 (Wide) | [product brief](https://datasheets.raspberrypi.com/camera/camera-module-3-product-brief.pdf) |
| Pololu D24V50F5, 5 V / 5 A buck | [product page](https://www.pololu.com/product/2851), with its dimension diagram. The pelvis bosses wait on the hole pattern |
| 3S LiPo pack | the chosen pack's manufacturer spec sheet: dimensions, capacity, C rating, connector |
| 3S protection board | the chosen board's manufacturer datasheet: continuous current, cut-off voltages, balance-lead pinout |
