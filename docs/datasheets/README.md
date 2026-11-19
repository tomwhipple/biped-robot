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

## E-Switch 100 series: the E-stop

The E-stop is an E-Switch **100SP1T1B1M1QEH** miniature toggle (#213); the
file is in [`switches/`](switches/).

| file | what it is | source |
|---|---|---|
| `e-switch-100-series.pdf` | 100 Series toggle switch catalog, dated 11.4.2025, 18 pp.: the part-number configurator and ratings p.1, function and body (SP1: 12.70 × 6.86 × 8.89 mm, terminal closures, keyway side) p.2, levers (T1) p.5, bushings (B1: 1/4-40, 8.89 mm long, keyway 1.02 × 0.67 mm) and panel cut-outs p.6-7, terminations (M1 solder lugs) p.8. Its hardware (H: nut and lockwasher) is not drawn | E-Switch (e-switch.com), retrieved 2026-11-04 |

## Not mirrored yet

| document | source | why |
|---|---|---|
| Feetech ST3215 memory table v3.7 (register map, defaults) | [xls](https://files.waveshare.com/upload/2/27/ST3215%20memory%20register%20map-EN.xls), cited in `firmware/components/scsbus/include/scsbus/registers.h` | the register reference behind [servo-map.md §4](../servo-map.md#4-registers) |
| Feetech communication protocol manual | [pdf](https://files.waveshare.com/upload/2/27/Communication_Protocol_User_Manual-EN(191218-0923).pdf) | packet format; the source of the `scsbus` golden packets |

## Power lite: board parts

The files are in [`power-lite/`](power-lite/). The BOM is
[`pcb/power-lite/fab/power-lite-bom.csv`](../../pcb/power-lite/fab/power-lite-bom.csv).
The ShillehTek CF-3SJH25A-C protection board is off-board. Its seller's datasheet
(below) is preliminary and publishes none of its thresholds or current rating, so it was
accepted on a bench check ([bringup.md](../../pcb/power-lite/bringup.md), Part 1).

| file | what it is (parts, revision/date, key pages) | source |
|---|---|---|
| `amass-xt60pw.pdf` | XT60PW-M/F board-mount connectors (J1, J2), spec sheet V1.2 | Amass |
| `jst-xh.pdf` | B2B/B4B-XH-A headers | JST |
| `jst-ph.pdf` | B2B/B8B-PH-K-S headers | JST |
| `keystone-3568.pdf` | MINI blade fuse holder | Keystone |
| `panasonic-eeu-fr.pdf` | C1, EEU-FR1E102 (1000 uF / 25 V). FR series radial-lead catalog sheet, 5 pp., 04 May 2013: ratings, ripple/ESR table, dimensions | Panasonic, via the [DigiKey-hosted copy](https://media.digikey.com/pdf/Data%20Sheets/Panasonic%20Electronic%20Components/EEU-FR_Series.pdf); industrial.panasonic.com timed out |
| `kemet-c322.pdf` | C2, C322C104K5R5TA (0.1 uF, 50 V, X7R, 5.08 mm). GoldMax 300 Comm X7R part sheet, 4 pp., KEMET-generated | [KEMET](https://connect.kemet.com:7667/gateway/IntelliData-ComponentDocumentation/1.0/download/specsheet/C322C104K5R5TA) |
| `goodark-1n400x.pdf` | D1, 1N4007. 1N4001 thru 1N4007 general-purpose rectifiers, 2 pp. | [Good-Ark](https://goodarksemi.com/docs/datasheets/general_purpose_rectifiers/1N400x.pdf) |
| `littelfuse-1-5ke.pdf` | D2, D3, 1.5KE16A. 1.5KE series TVS diodes, revised 11/20/15 (DigiKey F5624CT-ND copy of Littelfuse's sheet; littelfuse.com blocks scripted download) | [UTK mirror](https://web.eecs.utk.edu/~dcostine/ECE482/components/parts/PartsKit/datasheets/F5624CT-ND.pdf) |
| `molex-5267.pdf` | J4-J8, 22-03-5035 (3-circuit Mini-SPOX 5267 header). Molex's series catalog drawing, MX5267-NA-B104 (2002): features, ratings, the ordering table with dimensions A and B for 22-03-5035. It is the series sheet, not the part's own sales drawing or product spec (molex.com does not serve them to scripted clients) | Molex, via [ChipDip](https://static.chipdip.ru/lib/473/DOC047473976.pdf) |
| `te-v23134-f7.pdf` | K1, 1-1414610-0. Plug-in Maxi ISO relays, Power Relay F7 (V23134), 4 pp., 04-2013 Rev. 0413; the part is listed in the ordering table (V23134-J0052-X455) | TE, via the [RS Online copy](https://assets.rs-online.com/v1699044454/Datasheets/da2e19e2a00cf526f3d462032ef10e31.pdf); te.com returns Access Denied |
| `kingbright-wp710a10gd.pdf` | LED1, WP710A10GD (green T-1 lamp), 5 pp., Rev V.10B, 04/18/2020 | [Kingbright](https://www.kingbrightusa.com/images/catalog/SPEC/WP710A10GD.pdf) |
| `kingbright-wp710a10id.pdf` | LED2, WP710A10ID (high-efficiency red T-1 lamp), 5 pp., Rev V.12A, 08/11/2023 | [Kingbright](https://www.kingbrightusa.com/images/catalog/SPEC/WP710A10ID.pdf) |
| `onsemi-pn2222a.pdf` | Q1, Q2, PN2222A, 8 pp., Rev. 1.1.0, July 2014 (Fairchild datasheet carried by onsemi) | [onsemi](https://www.onsemi.com/pdf/datasheet/pn2222a-d.pdf) |
| `shillehtek-na405.pdf` | The BMS, ShillehTek CF-3SJH25A-C (SKU NA405). DS-NA405 rev 1.0, 2026-09-17, preliminary, 12 pp.: pads and the downward tap numbering (B1+ 11.1 V, B2 7.4 V, B3 3.7 V) §3, the sleep-until-charger note §4, wiring order (B− first) §6.2, 56 × 45 × 1.2 mm silkscreen §7. Thresholds, current rating and quiescent current are "not specified" | [ShillehTek](https://cdn.shopify.com/s/files/1/0837/4340/8415/files/shillehtek-na405-datasheet.pdf?v=1790018568), retrieved 2026-11-19 |
| `yageo-mfr.pdf` | R1-R8, MFR-25 metal film resistors, 4 pp., Revision 201304 | Yageo, via [ChipDip](https://static.chipdip.ru/lib/469/DOC013469277.pdf); yageo.com's own PDF link serves a web page |

## Raspberry Pi 4 Model B and Camera Module 3

The files are in [`raspberry-pi/`](raspberry-pi/).

| file | what it is | source |
|---|---|---|
| `rpi4-datasheet.pdf` | Raspberry Pi 4 Model B datasheet, release 1.1, March 2024, 13 pp. | [Raspberry Pi](https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-datasheet.pdf) |
| `rpi4-mechanical-drawing.pdf` | board outline and mounting holes, 1 p. | [Raspberry Pi](https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-mechanical-drawing.pdf) |
| `camera-module-3-product-brief.pdf` | Camera Module 3 (Standard, NoIR, Wide, NoIR Wide) product brief, published June 2024, 7 pp. | [Raspberry Pi](https://datasheets.raspberrypi.com/camera/camera-module-3-product-brief.pdf) |

## Pololu D24V50F5: 5 V / 5 A buck

| file | what it is | source |
|---|---|---|
| `pololu/d24v50f5-dimensions.pdf` | dimension diagram, drawing date 2 January 2018 (item 2851). The torso has no mount for it: its power bay holds power lite (#112) | [Pololu](https://www.pololu.com/file/0J1436/d24v50f5-step-down-voltage-regulator-dimensions.pdf) |

## To download

| part | manufacturer documents |
|---|---|
| Raspberry Pi 4 Model B product brief | [pdf](https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-product-brief.pdf): 21 MB, not committed |
| Molex 22-03-5035 sales drawing and product spec | [product page](https://www.molex.com/en-us/products/part-detail/22035035): molex.com does not serve the PDFs to scripted clients |
| 3S LiPo pack (OVONIC 3S 4200 mAh 130C XT60) | no maker spec sheet found; OVONIC's general [LiPo user manual](https://us.ovonicshop.com/pages/user-manual): never below 3.0 V a cell under load |
