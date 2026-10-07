# Power board

The robot's custom power board, as designed in
[docs/design-v6/2026-10-06-power-circuit.md](../../docs/design-v6/2026-10-06-power-circuit.md)
§5. Open `power-board.kicad_pro` in KiCad 10. [`power-board.pdf`](power-board.pdf)
is a plot of the schematic for reading without KiCad. [harness.md](harness.md) gives every
cable and connector to the rest of the robot.

**State: schematic and layout rev A, verified on paper, not ordered.**
- **Clean:** ERC and DRC report no violations.
- **Checked:** the simulations, footprint checks and copper analysis are in
  [the verification record](../../docs/design-v6/2026-10-07-power-board-verification.md).
- **In stock:** every part has a manufacturer datasheet and was in stock at Digi-Key on
  2026-10-07, except the XT60 (LCSC/TME).
- **Open:** physical checks P1–P7 ([harness.md](harness.md)) come before ordering. The
  outline and mounting holes wait on the torso redesign (#108).

## What is on it

| block | parts | function |
|---|---|---|
| pack input | J1 (XT60, right-angle), J2 (3S balance, JST XH), F1 (15 A MINI blade), R1 (1 mΩ) | the pack, a fuse for a hard short, and the coulomb-counter sense resistor in the negative lead |
| cell protection and main switch | U1 TI BQ76922 and its passives | per-cell cut-off without firmware; cells, current and charge over I²C (0x08); POWER wakes it on TS2; KILL (RST_SHUT) shuts it down |
| high-side FETs | Q1 (CHG), Q2 (DSG) CSD17556Q5B; gate networks; Q4 clamp | the BQ76922EVM topology, copied from TI's EVM schematic (SLVU957A): back-to-back FETs, 10 MΩ and 16 V zeners gate to source, a 4.7 kΩ turn-off path on DSG, and Q4 holding DSG off if PACK+ swings below ground |
| servo-rail switch | U2 TI TPS48111-Q1, Q3 CSD17556Q5B, R24 (2 mΩ) | soft start by gate slew, about 40 ms; overcurrent 20 A, latching off after 10 ms; short circuit at 40 A; the E-stop's NC contact in the EN/UVLO feed; SERVO_EN on INP; remote temperature diode Q5 at Q3 |
| servo distribution | J4–J7 (Molex 5267 3-pin), J8 (data in), C21 (1000 µF) | four branches; DATA and GND from the General Driver's H5, V+ from the switched rail |
| logic feed | F2 (3 A MINI blade fuse in a Keystone 3568 holder, replaceable by hand), J3 (JST XH 2-pin) | PACK+ to the General Driver's H1 |
| control | J9 (I²C, JST PH 4-pin), J12 (control, JST PH 8-pin), J10 (POWER), J11 (E-STOP) | I²C and 3V3 to the General Driver's P1; KILL, SERVO_EN, PWR_INT_N, ESTOP_N, RAIL_V, SERVO_IMON and SERVO_FLT_N to the ESP32; the two buttons on the torso shell |

Component values follow the datasheets' application sections: BQ76922 (SLUSE86A §8.2,
and the BQ76922EVM schematic) and TPS4811-Q1 (SLUSEE5E §8.3, §9.2–9.3). TI's TPS4811-Q1
design calculator (SLURB09) confirms the servo-rail switch values.

## The board

**97 × 64.5 mm, four layers, 1.6 mm, ENIG.** The stackup is in the board file, so the
Gerber job file carries it.
- **Copper:** 2 oz (70 µm) on the outer layers, 1 oz inside. 1 oz outer is enough by the
  copper analysis in the verification record; that choice is Tom's (cost).
- **Parts:** every part is on the top side. The through-hole parts are J1–J12, F1 and C21.

| layer | use |
|---|---|
| F.Cu | parts, signals, and the pours of the power path |
| In1.Cu | GND plane |
| In2.Cu | GND plane, with a BAT− island under the BQ76922 block (its VSS); the island takes in J1's BAT− pin |
| B.Cu | GND pour, signals, and copies of the BAT+, FBAT, PACK+ and SERVO+ pours stitched to the top |

**Placement.**
- **Power path:** J1 is a right-angle XT60 whose mating face is flush with the left edge.
  From it the power path runs along the top edge, left to right: F1, Q1 and Q2 drain to
  drain, R24, Q3, then SERVO+ around C21.
- **Servo headers:** a 2.6 mm SERVO+ strip runs down between the data and GND columns of the
  servo headers J4–J8 on the right edge.
- **Blocks:** the BQ76922 block sits under the left half, with the balance lead J2 above it.
  The TPS48111 block sits under R24 and Q3. The DSG gate network is above the strip.
- **Logic feed:** F2's holder and J3 are on a strip along the top edge. PACK+ copper runs up
  to F2's input, so the fuse can be pulled and replaced from above.
- **Control connectors:** J10, J9, J12 and J11 are along the bottom edge.

**High current.**
- **Pours:** every pad on the 15–20 A path sits in one solid pour per net, on top, with a
  copy on the bottom where the parts allow it. A script check confirms each pour fills as one
  piece covering all its pads.
- **Gate exits:** Q2's and Q3's gates leave through vias inside their pours' notches, so no
  gate trace crosses a pour.
- **Solid joints:** Q1–Q3, R1 and R24 join their pours solid. So do the ground pins of the
  through-hole connectors and C21: the servo returns carry amps, and thin thermal spokes would
  starve. That makes those pins harder to hand-solder.
- **R1's ground end** reaches the planes through ten 0.4 mm vias.

**Sensing.**
- **Kelvin traces:** SRP and SRN (through R7 and R8) leave R1's pads on their inner edges
  and touch nothing else. CS+ (through R25 and R26) and CS− leave R24's pads the same way.
- **Temperature:** Q5, the TPS48111's temperature diode, sits against Q3. RT1, the
  BQ76922's TS1 thermistor, sits under Q1 and Q2.

**Ground.** Every GND and BAT− SMD pad has its own via into its plane. GND stitching vias
on a 3 mm grid tie the planes and the bottom pour together.

**Rules.**
- Tracks and clearance: 0.15 mm minimum.
- Signal tracks: 0.2 mm by default; the logic feed is 1.2 mm.
- Vias: 0.6 mm with 0.3 mm drill.

These are within a standard 4-layer process at PCBWay or JLCPCB. JLCPCB lists 0.15/0.15 mm
as its minimum for 2 oz outer copper.

## Files

| file | what it is |
|---|---|
| `power-board.kicad_sch` | the schematic |
| `power-board.kicad_pcb` | the layout |
| `power-board.kicad_sym` | project symbols: BQ76922 (from TI's pin table, SLUSE86A table 5-1) and TPS48111-Q1 (SLUSEE5E table 5-1) |
| `power-board.pretty/TI_DGX0019A_VSSOP-19.kicad_mod` | the TPS48111-Q1 footprint, from TI's land pattern (drawing 4226944/A): 0.3 × 1.45 mm pads, 0.5 mm pitch, rows 4.4 mm apart, pad 16 left out |
| `power-board.pretty/TI_SON-8_5x6mm_P1.27mm_CSD17556Q5B.kicad_mod` | Q1–Q3, from TI's recommended PCB pattern and stencil (SLPS392D §7.2–7.3) |
| `power-board.pretty/AMASS_XT60PW-M_1x02_P7.20mm_Horizontal_EdgeMount.kicad_mod` | J1: KiCad's XT60PW-M footprint with the silkscreen stopping at the board edge |
| `power-board.pdf` | the schematic plotted by `kicad-cli` |
| `harness.md` | cables and connectors to the rest of the robot, and the physical checks P1–P7 |
| `fab.sh` | runs ERC and DRC, then writes `fab/` |
| `fab/power-board-gerbers.zip` | Gerbers (four copper layers, mask, paste, silk, outline, job file) and Excellon drill, for the board house |
| `fab/power-board-pos.csv`, `fab/power-board-bom.csv` | placement and bill of materials (MPN and datasheet per line) for assembly |
| `fab/erc.rpt`, `fab/drc.rpt` | the check reports behind the outputs |
| `../../docs/datasheets/power-board/` | the datasheets the design is drawn from |

Every other symbol and footprint comes from KiCad's own libraries. The KiCad files are
the source: edit them in KiCad, then run `fab.sh` again.

## Checks

```bash
sh pcb/power-board/fab.sh      # ERC 0 and DRC 0 (0 violations, 0 unconnected) or it stops
KC=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
$KC sch export pdf -o pcb/power-board/power-board.pdf pcb/power-board/power-board.kicad_sch
```

The schematic was also checked net by net against its design spec. All 65 nets match it
pin for pin. The netlist's 5 other nets are the single pins of no-connect pins.

## Open before ordering

- **The physical checks P1–P7** in [harness.md](harness.md):
  - the pack's XT60 polarity;
  - the balance-lead order;
  - the General Driver's H1 polarity and H5 orientation;
  - the families of its control headers;
  - the servo leads' wire gauge;
  - the pack's internal resistance.
- **Q1 and Q2 against the ordered pack.** The pack is an OVONIC 4200 mAh 3S 130C. If it
  measures under about 15 mΩ, a hard short on the board's own PACK+ copper takes a single Q2
  past its 400 A pulse rating. Accepted (Tom, 2026-10-07, verification record C13 and D-FET),
  with two handling rules at bring-up.
- **Outline and mounting holes.** Fix them to the torso (#108). The four M2.5 holes now sit
  at the corners, 3.2 mm in.
- **Buttons.** Choose the POWER button (momentary, NO) and the E-STOP (latching, two NC
  contacts), each from a maker with a datasheet.
- **Datasheets still missing.** Molex 22-03-5035, Panasonic EEU-FR1E102 and Littelfuse
  0297015 are in stock but their datasheets are not yet in `docs/datasheets/`; the makers'
  sites refused the download.
- **The BQ76922 configuration** (cell count, UV/OV, OCD/SCD, TS1 thermistor, TS2 wake,
  FET control): written in RAM first, then burned to OTP at bring-up with BAT at 10–12 V.
  - Until the OTP is programmed, the part does not protect without the host.
  - Set short-circuit-in-discharge to 150 mV (150 A on R1) with the fastest delay.
  - The datasheet gives that threshold ±35 %, so it trips at 97.5 A at the lowest. That is
    above the 56–76 A a servo-rail short draws from the pack, and the servo switch clears
    that short first.
