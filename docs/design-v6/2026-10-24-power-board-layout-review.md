# Power board rev A: layout, footprint and assembly-file review (2026-10-24)

*A dated record, the third pre-order check after the
[pin audit](2026-10-24-power-board-pin-audit.md) (schematic) and the
[whole-board simulation](2026-10-24-power-board-full-simulation.md). It reviews
`pcb/power-board/power-board.kicad_pcb`, `power-board.pretty/` and `fab/` on branch
`pcb/power-board-cost` at 8bc4aba, the board as it would be ordered (1 oz copper).*

*The review was done independently by a separate reviewer, read-only, with KiCad 10 (kicad-cli and pcbnew)
and the datasheets in `docs/datasheets/power-board/`. Its scratch scripts and renders stayed
in the build directory, not the repository. Re-checked when this record was written:*
- *W1, from the netlist;*
- *M1: `Net-(Q3-G)` is 37.5 mm of track;*
- *M2: Q3's footprint carries 0.508 × 0.286 mm drain-finger paste apertures.*

*Coordinates are KiCad board coordinates in mm, with y increasing downward. The board edge
runs from x 100.0 to 197.0 and from y 60.0 to 124.5.*

## Summary

There is no board-killer. **Verdict: order after the rev B list** in
[pcb/power-board/README.md](../../pcb/power-board/README.md), adding:
- W1, the mis-mating connectors;
- M1, Q3's gate route;
- M2, the Q1–Q3 stencil;
- M3, REG18's capacitor;
- the C1 BOM cleanup.

Rev B needs a re-layout anyway, and all of these are cheap. M4–M9 are optional
improvements. M10 lists settings to get right on the order form.

This record corrects two earlier statements:
- the README's "no gate trace crosses a pour" is false for Q3 (M1);
- the verification record C8's "TI's stencil" does not hold for the drain apertures (M2).

## Findings

### Wrong in some state

**W1. Two control connectors take the same plug, and a stray silk label sits beside the wrong one.**
- **Where.**
  - J9 (I²C) at x 126–132 and J11 (E-STOP) at x 158–164 are both JST B4B-PH-K-S on the bottom edge, at y 120.5.
  - J10 (POWER) is a 2-pin PH header beside them.
  - A board-level silk text "J10" sits at (122.98, 120.5), immediately left of J9 pin 1. J10's own reference is already at (114.75, 121.05).
- **Evidence.**
  - Pin nets from the layout: J9 is 1 = +3V3_HOST, 2 = GND, 3 = SDA, 4 = SCL. J11 is 1 = ESTOP_FEED (PACK+ through R29), 2 = ESTOP_RET, 3 = ESTOP_N, 4 = GND.
  - The E-STOP lead plugged into J9 bridges J9 pins 1–2 through the button's normally-closed contact A (harness.md J11 table). That shorts the General Driver's 3V3 rail to GND, and contact B shorts SDA to SCL.
  - The POWER lead in J9 pins 1–2 does the same whenever the button is pressed.
  - J8 (servo data in) uses the same 22-03-5035 header as J4–J7, 6.4 mm away. If the H5 lead lands in J4–J7, H5 pin 2 meets SERVO+. *Unconfirmed:* whether H5 pin 2 is live on the General Driver; if it is, the servos could be fed around the E-stop.
- **Fix.**
  - Give J11 a different pin count or family from J9, for example a 5-pin PH with one position blanked, or an XH.
  - Delete the stray "J10" text.
  - Make the connector labels legible (see C3).

### Margin

**M1. Q3's gate route is a 37 mm hairpin through the SERVO+ pour, with no series-resistor placeholder.**
- **Where.**
  - Net Net-(Q3-G) starts at Q3 pad 4 (169.57, 77.095), then:
    - via (169.57, 75.75);
    - B.Cu diagonal to (176.31, 82.49);
    - via (177.33, 82.49);
    - F.Cu west along y = 82.488 to (163.75, 82.49);
    - R28/R27, then U2 pin 14 (PD) at (160.75, 88.8).
  - The direct R28-to-Q3-gate distance is about 10 mm.
  - The return path runs from Q3 source through the SERVO+ pour and vias (167.77, 84.97) and (165.15, 87.68) to U2 pin 13. That is about 10 mm.
- **Evidence.**
  - TPS4811-Q1 SLUSEE5E §9.6.1: "The external MOSFETs must be placed close to the controller such that the GATE of the MOSFETs are close to PU/PD pins to form short GATE loop. Consider adding a place holder for a resistor in series with the Gate…"
  - The trace cuts a 0.8 mm slot through the F.Cu SERVO+ pour (x 168.7–177.3) and a diagonal slot through the B.Cu SERVO+ copy.
  - The README's claim that "no gate trace crosses a pour" is therefore false for Q3.
  - The pour still carries current: the F.Cu slot runs parallel to the flow, and below it a lobe about 2.3 mm wide remains. So this is a gate-loop issue, not a copper issue.
  - Not quantified: ringing or turn-off delay from about 25 mm of extra loop. The whole-board simulation does not model layout parasitics.
- **Fix.**
  - Route from the via at (169.57, 75.75) on B.Cu under Q3, where B.Cu is the GND pour, straight to R28. Keep it under about 12 mm.
  - Add an 0603 0 Ω placeholder in series at Q3's gate.
  - Place rev B's F-5 15 V zener directly from Q3 pad 4 to pads 1–3.

**M2. The Q1–Q3 stencil does not follow TI's §7.3 stencil, though the copper is correct.**
- **Where.** `power-board.pretty/TI_SON-8_5x6mm_P1.27mm_CSD17556Q5B.kicad_mod`, the unnamed F.Paste pads. The footprint is in its own orientation, with the drain to +x.
- **Evidence.** CSD17556Q5B SLPS392D §7.3, re-plotted at 300 dpi (re-plotted at 300 dpi):

  | | TI §7.3 (mirrored to the footprint) | footprint |
  |---|---|---|
  | drain-finger apertures | 0.766 × 0.508 mm at x ≈ +2.90 | 0.508 × 0.286 mm at x = +3.152 |
  | 1.294 × 0.746 columns | x ≈ −0.06 and +1.58, gap 0.350 | x = −0.087 and +1.969, gap 0.762 |
  | drain paste total | 9.28 mm² | 8.30 mm² (−11 %) |
  | finger apertures | 0.389 mm² each | 0.145 mm² each (−63 %) |

  - The chain looks mis-transcribed. The finger width (0.766) was used as the gap between the columns, and the finger height (0.508) and the gap (0.286) were used as the finger aperture.
  - The source and gate apertures (1.072 × 0.562) and all copper (§7.2: 4.44 × 4.52 drain with three 0.59 × 0.56 notches, 1.372 × 0.71 pads) match TI.
  - The verification record C8's statement "with TI's stencil" is not accurate.
- **Fix.** Rebuild the eight drain apertures and four finger apertures from the §7.3 chain above.

**M3. The REG18 capacitor C4 is about 8 mm and two vias from U1 pin 15.**
- **Where.**
  - U1 pin 15 at (124.51, 105.5) runs on F.Cu to the via at (120.72, 106.35), then on B.Cu to the via at (119.70, 108.88), then to C4 pad 1 at (120.63, 108.6).
  - The REG18 via sits 0.19 mm from R8's SRN pad.
- **Evidence.** BQ76922 SLUSE86A §10.1: the REG18 capacitor "should be placed as close to the REG18 pin as possible, with minimized trace inductance". The direct distance is about 4.4 mm.
- **Fix.** Put C4 at U1's pin-15 corner on F.Cu, with its BAT− end on an EP or island via.

**M4. R1 and R24 use KiCad's generic 2512 pads, not Bourns' Kelvin pattern, and R1's TCR is outside TI's guidance.**
- **Where.** R1 at (114, 95.5) and R24 at (159, 79) use `R_2512_6332Metric`: 1.225 × 3.35 mm pads, inner edges ±2.35 from centre.
- **Evidence.**
  - Bourns CSS2H-2512 datasheet p. 2, "Recommended Pad Layout": 1.80 × 3.40 mm solder pads with a 3.40 mm gap (inner edges ±1.70), plus separate sense terminals from the inner corners.
  - With the generic pads, each solder joint starts about 0.65 mm further out on the copper terminal. The sense points therefore include extra terminal copper.
  - Estimate, assuming 0.5–0.8 mm terminal thickness, which the datasheet does not give: about +0.9 to 1.5 % on R1 and +0.5 to 0.7 % on R24, plus copper's TCR on that fraction.
  - Separately, CSS2H-2512R-1L00 is specified at ±75 ppm/°C including the terminals (datasheet p. 1). BQ76922 §10.1 asks for ≤ 50 ppm.
  - The Kelvin pick-offs themselves are good. SRP leaves R1 pad 1 at (115.0, 92.9) on its inner half and SRN leaves pad 2 at (115.0, 98.1). CS+ leaves R24 pad 1 at (156.3, 80.4) and CS− at (161.5, 80.6).
- **Fix.**
  - A Bourns-pattern footprint with sense tabs; or
  - accept the offset and calibrate the coulomb-counter gain at bring-up.

**M5. Q1–Q3 have no thermal vias or bottom copper under their drain pads.**
- **Where.** The Net-(Q1-D) pour (137.7–148.0, 76.4–81.6, 53.5 mm², shared by Q1 and Q2) and the Net-(Q3-D) pour (34.9 mm²) are F.Cu only, with zero vias.
- **Evidence.**
  - CSD17556Q5B §5.2 gives RθJA max 50 °C/W on 1 in² of 2 oz copper and 125 °C/W on minimum copper.
  - Here each FET has about 25–30 mm² of 1 oz copper, plus In1 0.2 mm below. Rds(on) is 1.4 mΩ max at 25 °C and higher hot (Fig. 8).
  - Estimate, not simulated:
    - At 15 A continuous: about 0.3–0.45 W per FET, giving roughly 30–55 °C rise per FET, more for Q1 and Q2 sharing one pour.
    - At the harness's walking RMS (about 3.4 A total): about 20× less.
  - The 1 oz decision weakened this heat path more than it raised I²R.
- **Fix.**
  - Add B.Cu copies of Q1-D and Q3-D, stitched with vias.
  - Vias placed in the drain pads must be tented or plugged.
  - Or accept, given the real duty.

**M6. Four of U1's exposed-pad thermal vias sit under paste and are tented only from the bottom.**
- **Where.** U1 at (126.5, 104.5). The paste is four 1.17 mm squares at (±0.7, ±0.7). There are nine Ø0.3 mm plated pads at 1.1 mm pitch.
  - The four corner holes lie wholly inside paste windows; the four edge holes are partly covered.
  - These are footprint pads, not vias, so the order's via-plugging option does not apply. Only B.Mask covers them.
- **Evidence.** BQ76922 RSN0032B note 5: "vias under paste be filled, plugged or tented". Expect some solder draw-down and voiding on the exposed pad of a 0.4 mm-pitch part.
- **Fix.** Pull the paste windows off the holes, or use the 5-via variant outside the paste.

**M7. Solid (no thermal relief) through-hole joints into planes and pours.**
- **Where.**
  - zc = full on: J3 pin 2, J4–J8 pin 3, J9 pin 2, J10 pin 2, J11 pin 4, J12 pin 3 and C21 pin 2, into In1, In2 and B.Cu GND;
  - J4–J7 pin 2 into the F and B SERVO+ pours;
  - J1, F1 and F2 into their pours.
- **Evidence.**
  - JLCPCB's PCBA page (saved in the 2026-10-24 quote round's saved page) says through-hole parts are wave-soldered. PCBWay hand-solders them (quote record).
  - A branch peak of 4.3 A (harness.md) needs nothing like a solid joint. The zones' four 0.6 mm spokes on each of three layers give about 7 mm of 1 oz copper.
  - Solid joints risk incomplete hole fill.
- **Fix.** Use thermal relief at least on the signal connectors' GND pins (J3, J8–J12). Wide spokes are enough for J4–J7.

**M8. All return current passes through ten 0.4 mm vias at R1.**
- **Where.** GND vias 0.8/0.4 mm at (111.4–115.4, 100.0 and 101.2). The only other F.Cu GND path is the SRN Kelvin trace, so everything from the planes goes through these ten.
- **Evidence.** That is 1.5 A per via at 15 A and 2 A per via at 20 A. Barrel I²R is negligible (about 0.15 mΩ from F.Cu to In1 per via). This is near commonly quoted single-via ratings, but not over them. *Estimate.*
- **R1's own pad.** The F.Cu GND pour reaches only the south 0.375 mm of R1's GND pad
  (y 98.70–99.075 of 97.85–99.075; 30 % of the pad's area, measured with pcbnew from the
  zone fill). That costs nothing electrically (about 54 µΩ at 1 oz), but it is joint
  area. Percy's circuit review part 2, P3-6.
- **Fix.** Extend the R1 GND pour about 1 mm north over the pad, and double the via
  count. There is room to the left.

**M9. Unfused BAT+ copper is close to GND and BAT−.**
- **Where.**
  - The B.Cu BAT+ copy (113.4–125.2, 74.6–82.2) borders the B.Cu GND pour at 0.3 mm, along about 39 mm of perimeter.
  - On F.Cu, BAT+ (lower edge y ≈ 82.1) and BAT− (upper edge y ≈ 82.6) are 0.5 mm apart under J1.
- **Evidence.** A short between them bypasses F1 and Q1/Q2, and nothing on the board can interrupt it. The copper is under solder mask, so the likelihood is low, but the consequence is a dead short of a 130C pack.
- **Fix.** Pull GND back 1 mm from the BAT+ copy, or drop the B.Cu BAT+ copy (F.Cu alone is enough over this 6 mm run), and open the F.Cu gap to 1 mm.

**M10. Settings to get right when the order is placed (these are not file defects).**
- **Inner copper 1 oz.** JLCPCB's capabilities page says "Finished copper weight of inner layer is 0.5oz by default", and its quote page pre-selected 0.5 oz before the script changed it (2026-10-24 quote readback). The planes carry the whole servo return current.
- **Stencil thickness.** TI's RSN stencil is "based on 0.1 mm thick stencil", and the DGX stencil on 0.125 mm. U1's 0.775 × 0.2 mm apertures give an area ratio of about 0.66 at 0.12 mm.
- **J1 overhang.** J1's mating face sits 0.25 mm beyond the left edge (footprint front at x = 99.75). Tell the assembler before it panels the board or adds rails.
- **Economic PCBA pitch limit.** JLCPCB Economic PCBA's minimum IC pitch is 0.4 mm, exactly U1's pitch. Its Standard service needs a board of at least 70 × 70 mm or a panel; this board is 64.6 mm tall.

### Cosmetic

**C1. BOM** (`fab/power-board-bom.csv`, 65 lines):
- TP1–TP8 appear as 8 lines with no MPN. The PCB footprints are excluded from the BOM but the schematic symbols are not; DRC schematic parity reports this as 8 × "Exclude from bill of materials settings differ".
- The F1 and F2 lines each hold two MPNs ("Keystone 3568 + Littelfuse 029700x.WXNV"). Split each into the holder and a separate, unsoldered fuse line.
- One MPN appears on several lines:

  | MPN | lines |
  |---|---|
  | CC0603KRX7R9BB104 | C5–C8, C9/C23, C14 |
  | CC0603JRNPO9BN101 | C1, C10 |
  | 22-03-5035 | J4–J8, on 5 lines |
  | B4B-PH-K-S | J9, J11 |

- C22's value says "10u 25V", but CL31B106KLHNNNE is 10 µF, **35 V**, X7R, 1206 (Samsung spec sheet). The 1206 footprint is consistent with the part; only the voltage text differs.
- Wrong datasheet links: C15 points to the …101 page, and C17 points to CL21B334. J1's link is Amass's homepage. Q1–Q3 have no maker name in the MPN field.

**C2. Placement file.**
- It matches a fresh export byte for byte, and its 94 references match the BOM, value for value and footprint for footprint.
- The through-hole origins (J1–J12, C21, F1, F2) are at pin 1, not the body centre.
- Rotations run from −180 to 180 under KiCad's header names. Rename the headers for JLCPCB's CPL importer.
- Y is KiCad's absolute coordinate negated (y = −y for all 94 rows; Percy's part 2, P3-7). State the convention when the file is uploaded to either assembler.
- Confirm every polarized part in the assembler's preview. The pin-1 positions the file implies:

  | part | pin-1 corner |
  |---|---|
  | U1 | top-right |
  | U2 | bottom-left |
  | Q1 | top-left |
  | Q2, Q3 | bottom-right |
  | Q4, Q5, D2 | top-left |
  | D1, D8, D9 | top |
  | D3, D5, D6 | right |
  | D4, D7 | left |

**C3. Silk text size.** All references and the connector legend are 0.8 mm high with 0.12 mm strokes. JLCPCB's capability page asks for at least 1.0 mm and 0.15 mm, or the text "will be unidentifiable". The pinout legend at (173–174, 107–113) matters for cabling.

**C4. Keystone 3568 holes.** The F1 and F2 drills are 1.78 mm; Keystone's mounting detail gives Ø1.60 (.063") at 9.9 × 3.4 mm. The pitch is correct. The holes are looser than specified but still solderable.

**C5. J1 peg slot pads.** They are 0.9 × 2.0 mm on 0.6 × 1.7 mm plated slots, a 0.15 mm ring. That is JLCPCB's absolute minimum for a multilayer plated hole (recommended 0.20). The pads carry no net.

**C6. U1 footprint versus TI's land pattern.**
- KiCad's pads are 0.775 × 0.2 mm centred at 1.988 mm; TI's (RSN0032B) are 0.55 × 0.2 mm at 1.925 mm. The pad-to-exposed-pad gap is 0.2 mm against TI's 0.25.
- Acceptable: the mask webs are 0.2 mm, and JLCPCB's minimum bridge is 0.10 mm for green mask.

**C7. U2's local grounds and Q5's emitter.**
- TPS4811-Q1 §9.6.1 asks for a local star ground and a Kelvin connection from Q5's emitter to U2 GND. Here every GND pad has its own via to the plane.
- Estimated plane gradient from the servo return: about 0.05 mV/mm across three planes, so sub-mV offsets. Negligible.

**C8. Documentation.**
- README says "no gate trace crosses a pour" (see M1); verification C8 says "TI's stencil" (see M2).
- The Gerber job's Revision field reads "rev?".

### Unconfirmed: no datasheet in the repo, or a physical check needed

- **J4–J8 (Molex 22-03-5035).**
  - KiCad's 5267-03A footprint (0.85 mm drill, 1.7 × 1.85 mm pads) could not be checked: there is no Molex drawing in the repo.
  - Whether five 5264 plugs with latches fit side by side at 6.4 mm pitch is also unknown.
  - Pin order against H5 is P4.
- **C21 (Panasonic EEU-FR1E102).** No datasheet. The footprint assumes Ø10 mm, 5.0 mm pitch and a 1.0 mm drill. Height above the board is unknown; check it against the torso (#108).
- **F1 (Littelfuse 0297015.WXNV).** No datasheet, so its dissipation at 15 A is unknown. It is likely the largest single heat source on the main path.
- **JLCPCB and PCBWay rotation libraries for these KiCad footprints.** Not knowable from the files; see the C2 checklist.


## Verdict

**Order after the README's rev B list.** Rev B adds parts and needs a re-layout anyway, so fold in M1–M3 and W1 (all cheap), and fix the C1 BOM issues. M4–M9 are optional improvements. Then set M10 on the order form.


## Checked and found correct

- **Fab outputs match the board.**
  - Fresh Gerber, drill and placement exports equal the committed `fab/`, except creation dates and one B.Cu fill vertex.
  - The job file carries 4 layers, 1.6 mm, ENIG and the 1 oz stackup.
  - DRC with zone refill is clean.
  - DRC with the four project-ignored checks re-enabled (courtyards, footprint type, filters, via centring) is clean.
  - Schematic parity reports no footprint, value or net mismatches; only the MPN-field and test-point BOM-flag warnings.
- **U2 (DGX0019A).** 20 positions with pad 16 omitted, 0.3 × 1.45 mm pads, 0.5 mm pitch, rows 4.4 mm apart, pin 1 top-left with a silk mark. Pins match SLUSEE5E Table 5-1 (INP_G to GND, G floating, PD on the gate, PU through R27, VS, SRC, BST, CS±, ISCP, DIODE). This was spot-checked.
- **U1 (BQ76922 RSN).** Pin order matches SLUSE86A Fig. 5-1: VSS/CFETOFF/DFETOFF/REGIN/BREG/FUSE to BAT−, VC3/VC4 tied to VC2. The 2.8 mm exposed pad and pin-1 triangle are correct; the exposed pad is on the BAT− island.
- **Q1–Q3 copper.** Correct against SLPS392D §7.2. Pin 1 is S, pins 1–3 are S, pin 4 is G. The orientation is back-to-back common drain (Q1 S = fused BAT+, Q2 S = PACK+) and Q3 has its drain to R24. Silk pin-1 dots are present.
- **J1 (XT60PW-M).**
  - Matches Amass V1.2: Ø2.7 mm holes at 7.2 mm, 0.6 × 1.7 mm slots at 13.5 mm, 6 mm back.
  - Mating face at the edge.
  - The chamfered contact maps to pad 1 (BAT−) by the front-view geometry, consistent with harness.md and verification C8.
  - The bare-board silk "+" and "−" agree.
- **Through-hole connectors.** JST XH holes 0.95/1.0 mm (spec 0.9–1.0) and PH holes 0.75 mm (spec 0.7–0.8). Pin-1 marks are present.
  - J2 order B−, C1+, C2+, B+ goes through R3–R6 to VC0, VC1, VC2, VC5.
  - J4–J7 order DATA, SERVO+, GND; J8 pin 2 is unconnected.
  - J3 is LOGIC_OUT/GND through F2 on a 1.2 mm trace.
- **Discrete land patterns.**
  - D1 matches Nexperia SOD323F (0.5 × 0.5 mm at 2.2 mm), with the cathode on pin 1.
  - D3–D7 SOD-123 pads (0.9 × 1.2 mm at ±1.65) are within Diodes' suggested layout (0.9 × 0.95 mm, 4.05 mm overall).
  - D8/D9 SMB pads (2.5 × 2.3 mm, 1.8 mm gap) meet Bourns' minima.
  - All diode cathode bars and orientations match their nets: D3/D5/D6 gate clamps, D4/D7, TVS cathodes to the rail, BAT54C common cathode to J10.
  - C21's "+" and shaded negative half are correct.
- **Sensing.**
  - SRP and SRN are Kelvin from R1's inner pad halves and are not touched by any pour; R7/R8 pads have no zone connection. They run as a pair (x 117 / 116.2) to C9/C10, about 2 mm from U1 pins 11/12.
  - CS+ (R25) and ISCP (R26) share one pick-off from R24 pad 1. CS− is taken from the dead-end copper under R24 pad 2.
  - C15 (ISCP–CS−) is about 4 mm from U2.
- **TPS4811-Q1 layout items that pass.**
  - VS bypass C14 is about 2.8 mm from pin 20.
  - The BST loop (C17 to pins 12/13) is about 4 mm.
  - Input TVS D8 is at R24 and output TVS D9 at SERVO+.
  - Q5 is MMBT3904 with B and C tied, about 2.7 mm from Q3's body. The 1 nF across it is already rev B.
- **BAT− island.** In2 island (875 mm²) is tied only at J1 pin 1, through spokes. No main current flows through it, VSS references the pack-side terminal, and the BQ's B.Cu signals run over it.
- **Copper.** Copper sizing agrees with verification C11:
  - The pours are wide (5–13 mm), with no neck-downs.
  - I²R in each main-path segment is 18–140 mW at 15–20 A.
  - The SERVO+ strip is 2 × 2.65 mm (F + B). Its worst segment carries ≤ 10.3 A (get-up peak), about 10 mW/mm.
  - Hot spots are the parts (FETs, F1, R1, R24, the XT60 contacts at 2 × 0.5 mΩ), not the copper.
- **Rules and manufacturability.**
  - 0.15 mm track and space; vias 0.6/0.3 mm, within JLCPCB's via spec of 0.15/0.25 mm minimum.
  - Copper 0.3 mm from the edge.
  - No via in any SMD pad (closest same-net escapes ≥ 0.15 mm).
  - Silk is clipped by mask in the Gerbers.
  - Clearances are ≥ 0.15 mm for nets up to about 25 V (gate drive).
  - Mounting holes are NPTH Ø2.7 mm with no copper ring.
