# Power board rev A: pin-by-pin audit against the datasheets (2026-10-24)

*A dated record, the static half of the pre-order check; the simulation half is
[2026-10-24-power-board-full-simulation.md](2026-10-24-power-board-full-simulation.md),
which re-tests the findings below on the whole board. This audit was done
independently of the simulation: a separate reviewer read the netlist exported
from `pcb/power-board/power-board.kicad_sch` and the datasheets in
`docs/datasheets/power-board/`, without the verification record's conclusions.
The datasheet passages behind F4, F5, F6 and C2 were re-read when this record
was written.*

## Result

No wiring error stops the board working:

- every pin of U1 (BQ76922) and U2 (TPS48111-Q1) is connected as its pin table,
  application section and unused-pin guidance require;
- every discrete's pinout matches its datasheet;
- every logic signal between the board and the ESP32 lands on the right side of
  its receiver's thresholds.

What it found are three faults that appear only in some states, seven margin
items, and BQ76922 defaults that must change in its configuration.

| # | severity | finding | simulated? |
|---|---|---|---|
| F1 | wrong in some state | SERVO+'s only bleed is R38 + R39 (57 kΩ; τ ≈ 57 s with C21 alone). After KILL, SERVO+ holds PACK+ up through Q3's body diode, so the BQ76922 waits in "soft shutdown" and ignores POWER; on the bench, RAIL_V stays "up" long after an E-stop press | **yes**, confirmed (simulation record F-1) |
| F2 | wrong in some state, temperature-dependent | with the General Driver unpowered, D2's second junction leaks TS2's 5 MΩ wake bias into PWR_INT_N; warm, the BQ76922 wakes itself | **yes**, confirmed (F-2) |
| F3 | wrong under a harness fault | R29 (1 kΩ 0603, 0.1 W) dissipates 159 mW if J11 pin 2 shorts to pin 3 or to GND; ESTOP_N has no series resistor (design rule §5.5) | arithmetic below |
| F4 | margin | no 1 nF across Q5; TPS4811-Q1 §9.6.1: "a small 1000pF bypass capacitor must be placed in parallel with the MMBT3904". A noise trip latches the rail off until an E-stop cycle | no (noise) |
| F5 | margin, configuration | TI ships REG0 and REG1 enabled (DS §7.7.3.2); with BREG and REGIN on VSS (right, per Table 8-3) BREG sources its 2.5–3.33 mA short-circuit current until REG0_EN is cleared | no (configuration) |
| F6 | margin | VC0's resistor R3 goes only to J2.1; DS §8.3: "If VC0 is not connected through the series resistor to VSS on the PCB, then cells cannot be connected in random sequence" | **yes** (F-9) |
| F7 | margin | C5–C8 are 100 nF ±10 % against DS 6.3's 0.1 µF minimum (TI typical 0.22 µF) | no |
| F8 | margin | C4 (REG18) 2.2 µF ±10 % X7R against a 1.8 µF minimum | no |
| F9 | margin | U2's drain-side pins (VS, CS+, CS−, ISCP) are rated 60 V/µs; no VS filter is fitted | **yes**, confirmed (F-7) |
| F10 | accuracy | R1's TCR is ±75 ppm/°C including its terminals (Bourns); DS §10.1 asks for ≤ 50 ppm. Coulomb counting only | no |
| F11 | inside the accepted PACK+ short | in a PACK+ short C21 discharges backwards through Q3 and R24; CS+ − CS− can pass −0.3 V; R25 limits the pin current | partly (F-7 run) |

## F3, the arithmetic

- A short from J11.2 to J11.3 (adjacent pins and wires) with the E-stop
  released, or from J11.1 or J11.2 to GND, puts R29 across PACK+: 12.6 V / 1 kΩ =
  12.6 mA, 159 mW, 1.6 × the 0603's 0.1 W (Yageo RC_L) for as long as the fault
  lasts.
- A J11.1 to J11.3 short inside the switch with the E-stop pressed pushes
  (12.6 − 3.6) V / 1 kΩ ≈ 9 mA into IO4's clamp.
- Fix: R29 → 10 kΩ (16 mW under the same fault, ≤ 0.9 mA of injection) with
  R30 → 549 kΩ to keep the UVLO at 1.18 V × 659 / 100 = 7.78 V; optionally
  1 kΩ in series between J11.3 and the R40 / J12.6 node (ESTOP_N then reads
  0.30 V with contact B closed, still below V(IL)).

## BQ76922 configuration prerequisites

Each of these defaults would make the board respond wrongly if left as shipped:

| # | default (TRM SLUUCG7) | why it is wrong here | set |
|---|---|---|---|
| C1 | REG0 Config 0x01, REG1 Config 0x0D (TI's factory OTP, §3.2) | F5 | REG0_EN = 0, REG1_EN = 0 |
| C2 | TS1 Config 0x07: 18 kΩ pull-up, 18K model, cell-temperature protections | RT1 sits under Q1 and Q2 (README "Temperature"), not on a cell, and is a Murata NCP18XH103 (B 3380 K), not the Semitec 103-AT the 18K polynomial models | OPT[1:0] = 11 (FET temperature) or 10; a custom polynomial |
| C3 | TS2 Config 0x00, unused | must stay unused: as a thermistor or ADC input a POWER press reads as a hot thermistor, and TS2 floats to PWR_INT_N's level in NORMAL (simulation record, power-on) | keep 0x00 |
| C4 | CFETOFF / DFETOFF Pin Config 0x00, unused | both pins are tied to VSS; any active-low setting holds the FETs off | keep unused, or active-high |
| C5 | Mfg Status Init 0x0040: FET_EN = 0 | the FETs stay off after every wake | FET_EN = 1 |
| C6 | Enabled Protections A 0x88 (SCD, COV only); B and C 0x00 | CUV and OCD are off | per design §5.5; see the simulation record for SCDL, OCD recovery and shutdown voltage |
| C7 | cell mode | 3S on VC1, VC2 and VC5 (Table 8-3) | enable cells 1, 2 and 5 only |
| C8 | OTP programming needs V(BAT) 10–12 V (DS 6.3) | a full pack (12.6 V) is too high; BAT sits 0.3–0.7 V below the pack (D1, R2) | program at 10.7–12.0 V of stack |
| C9 | FET Options SFET = 1, Body Diode Threshold 50 mA | correct: CHG turns on when discharge would run through Q1's body diode | keep |

## Notes for firmware and documentation

- **N1. OT latch.** U2's over-temperature latch is reset only by EN/UVLO below
  V(ENF) or a VS power cycle (TPS §8.3.6), not by SERVO_EN; the over-current
  latch is reset by toggling INP (§8.3.3.2). FLT_T and FLT_I are wired together
  on SERVO_FLT_N, so the two cannot be told apart.
- **N2. Delay after an E-stop release.** The rail returns after
  T(DRV_EN) = C(BST) × V(BST_UVLOR) / I(BST) (TPS eq. 1): 0.36 s typical,
  0.24–0.52 s over tolerances, if BST has discharged; then about 40 ms of gate
  slew. At power-on BST charges before the ESP32 has booted.
- **N3. ESTOP_N polarity.** ESTOP_N is high when the E-stop is pressed; the
  `_N` reads as active-low.
- **N4. KILL and POWER.** KILL left high: a POWER wake is held in reset and shut
  down again. POWER still held when the ESP32 boots reads as a press. A stuck
  POWER button holds TS2 low and SHUTDOWN never completes (TRM §7.5).
- **N5. IMON range.** Gain 0.9 × 9.1 kΩ / 100 Ω = 81.9 (eq. 13); IMON passes
  the 5 V recommended limit above about 30 A, up to its 6.5 V clamp near the
  40 A short-circuit trip. It matters only if J12.7 is ever wired to an ADC.
- **N6. C22's label.** The schematic says "10u 25V"; Samsung's
  CL31B106KLHNNNE is a 35 V part.
- **N7. Test points.** None on BAT+/FBAT, REG18, TS2 or RST_SHUT.
- **N8. E-stop contacts.** Contact A switches 19 µA (12.6 V / 661 kΩ), a
  dry-circuit duty: choose gold-plated or low-level-rated contacts.

## Checked and found correct

**U1, all 33 pins** (SLUSE86A Table 5-1, §6, §8.2, §8.5, Table 8-3; EVM
SLVU957A Fig. 6-9 and 6-10, read at 600–1200 dpi):

| pin | net and parts | requirement | result |
|---|---|---|---|
| 1 BAT | BAT+ → D1 → R2 100 Ω; C1 100 pF, C2 2.2 µF | series diode, R and C (EVM: 40 V diode, 100 Ω, 100 pF, 1 µF) | ✓ 8.3–12.3 V over a 9.0–12.6 V pack |
| 2 VC5 | J2.4 via R6 20 Ω; C8 to VC2 | top cell VC4A–VC5; R(C) 20–100 Ω | ✓ (C8: F7) |
| 3–6 VC4A/B, VC3A/B | shorted to VC2 | "Pins 3 and 4 must be shorted", "5 and 6"; unused VC pins to an adjacent pin | ✓ |
| 7 VC2, 8 VC1 | J2.3 / J2.2 via R5 / R4; C7, C6 | cells 2 and 1 | ✓ |
| 9 VC0 | J2.1 via R3; C5 to VSS | "through a resistor and capacitor on the pcb to pin 10" | F6 |
| 10 VSS, 33 PAD | BAT− | | ✓ |
| 11 SRP, 12 SRN | R7, R8 100 Ω; C9 100 nF, C10 100 pF differential; SRP on the cell side | §8.2 filter; SRP is the top of the sense resistor | ✓ |
| 13 TS1 | RT1 to VSS, no capacitor | < 4 nF with the 18 kΩ pull-up | ✓ (C2) |
| 14 TS2 | R9 → D2 → POWER | wake below V(WAKEONTS2) | ✓ (F2) |
| 15 REG18 | C4 only | internal use only | ✓ (F8) |
| 16 ALERT | TP1 | may float | ✓ |
| 17 SCL, 18 SDA | R13, R14 100 Ω to J9 | VSS + 6 V maximum | ✓ |
| 19 CFETOFF, 20 DFETOFF | VSS | may float or go to VSS | ✓ (C4) |
| 21 RST_SHUT | R11 10 kΩ, C11 10 nF; R12 1 kΩ from KILL | EVM R13, C12 | ✓ |
| 22 REG1 | open | may float | ✓ (C1) |
| 23 REGIN, 24 BREG | VSS | "both pins should be connected to pin 10" | ✓ (F5) |
| 25 FUSE | VSS | | ✓ |
| 26 PDSG, 27 PCHG | open | "should be left floating" | ✓ |
| 28 LD, 29 PACK | R16, R15 10 kΩ to PACK+ | EVM R56, R55 | ✓ |
| 30 DSG | R19 10 kΩ; R20 4.7 kΩ + D4 turn-off (cathode at DSG) | EVM R50, R51 + D9; §8.5 | ✓ |
| 31 CHG | R17 5.1 kΩ | EVM R47 | ✓ |
| 32 CP1 | C3 470 nF to BAT | 100 / 470 / 2200 nF | ✓ |

The FET network matches the EVM: Q1 CHG (source on FBAT), Q2 DSG (source on
PACK+), common drains; R18, R21 10 MΩ gate–source; D3, D5 16 V (cathode at the
gate); Q4 (source PACK+, drain Q2 gate) with R22, D6, R23 and D7 as EVM R40, D6,
R46, D8. Gate drive 10–13 V (DS 6.24) is below the zeners' 15.2 V minimum and the
FETs' ±20 V.

**U2, all 19 pins** (SLUSEE5E Table 5-1, §6, §8.3, §9.2, §9.5–9.6):

| pin | net and parts | result |
|---|---|---|
| 1 EN/UVLO | PACK+ → R29 → contact A → R30 560 kΩ / R31 100 kΩ, C18 10 nF | ✓ on 7.80 V (7.67–7.93), off 7.34 V (7.27–7.47) |
| 2 INP_G | GND ("if the G drive functionality is unused") | ✓ |
| 3 INP | R32 1 kΩ / R33 100 kΩ | ✓ 3.27 V (2.61 V at the ESP32's VOH minimum) |
| 4 FLT_T, 5 FLT_I | wired-OR, R37 10 kΩ to +3V3_HOST | ✓ |
| 7 IMON | R36 9.1 kΩ, C20 10 nF | ✓ 3.26 V at 19.8 A (N5) |
| 8 IWRN | R34 30.1 kΩ | ✓ 19.8 A (18.8–20.7 A) |
| 9 TMR | C19 680 nF, R35 100 kΩ (latch-off) | ✓ t(OC) 10.8 ms; retry charge 0.21–0.33 V < V(TMR_FLT): stays latched |
| 10 DIODE | Q5 base and collector, emitter GND | ✓ (F4) |
| 11 G | open | ✓ |
| 12 BST | C17 4.7 µF to SRC | ✓ |
| 13 SRC | SERVO+ | ✓ |
| 14 PD | Q3 gate, direct | ✓ |
| 15 PU | R27 100 kΩ to the gate; R28 10 Ω + C16 330 nF to GND (TI Fig. 8-6) | ✓ |
| 17 CS−, 18 CS+ | Q3 drain; R25 100 Ω to R24's top | ✓ |
| 19 ISCP | R26 4.64 kΩ; C15 1 nF to CS− (§9.2.2.4) | ✓ 39.8 A (35–45 A) |
| 20 VS | PACK+, C14 100 nF | ✓ (F9) |

**Discretes:** D1 (pin 1 K), D2 (pins 1, 2 anodes, 3 common cathode), D3–D7,
D8–D9 (cathode on PACK+ / SERVO+), Q1–Q3 (S 1–3, G 4, D 5), Q4 (G 1, S 2, D 3),
Q5 (B 1, E 2, C 3), R1, R24, RT1, every capacitor's voltage rating against its
node, and J1–J12 against harness.md: all ✓, except R29 (F3) and C22's label
(N6).

**Logic crossings** (ESP32 V(IL) 0.83 V, V(IH) 2.48 V, V(OH) ≥ 0.8 × VDD):
KILL → RST_SHUT 3.0 V against ≤ 1.34 V; SERVO_EN → INP 3.27 V against ≤ 2 V;
POWER → TS2 0.1–0.25 V against ≥ 0.7 V; POWER → PWR_INT_N ≤ 0.32 V;
E-stop → EN/UVLO 1.36–1.91 V / 0 V; ESTOP_N 0 / 3.3 V; FLT ≤ 30 mV / 3.3 V;
RAIL_V 2.21 V at 12.6 V: all ✓.

## Not confirmed

- The ESP32's thresholds and injection limits (its datasheet is not in the
  repository); the General Driver's 3.3 V rail impedance when unpowered; the
  STS3215 electronics' current at low voltage; DC-bias derating of C4, C16, C17;
  C21's ESR (no Panasonic datasheet); the unpowered I/O structure of U1's
  SDA/SCL/RST_SHUT and U2's INP/FLT (the abs-max tables reference them to
  VSS/GND only, consistent with no clamp to a supply).
- DS 6.1's row "PACK, PCHG, PDSG, LD: min = max(VBAT − 10, VLD − 10)", read
  literally, would forbid PACK+ = 0 V with the FETs off, which is the normal
  off state of TI's own EVM; taken as a table ambiguity.
- Layout, beyond the custom footprints' pad numbering.
