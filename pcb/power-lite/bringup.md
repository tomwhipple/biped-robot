# Power lite: bench bring-up of the BMS and the board

*The step-by-step version of the README's
[Before it goes in the robot](README.md#before-it-goes-in-the-robot). Each measurement is
checked against the simulation in the README's [Checks](README.md#checks). Parts 1–4 match
README steps 1–3 and the assembly guide's
[order of work](assembly.md#order-of-work). The BMS facts come from its datasheet,
[DS-NA405](../../docs/datasheets/power-lite/shillehtek-na405.pdf) (ShillehTek, rev 1.0,
preliminary).*

The board is assembled, cleaned, inspected and metered before this guide starts (the assembly
guide's steps 1–12 and [Clean, inspect and check](assembly.md#clean-inspect-and-check)). The
leads are made too. Work through the parts in order. Each part has a result table: fill it in
as you go, and stop at the first reading that is outside its range (see
[If a reading is wrong](#if-a-reading-is-wrong)).

| part | what it proves | needs |
|---|---|---|
| [0. Set up](#part-0-set-up) | the bench, the camera and the BMS's real size | — |
| [1. The BMS alone](#part-1-the-bms-alone) | where it cuts off, how it comes back, that it carries 12 A | lab supply, resistors |
| [2. The board on the lab supply](#part-2-the-board-on-the-lab-supply) | the E-stop, the relay and the ESP32 signals, with no pack | lab supply, resistors |
| [3. The BMS and the pack on the board](#part-3-the-bms-and-the-pack-on-the-board) | the whole power path, on the real pack | the pack |
| [4. The relay closing onto the servos](#part-4-the-relay-closing-onto-the-servos) | the inrush does not trip the BMS, and the E-stop always opens the rail | the robot's servos |

**`SERVO_EN` is driven by hand throughout.** The firmware that drives the relay, reads the
E-stop and checks for a stuck rail is in PR #229 and is not on `main` yet (#218). One lab
supply is enough: the board's own supply stands in for the ESP32's 3.3 V, on the breadboard's
**+ rail**.
- **`SERVO_EN`** is driven from the + rail through **three 470 Ω in series** (the **drive
  row**). With R1 that sets Q1's base current to about what an ESP32 pin gives: 5.9 mA at
  12.0 V, against 4.9 mA from 3.3 V through R1 alone (simulation).
- **`ESTOP_N`** is pulled up to the + rail through **47 kΩ**, in place of IO4's pull-up. So
  its high reads the + rail's voltage (12.0 V on the lab supply, the pack voltage in Parts 3
  and 4), not 3.3 V. Q2 is rated 40 V.
- **The BMS's wake-up** (1.2) uses the same supply as the "charger", rewired.

## Safety

- **The pack is a 4.2 Ah lithium-polymer pack.** A short across it, a crushed or punctured
  cell, or overcharge can start a fire. For Parts 3 and 4, work on a non-flammable surface and
  keep a LiPo bag and sand or an extinguisher within reach. Never leave the pack connected
  and unattended.
- **The BMS's corner pads are live at pack voltage** once it is wired: B−, B3 (3.7 V), B2
  (7.4 V) and B1+ (11.1 V). Keep tools, probes and loose wire off them, and don't touch two
  at once.
- **Use the supply's current limit** for every step. Set it before turning the output on. If
  the supply goes into current limit (its CC light) when you didn't expect it, turn the output
  off: something is shorted.
- **The 12 A load gets hot.** At 12 V it takes 144 W. Put it on a heat-proof surface and
  don't touch it for a few minutes after the test.
- **Break current with the supply's output button, not the main switch.** The main switch
  has no DC rating for breaking a load. Turn it off only after the relay has dropped the
  servo rail (README, How power flows).
- **Never connect the + rail straight to a J12 wire.** `SERVO_EN` goes only through the three
  470 Ω, `ESTOP_N` only through the 47 kΩ. Straight to 12 V, `SERVO_EN` would push about 24 mA
  into Q1's base, and `ESTOP_N` would put the supply across Q2 whenever the E-stop is at RUN.
- **Film every powered step** (AGENTS.md), from before the first connection until after the
  last disconnection, with the meter's display in frame where you can.

## What you need

| item | for | notes |
|---|---|---|
| lab supply, current-limited, 0–13 V | the "pack" in Parts 1 and 2 | **15 A or more** for the 12 A test (1.7): the supply the robot runs on |
| multimeter | V, Ω, diode, mA; µA if it has it | |
| 4 × 1 kΩ, ¼ W | three as the "cell" divider, one as a light load | |
| 1 × 47 kΩ, ¼ W | `ESTOP_N`'s pull-up | |
| 4 × 470 Ω, ¼ W | three in series as `SERVO_EN`'s drive; one to wake the BMS (1.2) | spares of R1's 470R |
| a short 22 AWG solid wire | the + rail from the J3 lead in Parts 3 and 4 | |
| 1 Ω, 150 W or more | the 12 A load | or 2 × 2 Ω 100 W in parallel, or an electronic load at 12 A |
| 14 AWG wire, clip leads, a small breadboard | rig wiring; 14 AWG for anything carrying 12 A | |
| the J12 lead and the E-stop lead | from the assembly guide's [Make the leads](assembly.md#make-the-leads) | |
| a PHR-2 housing with its two contacts joined | a J11 jumper | or the E-stop lead with the toggle at RUN |
| camera on a stand | filming | |
| optional: an oscilloscope, an IR thermometer | the inrush trace (Part 4); the load and the BMS's temperatures (1.7) | |

## Part 0: Set up

1. **Measure the BMS** before anything is soldered to it, because the bracket in the power bay
   is drawn from the datasheet's figures, not from a measured board
   (`BMS_*` in `cad/v6/dimensions_v6.py`). Photograph both faces with a ruler, then measure:
   - the outline (silkscreen: 56 × 45 mm) and the PCB thickness (1.2 mm);
   - the overall height over its tallest part (the CAD assumes 8 mm);
   - **the part-free strip along each edge of the component side.** The CAD assumes 2 mm. The
     bracket's hooks and snaps grip 0.8 mm of the PCB's component face: over the P+/P− edge
     between P+ and B2, along the B1+–B3 edge, and on the B3–B− edge. Note anything taller
     than the PCB within 2 mm of those edges.
   - Check the silkscreen: `CF-3SJH25A-C`, with B1+ 11.1V and B2 7.4V on one edge, B3 3.7V and
     B− on the other, and P+ and P− on the edge between B2 and B−. **The taps number
     downward:** B1+ is the top of the pack (11.1 V), B3 the bottom cell (3.7 V). Many 3S
     boards number the other way; wiring to the digits rather than the printed voltages
     destroys the board and a cell (DS-NA405 §3).
2. **Start the camera** (`tools/cam_record.sh`), with the bench and the meter in view.
3. **Write the date, the board's number** (if you have more than one, mark them) **and the
   BMS's number** at the top of the result tables.

## Part 1: The BMS alone

*README step 1. The BMS's thresholds and current rating are "not specified" in its datasheet,
so this part measures them. The lab supply plays the pack; a divider of three equal resistors
gives the two middle taps a third and two thirds of it, so every "cell" is equal.*

### 1.1 Wire the rig (supply off)

```
supply + ──┬───────────────────────── B1+   (11.1 V pad)
         1 kΩ
           ├───────────────────────── B2    (7.4 V pad)
         1 kΩ
           ├───────────────────────── B3    (3.7 V pad)
         1 kΩ
supply − ──┴───────────────────────── B−    (0 V pad)

P+ ──── 1 kΩ load ──── P−      meter across P+ and P−: the BMS's output
```

- Solder short leads to the BMS's pads. Connect **B− first**, then B3, B2 and B1+ (DS-NA405
  §6.2): the order that never puts the full voltage across one monitoring input.
- Set the supply to **11.1 V (3.7 V a cell), current limit 0.1 A**, and turn it on.

### 1.2 First reading and wake-up

1. Meter each tap against B−: B3 about 3.70 V, B2 about 7.40 V, B1+ 11.10 V.
2. Meter the output, P+ to P−. It should read about 11.1 V.
3. **If it reads 0 V, the BMS is asleep.** Boards of this family often ship asleep and wake
   when a charger is put across P+ and P− (DS-NA405 §4). The BMS switches only P−, not P+
   (DS-NA405 §3), so a 12.6 V charger on an 11.1 V pack holds P− about 1.5 V below B−. The
   one supply does the same when its − moves to P− and a 470 Ω between B− and P− sets the
   cells:

   ```
   supply + ──── B1+ (and P+), the divider as in 1.1
   supply − ──── P−
   B− ──── 470 Ω ──── P−        meter: + on B−, − on P−
   ```

   1. Take the 1 kΩ load off P+/P−. Raise the supply to **12.6 V** (4.2 V a cell, a full
      pack), limit still 0.1 A.
   2. Clip the 470 Ω between B− and P−, and a second lead from the supply's − to P−. Nothing
      changes yet: B−, P− and the supply's − are joined.
   3. **Unclip the supply's − from B−.** The divider's 3.6 mA now returns through the 470 Ω:
      B− sits **1.7 V above P−** and the cells at 10.9 V (3.63 V each), the voltages a
      12.6 V charger puts on a 10.9 V pack. Hold 3 s, watching the meter.
   4. **Awake:** B− to P− falls to about 0 V, because the BMS's MOSFETs short the 470 Ω.
      **Asleep:** it stays at 1.7 V.
   5. **Clip the supply's − back onto B− first,** then remove the P− lead and the 470 Ω, so
      the BMS never loses power. Supply back to 11.1 V, the load back on, and measure the
      output again.

   Record whether it needed waking, and whether this woke it.

### 1.3 Standing drain

This is what drains a pack left plugged into the robot with the main switch off: the BMS is
on the pack's side of the switch.

1. Take the 1 kΩ load off P+/P−.
2. Move the supply's − lead off B− and put the meter (mA or µA range) between them, so the
   BMS's own current flows through it. Leave the divider's bottom end on the supply's −, so
   the divider's 3.7 mA does not flow through the meter.
3. Read the current, then put the lead back.

Expected: microamps to a few hundred microamps. Not specified by the maker. At 100 µA a full
4.2 Ah pack would take years to drain; at 1 mA, about 6 months.

### 1.4 Under-voltage cut-off

1. Load back across P+/P−. Supply at 11.1 V.
2. Lower the supply to 10.5 V, then in **0.1 V steps, holding each step 5 s** (the cut-off has
   a delay, not published). Watch the output.
3. **The trip** is the step where the output falls to about 0 V. Record the supply voltage
   and divide by 3 for the voltage per cell.
4. Stop at 7.5 V (2.5 V a cell, the representative cell's discharge cut-off in DS-NA405 §2.2)
   if it has not tripped: that is a fail.

**Pass:** it trips at or below **9.6 V (3.2 V a cell)** and at or above 7.5 V.
**Why 9.6 V:** the firmware warns at 10.5 V and starts landing the robot at **9.9 V
(3.3 V a cell)** (`firmware/components/battguard`). If the BMS cut off above that, the robot
would lose all power mid-step instead of crouching. The 0.3 V below the landing line is a
margin for the landing itself, chosen here, not measured.

### 1.5 Recovery after a cut-off

This decides what happens in the robot after a low-battery cut-off.

1. Straight after the trip, raise the supply in 0.1 V steps up to 11.1 V, holding each 5 s.
   Record the voltage where the output comes back, if it does.
2. If it has not come back at 11.1 V: turn the supply off for 10 s and back on. Record whether
   that brings it back.
3. If not: wake it as in 1.2, steps 3.1–3.5 (the supply as the charger). Record whether that
   brings it back.

### 1.6 A power cycle: the pack swap

Every battery swap unplugs the pack (J1 and J2), so the BMS loses all power. It must come
back on its own when the pack is plugged in again.

1. With the output on at 11.1 V, turn the supply off. Wait 10 s. Turn it on.
2. Record whether the output came back by itself. Do this three times.

**If it needs the wake (1.2) every time,** the robot cannot use it as wired: every swap would
need a charger on P+/P−. Stop here and decide what to do.

### 1.7 12 A for 2 s

The get-up draws 12 A for about 2 s. The BMS must carry it without cutting off.

1. Supply output off. Change the load: the **1 Ω power resistor across P+ and P−, in 14 AWG**,
   on a heat-proof surface. The divider stays as it is.
2. Supply at **12.0 V, current limit 15 A**.
3. Meter across **B− and P−**: the voltage drop across the BMS itself.
4. Turn the output on for **2 s**, then off. Read the drop while it is on, and whether the
   output stayed on.
5. Repeat twice more, 60 s apart. Then check the BMS's MOSFETs and the load for heat (by IR
   thermometer, or carefully by touch on the BMS only).

**Expected:** no cut-off, and a drop of roughly 0.03–0.1 V. Eight 5 mΩ MOSFETs (5–7 mΩ each
at 10 V gate drive, DS-NA405 §2.3), four in parallel twice in series, make about 2.5–3.5 mΩ;
the two 4 mΩ shunts add to that (the simulation takes 2.5 mΩ for the whole BMS). At 0.1 V
the BMS dissipates 1.2 W.
**If it cuts off,** note after how long. The BMS cannot carry the get-up: stop, and decide.

### 1.8 Not tested here

- **The short-circuit cut-off.** It needs a source as stiff as the pack, and a deliberate short
  across a LiPo is not worth the risk. F1 and F2 protect against shorts downstream.
- **The over-charge cut-off.** The board has no charge input: the pack is charged on its own
  charger, off the board.

Desolder the test leads, keeping the pads clean for the 14 AWG and 24 AWG wires of Part 3.

### Part 1 results

| step | reading | expected | measured | pass |
|---|---|---|---|---|
| 1.2 | output at 11.1 V | ≈ 11.1 V | | |
| 1.2 | needed waking? | — | | — |
| 1.3 | standing current | µA (not specified) | | — |
| 1.4 | trip, supply V (÷ 3 = per cell) | 7.5–9.6 V | | |
| 1.5 | recovers at, or how | — | | — |
| 1.6 | back by itself after a 10 s power-off, 3 of 3 | yes | | |
| 1.7 | drop B− to P− at 12 A | ≈ 0.05–0.1 V | | |
| 1.7 | cut off during 3 × 2 s at 12 A | no | | |

## Part 2: The board on the lab supply

*README step 2: the board with no BMS and no pack. The lab supply goes where the main switch's
output would be. Expected values are worked from the parts in `tools/parts.py`, and from the
simulation in the README's [Checks](README.md#checks) where it gives one.*

### 2.1 Before power

- The fuses are in: **15 A in F1** (the upper holder), **3 A in F2** (the lower one).
- Nothing in J1, J2 or J3–J8. W1–W7 are empty: no BMS and no switch yet.
- **Supply:** 12.0 V, **limit 0.5 A**, output off. + to **W7** (silkscreen SW, the right pad of
  the pair on the top edge), − to **W2** (BMS P-). Use short solid wires pushed through the
  pads and clipped.
- **The breadboard's + rail:** a second clip lead from the **lab supply's + terminal**. It
  stands in for the ESP32's 3.3 V. It is taken before F2, so `ESTOP_N` keeps its pull-up with
  F2 out (2.8).
- **J12 test lead:** plug the J12 lead in and bring its four wires to the breadboard:
  - pin 3, GND: to the breadboard's − rail;
  - pin 6, `ESTOP_N`: 47 kΩ to the + rail;
  - pin 5, `RAIL_V`: free, for the meter;
  - pin 2, `SERVO_EN`: free. Put three 470 Ω in series from the + rail to a spare row, the
    **drive row**. Touching the `SERVO_EN` wire to the drive row is "`SERVO_EN` high".
- **J11:** nothing plugged in yet. Open means "E-stop pressed".

Measure every voltage below against **W2 (ground)**, or J12 pin 3, which is the same net.

### 2.2 Power on: E-stop open, `SERVO_EN` free

Turn the supply on.

| check | expected | why |
|---|---|---|
| supply current | about 2 mA | LED1 and its 4.7 kΩ; anything over 50 mA, turn off |
| LED1 (green) | on | it shows the logic feed, after F2 |
| LED2 (red), K1 | off, silent | |
| J3 pin 1 to pin 2 | 12.0 V, pin 1 + | the General Driver's feed |
| `ESTOP_N` (J12.6) | 12.0 V, the + rail | J11 open reads as "stop" |
| `RAIL_V` (J12.5) | 0.00 V | |

### 2.3 E-stop at RUN

Plug the J11 jumper in (or the E-stop lead, toggle at RUN).

| check | expected | why |
|---|---|---|
| `ESTOP_N` | below 0.1 V (simulation: 0.009 V) | Q2 conducts while the coil feed is up |
| K1, LED2 | still off | `SERVO_EN` is free: R2 holds Q1 off |
| supply current | up about 0.2 mA | R5 and R6 |

### 2.4 Rail on

Touch the `SERVO_EN` wire to the drive row and hold it there; a clip is easier.

| check | expected | why |
|---|---|---|
| K1 | clicks | |
| LED2 | on | it shows the servo rail |
| `RAIL_V` | 2.11 V (12.0 × 10/57) | R3/R4 divide SERVO+ by 5.7 |
| J4–J7, pin 2 to pin 3 | 12.0 V | the rail reaches the servo headers |
| supply current | about 0.17 A | the coil's 90 Ω, its built-in 470 Ω, LED2 |
| Q1's saturation: D1's unbanded end to ground | 0.17–0.23 V (simulation: 0.22 V) | the coil's low side |
| `SERVO_EN` current: the meter (mA) between the drive row and the wire | about 5.9 mA (simulation) | an ESP32 pin, through R1, gives 4.9 mA; it can give up to 20 mA |

### 2.5 Rail off by `SERVO_EN`

Let go of `SERVO_EN` (leave it free), then try it tied to ground (the − rail).

| check | expected |
|---|---|
| K1 | drops at once |
| LED2, `RAIL_V` | fade to off over a few seconds: C1 (1000 µF) drains through LED2 |

### 2.6 Rail off by the E-stop

`SERVO_EN` back on the drive row, so the rail is on. Then unplug J11 (or the toggle to STOP).

| check | expected |
|---|---|
| K1 | drops at once |
| `ESTOP_N` | 12.0 V |
| `RAIL_V` | fades to 0 as in 2.5 |

Plug J11 back in (RUN) with `SERVO_EN` still high: **K1 closes again.** That is the hardware
as designed: re-arming after an E-stop is the firmware's job (PR #229).

### 2.7 Over the pack's range

With the rail on (`SERVO_EN` high, J11 in). The + rail follows the supply, so at 9.0 V
`SERVO_EN` gets 4.3 mA, less than an ESP32 pin's 4.9 mA: a harder test. The simulation still
closes K1 there (coil 97 mA, Q1 at 0.18 V).

| supply | check | expected |
|---|---|---|
| 12.6 V (full pack) | `RAIL_V` | 2.21 V |
| 9.0 V (empty pack) | `RAIL_V` | 1.58 V |
| 9.0 V | K1 | stays closed; the coil sees about 8.8 V across D1 (simulation: 8.77 V) |
| 9.0 V | `SERVO_EN` off, then on again | K1 closes again: it operates at 7.2 V |

Set the supply back to 12.0 V.

### 2.8 A missing fuse reads as "stop"

Turn the supply off before pulling or fitting a fuse.

| fuse out | power on, J11 in, `SERVO_EN` high | expected |
|---|---|---|
| F2 | LED1, J3, `ESTOP_N`, K1 | LED1 off, J3 0 V, `ESTOP_N` 12.0 V, K1 does not close |
| F1 | K1, LED2, `RAIL_V` | K1 clicks, but LED2 stays off and `RAIL_V` reads 0 |

Put both fuses back. `SERVO_EN` free, supply off.

### Part 2 results

| step | reading | expected | measured | pass |
|---|---|---|---|---|
| 2.2 | supply current | ≈ 2 mA | | |
| 2.2 | J3 | 12.0 V, pin 1 + | | |
| 2.2 | `ESTOP_N`, J11 open | 12.0 V | | |
| 2.3 | `ESTOP_N`, J11 in | < 0.1 V | | |
| 2.4 | `RAIL_V` at 12.0 V | 2.11 V | | |
| 2.4 | J4–J7 pin 2 | 12.0 V | | |
| 2.4 | supply current, rail on | ≈ 0.17 A | | |
| 2.4 | Q1 saturation | 0.17–0.23 V | | |
| 2.4 | `SERVO_EN` current | ≈ 5.9 mA | | |
| 2.5 | `SERVO_EN` free / grounded drops K1 | yes / yes | | |
| 2.6 | E-stop drops K1; `ESTOP_N` | yes; 12.0 V | | |
| 2.7 | `RAIL_V` at 12.6 / 9.0 V | 2.21 / 1.58 V | | |
| 2.7 | K1 holds and re-closes at 9.0 V | yes | | |
| 2.8 | F2 out: all off, `ESTOP_N` high | yes | | |
| 2.8 | F1 out: no rail | yes | | |

## Part 3: The BMS and the pack on the board

*The assembly guide's [step 13](assembly.md#step-13-wire-the-bms-and-the-main-switch), then the
first connection of the pack.*

### 3.1 Wire the BMS and the main switch (no pack)

Follow [step 13](assembly.md#step-13-wire-the-bms-and-the-main-switch): W1 → B−, W2 → P−
(14 AWG black); W3 → B3, W4 → B2, W5 → B1+ (24 AWG); W6, W7 → the main switch (14 AWG
red). Match the BMS's printed voltages, not the digits: **W3 goes to the 3.7 V pad, W5 to the
11.1 V pad.** The BMS's P+ pad stays empty: the board takes the pack's + straight from J1.

Then meter, with no pack:

| between | reads |
|---|---|
| W3 and the BMS's 3.7 V pad; W4 and the 7.4 V pad; W5 and the 11.1 V pad; W1 and B−; W2 and P− | 0 Ω |

### 3.2 Check the pack

- **P1:** the pack's XT60 is female, with + on the housing's straight side
  ([harness](harness.md#j1-the-pack)).
- **P2:** meter the balance plug from the pin-1 side: 0, about 3.7, 7.4, 11.1 V.
- The three cells match within 0.05 V. If not, balance-charge it first.
- Write down the pack voltage.

### 3.3 Plug in the pack, switch off

Set up as in Part 2: J4–J8 empty, the J12 test lead with its pull-up and drive row, and the
**E-stop lead in J11 at STOP**. Remove the lab supply. **Main switch off.**
- **The + rail now comes from J3 pin 1**, the logic feed: the pack voltage once the main
  switch is on, nothing while it is off. Plug the J3 lead into J3. Push the 22 AWG solid wire
  into its far housing's pin 1 (the + wire, harness check P3) and take it to the + rail.
  Leave the far pin 2 empty.

1. Plug the **balance plug into J2 first**, then the **XT60 into J1**.
2. Measure:

| between | expected | if not |
|---|---|---|
| W6 to W1 (pack + to −) | the pack voltage | pack or J1 |
| W3 to W1, W4 to W1 | one cell, two cells | the balance wiring |
| W6 to W2 (pack + to the BMS's output) | the pack voltage | 0 V: the BMS is asleep or tripped |
| LED1 | off | the switch is off |

**If W6 to W2 reads 0:** wake the BMS the way 1.2 found. With the pack in, the lab supply is
free to be the charger: 12.6 V, limit 0.2 A, + to W6 and − to W2 for 3 s. Measure again.

### 3.4 Switch on

Main switch on.

| check | expected |
|---|---|
| LED1 | on |
| J3 | the pack voltage, pin 1 + |
| `ESTOP_N` (E-stop at STOP) | the pack voltage, the + rail |
| E-stop to RUN: `ESTOP_N` | below 0.1 V |
| `SERVO_EN` to the drive row: K1, LED2 | clicks, on |
| `RAIL_V` | pack voltage ÷ 5.7 |
| E-stop to STOP | K1 drops |

### 3.5 Switch off, and the pack swap

1. **In this order:** `SERVO_EN` free, E-stop at STOP, **then** the main switch off. The switch
   then breaks only the logic's few milliamps.
2. Unplug J1, then J2. Wait 10 s. Plug J2, then J1, switch on. Record whether the BMS came
   back by itself (W6 to W2 reads the pack voltage, LED1 on). Part 1.6 says what to expect.

### Part 3 results

| step | reading | expected | measured | pass |
|---|---|---|---|---|
| 3.2 | pack voltage; cells | ≈ 11.1–12.6 V; within 0.05 V | | |
| 3.3 | W6 to W2, switch off | pack voltage | | |
| 3.4 | `RAIL_V` | pack ÷ 5.7 | | |
| 3.4 | E-stop STOP drops K1 | yes | | |
| 3.5 | BMS back by itself after the swap | yes | | |

## Part 4: The relay closing onto the servos

*README step 3. When K1 closes, C1 and every servo's own input capacitor charge at once from
the pack. The simulation puts the peak at 176–341 A, above 100 A for 52–132 µs. The BMS's
short-circuit detector may read that as a short. Its delay is not published.*

### 4.1 Set up

- The robot on its stand, legs hanging free.
- Servo leads in J4–J7. **J8 empty and the General Driver not connected**, so nothing can
  command the servo bus. Any bus write is a motion command (AGENTS.md, Bench safety).
- The pack in, as in Part 3. The J12 test lead with its pull-up and drive row, and the + rail
  from J3 (3.3). The E-stop lead in J11.
- Optional: an oscilloscope on SERVO+ (a J4 pin 2) and on J3 (the logic feed), triggered on
  SERVO+ rising.
- Camera running.

### 4.2 Ten closures

Main switch on, E-stop at RUN. Then, ten times:

1. `SERVO_EN` to the drive row. K1 closes onto the servos.
2. **Watch that the BMS holds.** LED1 stays on, and J3 (or the meter on `RAIL_V`) does not
   drop to 0. If the board goes dark, the BMS has cut off: record which closure, and go to
   [If the BMS cuts off on closing](#if-the-bms-cuts-off-on-closing).
3. Wait 5 s. E-stop to STOP. **K1 must drop:** `RAIL_V` falls to 0, faster than in Part 2
   because the servos now load C1.
   - **If `RAIL_V` stays at the pack voltage with the E-stop at STOP, K1's contact has welded**
     and the E-stop no longer cuts the servos. Unplug the pack at once. K1 must be replaced
     before anything else.
4. `SERVO_EN` free, E-stop back to RUN, and **wait 30 s** so every closure starts from empty
   capacitors.

Afterwards: `SERVO_EN` free, E-stop at STOP, main switch off, unplug J1, then J2.

### Part 4 results

| closure | BMS held | K1 dropped on STOP | notes (scope peak, if taken) |
|---|---|---|---|
| 1 | | | |
| 2 | | | |
| 3 | | | |
| 4 | | | |
| 5 | | | |
| 6 | | | |
| 7 | | | |
| 8 | | | |
| 9 | | | |
| 10 | | | |

**Pass:** the BMS holds on all ten, and K1 drops on every STOP.

### If the BMS cuts off on closing

Recover it the way Part 1.5 found. Then the choices, from the README, are:
- a smaller C1;
- a pre-charge resistor in series with C1.

Both can be simulated first with `sim/lite_sim.py`. Use the measured BMS values from Part 1.

## If a reading is wrong

- **Stop at the first wrong reading.** Turn the supply off, or unplug the pack (J1, then J2).
  Write the reading down before you change anything.
- **A supply current far above expected** (in current limit) is a short or a backwards part:
  D2, D3 or C1 first (the assembly guide's diode checks), then a solder bridge.
- **`ESTOP_N` reads high (the + rail's voltage) with the E-stop at RUN**: F2, the J11 lead, or
  Q2's orientation.
- **K1 does not click with `SERVO_EN` high**: J11 (it must be closed), the drive row (about
  6 mA flows through it), the − rail to J12 pin 3, or Q1's orientation.
- **The supply hits its current limit the moment `SERVO_EN` goes high**: D1 may be in
  backwards, shorting the coil's feed through Q1. Turn off at once. D1's band goes on the
  coil's high side (the assembly guide's step 2).
- Compare with the simulation before changing a part. A difference is a question about the
  board or about the model; write down which.

## After bring-up

1. **Write the record** as `docs/design-v6/<date>-power-lite-bringup.md`: the four result
   tables filled in, the BMS's measured size (Part 0), and the paths to the video and photos.
2. **Put the measured BMS size into `BMS_*`** in `cad/v6/dimensions_v6.py` and re-run
   `make check-cad` if it changed. Put its cut-off, its drop and its standing current into
   `sim/lite_sim.py`'s BMS model.
3. **Update #207** with the results. Then the firmware's bench check (#218, PR #229) can
   start, and so can the harness checks that need the servos: the 5264 contacts' temperature
   (harness, J4–J8) and, on a bench supply, check T1.
