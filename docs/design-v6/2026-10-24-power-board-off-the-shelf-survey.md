# Power board: is anything off the shelf similar and cheaper? (2026-10-24)

*A dated record. Tom's question before ordering: "make sure that there's not a substantially
similar product available off-the-shelf for less money, possibly as an RPi hat or something."
The custom board is [pcb/power-board](../../pcb/power-board/README.md). Its price is in
[2026-10-24-power-board-quote.md](2026-10-24-power-board-quote.md): about $101 a board for 2
assembled, $78 for 3, $60 for 5, from instant quotes. The 2026-10-06 options study is
[2026-10-06-power-circuit.md](2026-10-06-power-circuit.md) §5.2 and §13.*

## Answer

**No.** Nothing off the shelf does substantially the same job for less.

- **No single product meets more than two of the board's four core requirements.** No Pi
  HAT, robot power board, FPV power module, BMS module or evaluation board combines them:
  1. cell protection that needs no firmware;
  2. I²C telemetry;
  3. a soft main switch;
  4. a servo rail with a hardware E-stop.
- **Requirement 4 has no buy-ready product under $100:** a switched servo rail with a
  no-firmware E-stop, a ~20 A breaker and soft start.
- **The closest combination costs more and does less.** It is about $120–125 a robot plus
  a wiring harness, against the custom board's $101 at 2.
- **Recommendation: order the custom board**, after rev B and the reviews in progress.

## What was searched

- **Raspberry Pi power HATs** (Waveshare, Geekworm, PiJuice, SugarPi, Pimoroni, Sequent).
  All are 5 V UPS boards that hold their own cells. The nearest, Waveshare's UPS Module
  3S, takes three 18650 cells:
  - 5 V at 5 A out, plus the series pack voltage;
  - INA219 monitoring and an S-8254 protector;
  - 60 × 93 mm, $21.99.

  It cannot use the robot's LiPo, reports no per-cell voltages, and has no E-stop or
  switched high-current rail.
- **Robot power boards.** ROBOTIS and Lynxmotion use the wrong connectors and have no
  protection. REV's Power Distribution Hub is $250 and 517 g, with fuses and no E-stop path.
- **FPV and RC power modules** (Matek I2C-INA-BM, Holybro PM02D). They give pack voltage and
  current, and the INA228's charge count, over I²C. They have no per-cell voltages, no
  protection and no switching.
- **BMS modules.** JBD and Daly smart BMSs speak UART, Bluetooth or CAN, not I²C, and were
  rejected on 2026-10-06 for size and ground switching. Rotoye's BATMON v5 per-cell SMBus
  monitor is from $155, and ships six weeks after order.
- **Electronic switches and evaluation boards.** TI's eFuse boards cost $251–328, and ADI's
  hot-swap board is out of stock. Infineon's 4-channel PROFET shields are rated 10 A a
  channel, and the smaller eFuse and click boards 1–6 A.
- **Not reached in the time:** Yahboom, Kondo, UBTECH, AndyMark, CTRE, VEX, MikroE's BMS
  click boards and Daly's own pages. None came up in the broad searches.

## The closest combination (C2) against the custom board

| requirement | C2 | what it gives up |
|---|---|---|
| 1. cell protection without firmware | JBD 3S 20 A protection board ($21.20–37.80 for five) | Over-voltage 4.25 V and under-voltage 2.7 V are on its page, but there is no datasheet, and it does not say which lead it switches. Common 3S boards switch the negative lead, which splits the ground the ESP32, the Pi and a USB laptop share |
| 2. I²C telemetry | Matek I2C-INA-BM (INA228, about $15) | no per-cell voltages, no protection flags |
| 3. soft main switch | Pololu Big Pushbutton Power Switch HP #2813 ($8.95) | No press report: it needs extra wiring for an orderly shutdown. Rated 6 A continuous at 55 °C, so it suits the logic feed only |
| 4. servo rail with E-stop | Infineon 12 V Protected Switch Shield, SHIELD-BTS50015-1TAD ($92.68) | No adjustable 20 A / 10 ms breaker, no 40 A short-circuit trip, no latch. Infineon calls its evaluation boards "not commercial products". 0 in stock at Digi-Key, 1 due 2026-11-10 |
| 5–6. distribution, fit | a harness and an inline fuse | four modules and their wiring instead of one board |
| **cost per robot** | **≈ $121–125 + harness** | against $101 at 2 boards, $78 at 3 |

- **The only cheaper set is C1, at about $30:** C2 without the Infineon shield. It drops
  the E-stop altogether and still has no per-cell telemetry, so it is not substantially
  similar.
- **A single robot is the one case where outlay differs.** The custom board's minimum order
  is two boards (~$203, one a spare), more than C2's ~$125. But C2 cannot be ordered before
  November and misses parts of requirements 1, 2 and 4.

## Corrections to what the design record assumed

- **TI's BQ76922EVM is in stock again**: $232.96 at Digi-Key, 6 units. It was out of stock
  on 2026-10-06.
  - It is 3.0 × 4.9 in, rated 0–10 A continuous and populated for 5 cells (user guide
    SLVU957A, in `docs/datasheets/power-board/`).
  - That rules it out for the robot, but it is again usable for bring-up on the bench:
    design-record decision D6 tests the configuration and sequences on it while the board
    is made.
- **TI's TPS48111Q1EVM** is $232.96, with 18 in stock. It has the same IC as the servo
  switch, but its user guide sets it up for 24–60 V with a 24 V UVLO, so 9–12.6 V would need
  SMD rework.

## Sources and what was re-checked

- **The survey** was done by a research sub-agent, which fetched the makers' and
  distributors' pages on 2026-10-24. It kept its source list and marked fact, snippet and
  inference apart in its working notes (under `build/` on the machine that ran it; not in
  git).
- **Re-checked here, on the makers' pages:**
  - Pololu #2813: $8.95; 6.0 A continuous at 55 °C; under 0.2 µA off; its warning "Do not
    use this switch as an emergency cutoff or similar safety disconnect".
  - Waveshare UPS Module 3S: three 18650 cells; 5 V at 5 A; INA219 and S-8254AA; 60 × 93 mm;
    $21.99.
  - TI's TPS48111Q1EVM page: "5-A to 50-A adjustable over current protection using onboard
    jumpers".
- **Taken from the survey without a second check:** the Infineon shield's figures. Its
  product page renders by script, and its guide did not load here. Treat them as the
  survey's reading.
