# Datasheets

Vendor documentation for off-the-shelf parts, downloaded from primary sources so
bringup doesn't depend on a live internet connection or a listing that vanishes.

## Feetech STS3215 / ST3215 serial bus servo (the 10× plant servo)

Files live in [`st3215/`](st3215/). The 3D model is at
[`cad/vendor/ST3215.step`](../../cad/vendor/ST3215.step).

`cad/dimensions.py` derives every servo constant from these documents but only
*cited* their URLs — the files themselves now live here so CAD work does not
depend on Waveshare staying up.

### Which part this is

Sold under many reseller brands (RCmall, DIYmall, Stemedu, Waveshare, Seeed).
They are all the same Shenzhen Feetech part; the **voltage class** is the only
thing that matters when ordering, and the two classes share one case.

| Class | Feetech model | Stall torque | Voltage | Ours? |
|---|---|---|---|---|
| 12 V | **ST-3215-C018** | 30 kg·cm @ 12 V | rated 12 V, 4–14 V | ✅ **yes** — [bom-sourced.md](../bom-sourced.md) row 1 |
| 7.4 V | STS3215 (C001) | 19.5 kg·cm @ 7.4 V | 4–7.4 V | ❌ cannot run on 3S |

Verified against [Amazon B0FLPQQ4FR](https://www.amazon.com/dp/B0FLPQQ4FR?tag=tommwhipple-20)
("RCmall 30KG … 12V …", 6-pack) — that ASIN is the **12 V** class, i.e. C018.

### The files

| File | What it is | Source |
|---|---|---|
| `ST-3215-C018-datasheet-12V-30kg.pdf` | **The authoritative spec for the servo we actually run.** Ed. A/0, 2023-07-20, 8 pp. Chinese, with English column headings. §9 = outline drawing, §10 = torque/speed/current/efficiency curves. | [feetechrc.com](https://www.feetechrc.com/525603.html) |
| `ST3215-outline-drawing.pdf` | **Standalone A4 mechanical drawing** — 3 views, fully dimensioned, with mounting-hole positions. Title block reads "SCS215": same case family, and every dimension matches the C018 outline. | Waveshare [ST3215-2D.zip](https://files.waveshare.com/upload/0/08/ST3215-2D.zip) |
| `ST3215-outline-drawing.dxf` | Same drawing as **editable CAD geometry** — import straight into FreeCAD/OrcaSlicer for tracing bracket profiles. | same zip |
| `ST3215-Servo-User-Manual-EN.pdf` | Waveshare user manual, **English** — register map, packet format, wiring, mode switching. The C018 sheet does not cover the protocol. | [Waveshare wiki](https://www.waveshare.com/wiki/ST3215_Servo) |
| `STS3215-datasheet-7.4V-19kg-EN-machine-translated.pdf` | The **7.4 V** sheet, machine-translated to English. Wrong voltage class — keep it only because it is the one fully-English version of the spec layout. Read torque/current numbers from the C018 sheet, never this one. | [Core Electronics](https://core-electronics.com.au/attachments/uploads/sts3215-smart-servo-datasheet-translated.pdf) |
| `*-outline-*.png` | Rendered quick-look of the two dimensioned pages, so the numbers are greppable from a diff. | rendered locally |
| `../../cad/vendor/ST3215.step` | 3D solid. Loads in build123d; bbox **45.22 × 37.8 × 24.72 mm**. | Waveshare [ST3215-3D.zip](https://files.waveshare.com/upload/5/59/ST3215-3D.zip) |

### Dimensions the drawing calls out (mm)

Case **45.22 × 24.72**, 35 overall across the output axis (case 34.7 + boss),
**37.25** horn face → idler disc face. Output axis **10.11** from the output
end, centred across the width. Horn/idler discs **Ø19.2** on a **Ø14** 4× M3
bolt circle; output spline **25T / OD 5.9**; horn screw **M3×6**. Case mounting
bosses span **20.5** across the width (= ±10.25) with rows at **18.41** and
**6.11** from the two ends — i.e. 8.30 and 29.00 behind the output axis, which
is how `cad/dimensions.py` states them. Connector **5264 3P / 2.54 mm**, 150 mm
lead. Weight **55 ± 1 g**.

These are the same numbers already hard-coded in `cad/dimensions.py` — the
drawing is the receipt for them, not a correction.

## BNO08X (Teyleten Robot "GY-BNO085" 9-DOF IMU breakout)

Purchased as [Amazon B0CL26J81F](https://www.amazon.com/dp/B0CL26J81F?tag=tommwhipple-20) —
"Teyleten Robot GY-BNO085 AR VR IMU High Accuracy Nine-Axis 9DOF AHRS Sensor Module".

There is no manufacturer datasheet for the Teyleten breakout itself (it's an
unbranded carrier board). The authoritative documents are for the **CEVA BNO085**
System-in-Package it carries:

| File | Doc # | Rev | Pages | What it covers |
|---|---|---|---|---|
| `BNO08X-datasheet.pdf` | 1000-3927 | 1.17 (2023) | 59 | Electrical specs, pinout, protocol select, timing, sensor performance |
| `SH-2-Reference-Manual.pdf` | 1000-3625 | 1.9 (Jun 2021) | 95 | Firmware/API: sensor reports, feature commands, calibration, FRS records |
| `SHTP-Reference-Manual.pdf` | 1000-3535 | 1.10 (Jun 2021) | 20 | Sensor Hub Transport Protocol — the framing layer under SH-2 |

Sources: <https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf>,
<https://www.ceva-ip.com/wp-content/uploads/SH-2-Reference-Manual.pdf>,
<https://www.ceva-ip.com/wp-content/uploads/Sensor-Hub-Transport-Protocol.pdf>

### What the part is

BNO085 = triaxial accelerometer + gyro + magnetometer + a 32-bit Cortex-M0+
running CEVA's SH-2 firmware with MotionEngine sensor fusion. Fusion runs
**on-chip** — it emits rotation vectors / quaternions directly, so the host does
no filtering. That is the reason to pick it over a raw IMU.

### Supply (datasheet §1.2.1)

- `VDD` (sensors): **2.4 V – 3.6 V**
- `VDDIO` (core + I/O): **1.65 V – 3.6 V**

Not a 5 V part at the chip level. Whether the Teyleten carrier adds a regulator
and level shifters is **not documented by the seller** — verify on the physical
board before applying 5 V (see "To verify on the bench" below).

### Protocol select — PS1/PS0 (datasheet Figure 1-5)

Sampled at reset, along with `BOOTN`.

| PS1 | PS0 | Interface (BOOTN=1) |
|---|---|---|
| 0 | 0 | **I²C** |
| 0 | 1 | UART-RVC |
| 1 | 0 | UART (SHTP) |
| 1 | 1 | SPI |

Most breakouts ship with PS1/PS0 tied low → I²C by default.

- **I²C**: tie both PS pins to ground. `SA0` (pin 17) selects the low address
  bit → **0x4A** (low) or **0x4B** (high).
- **UART-RVC**: PS1→GND, PS0→VDDIO. Streams heading at 100 Hz over TX with no
  host protocol stack at all. Requires the external crystal/clock — the internal
  oscillator is not accurate enough for UART.
- **SPI**: both PS pins high from before reset until after the first `H_INTN`
  assertion. PS0 doubles as `WAKE` after reset, so it must go to a GPIO, not a
  fixed pull-up.

`BOOTN` low at reset enters the bootloader (DFU); pull it high through 10 kΩ.

### Key chip pins (datasheet Figure 1-6)

| Pin | Name | Function |
|---|---|---|
| 3 | VDD | Sensor supply, 2.4–3.6 V |
| 28 | VDDIO | Core/IO supply, 1.65–3.6 V |
| 4 | BOOTN | Bootloader select, sampled at reset |
| 5 / 6 | PS1 / PS0-WAKE | Protocol select; PS0 = WAKE in SPI mode |
| 11 | NRST | Reset, active low |
| 14 | H_INTN | Interrupt to host, active low |
| 17 | SA0 / H_MOSI | I²C address LSB; SPI data in |
| 18 | H_CSN | SPI chip select, active low |
| 19 | H_SCL / SCK / RX | I²C clock, SPI clock, or UART RX |
| 20 | H_SDA / H_MISO / TX | I²C data, SPI data out, or UART TX |
| 15 / 16 | ENV_SCL / ENV_SDA | Secondary I²C for environmental sensors — **must be pulled up even if unused**, the firmware polls it at reset |

`H_INTN` is not optional in practice: it is how the chip tells the host a report
is ready, and the datasheet calls it required for stable SPI.

### To verify on the bench

These are board-level facts the seller does not publish and that the chip
datasheet cannot answer. Check them against the physical module before wiring:

1. Silkscreen labels and header pin order (clone layouts differ between batches).
2. Whether `VIN` is regulated (5 V-tolerant) or straight to `VDD` (3.3 V only).
3. Whether SDA/SCL have on-board pull-ups, and their value.
4. Whether PS0/PS1 are hard-tied low or exposed as solder jumpers.
5. Measured I²C address — 0x4A or 0x4B — with a bus scan.

## Waveshare General Driver for Robots (candidate controller swap)

Files live in [`general-driver/`](general-driver/). Downloaded 2026-07-28 while
answering "can the IMU daisy-chain onto the servo bus?" — it can't (see
[wiring.md](../wiring.md)), and this board is the proposed alternative because
it carries a 9-axis IMU on board *and* breaks I²C out to a connector. The
proposed harness is drawn in
[`wiring-general-driver.svg`](../wiring-general-driver.svg).

| File | What it is | Source |
|---|---|---|
| `General_Driver_for_Robots-schematic.pdf` | **Full board schematic**, single A3 sheet. The authority for everything below. | [files.waveshare.com](https://files.waveshare.com/upload/3/37/General_Driver_for_Robots.pdf) |
| `JST-XH-connector-datasheet.pdf` | JST XH series, 9 pp. Downloaded for one number: the **current rating of the board's power inlet**. | [jst-mfg.com](https://www.jst-mfg.com/product/pdf/eng/eXH.pdf) |

### What the schematic establishes

- **P1, the IIC header, is `Header 4`: 1 = VDD3V3 · 2 = GND · 3 = IIC_SDA ·
  4 = IIC_SCL** — exactly the BNO085's four wires, in order. I²C is on
  **GPIO 32 (SDA) / GPIO 33 (SCL)**, not 21/22.
- Bus already carries QMI8658C (6-axis, behind an LSF0204PWR level shifter),
  AK09918C (magnetometer), BMP280 and the INA219 at 0x42. **Nothing collides
  with the BNO085's 0x4A/0x4B.**
- **Two** bus-servo ports, H5 and H6, both `Header 3`:
  **1 = DATA · 2 = DC_IN · 3 = GND**, same nets on both — so the one-port-per-leg
  routing survives the swap. Bus UART is GPIO 18 RX / 19 TX, as before.
- **Power inlet H1 is a 2-pin `Header 2+0` (XH2.54).** JST rates XH at
  **3 A AC/DC (AWG #22)**. H5/H6 tap `DC_IN`, so the *whole* servo bus current
  crosses H1, the AO4407 P-FET and SW1 — a 1P2T slide switch whose rating the
  schematic does not state. Against 1.4 A RMS that is fine; against the 6.8 A
  sim-derived peak it is not, and the old board's barrel jack was 5 A.
- **The INA219 does not measure servo current.** Its 0.01 Ω shunt (R11) sits
  between `DC_IN` and `VIN`, i.e. in the buck's feed only, and the servo ports
  tap `DC_IN` upstream of it. What it does give is a **pack-voltage sense
  independent of the servo bus** — which is exactly `battguard`'s blind spot on
  the current board.

### IMU decision (user, 2026-07-28)

**Start on the board's own QMI8658C.** The BNO085 stays on the shelf as the
upgrade path — already owned, so nothing is wasted, and swapping it in later is
a firmware driver change plus one 4-wire cable into P1, with no board or CAD
rework. Consequences of starting onboard:

- The ESP32 must do the fusion the BNO085 did on-chip. Gyro feeds the obs
  angular-velocity terms as-is; the **up-vector needs a complementary/Madgwick
  filter over accel + gyro**. The magnetometer is *not* required — the policy
  consumes up-vector and rates, not heading.
- Cheaper than the read it replaces: ~0.4 ms for the I²C burst plus ~50 µs of
  math, against the 1.0 ms `firmware-design.md` budgeted for the BNO085, in a
  tick with ~16 ms spare. The new risk is filter tuning, not timing.
- **The IMU moves from the tower top to the deck**, so `imu_carrier` is no
  longer needed to carry it — roughly 8 g (carrier + breakout) off the highest
  point on the robot, which lowers the CoM. Attitude and rates are identical
  anywhere on a rigid torso, and sitting nearer the CoM means less lever-arm
  acceleration corrupting the gravity estimate, so the move is neutral-to-better.

Still unverified: SW1's current rating, and — only if the upgrade path is ever
taken — P1's physical connector type and whether it carries I²C pull-ups.
