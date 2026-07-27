# Datasheets

Vendor documentation for off-the-shelf parts, downloaded from primary sources so
bringup doesn't depend on a live internet connection or a listing that vanishes.

## BNO08X (Teyleten Robot "GY-BNO085" 9-DOF IMU breakout)

Purchased as [Amazon B0CL26J81F](https://www.amazon.com/dp/B0CL26J81F) —
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
