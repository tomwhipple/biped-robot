# Sensors and perception on the General Driver

This page covers three things: what the controller board's connectors carry, where
a new sensor would plug in, and how the robot's perception (a Raspberry Pi 4B and
a head camera) connects to the ESP32.

- Everything about nets and GPIOs is read off the vendor schematic, `datasheets/general-driver/General_Driver_for_Robots-schematic.pdf`.
- Power and current are in [wiring.md](wiring.md).

## 1. The board's connectors

The item numbers (№) refer to the vendor's annotated photo,
`datasheets/general-driver/General_Driver_for_Robots-connector-diagram.jpg`.

| ref | № | connector | nets | robot use |
|---|---|---|---|---|
| H1 | 10 | JST XH 2-pin | 1 V+ (`VDD_DC_JACK`) · 2 GND, then the AO4407 reverse-polarity FET and SW1, onto `DC_IN` | **power in** |
| SW1 | 12 | slide switch | switches `DC_IN` | on/off |
| H5, H6 | 13 | 3-pin, "D V G" | 1 `DATA` · 2 `DC_IN` · 3 GND. UART1, GPIO 18 RX / 19 TX | **the servo bus** |
| USB-C `USB` | 9 | CP2102N | ESP32 UART0, with the DTR/RTS auto-program circuit on `EN`/`GPIO0` | **flashing, CLI** |
| USB-C `LIDAR` | 8 | CP2102N | its RXD takes `CP_RX` from H7; its TXD is unconnected | free; not wired to the ESP32 |
| P1 | — | 4-pin | 1 3V3 · 2 GND · 3 `IIC_SDA` (GPIO 32) · 4 `IIC_SCL` (GPIO 33) | free |
| P2 + P4 | 23, 24 | 2 × 40-pin, the Raspberry Pi footprint, **numbered mirrored** in the schematic | Per the schematic (re-read 2026-10-02): pins **1 and 3 are 5 V**, **4 is `IIC_SDA`**, **6 is `IIC_SCL`**, 7/9 are `P_TX`/`P_RX` (ESP32 UART0). The earlier "pins 3/5 are I²C" was wrong. Which physical corner is pin 1 is unverified, so wire I²C sensors to P1, whose silkscreen names each pin | free |
| P3 | — | 7-pin | 1 IO5 (10 Ω) · 2 3V3 · 3 GND · 4 IO16 · 5 IO27 · 6 `CP_RX` · 7 `U0RX` | free |
| H7 | — | PH2.0 4-pin, "LIDAR" | 1 `CP_RX` · 2 n/c · 3 GND · 4 5 V | free |
| H2 | — | 3-pin | 1 IO4 (10 Ω) · 2 5 V · 3 GND. The schematic labels it 舵机接口 ("servo port") | free. The firmware names GPIO 4 `kRgbLed` |
| H3 | 15 | 6-pin | 1 motor A · 2 GND · 3 `A_C2` = GPIO 35 · 4 `A_C1` = GPIO 34 (10 Ω each) · 5 3V3 · 6 motor A | free |
| H4 | 14 | 6-pin | 1 motor B · 2 GND · 3 `B_C2` = GPIO 16 · 4 `B_C1` = GPIO 27 (10 Ω each) · 5 3V3 · 6 motor B | free |
| MOTOR-A1/A2, B1/B2 | 16, 17 | 2-pin connectors | TB6612 motor outputs; the driver is powered from `DC_IN` | free |
| TF1 | 22 | microSD | SPI: CS IO15 · MOSI IO13 · CLK IO14 · MISO IO12 | free |

Physical layout:

- The service edge is a 56.01 mm edge. It carries the XH inlet and both USB-Cs.
- SW1 sits on the adjacent bus edge, near that corner (`BOARD_GD_U_*` in `cad/dimensions.py`).
- In the robot the board stands vertical on the pelvis front wall. Its service edge faces +y through a side window (`cad/v6/dimensions_v6.py`).

The on-board I²C bus (GPIO 32/33, 400 kHz) already carries these parts:

| address | part | status |
|---|---|---|
| 0x6B | QMI8658C 6-axis IMU | the robot's IMU. It sits behind an LSF0204 level shifter. If the bus is flaky at 400 kHz, try 100 kHz before suspecting the driver (`board.h`) |
| 0x0C | AK09918C magnetometer | unused. Beside the servos it measures their current, not north |
| 0x77 | BMP280 barometer | on the schematic; it did not answer the bench scan |
| 0x42 | INA219 | unused. It measures pack voltage and the board's own logic current, never servo current ([wiring.md](wiring.md#what-the-ina219-measures)) |
| 0x4A / 0x4B | reserved | for a BNO085 on P1, the firmware's named upgrade path from the QMI8658C. `imu scan` recognises it |

What the robot uses: H1, SW1, H5/H6, the `USB` port and the on-board IMU.
Everything else is free.

A few things about the USB ports:

- Both USB-C ports power the logic. Their VBUS lines are diode-OR'd into the 5 V rail.
- Neither port powers `DC_IN`. The servos need the pack.
- Only `USB` flashes. `LIDAR` enumerates as a serial port that never syncs.

## 2. Where a new sensor goes: does it touch the 20 ms tick?

The tier a sensor falls into, not its wiring, decides what integrating it costs.

- **Tier 1: in the policy observation.**
  - The sensor is read on the ESP32 inside the tick. It must also exist in the MuJoCo plant, with a noise model, and the policy must be retrained.
  - Adding a channel changes the observation width (`obs_spec.h`), which invalidates every trained policy.
  - Foot contacts would be Tier 1.
- **Tier 2: supervisory.**
  - Pack voltage, temperature, current, fault flags.
  - Read off-tick at a few Hz. Never fed to the network.
  - It needs a driver, not a retrain.
- **Tier 3: perception.**
  - The camera, lidar, and anything else that produces more data than a 240 MHz Xtensa should look at inside a hard 20 ms loop.
  - Tier 3 never enters the observation. It enters through **`cmd`**, the command block the policy already consumes. That block is plumbed end to end over the link ([control-channel.md](control-channel.md)).
  - A camera setting `cmd` at 10 Hz needs no retrain, no sim change and no new observation dimension.

The tick has room, though the robot's figure is unmeasured:

- On the 10-joint prototype, policy inference (a 128 × 128 network) measured 7.2 ms of the 20 ms tick.
- `stat` reports the per-phase times.
- The robot's 17-joint network has not been measured.

## 3. Perception: the Pi and the head camera

- **Camera.** A Raspberry Pi Camera Module 3 **Wide** (IMX708, 102° horizontal FOV).
  - It sits on M2 bosses in `head_face`, about 30 mm above the neck horn; the camera is 0.53 m above the floor.
  - Its 300 mm ribbon runs through a slot beside the neck axis to the Pi's CSI port.
- **Neck.** An STS3215 standing on the deck, with ±90° of yaw.
  - It is on the ESP32's servo bus like every other joint, so the Pi can only turn the head through the ESP32.
  - How head yaw is commanded is part of the open firmware port.
- **Pi.**
  - A Raspberry Pi 4B stands vertical on the pelvis aft wall.
  - It has a low-profile heatsink and no fan, because there is 16 mm of component depth.
  - USB and Ethernet come out through a side window.
  - It is powered from its own D24V50F5 buck ([wiring.md](wiring.md#pi-supply)).
  - It is not on the board's 40-pin header.
- **Division of labour.**
  - The ESP32 runs the 50 Hz loop, the servo bus, the IMU and the guards.
  - The Pi runs vision and navigation. It sends intent (`cmd`: velocity, crouch, and so on), never joint targets.

**The Pi ↔ ESP32 link is not chosen yet.** These are the options:

| option | wiring | what it takes | caveats |
|---|---|---|---|
| Wi-Fi UDP | none | Works today. The Pi runs a commander that speaks `link/protocol.py` to port 4210 and reads the 10 Hz beacon on 4211 | Two boards 10 cm apart talk through the access point. One commander at a time (control-channel "Taking control") |
| USB to the `USB` port | one cable | The CP2102N reaches UART0, the CLI console at 115200. Needs a binary host mode in firmware | Opening the port toggles DTR/RTS, which the auto-program circuit turns into an ESP32 reset. UART0 is also the console |
| Wired UART | 3–4 wires | Option 1: UART0, on P3 pin 7 (`U0RX`) and on the 40-pin's `P_TX`/`P_RX`, shared with the `USB` bridge through 1 kΩ. Option 2: a second UART (the ESP32's UART2 through the GPIO matrix) on P3's IO16/IO27/IO5. Both sides are 3.3 V logic | Either option needs firmware work. The Pi is not on the header, so it is a small harness |

Also open: pack voltage for the Pi's low-battery shutdown. The telemetry beacon
carries `vbat` from register 62, so the Wi-Fi option gets it for free. The INA219 is
not read.

## 4. Other sensors, by connector

- **Foot contact switches → H3 / H4.**
  - The wiring is free: 3V3, GND and two inputs per housing, no soldering.
  - H3's inputs are GPIO 34/35. These are input-only pins with **no internal pull-up or pull-down**, so each switch needs an external pull-up. The 10 Ω series parts are protection, not pull-ups.
  - H4's GPIO 27/16 are normal bidirectional pins, but they are shared with P3 pins 5/4. You can use H4 or those two P3 pins, not both.
  - Contacts are Tier 1. They need MuJoCo touch sensors, a bounce/chatter model (real switches chatter; simulated ones do not) and a retrain.
- **I²C sensors → P1.**
  - Four wires on GPIO 32/33. Avoid the addresses listed in §1.
  - The IMU sampler reads this bus every 4 ms (250 Hz), so every added device shares the IMU's critical path.
  - Unverified: P1's physical connector type, and whether it has pull-ups. Check both on the board in hand before ordering a cable.
  - A BNO085 on P1 is the named IMU upgrade path. Fitting one is a firmware driver plus one 4-wire cable.
- **Lidar → H7.** A PH2.0 4-pin connector wired for an LD19/LD06-class unit: its data line (`CP_RX`) goes to the `LIDAR` USB-C bridge and to P3 pin 6. It is Tier 3. Nothing in the plant or the policy consumes range data.
- **Serial sensors → P3.** IO5, IO16 and IO27 with 3V3 and GND, through a UART routed in firmware. Mind the H4 sharing above.
- **Logging → TF1.**
  - A microSD card on SPI. It is not a boot device: the ESP32 boots from the module's own flash.
  - **GPIO 12 (`SPI_SO`) is the MTDI strapping pin.** A card that drives it high at reset selects the wrong flash voltage.
  - Check that the board boots with a card inserted before relying on it.
- **Signal header → H2.** IO4, 5 V and GND.

## 5. The constraints that bind

1. **The observation contract.** Tier 1 sensors invalidate trained policies and need a sim model that includes their noise. Tier 3 sensors that feed `cmd` cost nothing on that side.
2. **Input current.** The whole servo bus crosses the 3 A XH contact and a path that Waveshare rates at 5 A continuous ([wiring.md](wiring.md#current-the-open-constraint)). A 5 V sensor load rides the board's buck and the INA219 shunt instead, so small sensors are not the problem.
3. **Volume and mass.** The pelvis has the Pi bay and the head, and no reserved sensor volume beyond them. Mass above the deck moves the CoM, and the plant and the policy are downstream of that.
4. **Tick time and the I²C bus.** Anything read inside the tick, or on the IMU's bus, spends the loop's slack.
5. **Pins and connectors.** The most abundant resource on the board.
