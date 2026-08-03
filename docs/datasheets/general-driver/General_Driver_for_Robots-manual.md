# General Driver for Robots — user manual (captured from the Waveshare wiki)

*Waveshare publishes no user-manual PDF for this board — the wiki page IS the
manual. Captured 2026-08-03 from
<https://www.waveshare.com/w/index.php?title=General_Driver_for_Robots&oldid=103968>
(stable permalink). Companion files in this directory:
[schematic](General_Driver_for_Robots-schematic.pdf) ·
[dimensions](General_Driver_for_Robots-dimensions.pdf) ·
[JST XH connector datasheet](JST-XH-connector-datasheet.pdf).*

> ## ⚠️ Two facts that matter for the bimo swap
>
> **Power inlet is XH2.54, not a barrel jack.** The current Servo Driver with
> ESP32 takes the battery lead through a DC-044 5.5×2.1 mm barrel plug; this
> board takes a **JST XH 2.5 mm-pitch 2-pin plug** (wiki: "Input DC 7~13V,
> this interface directly powers the serial bus servo and motor"). The XH
> contact is rated **3 A with AWG #22** and physically accepts **nothing
> thicker than #22** (see the JST datasheet here) — the existing 20 AWG
> switch lead cannot be crimped into it.
>
> **The bus-servo path is limited to 5 A continuous.** Waveshare's FAQ, on
> this exact board: *"the current output limit for bus servos is controlled
> by a switch, with a maximum current of 5A for long-term operation … with
> the ST3215 Servo, it can support up to 5 servos (ensuring that these 5
> servos are not stalled simultaneously)."* bimo has **8× ST3215** and a
> wiring transient budget of ~10 A ([wiring.md](../../wiring.md), BOM
> item 20). A biped catching itself stalls several servos at once. This
> needs an explicit power plan (see [bom-sourced.md](../../bom-sourced.md)
> power-pigtail item) before the swap is committed on hardware.

## Introduction

Multifunctional driver board for robots based on the ESP32-WROOM-32 module.
Develop with the Arduino IDE; supports WIFI, Bluetooth, and ESP-NOW wireless
communication. Onboard interfaces: DC motor with/without encoder, bus servo,
IIC, Lidar, PWM servo, SD card. Onboard resources: 9-axis IMU, temperature
sensor, automatic download circuit, Lidar serial-to-USB circuit, bus servo
control circuit.

Note (Waveshare): the PWM servo headers do **not** support MG996R/MG90S or
other high-power PWM servos; recommended PWM servo is the WP90.

## Features

- ESP32-WROOM-32; WIFI / Bluetooth / ESP-NOW.
- Motor control: 2× DC with encoder, or 4× DC (two groups) without encoder.
- Serial bus servo interface: up to 253 ST3215 servos with feedback
  (electrically limited per the FAQ above).
- Onboard 9-axis IMU (AK09918C compass + QMI8658 6-axis).
- 7~13 V input — 2S or 3S lithium battery direct.
- Automatic download circuit; input voltage/current monitoring (INA219);
  TF-card slot; Lidar interface with UART-to-USB; IIC expansion;
  multi-functional extended interface; 40-pin header for a Raspberry
  Pi / Jetson host (communicates and powers it).

## Parameters

| Parameter | Value |
|---|---|
| Main controller | ESP32-WROOM-32 |
| Power supply | DC 7–13 V |
| Power port | **XH2.54 (JST XH 2-pin)** |
| Antenna connector | IPEX1 |
| Download interface | Type-C |
| Wireless | WIFI, Bluetooth, ESP-NOW |
| Dimensions | **65 × 65 mm** |
| Mounting holes | **49 × 58 mm spacing, Ø3 mm** |

(The current Servo Driver with ESP32 is 65 × 30 mm, holes Ø2.75 on 58 × 23 —
the tower mount in `cad/dimensions.py` does not fit this board without
redesign.)

## Onboard resources

| # | Resource | Notes |
|---|---|---|
| 1 | ESP32-WROOM-32 | Arduino-IDE developable |
| 2 | IPEX1 WiFi connector | external antenna |
| 3 | LIDAR interface | integrated radar adapter function |
| 4 | IIC expansion | OLED / IIC sensors |
| 5 | Reset button | reboot ESP32 |
| 6 | Download button | hold at power-on → download mode |
| 7 | DC-DC 5 V regulator | powers a host SBC (Pi / Jetson) |
| 8 | Type-C (LIDAR) | LIDAR data |
| 9 | Type-C (USB) | ESP32 UART / program upload |
| 10 | **XH2.54 power port** | **DC 7~13 V in; directly powers servos + motors** |
| 11 | INA219 | voltage/current monitor |
| 12 | Power ON/OFF | switches external power |
| 13 | ST3215 bus servo interface | |
| 14, 15 | Motor PH2.0 6P (groups B, A) | motors with encoder |
| 16, 17 | Motor PH2.0 2P (groups A, B) | motors without encoder |
| 18 | AK09918C | 3-axis compass |
| 19 | QMI8658 | 6-axis motion sensor |
| 20 | TB6612FNG | motor driver |
| 21 | Bus-servo control circuit | multiple ST3215 + feedback |
| 22 | SD card slot | logs / WIFI config |
| 23, 24 | 2× 40-pin headers | Raspberry-Pi-format host connection |
| 25 | CP2102 | UART→USB, radar data |
| 26 | CP2102 | UART→USB, ESP32 console |
| 27 | Auto-download circuit | flash without EN/BOOT buttons |

(Connector→GPIO net mapping read off the schematic is in
[sensor-expansion.md](../../sensor-expansion.md) §1.)

## Tutorials (on the wiki)

How To Install Arduino IDE (uses the UGV01 demo); then per-peripheral demos:
I–III motor with encoder, IV motor without encoder, **V ST3215 serial bus
servo control**, VI PWM servo, VII IMU data reading, VIII SD card, IX INA219
voltage/current monitoring, X OLED screen, XI Lidar + ROS2 topics. All linked
from the wiki page above.

## Resource downloads

- Demo code: <https://files.waveshare.com/upload/0/0c/UGV01_BASE.zip>
- Schematic: <https://files.waveshare.com/upload/3/37/General_Driver_for_Robots.pdf>
- Dimensions PDF: <https://files.waveshare.com/upload/0/0a/GENERAL-DRIVER-FOR-ROBOTS-STR-PDF.zip>
- Dimensions DXF: <https://files.waveshare.com/upload/5/50/GENERAL-DRIVER-FOR-ROBOTS-STR-DXF.zip>
- STEP model: <https://files.waveshare.com/upload/8/8e/General_Driver_for_Robots_STEP.zip>

## FAQ (as published, abridged)

**Q: Maximum current of the bus-servo interface? How many servos?**
A: Output limit is controlled by a switch, **maximum 5 A for long-term
operation**; with ST3215, up to 5 servos, ensuring they are not stalled
simultaneously.

**Q: No new COM port when plugged into a computer?**
A: Check Device Manager for an unidentified "CP2102" device → install the
CP2102 driver; if absent entirely, contact Waveshare support.
