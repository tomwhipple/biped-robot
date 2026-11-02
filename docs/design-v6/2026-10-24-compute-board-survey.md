# Compute board survey: a replacement for the Pi 4B (2026-10-24)

**Question:** is there a board in the Raspberry Pi form factor with more compute,
or less power draw, than the Pi 4B, that this robot can take?

**Answer:** nothing draws meaningfully less than the Pi 4B. Two boards offer
more compute and fit the torso's board stack unchanged:

- the **Raspberry Pi 5**, the only drop-in;
- the **Radxa ROCK 5C (V2.1)**, which adds a 6 TOPS NPU but cannot run the
  Camera Module 3 Wide.

Neither is worth fitting before the vision workload exists.

## Constraints, from the design on this date

| constraint | value | where it comes from |
|---|---|---|
| mount | 85 × 56 mm board, M2.5 holes on 58 × 49 mm, 40-pin header in the Pi position. The General Driver plugs onto it as a HAT on a 17 mm riser | DESIGN.md, *the board stack* (PR #136) |
| power | 5 V into the 40-pin header from the General Driver's MP8759. No USB-PD negotiation | [power-circuit record](https://github.com/tomwhipple/biped-robot/blob/design/power-circuit/docs/design-v6/2026-10-06-power-circuit.md) P10 (on `design/power-circuit`, PR #112) |
| cooling | low-profile heatsink, no fan, enclosed bay | DESIGN.md; #87 |
| camera | Camera Module 3 Wide (Sony IMX708, 102° HFOV) | DESIGN.md R6 |
| runtime | ≥ 45 min holds unless the Pi draws ≥ 12 W | [verification record](https://github.com/tomwhipple/biped-robot/blob/design/power-circuit/docs/design-v6/2026-10-07-power-board-verification.md) C14 (same branch) |
| workload | vision and navigation, not yet written. Moving them onboard waits on fall-free locomotion | #87 (Tom, 2026-08-31) |

## Candidates

"Measured" power is the whole board, from the source named. No two sources used
the same rig, so the figures compare only roughly.

| board | fits the mount | 5 V in on the header | Camera Module 3 Wide | compute vs Pi 4B | measured power, idle / CPU stress |
|---|---|---|---|---|---|
| **Pi 4B** (today) | — | yes, ≥ 3 A [1] | yes | — (4 × A72, no NPU) | 0.6 A idle, 1.25 A max at 5 V [2]; 275 mA idle, 1.3 A stress-ng with peripherals attached [3] |
| **Pi 5** | **yes**: 85 × 56 mm, holes 58 × 49 mm, 3.5 mm inset, Ø2.7 mm [4]. Its connector set differs [5] | yes, with `PSU_MAX_CURRENT=5000` (C14) | **yes**, with a 22-to-15-pin "Standard-Mini" cable [6] | "2 to 3×" CPU [5]. 4 × A76, no NPU | 2.4 W / 8.9 W (2 GB, D0 stepping); 3.2–3.3 W / 9.8 W (4 and 8 GB, C1) [7] |
| **ROCK 5C V2.1** (RK3588S2) | **yes**: 85.0 × 56.0 mm, holes 58.0 × 49.0 mm, 3.5 mm inset, 40-pin in the same place [8] | yes: "5V Power applied to the GPIO PIN 2 & 4"; source ≥ 10 W [8] | **no**: the supported list is OV5647, IMX219 (Pi Camera v2), IMX415 [9] | 4 × A76 + 4 × A55, Mali-G610, 6 TOPS NPU [8]. No independent benchmark found | not found |
| ROCK 5C Lite V2.1 (RK3582) | as the 5C [8] | as the 5C [8] | as the 5C | 2 × A76 + 4 × A55, no GPU, 5 TOPS [8] | not found |
| ROCK 4D (RK3576) | **unverified**: 87 × 58 mm [10]; hole grid not published | yes, GPIO pins 2 and 4 [11] | no (not on Radxa's list) | 4 × A72 + 4 × A53, 6 TOPS [11] | 2.7 W / 7 W [12] |
| Radxa Cubie A7A (A733) | 85 × 56 mm; hole grid not found | yes, per Radxa's docs (agent-read, not re-checked) | no (not on Radxa's list) | 2 × A76 + 6 × A55, 3 TOPS | 3.9 W / 9 W [13] |
| Radxa Dragon Q6A (QCS6490) | 85 × 56 mm | **no**: takes 12 V; a forum reply says the 5 V header pins cannot power it [14] | not listed | 12 TOPS | not found |

**Ruled out on size or power input:**

| board | why |
|---|---|
| Orange Pi 5 Pro, Orange Pi 4 Pro | 89 mm long |
| Banana Pi BPI-M5 Pro | 92 × 62 mm |
| Thundercomm RUBIK Pi 3 | 100 × 75 mm |
| Radxa X4, LattePanda IOTA (x86) | no CSI; 10–17 W under load [15] |
| Jetson Orin Nano | no carrier in the Pi 4B footprint found |
| Cool Pi 4B (RK3588S) | 5 V header power and camera support not found |

## What it shows

- **Less power is not on offer.** The lowest-power board measured, the ROCK 4D
  (2.7 W idle, 7 W under stress), draws about what a Pi 4B does. The logic rail
  is also the small load: the servos take 11–34 W walking, against 5–9 W for
  the Pi, the camera and the General Driver (power-circuit record §2).
- **The Pi 5 is the drop-in.** Same outline and hole grid, the same camera and
  software (libcamera, picamera2), and the power board already cleared for it
  (C14: 48 min upper-model runtime). What it costs:
  - About 1–4 W more, depending on the source [3][7].
  - It throttles without a fan. With only a passive heatsink, Raspberry Pi's
    own test reaches thermal throttling about 200 s into sustained load, in
    open air [16]. Above 85 °C the firmware drops the CPU from 2.4 to 1.5 GHz,
    and Raspberry Pi says a fully throttled Pi 5 still outruns a Pi 4 [16].
  - Its connectors are not where the Pi 4B's are: two 22-pin MIPI, PCIe FPC,
    power button, RTC [5]. The ribbon route to the neck and the General
    Driver's riser over the Pi's parts need re-checking in CAD.
- **The ROCK 5C V2.1 is the NPU route.** It is mechanically identical to the
  Pi 4B and powers from the header. What it costs:
  - **The Camera Module 3 Wide:** IMX708 is not on its list [9]. A Radxa forum
    thread reports the sensor answering on I²C but failing to stream on a
    Rockchip ISP [17]. The head would take an IMX219- or IMX415-based module
    instead, giving up the 102° lens and autofocus unless a wide IMX219 module
    is found.
  - **Power:** unmeasured. Radxa asks for a ≥ 10 W source and says sustained
    high load "may" need a heatsink or fan [8].
  - **Software:** Radxa's Debian and the RKNN toolchain instead of Raspberry
    Pi OS.
- **Accelerators on the Pi.**
  - The AI HAT+ (Hailo-8L / Hailo-8) and AI HAT+ 2 (Hailo-10H) need a Pi 5,
    and would be a third board in the stack. That stack has not been checked
    against the bay.
  - The AI Camera (IMX500) infers on the sensor and works with any Pi, but it
    sees 66° horizontally against the Wide's 102° [6].

## Recommendation

**Keep the Pi 4B until the vision stack exists** (the #87 gate), then choose
by what bounds it:

- **CPU-bound:** a **Pi 5**. Before ordering:
  - check its connector positions in the torso CAD;
  - set `PSU_MAX_CURRENT=5000`;
  - buy a 22-to-15-pin camera cable of ≥ 300 mm;
  - accept throttling under sustained load, or measure it in the bay.
- **Neural-net-bound:** a **ROCK 5C V2.1** with a new head camera. First bench
  its draw on 5 V header power, and confirm the camera streams on Radxa's
  image.

## Not checked

- ROCK 5C power under load, and any RK3588S figure measured fanless in an
  enclosure.
- Hole grids for the ROCK 4D and Cubie A7A.
- Whether wide-angle (≥ 100°) IMX219 modules work on the ROCK 5C.
- Pi 5 connector positions against the torso CAD.
- Pi 5 throttling inside the closed bay. Raspberry Pi's figure is in open air.

## Sources

1. Raspberry Pi 4 Model B product brief: "5V DC via GPIO header (minimum 3A)". https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-product-brief.pdf
2. Raspberry Pi documentation, *power supplies*, as cited in the power-circuit record §2.
3. Core Electronics, *Raspberry Pi 5 vs Raspberry Pi 4 Model B*. Ottiarc meter. Idle with no peripherals: Pi 4 275 mA, Pi 5 500 mA. stress-ng with a monitor, keyboard and mouse: 1.3 A and 2.15 A. https://core-electronics.com.au/guides/raspberry-pi/raspberry-pi-5-vs-raspberry-pi-4-model-b-comparison-and-benchmarking/
4. Raspberry Pi 5 mechanical drawing. https://datasheets.raspberrypi.com/rpi5/raspberry-pi-5-mechanical-drawing.pdf
5. Raspberry Pi 5 product brief, RP-008348-DS. https://pip-assets.raspberrypi.com/categories/892-raspberry-pi-5/documents/RP-008348-DS-8-raspberry-pi-5-product-brief.pdf
6. Raspberry Pi documentation, *camera*. https://www.raspberrypi.com/documentation/accessories/camera.html
7. Jeff Geerling, *New 2GB Pi 5 has 33% smaller die, 30% idle power savings* (stress-ng). https://www.jeffgeerling.com/blog/2024/new-2gb-pi-5-has-33-smaller-die-30-idle-power-savings/
8. Radxa ROCK 5C product brief, RAD-DOC-0115 rev 1.6, 2026-09-04: §4.1 power, §4.1 V2.1 mechanical drawing, operating conditions. https://dl.radxa.com/rock5/5c/docs/rad-doc-0115_radxa_rock5c_product_brief__revision_1.6_g9d14665.pdf
9. Radxa docs, ROCK 5C MIPI CSI support list. https://docs.radxa.com/rock5/rock5c/getting-started/interface-usage/mipi-csi
10. CNX Software, ROCK 4D launch. https://www.cnx-software.com/2025/05/28/radxa-rock-4d-sbc-raspberry-pi-lookalike-powered-by-rockchip-rk3576-edge-ai-soc/
11. Radxa ROCK 4D product brief. https://dl.radxa.com/rock4/4d/docs/radxa_rock4d_product_brief.pdf
12. LinuxLinks, ROCK 4D power consumption (idle; s-tui/stress). https://www.linuxlinks.com/radxa-rock-4d-single-board-computer-running-linux-power-consumption/
13. LinuxLinks, Cubie A7A power consumption. https://www.linuxlinks.com/radxa-cubie-a7a-single-board-computer-running-linux-power-consumption/
14. Radxa forum, *Powering the Q6A via the 5V pin on the 40-pin header*. https://forum.radxa.com/t/powering-the-q6a-via-the-5v-pin-on-the-40-pin-header/30833
15. Tom's Hardware, Radxa X4 review (4.84 W idle, 16.9 W peak, 10.89 W sustained). https://www.tomshardware.com/raspberry-pi/radxa-x4-review
16. Raspberry Pi, *Heating and cooling Raspberry Pi 5*, with a staff reply in its comments on the 2.4 → 1.5 GHz throttle. https://www.raspberrypi.com/news/heating-and-cooling-raspberry-pi-5/
17. Radxa forum, *IMX708 sensor detected on Radxa Zero 3W but ISP pipeline cannot start*. https://forum.radxa.com/t/issue-imx708-sensor-detected-on-radxa-zero-3w-but-isp-pipeline-cannot-start-vidioc-streamon-enodev/29875

Rows not re-checked here; the research agents read these sources:
- Cubie A7A: https://docs.radxa.com/en/cubie/a7a
- Dragon Q6A: https://docs.radxa.com/en/dragon/q6a
- Orange Pi 4 Pro: https://boilingsteam.com/orange-pi-4-pro-review/
- Orange Pi 5 Pro: https://www.cnx-software.com/2024/04/12/orange-pi-5-pro-low-cost-rockchip-rk3588s-sbc-16gb-lpddr5-ram-dual-hdmi/
- BPI-M5 Pro: https://docs.banana-pi.org/en/BPI-M5/BananaPi_BPI-M5_Pro
- RUBIK Pi 3: https://www.cnx-software.com/2024/10/10/qualcomm-qcs6490-rubik-pi-ai-sbc-supports-android-linux-and-lu-operating-systems/
- Cool Pi 4B: https://www.notebookcheck.net/Cool-Pi-4-Model-B-Rockchip-RK3588S-based-SBC-launches-with-Raspberry-Pi-4-Model-B-form-factor.672854.0.html
- LattePanda IOTA: https://www.notebookcheck.net/LattePanda-SBC-packs-four-x86-cores-M-2-and-PCIe-FPC-into-a-Raspberry-Pi-form-factor.1118027.0.html
- Jetson Orin Nano carriers: https://openelab.io/nl/products/seeed-studio-recomputer-j401-carrier-board
