# Adding a camera or other sensors to the General Driver board

*2026-07-29. Answers "suppose I want to add a camera or other sensors — how
would they be integrated?" for the proposed controller swap
([wiring-general-driver.svg](wiring-general-driver.svg),
[datasheets/general-driver/](datasheets/general-driver/)).*

Everything below about connectors and nets is read off the board's own
schematic, which is in the repo. Where the schematic does not answer a
question it says so rather than guessing.

## 1. What the board actually gives you

The current harness uses three connectors: power in, and one servo port per
leg. Everything else on the board is unused. Full inventory:

| Ref | Connector | Nets | Used by bimo? |
|---|---|---|---|
| H1 | XH2.54 2-pin | V+ (`DC_IN`), GND — power inlet | **yes** |
| H5, H6 | Header 3 | `DATA` / `DC_IN` / GND — bus servo, GPIO 18 RX / 19 TX | **yes** (one per leg) |
| P1 | Header 4 | 3V3 · GND · `IIC_SDA` (GPIO 32) · `IIC_SCL` (GPIO 33) | free — BNO085 upgrade path |
| P2 + P4 | 2 × 40-pin | Raspberry-Pi-format GPIO header — the board is built to carry a host SBC | free (**but see §4**) |
| P3 | Header 7 | 3V3 · GND · `IO16` · `IO27` · UART (`P_RX`/`U0RX`) | free |
| H7 | PH2.0 4-pin | **`LIDAR`** — the LD19/LD06-class interface | free |
| H2 | Header 3 | 5V · GND · `IO4` through a 10 R series — WS2812 RGB LED | free |
| H3, H4 | Header 6+0 | motor lead ×2 · GND · **3V3** · **two encoder inputs** (`A_C1`/`A_C2`, `B_C1`/`B_C2`, each through 10 R) — A_C1/A_C2 = **GPIO34/35**, B_C1 = **GPIO27** | free |
| MOTOR-A1/A2/B1/B2 | Header 2+0 | DC motor screw terminals (TB6612) | free |
| TF1 | microSD socket | SPI mass storage — GPIO 12–15. **Not a boot device** | free |
| Type-C ×1 | CP2102N + auto-program | **ESP32 console / flashing** — DTR/RTS → RST/GPIO0 | bring-up only |
| Type-C ×1 | second CP2102N | USB-to-UART bridge **to a host computer** (`P_TX`/`P_RX`) | free |

Two things worth noticing immediately:

- **The board is designed around a host SBC.** The 40-pin header, the second
  Type-C, and the `P_TX`/`P_RX` UART all exist so a Raspberry Pi or Jetson can
  sit on top doing perception while the ESP32 does real-time servo and IMU
  work. That is the vendor's intended camera story.
- **H3/H4 are four free digital inputs with power, on plug-in connectors.**
  They are meant for quadrature encoders, but electrically they are GPIO + 3V3
  + GND on a 6-pin housing. That is exactly a foot-contact-switch harness with
  no soldering.

## 2. The question that decides everything: does it touch the 20 ms tick?

Sensors on this robot fall into three tiers, and the tier — not the wiring —
is what makes integration cheap or expensive.

**Tier 1 — in the policy observation.** Must be read on the ESP32, inside the
tick, *and* must exist in the MuJoCo model, *and* the policy must be retrained.
The obs frame (`sim/mjx/env_mjx.py:_obs`) is

```
[ joint qpos | joint qvel | up-vector | linvel | gyro | prev_action | height | sin φ, cos φ | cmd ]
```

with `linvel` and `height` **zeroed** whenever `imu_obs` is set, because the
robot cannot measure them. Adding a channel changes the input width, which
invalidates every trained policy. This tier is a training project, not a
wiring job.

**Tier 2 — supervisory / telemetry.** Battery voltage, temperature, current,
fault flags. Runs off-tick on the other core or at a few Hz, never feeds the
net. Cheap: read the datasheet, write a driver, ship it.

**Tier 3 — off-board perception.** Camera, lidar, anything that produces more
data than a 240 MHz Xtensa should be looking at during a hard 20 ms loop.
These never enter the obs at all. They enter through **`cmd`** — the last
block of the obs frame, already plumbed end to end as `ext_cmd` over the
wireless link ([control-channel.md](control-channel.md)) at whatever rate the
perception stack can manage.

That last point is the important one, and it is the clean seam this design
already has: **perception sets the goal; the policy walks.** A camera at 10 Hz
feeding `cmd` needs no retrain, no sim change, and no new obs dimension. A
camera trying to enter the obs needs all three plus a renderer in the sim.

Tick budget is not the constraint. Measured worst case today is 4.58 ms of 20,
and `firmware-design.md` expects ~8 ms once the real policy net replaces the
placeholder. There is ~12 ms of slack for sensor reads.

## 3. Camera — the recommendation

**The ESP32-WROOM-32UE cannot take a camera directly.** No DVP interface, no
PSRAM for a framebuffer. This is not a board limitation to work around; it is
why the vendor put a 40-pin SBC header on the board. Two real options:

### (a) Separate WiFi camera module — recommended, and already the plan

A **Seeed XIAO ESP32S3 Sense** (~5 g, OV2640/OV3660, 21 × 17.8 mm) streaming
MJPEG to the laptop, powered from the board's 5 V rail. The laptop does
perception and goal inference and sends `cmd` to the robot at ≤50 Hz over the
existing control channel. Latency ~100 ms, or 90–110 ms measured with the
`hx-esp32-cam-fpv` firmware. This is the path already worked out in
[gopro-vision-input.md](gopro-vision-input.md), under "Recommended perception path".

Why it wins here:

- **Zero coupling to the real-time loop.** The camera cannot make the servo
  tick late, because it is not on the same processor.
- **Zero connectors consumed.** 5 V and GND off any convenient point; nothing
  plugs into P1, P2 or P3.
- **5 g at the top** against a 921 g robot, versus the 154 g GoPro the plant
  already carries and was retrained for.
- Power is negligible: <300 mA at 5 V worst case, and it rides the buck, so
  the onboard INA219 sees it.

Cost: it is a second 2.4 GHz radio next to the robot's own link. Channel
planning matters — see gopro-vision-input.md §7.

### (b) Host SBC on the 40-pin header — the vendor's path, blocked by the tower

Pi Zero 2 W on P2, camera on its CSI ribbon, Pi ↔ ESP32 over `P_TX`/`P_RX`.
This is architecturally the nicest answer — on-robot autonomy, no laptop in
the loop, and the ESP32 keeps its hard loop untouched.

**It does not physically fit the tower as currently dimensioned.** From
`cad/dimensions.py`: the PCB's +x face sits at 15.23 mm and the tower interior
wall is at 25.4 — **10.2 mm of clearance**, of which `BOARD_GD_COMP = 9.0` is
the fitted 40-pin header itself. There is about 1 mm above the header. Nothing
can plug into it.

Taking this path means one of:

- grow `TOWER_W` past 56 and re-roll the plant (again — the last tower resize
  cost a 31.5 mm GoPro rise and a full retrain), or
- use a low-profile FFC off the header and remote the Pi to the deck or the
  top plate, or
- skip the header and talk to the Pi over the second Type-C instead, mounting
  it wherever it fits. **This is the cheap version of (b)** and it costs no
  CAD work at all.

Mass is the other gate: a Pi Zero 2 W is ~10 g and defensible; a Pi 4/5 is not,
on a 921 g biped running a 3S 850 mAh pack.

**Verdict:** ship (a). Revisit (b) — in its Type-C form — only when you want
the laptop out of the loop.

## 4. Other sensors, by where they land

### Foot contact switches — best value, and the connector already exists

Two microswitches per foot into H3 and H4: 3V3 and GND are on the housing, the
two encoder lines are the inputs. No soldering, no board mods, ~2 g.

Three cautions, the first two read off the module pin block on the sheet:

- **H3's two inputs are input-only pins with no internal pull-up.**
  `A_C1` = **GPIO34** (module pin 6), `A_C2` = **GPIO35** (pin 7). GPIO34–39 on
  the ESP32 cannot be outputs *and* have no internal pull-up or pull-down at
  all — a switch on either needs an **external pull-up resistor**. The 10 R
  series parts on the header are protection, not pull-ups. H4's `B_C1` =
  **GPIO27** (pin 12) is a normal bidirectional pin with internal pulls.
- **H4 and P3 share pins.** `B_C1` is GPIO27, which is *also* P3 pin 5; P3
  pin 4 is GPIO16, which is almost certainly `B_C2` (confirm on the sheet).
  So the four inputs are real, but you cannot use both encoder ports *and*
  P3's GPIOs — pick one.
- **Contact only helps if it is in the obs**, i.e. Tier 1. That means MuJoCo
  touch sensors on the foot geoms, a noise/bounce model for the switches
  (real ones chatter, sim ones do not), and a retrain. Budget it as a
  training project.

### I²C sensors → P1

Four wires, GPIO 32/33. Address space already occupied: **0x6B** QMI8658C,
**0x0C** AK09918C, **0x77** BMP280, **0x42** INA219, and **0x4A/0x4B** should
stay reserved for the BNO085 upgrade. Everything else is free.

This bus is read every tick for the IMU, so each added device is in the
critical path unless you move it to the other core. A 400 kHz burst is ~0.4 ms;
two or three more sensors still fit the slack comfortably.

Still unverified on P1: its physical connector type, and whether it carries
pull-ups. Confirm on the board in hand before ordering a cable.

### Lidar → H7

A dedicated PH2.0 4-pin LD19/LD06 interface, free. But nothing in the current
policy or sim consumes range data, and the unit rides at the top of the robot
where mass hurts most — weigh it and re-roll the torso inertial before
believing any policy trained without it. Tier 3: it would feed goal inference,
not the obs.

### Serial sensors (GPS, ToF, a second IMU in UART-RVC) → P3

Seven pins: 3V3, GND, `IO16`, `IO27`, and UART. Note the sharing with H4 above
— these are the same two nets as encoder channel B.

### microSD logging → TF1, already fitted

**The SD slot has nothing to do with firmware.** The ESP32 boots from the SPI
flash die inside the WROOM-32UE module (module pins 17–22 go to that flash and
are not brought out), and you flash it over USB exactly as on the current
board — see "Flashing" below. TF1 is plain optional mass storage.

`firmware-design.md` lists full-rate logging as a v1 non-goal. The slot removes
the reason for that. No mass, no power, no CAD, no retrain — do it first.

Wired for SPI mode: socket pin 2 (`CD/D3`) → `SD_CS`, pin 3 (`CMD`) →
`SPI_MO`, pin 5 (`CLK`) → `SPI_CK`, pin 7 (`D0`) → `SPI_SO`, on **GPIO 12–15**.
Confirmed off the module pin block: `SPI_CK` = **GPIO14** (pin 13), `SPI_SO` =
**GPIO12** (pin 14); CS and MOSI take the remaining IO13/IO15.

⚠ **`SPI_SO` is GPIO12 = MTDI, a strapping pin.** At reset the ESP32 samples
MTDI to choose its internal flash voltage (high → 1.8 V), so a card driving
that line high during reset is the classic "board won't boot with a card
inserted" failure. WROOM-32 modules normally have the flash-voltage eFuse
burned, which makes the strap moot — but do not assume it. **Bring-up check:
power-cycle the board with a card in the slot and confirm it boots.**

### Flashing → Type-C, not the SD slot

Same story as the current board: **CP2102N** bridge onto the ESP32's
`U0TXD`/`U0RXD`, plus the sheet's `AUTO PROGRAM CIRCUIT` — DTR/RTS through two
S8050 transistors driving `RST` and `GPIO0`, with the truth table printed on
the schematic. So `esptool`/`idf.py flash` works with no button-holding, and
macOS needs the same CP210x VCP driver already installed
([bringup-day1.md](bringup-day1.md)). `S2` (`KEY_RST/USER`) is there for a
manual reset.

One new gotcha: **the board has two USB-C ports and only one flashes the
ESP32.** The other is a second CP2102N acting as a host-computer bridge on
`P_TX`/`P_RX`. Label them the first time you find out which is which.

### Status LED → H2

5 V, GND, `IO4` through 10 R: a WS2812 header. Free, and genuinely useful at
bring-up for signalling link-dead / watchdog-tripped states without a laptop.

## 5. The constraints that actually bind

Ranked by how much trouble they cause, which is *not* the order you would
guess:

1. **Volume and mass above the deck.** ~10 mm of clearance over the PCB face,
   9 of it already spoken for. Every gram at tower height moves the CoM, and
   the plant + policy are downstream of that.
2. **The obs contract.** Tier-1 sensors invalidate trained policies and need a
   sim model including their noise. Tier-3 sensors feeding `cmd` cost nothing.
3. **Input current.** H1's XH2.54 is rated 3 A and the whole servo bus crosses
   it, against a 6.8 A sim-derived peak — already the board's weak point. New
   5 V loads ride the buck rather than `DC_IN`, so a camera module is not the
   problem; just do not add anything large.
4. **Tick time.** ~12 ms spare. Effectively a non-issue.
5. **Pins and connectors.** The most abundant resource on the board. Nine free
   connectors, of which the design needs at most two.

## 6. Suggested order

| # | Change | Cost |
|---|---|---|
| 1 | microSD logging on TF1 | firmware only |
| 2 | Status LED on H2 | one part, bring-up quality of life |
| 3 | Camera as XIAO ESP32S3 Sense → laptop → `cmd` | no board change, no CAD, **no retrain** |
| 4 | Foot contacts on H3/H4 | wiring is free; sim model + retrain is not |
| 5 | Host SBC over the second Type-C | mass + power budget |
| 6 | Lidar on H7, or a Pi on the 40-pin | needs tower rework and a plant re-roll |
