# Wiring & control architecture

*Status 2026-07-11. One serial bus, one battery, no custom electronics —
the entire harness is the servo leads that ship in the box plus one XT30
pigtail and a switch.*

**Circuit diagram** (pin-level: connectors, nets, wire colors):

![Circuit diagram](circuit-diagram.svg)

**System block diagram** (physical layout view of the same thing):

![Wiring diagram](wiring-diagram.svg)

## Power path

```
3S LiPo (XT30) ── inline switch ── screw terminal / DC 5.5×2.1 on driver board
                                        │
                                        └── servo bus V+ (battery voltage,
                                            passed straight through to all 8)
```

- The battery is the [Zeee 3S 850 mAh 100C XT30](https://www.amazon.com/dp/B08H5GD35D)
  (decided 2026-07-11; a 2-pack, so one charges while one flies). It swaps
  tool-free through the window in the rear tower wall: peel the belt, tug the
  pull-ribbon, tilt the pack out; reverse to insert (lead-end toward whichever
  y-pocket the XT30 pigtail lives in).
- The **Servo Driver with ESP32** accepts 6–12.6 V, so it runs directly from
  2S (7.4 V) or 3S (11.1 V, 12.6 V fully charged) — no regulator needed. Its
  logic is powered from an onboard buck; the servo bus carries raw battery
  voltage, which is what sets torque (the whole 2S/3S performance story in
  [hardware-order.md](hardware-order.md)).
- **Current sizing**: each ST3215 can pull ~2.5–3 A at stall; the gait
  gauntlet shows only 2–3 joints near peak torque simultaneously, so budget
  ~10 A transient. XT30 (30 A) and 20 AWG battery leads are comfortable;
  this is why JST-terminated packs (~3 A) are ruled out.
- The board's OLED shows measured bus voltage — the low-battery check.
  Land the robot by **10.5 V on 3S** (3.5 V/cell) / 7.0 V on 2S.
- GoPro MAX is self-powered; zero wiring to the robot.

## Servo bus

- 3-pin daisy chain, half-duplex TTL at **1 Mbaud** (driver board UART1,
  GPIO 18/19). Pin order on every connector (Molex-5264-style): **1 GND
  (black) · 2 V+ (red) · 3 DATA (white/blue)**. Every ST3215 has two
  identical, internally-paralleled ports, so chains just hop case to case
  with the included 150 mm leads.
- Per-servo current (12 V class): **2.7 A stall, 0.18 A idle** — the ~10 A
  transient budget in the circuit diagram comes from 2–3 joints near stall
  simultaneously in the gauntlet's worst gaits.
- The board's two bus ports are **electrically the same bus** — we use one
  per leg purely for cable routing:
  - **Port A → left leg**: ID 1 hip roll → ID 2 hip pitch → ID 3 knee → ID 4 ankle
  - **Port B → right leg**: ID 5 hip roll → ID 6 hip pitch → ID 7 knee → ID 8 ankle
- ID order matches the sim's joint order in `sim/walker_env.py`
  (left leg then right, proximal to distal), so the policy's action vector
  maps to IDs 1–8 with no permutation table.
- Servos ship with ID 1: at bring-up, connect **one at a time** and assign
  IDs via the board's web UI (AP mode, `192.168.4.1`), then chain them.
  Two same-ID servos on the bus fail to enumerate.
- Hip-roll servos sit closest to the board; the shin→ankle hop is the
  longest run — verify the 150 mm leads reach with the service loop, else
  use the two extension leads on the order list.

## Control path (and where latency lives)

| Link | Rate | Role |
|---|---|---|
| Laptop ↔ ESP32 WiFi | 802.11n | development: telemetry, web UI, remote policy |
| Laptop ↔ USB-C (UART0) | 115200 | flashing, serial bridge, tethered debug |
| ESP32 ↔ servos (UART1) | 1,000,000 | position commands + state readback |

The latency work in DESIGN.md bears directly on this: at 115200 baud a full
8-servo command + state readback cycle eats most of a 20 ms control tick,
and WiFi adds jitter on top. The servo bus itself at 1 Mbaud is ~10× faster
than the link to the laptop. So the plan of record is the one the latency
sims validated: **run the 50 Hz policy loop on the ESP32** (or at minimum
raise UART0 baud), keeping laptop links for telemetry only. No parts change
either way — the ESP32 on the order sheet does both jobs.

## Bring-up checklist

1. Bench-power the bare board (no servos), confirm OLED + web UI.
2. Set servo IDs 1–8 one at a time via web UI; label each case.
3. Chain one leg at a time; confirm enumeration count on the OLED.
4. Torque-off ("Release") all servos, assemble, then use "Set Middle
   Position" at the CAD-neutral pose before first powered stand.
