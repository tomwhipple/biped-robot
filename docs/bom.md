# Bill of materials

Everything needed to build the robot described in [DESIGN.md](../DESIGN.md): 17
servos, the controller board, a Raspberry Pi 4B and its head camera, a 3S pack
and its protection, fasteners, filament and tools. The printed parts are in
[cad/PRINT_LIST.md](../cad/PRINT_LIST.md); how it goes together is
[assembly.md](assembly.md); power and wiring detail is [wiring.md](wiring.md).

**Status.** Nothing for the robot has been bought. The servo purchase waits on
the bench test in issue #73, then #74. The non-servo order is #78.

**Buying rules**

- **Buy to the spec filter in each row, not to a listing.** Listings churn: a
  pack the prototype was designed around disappeared mid-project. Anything that
  meets the filter fits. A cached listing title is not a live listing, so check
  stock and price in a browser.
- **Only parts with real manufacturer documentation**: a datasheet with specs
  and a connector or outline drawing. Reseller renders and self-authored
  "manuals" do not count. When a part is bought, mirror its datasheet into
  `docs/datasheets/`.
- Prices are indicative and dated, and none has been re-checked.

In the **have** column, *on hand* means the repo's records say it is in the
lab. A blank means buy it.

## 1. Servos

| item | qty | spec | have |
|---|---|---|---|
| **Feetech STS3215, 12 V class: ST-3215-C018** | 15 + spares | 12 V, 30 kg·cm; case 45.22 × 24.72 × 35 mm; 55 ± 1 g; 25T spline; half-duplex TTL bus; 5264 3-pin connector with a 150 mm lead | **12 on hand** (10 of them are in the prototype today) |
| **Feetech STS3250 (ST-3250-C001 / C002)** | 2 | the two hip rolls; same case, horn and connector; 12 V, 50 kg·cm, 74.5 ± 1 g | **2 ordered** 2026-10-02 (#74) |

- **Why 15 + 2:** 12 in the legs, the neck, and 4 in the arms, with the two hip
  rolls on STS3250s ([DESIGN.md §4](../DESIGN.md)). How many STS3215s to buy,
  with spares, is #74.
- **Datasheet, drawing and STEP model:** `docs/datasheets/st3215/` and
  `cad/vendor/ST3215.step`.
- **Never buy the 7.4 V class** (STS3215 C001, rated 4–7.4 V). It cannot run
  on 3S. Both classes share one case, so the model number on the label is what
  tells them apart.
- **Sources:**
  - Waveshare direct, <https://www.waveshare.com/st3215-servo.htm>. Choose the
    12 V / 30 kg·cm version. It was $21.99 on 2026-07-11.
  - Amazon RCmall 6-pack, B0FLPQQ4FR. This listing was checked and is the
    12 V class.
- **Gain setup** ([DESIGN.md §4](../DESIGN.md)): on the four ankle-roll and
  knee STS3215s, raise the position-loop P (register 21, default 32) and set D
  (register 22) to the values #73 establishes (the sim assumes P 96 / D 0,
  ≈ 2.8×). Record the values per ID in [servo-map.md](servo-map.md).

**Incoming check.** Run it on every servo before it shares the bus with another:

1. **Weigh it and read the label.** An STS3215 weighs 55 ± 1 g and its label
   reads ST-3215-C018.
2. **Read Model_Number, alone on the bus.** Servos ship as ID 1, and two
   servos with the same ID do not enumerate.
   - The register is address 3, 2 bytes, little-endian.
     `firmware/components/scsbus/include/scsbus/registers.h` names the two bytes
     `kRegServoMajor` / `kRegServoMinor`.
   - **Expected values:** STS3215 = 777, STS3250 = 2825. These come from
     Feetech's model numbering as found during sourcing.
   - **Confirm the read on an on-hand STS3215 first.**
   - A read is not a motion command. Opening the serial port does reboot the
     board, though, so drain it before sending anything
     ([AGENTS.md](../AGENTS.md), bench safety).

*Why:* a relabelled or clone servo silently invalidates the stiffness
measurement the whole servo plan rests on.

**If #73 fails.** The fallbacks come in DESIGN.md §4's order, after trying the
I term. Both are the same case, horn and protocol, so no CAD change:

| item | qty | spec | role |
|---|---|---|---|
| Feetech STS3235 (ST-3235-C001) | 1 | 12 V, 30 kg·cm, 70.5 g, aluminium case, steel gears 1:345, backlash ≤ 0.5°, 2.7 A stall (Feetech spec A/0, 2021-11-19) | shows whether the gear train, not the loop gain, is what is soft |
| Feetech STS3250 (ST-3250-C001 / C002), beyond the two hip rolls | up to 4 more (ankle rolls, knees) | 12 V, 50 kg·cm, 74.5 ± 1 g, aluminium case, 4.2 A stall, measured loaded backlash 0.33° (third-party bench) | only if the bench comparison (#73) calls for them |

**STS3250 sources** (checked 2026-09-26):

- **Feetech direct on Alibaba.** The storefront is feetechrc.en.alibaba.com
  (supplier Shenzhen Feite Model Co., Ltd., Feetech's own company); the listing
  is 1601808172007.
  - $47.77 each for 2–9, $45 for 10–99, minimum order 2.
  - US shipping and import charges are included. Delivery is slow (quoted
    October to December).
  - The C001 / C002 / URT variants share one price ladder.
- **WowRobo** (Dongguan; sells LeRobot kits): $43 for the C002. The stock count
  is hidden, and US duties are not published on the servo page.
- **Avoid:**
  - AliExpress. The Feetech Official Store there is genuine, but the servo SKU
    costs $80.86 plus about 40 % import charges at checkout, and the listings
    headline the URT-1 board instead.
  - US resellers at $65–97: BABSCO $64.99 (out of stock), Amazon $67–75,
    AIFITLAB $88–97.
  - Any listing that gives the mass as about 50 g.
- **Incoming check:** 74.5 ± 1 g, an aluminium case, a label reading
  STS3250-C00x, and Model_Number 2825. Feetech's STS3250 datasheet is not yet
  in `docs/datasheets/`.

Feetech's TTL catalogue lists exactly three 12 V servos in this case (checked
2026-09-26): ST-3215-C018, ST-3235-C001 and ST-3250-C001. Ruled out:

- the RS485 SM series, because the General Driver's bus is TTL;
- 7.4 V parts, because a 3S pack is over their rating;
- anything without a manufacturer datasheet.

## 2. Controller, computer, camera

| item | qty | spec | have |
|---|---|---|---|
| **Waveshare General Driver for Robots** | 1 | ESP32 with onboard QMI8658C IMU, AK09918C magnetometer and INA219 (0x42); two servo ports H5/H6 (pin 1 DATA, 2 DC_IN, 3 GND); power inlet XH2.54 2-pin; outline 65.01 × 56.01 mm, holes 58 × 49 mm | **on hand** (in the prototype today) |
| Raspberry Pi 4B | 1 | 2–4 GB; 85 × 56 mm, M2.5 holes on 58 × 49 mm | not recorded |
| Pi heatsink | 1 | low profile, no fan: it must stay within the 16 mm the USB/Ethernet stack already occupies above the board | |
| microSD card | 1 | 32 GB | |
| Raspberry Pi Camera Module 3 **Wide** | 1 | Sony IMX708, 102° horizontal / 120° diagonal field of view; 25 × 24 × 12.4 mm; M2 holes on 21 × 12.5 mm; 15-pin 1 mm FPC | |
| camera cable | 1 (+1 spare) | Raspberry Pi 15-pin 1 mm FPC camera cable, **≥ 300 mm** | |

- **General Driver.** Its documents are in `docs/datasheets/general-driver/`.
  - The outline comes from the dimension drawing; the wiki's "65 × 65 mm" is
    wrong.
  - The input rating is 7–13 V on the wiki but "DC 9–12.6V" on the rev 1.2
    silkscreen. A 3S pack is inside both.
- **Pi 4B.** It slides vertically into the pelvis's aft column.
- **Camera cable.** The camera ships with a 200 mm cable, too short. The route
  runs from the Pi's CSI edge under the deck up to the head, plus a slack loop
  for ±90° of neck yaw. 300 mm is the design estimate; the routed length has
  not been measured.

The Pi ↔ ESP32 link is not chosen yet (#87).

**On hand, not in the build:** a Teyleten GY-BNO085 IMU breakout. It is the
upgrade path if the onboard IMU proves insufficient, and would plug into the
board's I²C header P1.

## 3. Power

The power path runs pack → protection board → fuse → switch → board inlet. A
bulk capacitor sits at the board, and a 5 V buck feeds the Pi.

**Current is open (#77):**

- The General Driver's XH inlet contact is rated **3 A**.
- Waveshare's FAQ limits the board's servo power path to **5 A continuous**
  (about 5 STS3215s, not stalled at once).
- The robot puts 17 servos on that path. The prototype measured 6.8 A peaks on
  10.
- The fix under consideration is to solder the power lead to the inlet pads
  instead of using the XH plug.
- The fuse, the bulk capacitor and a current budget are required before the
  first powered floor run.

| item | qty | spec / filter | have |
|---|---|---|---|
| **3S LiPo pack** | 1–2 | **11.1 V, not 11.4 V HV**; 2200–2600 mAh; **≤ 105 × 36 × 26 mm**; 150–190 g; XT60 or XT30; ≥ 25C | |
| **3S protection board** | 1 | ≥ 15 A continuous; over-discharge cut-off ≈ 3.0 V/cell; balance leads; must fit the pelvis power pocket together with the buck | |
| **5 V / 5 A buck** for the Pi | 1 | Pololu D24V50F5 (item #2851): 6–38 V in, 5 V, 5 A; 17.8 × 20.3 × 8.8 mm; two 2.18 mm holes for #2/M2 screws on 13.5 × 16.0 mm | |
| inline fuse holder | 1 | mini blade (ATM), 16–18 AWG leads | |
| mini blade fuses | 1 pack | 10 A and 15 A | |
| bulk capacitor | 1–2 | 470–1000 µF, ≥ 25 V, low-ESR, 105 °C electrolytic | |
| power switch | 1 | inline, rated at least the fuse current | |
| balance-lead low-voltage alarm | 1–2 | 1S–8S, plugs onto the JST-XH balance lead, per-cell display, settable alarm point | |
| 3S balance charger | 1 | with a storage mode | check what is on hand |
| inline watt meter | 1 | ≥ 60 A, XT60/XT30, shows peak current | |
| LiPo charging bag | 1 | fits the pack | |

- **Pack**
  - The pelvis battery layer is sized to this envelope (`BATT` in
    `cad/v6/dimensions_v6.py`, with 170 g modelled).
  - An HV pack charges to 13.05 V, above the 12.6 V the servo listing and the
    board's silkscreen allow.
  - 2600 mAh packs usually run over 105 mm long, so 2200 mAh is the class that
    fits.
  - Two candidates from the sourcing pass:
    - The Admiral EPR22003X6 (105 × 34 × 22 mm, XT60) fits.
    - The Yowoo 3S 2200 (105 × 36 × 26 mm, XT60) fits the box but is listed at
      208 g, over the mass filter.
  - Cross-check dimensions and mass at two independent stockists.
- **Protection board**
  - The pocket takes both boards standing on edge: one 60 × 25 mm face,
    8.4 mm deep (`PWR_BOARD`).
  - No board is selected yet.
  - The UART "smart" boards that give per-cell telemetry do not fit the pocket:
    JBD SP04S010 (80 × 60 × 12 mm) and Daly R05J (100 × 65 × 13 mm).
- **Buck**
  - Pololu lists it 8.8 mm thick, against the pocket's 8.4 mm depth. Check the
    fit.
  - Its holes take #2/M2 screws, but the pocket's two bosses are M2.5 pilots.
    How the power boards are fixed is open.
- **Fuse:** start at 15 A, and drop to 10 A only once the peak current has been
  measured. It is for a dead short, not the operating current. **Never use a
  polyfuse/PPTC**: it takes seconds to trip, and its series resistance sags the
  rail.
- **Bulk capacitor:** solder it across the servo bus at the board, observing
  polarity. It must be rated 25 V because a full 3S pack is 12.6 V.
- **Power switch:** the board's own slide switch SW1 sits in the servo path,
  but the schematic does not state its rating.
- **Low-voltage alarm:** it is the only protection that is independent of the
  firmware, and it is per cell. One weak cell hides inside a normal-looking
  pack voltage.
- **Charger:** storage-charge to 3.8 V/cell if a pack will sit for more than a
  few days.
- **Watt meter:** it measures the peak current #77 needs, and that number sets
  the fuse.

**Not needed:**

- a UPS HAT. The Waveshare UPS Module 3S's pack output is 2 A, an order of
  magnitude under the servo bus.
- an anti-spark connector. At 3S the connection arc is negligible.
- PCA9685-type servo boards. They drive PWM servos and cannot talk to ST/SC bus
  servos.

## 4. Leads and connectors

| item | qty | spec | notes |
|---|---|---|---|
| pack pigtail | 1 | XT60 or XT30 to match the pack; ≥ 20 AWG silicone lead | runs to the protection board, fuse and switch |
| board power pigtail | 1 + 1 spare | JST XH 2.54 mm 2-pin female (housing XHP-2, SXH contacts), pre-crimped, 22 AWG, ≥ 100 mm | for the General Driver inlet (connector diagram no. 10, silkscreen "− +") |
| servo extension leads | ~4 | 3-pin 5264 bus-servo cable, 200–300 mm | each servo ships with a 150 mm lead |
| Pi power lead | 1 | USB-C pigtail from the buck (or the Pi's 5 V GPIO pins) | |
| silicone wire, heat-shrink | — | 20 AWG for the power path | |
| USB-C data cable | 1 | for flashing the General Driver | likely on hand |

- **Board power pigtail.** XH crimps take 22 AWG at most, so splice the pigtail
  to the 20 AWG lead and heat-shrink the joint. Pigtail wire colours are not
  standard: check polarity against the silkscreen. This is not a balance-lead
  connector.
- **Servo extension leads.** Which hops need them has not been measured on this
  robot (#77). On the prototype, the hip-roll → hip-pitch hop routed at 170 mm.

## 5. Hip-yaw bearing

A thin-section bearing between each yaw carrier and the pelvis
([DESIGN.md §5.2](../DESIGN.md); background:
[study-yaw-bearing.md](design-v6/study-yaw-bearing.md), option A). Buy 2
fitted plus 2 spares:

| bearing | qty | spec | notes |
|---|---|---|---|
| **6810-2RS**, 50 × 65 × 7 mm, sealed deep-groove | 2 + 2 spares | C0 5.8 kN (one spec sheet); SKF 61810-2RS1 catalogue mass 0.052 kg | inner race on the carrier hub (+0.08 mm), outer race in the pelvis recess (−0.04 mm) |

- **Retaining compound**, on both seats: Loctite 641 (medium strength, fills a
  0.15–0.25 mm gap, can be taken apart), or 648 if it never needs to come
  apart. The CAD's 0.04 / 0.08 mm interference fits are inside the printer's
  error band, and PETG creeps.
- **Buy from a maker that publishes the ring dimensions.** The pelvis shoulder
  is sized against an *estimated* inner-ring OD and outer-ring ID. Verify those
  on the real bearing before printing the pelvis.
- **6710-2RS (50 × 62 × 6 mm)** can replace the 6810 if its rating is
  confirmed. Only `YAW_BRG_W` and `YAW_BRG_OD` in `cad/v6/dimensions_v6.py`
  would change.

## 6. Fasteners

The counts come from each part module's `SCREWS()` (the robot's build, with
arms), times the number of each part.

**The build uses no heat-set inserts and no washers.** All servo-case and
printed-pilot screws are M2.5 flat-head self-tappers. The servo case holes take
M2.5, not M3.

| fastener | qty | where | have |
|---|---|---|---|
| **M2.5 × 8 flat-head (90° countersunk) self-tapping**, stainless | **110** | leg links 24, ankle links 12, foot tabs 8, yaw-servo stators 8, yaw-carrier walls 16, neck floor to the neck tube 4, neck-servo stators 4, head face 4, girdle to deck 10, shoulder servos 8, elbow grips 12 | **24 on hand** |
| M2.5 × 10 pan self-tap | 4 | Pi and General Driver, lower row, into standoff bosses | |
| M2.5 × 6 pan self-tap | 4 | Pi and General Driver, upper row, into wall pilots | |
| M2.5 × 8 pan self-tap | 2 | the power-pocket bosses | |
| M2 × 4 self-tap | 4 | Camera Module 3 onto the head face | |
| **M3 button head, disc screws** | **116** = 56 × M3×5 + 52 × M3×6 + 8 × M3×8 | 4 per servo disc | **40 M3 × 6 on hand** |

- **Disc screw lengths.** The horn and idler discs are tapped through a thin
  flange: 2.5 mm on the horn and 2.1 mm on the idler, measured on the bench and
  confirmed on the vendor STEP.
  - A disc screw must engage 1.5 mm up to the flange depth, and no more. A
    longer screw bottoms out and jacks the joint apart instead of clamping it.
  - So the length follows the stack under the head, as tabulated per joint in
    [assembly.md §1](assembly.md#1-the-joint-the-servo-is-the-axle). The part
    modules' `SCREWS()` lists take it from the stack, and `check_assembly_v6`
    checks every one's engagement.
  - By length: **M3 × 5** at the leg-link horn pads 16, ankle-link tines 16,
    hip-yoke pitch horns 8, yaw carriers 8, elbow horns 8; **M3 × 6** at the
    leg-link idler pads 16, hip-yoke roll horns 8 and pitch idlers 8, shoulder
    horns 8, elbow idlers 8, head 4; **M3 × 8** at the hip-yoke roll idlers 8.
  - Buy a pack of each length.
- **The armless variant** (`ARMS=0`, not a print target) would need 84 M2.5 × 8
  flat-heads and 92 M3 disc screws (48 × M3×5, 36 × M3×6, 8 × M3×8).
- **Spares:** add about 20 % to every line, and buy the M2.5 × 8 flat-heads as a
  200-pack.

## 7. Materials

| item | qty | notes | have |
|---|---|---|---|
| PETG, 1.75 mm | 2 × 1 kg | the print set is ≈ 0.77–0.79 kg before supports and brims ([PRINT_LIST](../cad/PRINT_LIST.md)) | on hand (quantity not recorded) |
| TPU 95A, 1.75 mm | 1 spool (500 g is enough) | the two soles, 23 g each | |
| hook-and-loop strap, **15 mm** wide | 1 | the battery belt, through the pelvis belt slots (`BATT_BELT_W`) | |
| adhesive for the soles | — | bonds TPU 95A to PETG; type not chosen | |
| thread-locker, medium strength (removable) | 1 | on the M3 disc screws (the design calls for thread-locked horn screws); keep it off the printed parts | |
| retaining compound | — | §5 | |
| zip ties, 2.5 mm | 1 pack | cable dressing | |
| glue stick, isopropyl alcohol | — | glue is the release layer for PETG on the PEI plate; degrease the plate with IPA | glue ships with the printer |
| desiccant / dry box | — | keep PETG and TPU dry | |

## 8. Tools

| tool | notes | have |
|---|---|---|
| FlashForge Adventurer 5M Pro | enclosed CoreXY, 220 × 220 × 220 mm, 280 °C hotend, PEI plate, no filament lock ([PRINT_LIST](../cad/PRINT_LIST.md), Slicing) | **on hand** |
| digital calipers | | **on hand** |
| drivers for M2.5 flat-heads, M3 button heads and M2 | a slim shaft: the CAD checks a 7 mm driver cylinder over every screw, and a few screws near walls need slimmer | |
| flush cutters, deburring blade | support removal | |
| soldering iron, wire strippers, heat-shrink | power splices, the bulk capacitor, the XH pigtail | |
| lever and weights for 0.5 / 1.0 / 1.5 N·m | the #73 stiffness bench | |

Not needed: a heat-set insert tip, because the build has no inserts.
