# 3D Printer Order (standalone)

*Split out from the robot BOM 2026-07-13. This is the one-time "get a printer +
something to print with" purchase — separate from the robot parts in
[`bom-by-vendor.md`](bom-by-vendor.md). Prices USD, verify at checkout.*

## Decision drivers (why this changed from the A1)

Priorities, in order:
1. **No software lock-in.** The only program that has to run on your computer is
   a *slicer*; it must not chain you to one vendor's cloud/app. (See
   [What runs on your computer](#what-runs-on-your-computer).)
2. **Buy once.** Future projects are expected to run **outdoors** (→ UV-stable
   **ASA**, which needs an **enclosure**) and possibly include **drone frames**
   (→ carbon-fiber nylon **PA-CF**, which needs a **hardened nozzle** + enclosure
   + a filament dryer). So the machine must be an **enclosed CoreXY** that can
   grow into engineering materials — not the open-frame A1, which tops out at
   PLA/PETG/TPU.
3. **Printer as a tool, not a second hobby** — reliable, auto-calibrating,
   minimal tuning.

Leading pick: **FlashForge Adventurer 5M Pro** — an enclosed CoreXY that clears
all three, is highly rated, and is markedly **more open than Bambu** (no
firmware authorization gate, no RFID filament lock, standard Orca-based slicer).
Alternatives and the one thing it *can't* do are covered below.

---

## The order

| Item | Product | Qty | Est. price | Link |
|---|---|---|---|---|
| **3D printer** | FlashForge **Adventurer 5M Pro** — enclosed CoreXY, 220³ mm, 280 °C hotend, HEPA+carbon, 600 mm/s | 1 | **$449** | [FlashForge](https://www.flashforge.com/products/adventurer-5m-pro-3d-printer) · [Amazon B0CH4RG161](https://www.amazon.com/dp/B0CH4RG161) |
| PLA filament 1 kg | Any brand, 1.75 mm (structural prototyping) | 1 | ~$20–28 | [Amazon B0CGQYRSNT](https://www.amazon.com/dp/B0CGQYRSNT) (Bambu ex.) |
| PETG filament 1 kg | Any brand, 1.75 mm (final servo brackets) | 1 | ~$20–30 | [Amazon B0F68FKRWH](https://www.amazon.com/dp/B0F68FKRWH) (Bambu ex.) |
| TPU filament 500 g | Shore 95A, 1.75 mm (foot pads) | 1 | ~$16 | [Amazon B0DG8BZL6L](https://www.amazon.com/dp/B0DG8BZL6L) (Geeetech) |
| **Order subtotal (robot-ready)** | | | **~$523** | |

Filament is **brand-agnostic** here on purpose — no RFID lock means any spool
works; the Amazon links are just concrete examples, not required brands.

> **Buying check:** confirm the listing says **"5M Pro"** and shows the
> *enclosed* body — FlashForge also sells the open-frame base **5M** (~$239)
> under near-identical naming, and the enclosure is exactly what you need for
> ASA/PA-CF. Also watch for **bundles** that pad the ~$449 price with
> filament/accessories. Verified ASIN for the bare 5M Pro: `B0CH4RG161`.

### Add-ons for when the outdoor / drone projects actually start
*(not needed for the robot — buy when you reach these materials)*

| Item | Why | Est. price |
|---|---|---|
| 0.6 mm **hardened steel nozzle** | Carbon fiber shreds brass/stainless nozzles; required for PA-CF | ~$15–25 (FlashForge offers a hardened option) |
| **Filament dryer** | Nylon/PA-CF (and PETG/TPU) absorb water and print badly wet | ~$40–60 |
| **ASA filament** | UV-stable outdoor parts | ~$25/kg |
| **PA-CF filament** | Stiff, impact-tough drone frames | ~$40–60/kg |

---

## What runs on your computer

The pipeline is **CAD → slicer → printer**, and only the middle piece is new
software on your machine:

- **CAD** — already fully open: your `cad/` is code-based (build123d) → STL/STEP.
  Nothing changes, no vendor involved.
- **Slicer** — the one required desktop app. It turns an STL into machine
  instructions (G-code / `.3mf`). **Slicing is 100% local — no account or
  internet required.** The 5M Pro's official slicer is **Orca-Flashforge** (a
  fork of the open-source **OrcaSlicer**); plain OrcaSlicer also drives it.
- **Transfer** — send the file to the printer by **USB drive** (fully offline),
  or Wi-Fi/Ethernet; FlashCloud is optional, not required.

**Slicer system requirements** (Orca-Flashforge / OrcaSlicer / PrusaSlicer are
all similar; all ship native **macOS Apple-Silicon** builds — you're on a Mac):

| | Requirement |
|---|---|
| OS | macOS 11+ (also Windows 10/11, Linux) |
| RAM | 4 GB min, **8 GB+ comfortable** (CF/nylon geometry is heavy) |
| GPU | OpenGL 3.2+ — integrated Mac graphics fine |
| Disk | ~1–2 GB |
| Internet | Updates/optional cloud only — **not for slicing** |

Any Mac from ~the last 8 years handles this. **Standardize on OrcaSlicer**: it
drives FlashForge, Prusa, Creality, Voron, and even Bambu from one interface, so
your *software* skills stay portable even if you switch printer brands later.

---

## Why the 5M Pro leads (openness first)

- **No vendor gate between your slicer and the machine.** Unlike Bambu (whose
  2025 firmware added a "Bambu Connect" authorization layer that third-party
  slicers must pass through), the 5M Pro takes a standard Orca-based workflow
  over USB/LAN with no handshake, and **no RFID filament lock** — use any spool.
- **Enclosed CoreXY** — the enclosure is the hard requirement for ASA and PA-CF
  (both warp badly in open air). CoreXY is inherently steadier at speed than a
  bed-slinger.
- **Capability** — 280 °C hotend + enclosure + optional hardened nozzle covers
  PLA/PETG/TPU **now** and **ASA + PA-CF** later.
- **Well-regarded & good value** — strong Amazon ratings; HEPA + activated-carbon
  filtration and a silent mode at ~$449, roughly half a comparable Bambu P1S.

### The one limitation to know: no *active* heated chamber

The 5M Pro's enclosure is **passive** (trapped heat drifts to ~40–50 °C). That's
enough for ASA and **small-to-medium PA-CF**, but **large PA-CF or any PC-CF
frames** want an actively heated ~65 °C chamber to stay flat. If serious
large/PC drone frames become the goal, that's the upgrade trigger — and to stay
open, the upgrade path is a **Prusa Core One** (enclosed, open) rather than a
locked-down machine. For everything on the current horizon, the 5M Pro is enough.

### Alternatives, ranked by openness

| Printer | Price | Openness | Notes |
|---|---|---|---|
| **FlashForge 5M Pro** ⭐ | $449 | High | Enclosed CoreXY, Orca slicer, any filament. **The pick.** |
| **Prusa Core One** | ~$949+ | Highest (mainstream) | Fully open PrusaSlicer + standard G-code; the buy-it-for-a-decade open pick. Pricier, slightly slower. Step here if you outgrow the passive chamber. |
| **Voron / Klipper (open CoreXY)** | varies (kit) | Maximum | Fully open firmware, any slicer, self-hosted UI — but it's a build-and-tune project, against the "tool not a hobby" goal. |
| **Bambu P1S / X1C** | $699 / $1,449 | Low | Best out-of-box tuning, but the ecosystem *is* the lock-in (cloud/app default, firmware authorization, RFID). Only if you relax the openness constraint. |

---

## Materials rationale

| Material | Use | Enclosure needed? |
|---|---|---|
| **PLA** | Structural prototyping (cheap, stiff, easy) | No — actually prefers open air |
| **PETG** | Final servo brackets (tough, mild heat resistance) | No |
| **TPU 95A** | Foot soles / grip pads | No |
| **ASA** *(future)* | UV-stable outdoor parts | **Yes** (have it) |
| **PA-CF** *(future)* | Drone frames — stiff, crash-tough | **Yes** + hardened nozzle + dryer |

## Build-volume note

220³ mm is between the A1 mini (180) and the A1/P1S/Prusa (256). It's ample for
every robot part (all < 120 mm). For FPV drone frames it's adequate, but a large
one-piece frame plate may need diagonal orientation or splitting — the 256 mm
machines have more room there. Not a concern for the robot.

---

## Sources
- [FlashForge — Adventurer 5M Pro (official specs)](https://www.flashforge.com/products/adventurer-5m-pro-3d-printer)
- [Top3DShop — 5M Pro: 280 °C nozzle, 220³ build, $449](https://top3dshop.com/product/flashforge-adventurer-5m-pro-3d-printer)
- [SimplyPrint — 5M Pro specs, compatibility & slicer guide](https://simplyprint.io/compatibility/flashforge-adventurer-5m-pro)
- [Tom's Hardware — Adventurer 5M Pro review](https://www.tomshardware.com/reviews/flashforge-adventurer-5m-pro-3d-printer)
- [OrcaSlicer (open-source slicer, cross-brand)](https://github.com/SoftFever/OrcaSlicer)
- [Prusa Core One (open alternative)](https://www.prusa3d.com/product/prusa-core-one/)
