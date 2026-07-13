# 3D Printer Order (standalone)

*Split out from the robot BOM 2026-07-13. This is the one-time "get a printer +
something to print with" purchase — separate from the robot parts in
[`bom-by-vendor.md`](bom-by-vendor.md). Prices USD, verify at checkout (Bambu
list pricing has moved with 2025–26 tariffs).*

Decision: **full-size A1** (256 × 256 × 256 mm), not the A1 mini. The robot's
parts all fit the mini's 180 mm bed, but the bigger bed buys one-piece torso
shells, batch-printing a whole leg per plate, and headroom for future/larger
projects.

---

## The order

| Item | Product | Qty | Est. price | Link |
|---|---|---|---|---|
| **3D printer** | Bambu Lab **A1** (no AMS), 256³ mm | 1 | ~$339 (range $319–399) | [Bambu store](https://us.store.bambulab.com/products/a1) · [Amazon B0D17TMWFB](https://www.amazon.com/dp/B0D17TMWFB) |
| PLA filament 1 kg | Bambu Lab PLA 1.75 mm (Jade White, RFID auto-settings) | 1 | $28.00 | [Amazon B0CGQYRSNT](https://www.amazon.com/dp/B0CGQYRSNT) |
| PETG filament 1 kg | Bambu Lab PETG Translucent 1.75 mm (with spool) | 1 | $30.15 | [Amazon B0F68FKRWH](https://www.amazon.com/dp/B0F68FKRWH) |
| TPU filament 500 g | Geeetech TPU 1.75 mm, Shore 95A (foot pads) | 1 | $15.99 | [Amazon B0DG8BZL6L](https://www.amazon.com/dp/B0DG8BZL6L) |
| **Order subtotal** | | | **~$413** | |

**Optional add-ons** (not required for the robot):
- A1 **Combo** (bundles the AMS Lite for 4-color): **~$459–499** instead of the
  bare A1 — adds ~$120. Unnecessary for functional single-color robot parts.
- Spare 0.4 mm hotend / nozzle (~$8–15) — cheap insurance against a clog.
- Spare textured PEI build plate (~$20) — lets you keep one plate per material.
- Bambu TPU 1 kg ($42.99, [B0GG283YLZ](https://www.amazon.com/dp/B0GG283YLZ)) if
  you'd rather buy first-party TPU with RFID than the Geeetech 500 g roll.

Materials rationale: **PLA** for structural prototyping (cheap, stiff, easy),
**PETG** for the final servo brackets (tougher, mild heat resistance near warm
servos), **TPU 95A** for foot soles / grip pads. All three print fine on the
open-frame A1 — none of the robot's materials need an enclosure.

---

## Why Bambu (and when I'd pick something else)

Short version: for a project where the printer should be a *tool, not a second
hobby*, the A1 is the closest thing to "click print, get a good part," and its
ecosystem removes the most failure points for someone who wants to spend their
hours on the robot, not on printer tuning. But FlashForge is a genuinely strong
alternative now, and there's one axis (enclosure) where it beats the A1 — so
the honest answer is "Bambu by default, FlashForge if you want an enclosed
CoreXY at a similar price." Details:

### What actually makes Bambu the default here
- **Plug-and-play reliability.** Full auto-calibration (bed mesh, flow,
  resonance) every print; the highest first-print-success rate in class. Fewer
  wasted plates = faster path to physical robot parts.
- **First-party filament with RFID.** Bambu PLA/PETG spools carry an RFID tag
  the printer reads to auto-load the correct temperature/flow profile — no
  manual profile fiddling. This is why the filament lines above are Bambu-brand.
- **Ecosystem depth.** The largest user base of any of these printers, so for
  almost any error message or part-geometry question there's already a tuned
  profile, a forum thread, or a MakerWorld model. That community is worth real
  hours on a first build.
- **Slicer.** Bambu Studio (a Bamboo-flavored fork of the excellent open-source
  OrcaSlicer) ships free and is one of the best in the category; OrcaSlicer
  proper also drives the A1 if you prefer fully-open tooling.

### Where FlashForge legitimately competes — or wins
The relevant machines are the **FlashForge Adventurer 5M** (~$239, open CoreXY)
and **Adventurer 5M Pro** (~$399–599, *enclosed* CoreXY with HEPA + activated-
carbon filtration and a silent mode).
- **CoreXY vs bed-slinger.** Both 5M models are CoreXY (the bed moves only in Z),
  which is inherently steadier at high speed than the A1's bed-slinger (bed
  shuttles in Y). In practice both hit ~500 mm/s-class speeds and the A1's
  print quality is excellent, so this is more "nice engineering" than a
  day-to-day difference for parts this small.
- **Enclosure (the real one).** The 5M **Pro** is fully enclosed; the A1 is
  open-frame. An enclosure gives a stable thermal chamber for warp-prone
  materials (ABS/ASA/nylon) and captures fumes. **This robot only needs
  PLA/PETG/TPU, none of which require an enclosure** — so it doesn't change *this*
  order — but if you ever want to print engineering plastics, the Pro (or a
  Bambu **P1S**, ~$699) is the pick and the A1 is not.
- **Price/value.** The base 5M undercuts the A1; the Pro roughly matches an A1
  Combo while adding the enclosure + filtration instead of multicolor.

### Where FlashForge falls short vs the A1
- **No native multicolor.** No AMS-equivalent — a hard limitation if you ever
  want multi-material/color. (Irrelevant for functional robot parts, but it's
  the A1's headline advantage.)
- **Smaller ecosystem / less first-party-filament automation** — more manual
  profile setup, fewer community resources than Bambu's.

### The other usual suspects (why not these)
- **Prusa MINI+ / Core One** — the open-source, infinitely-repairable, "buy it
  for a decade" pick, but ~1.5–2× the price and slower. Choose it only if
  open-hardware repairability is a principle for you.
- **Creality (Ender / K1 series)** — cheapest entry, but historically more
  tuning and QC variance; the hours you save on the sticker price you spend
  dialing it in. Against the goal of "printer as tool," that's a bad trade here.

### Honest caveat on Bambu
Bambu's ecosystem is partly closed (cloud-tied firmware; there was a 2025
firmware-authorization controversy). **LAN-only mode exists and works** if you
want it off the cloud. If open-source-on-principle matters to you, that's the
case for Prusa — or, for an enclosed machine with a more hardware-driven and
less account-tied posture, the FlashForge 5M Pro.

**Bottom line for this build:** order the **Bambu A1** (bare, no AMS) — best
tool-not-a-hobby experience and filament automation for single-color PLA/PETG/
TPU parts. Switch to the **FlashForge 5M Pro** if you specifically want an
enclosed CoreXY (e.g. you plan to print ABS/ASA later) at a similar price; drop
to the base **5M** to save money if you don't need Bambu's ecosystem. Say the
word and I'll swap the line above.

---

## Sources
- [Bambu Lab A1 (official store)](https://us.store.bambulab.com/products/a1)
- [Micro Center — A1 Combo listing](https://www.microcenter.com/product/675479/bambu-lab-a1-combo-3d-printer)
- [3DTechValley — FlashForge Adventurer 5M review (2026)](https://www.3dtechvalley.com/flashforge-adventurer-5m-review/)
- [3DTechValley — FlashForge Adventurer 5M Pro review (2026)](https://www.3dtechvalley.com/flashforge-adventurer-5m-pro-review/)
- [goodprints3d — Bambu A1 vs FlashForge Adventurer 5M](https://www.goodprints3d.com/blogs/3d/bambu-lab-a1-vs-flashforge-adventurer-5m-which-3d-printer-makes-more-sense-for-buyers-deciding-between-a-full-size-easy-bambu-and-a-fast-corexy-value-pick)
- [Tom's Hardware — Best 3D Printers 2026](https://www.tomshardware.com/best-picks/best-3d-printers)
