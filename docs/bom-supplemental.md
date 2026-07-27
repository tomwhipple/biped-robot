# Supplemental BOM — battery protection & power bench

*Compiled 2026-07-27. Prices in USD, delivering to Lyons, CO 80540.*

Everything here came out of the "do we need to guard against overdraw?"
question. The reasoning behind each line is in
[wiring.md § Battery protection](wiring.md#battery-protection) — this file is
just the shopping list. It is **supplemental to** [bom-sourced.md](bom-sourced.md):
the robot is complete and functional without any of it.

> **Prices are indicative, not captured.** These are commodity RC parts whose
> listings churn constantly, so the links are *search queries* rather than
> ASINs — the same pattern the main BOM uses for its generic rows. Check price
> and stock in a browser; a cached product title is not a live listing.

---

## Buy first — the one real gap

| # | Component | Qty | Spec that matters | Est. | Source |
|---|-----------|-----|-------------------|------|--------|
| S1 | LiPo low-voltage alarm / buzzer | 1–2 | **1S–8S**, plugs onto the pack's **JST-XH balance lead**, adjustable or 3.3 V/cell alarm point, per-cell display | ~$7 (usually sold in 2–4 packs) | [search](https://www.amazon.com/s?k=lipo+low+voltage+alarm+buzzer+1s-8s+balance) |

This is the highest-value part on the page and it costs about as much as a
coffee. It is the **only** protection that is independent of the firmware: it
still sounds if the ESP32 hangs, if the control loop wedges, or if the servo
bus drops — and the bus dropping is precisely how the firmware guard goes
blind, because the robot's only voltage sense is the servos' register 62
(no ADC on the driver board).

It also monitors **per cell**, which pack voltage fundamentally cannot. One
weak cell sitting at 2.9 V while the other two are healthy still reads ~10.6 V
at the pack — a number the firmware guard calls fine, because from where it
sits, it is.

Buy 2 if you want one living on each of the two packs; buy 1 and move it if you
don't mind the swap step.

**Fit note:** it hangs off the balance lead, which is the *unused* connector on
the pack — the XT30 carries the load. Check it clears the tower window when the
pack is seated, or let it sit outside the bay on its lead during bring-up.

---

## Worth it — the short-circuit case

| # | Component | Qty | Spec that matters | Est. | Source |
|---|-----------|-----|-------------------|------|--------|
| S2 | Inline blade-fuse holder, **mini (ATM)** | 1 | 16–18 AWG pigtail, waterproof/heat-shrink body | ~$8 (2-pack) | [search](https://www.amazon.com/s?k=inline+mini+blade+fuse+holder+16+awg) |
| S3 | Mini blade fuses, assorted | 1 pack | Need **10 A and 15 A**; start at 12–15 A | ~$7 | [search](https://www.amazon.com/s?k=mini+blade+fuse+assortment+10a+15a) |

Nothing in the power path has a fuse, polyfuse, e-fuse, or protection FET —
confirmed from Waveshare's schematic. Operating current is a non-issue (1.4 A
RMS, 6.8 A millisecond peaks, against a 5 A jack rating). The case this covers
is a **dead short**, where an 850 mAh 75C pack can deliver ~60 A into a pinched
or chafed lead. A robot that falls over repeatedly, with servo leads under
strain and a pack that swaps in and out of a window, is where that happens.

**Sizing:** 15 A is the safe first choice — comfortably above the 6.8 A
sim-derived peak, well below a short. Drop to 10 A only after the bench
measurement (S5) confirms the real peak, otherwise you will nuisance-blow on a
hard landing and learn nothing.

**Integration caveat, stated honestly:** the routed pigtail run is only ~30 mm
(wiring.md), and an inline holder plus its leads is more like 40–60 mm. This
does *not* drop in — expect either a longer pigtail with the fuse body coiled
into the y-pocket beside the XT30, or the holder sitting proud of the bay.
Decide where it lives before ordering the pigtail, and re-check the
battery-swap path still works with it in place.

**Do not substitute a polyfuse/PPTC.** Its trip time is seconds and its series
resistance is meaningful at these currents — wrong instrument for a dead short,
and it would sag the rail the rest of the time.

---

## Already recommended in wiring.md

| # | Component | Qty | Spec that matters | Est. | Source |
|---|-----------|-----|-------------------|------|--------|
| S4 | Low-ESR electrolytic capacitor | 1–2 | **470–1000 µF, ≥ 25 V**, low-ESR / "low impedance", 105 °C | ~$9 (assortment) | [search](https://www.amazon.com/s?k=470uf+25v+low+esr+electrolytic+capacitor) |

Across servo V+/GND at the board. The `6-12V` net carries only 10 µF + 0.1 µF
of local bulk, so a 6.8 A step is sourced all the way through the pack leads
and the barrel jack; a bulk cap sources it locally, cutting rail sag and the
transient the jack actually sees. It also makes the firmware guard less twitchy,
since less sag means fewer excursions toward the trip line.

**25 V, not 16 V.** A fully charged 3S is 12.6 V; standard derating practice
wants roughly double, and 16 V parts leave nothing. **Polarity matters** —
electrolytics fail loudly when reversed. This is a soldering job onto the
pigtail or a spare header, not a plug-in part.

---

## Optional — closes an open question

| # | Component | Qty | Spec that matters | Est. | Source |
|---|-----------|-----|-------------------|------|--------|
| S5 | Inline RC watt meter / power analyzer | 1 | ≥ 60 A, **XT30 or XT60 with adapters**, shows peak A | ~$20 | [search](https://www.amazon.com/s?k=rc+watt+meter+power+analyzer+60a+xt60) |

wiring.md's current table is **sim-derived, not measured**, and explicitly asks
for a bench check: *"Bench-verify the peak with an inline shunt or clamp during
the first walks."* This is the cheapest way to do that. It also settles the
fuse sizing above (S2/S3) with a number instead of a margin, and gives a real
mAh-consumed figure per walk — which is the honest input to any future
runtime estimate.

Not required for safety; it is the instrument that turns three estimates in
this repo into measurements.

---

## Also worth having (generic LiPo hygiene)

| # | Component | Qty | Spec that matters | Est. | Source |
|---|-----------|-----|-------------------|------|--------|
| S6 | LiPo charge/storage bag or case | 1 | Fits a 60 × 30 × 23 mm pack with room to spare | ~$12 | [search](https://www.amazon.com/s?k=lipo+safe+charging+bag) |

Standard practice for any LiPo, independent of this robot. Worth pairing with a
habit rather than a part: **storage-charge to ~3.8 V/cell (11.4 V) if a pack
will sit for more than a few days.** Leaving a pack full or flat is what kills
packs that never see a fault. A smart charger has a storage mode; use it.

---

## Totals

| Tier | Lines | Est. |
|---|---|---|
| Buy first | S1 | ~$7 |
| Worth it | S2, S3 | ~$15 |
| wiring.md recommendation | S4 | ~$9 |
| Optional instrument | S5 | ~$20 |
| LiPo hygiene | S6 | ~$12 |
| **All of it** | | **~$63** |

The protection story is ~$31 (S1–S4). The rest is instrumentation and hygiene.

## What is deliberately NOT here

- **A BMS / protection board.** These exist for LiPo packs but are sized for
  low-current cells; a board that passes 6.8 A peaks without sagging costs and
  weighs more than the problem justifies, on a 0.9 kg robot where mass is the
  binding constraint. The firmware guard plus the balance-lead alarm covers the
  same failure at ~10 g.
- **A current sensor (INA219 etc.).** The driver board has no sense resistor
  and no free path to add one without cutting the servo V+ net. S5 does the same
  job on the bench, which is where the question actually lives.
- **An anti-spark connector.** At 3S and this capacitance, XT30 connection arc
  is negligible. This matters at 6S and multi-thousand-µF, not here.
