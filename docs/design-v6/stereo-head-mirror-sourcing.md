# Stereo head: where to buy the mirrors (sourcing record)

*Searches 2026-09-26 and 2026-09-27. Companion to [`stereo-head.md`](stereo-head.md)
§3.1, which carries the short list. This file keeps everything: every candidate
found, how and when it was checked, what was rejected and why, and which V
square sizes fit the head.*

Tom, on what the glass has to be:

> *I'd think we could get some cheap mirrors and glue them in, then calibrate*
> — *don't use a prism then* — *they don't have to come from etsy. Look for
> another source.* (2026-09-26)

> *the design appears to call for smaller reflectors.. where should I get
> them? I don't want to try to cut these.* (2026-09-27)

---

## 1. What to buy

The head needs four front-surface glass mirrors, all sold **pre-cut**. Nothing
needs cutting.

| part | recommended | alternative | used |
|---|---|---|---|
| **V squares**, 25 × 20 × **1.1** mm | Amazon [B0FDB2KL5F](https://www.amazon.com/dp/B0FDB2KL5F), 5 pcs, **$23.41** delivered, Oct 9–22 (read 2026-09-27). **Pick the 1.1 mm option**: the same listing's 2 mm and 3 mm versions cost the same and lose field (§4). | eBay *explore-space* [158008301027](https://www.ebay.com/itm/158008301027), 5 pcs, **$12.66** (read 2026-09-26; eBay has blocked re-checks since, so confirm in a browser) | 2, plus 3 spares |
| **outer mirrors**, 50 × 50 × **1.1** mm | eBay *hnpiwxny* [188927818845](https://www.ebay.com/itm/188927818845), 5 pcs, **$16.34** delivered, import fees included (read 2026-09-26). Pick the 1.1 mm option. | Amazon MPXBM [B0GKG4LC9H](https://www.amazon.com/dp/B0GKG4LC9H), 2 pcs, **$18.22**, free delivery (read 2026-09-26). No spares. | 2 |

The whole order comes to:

| combination | glass, delivered |
|---|---|
| eBay (V squares) + eBay (outer mirrors) | **$29.00** |
| Amazon (V squares) + eBay (outer mirrors) | **$39.75** |
| all Amazon | **$41.63** |

**No manufacturer datasheets.** None of these sellers publishes one. That's
Tom's call for this part (cheap mirrors, glued in and calibrated), and it is a
deliberate exception to the repo's sourcing rule. So the glass is measured on
arrival (§6).

---

## 2. What a part has to be

1. **Front-surface coating.** The reflective layer must be on the front of the
   glass. Craft and household mirrors are back-silvered, and they fail at the
   V: the notch between two back-silvered tiles' glass ends blinds *both* eyes
   at the apex (traced, `periscope_optics.py`, `v_back_t`).
2. **Glass, rectangular, straight factory edges.** One square's edge meets the
   other at the V's point, so round mirrors (the 20/25 mm CO2-laser kind) don't
   work.
3. **Sold at size.** No cutting.
4. **1.1 mm thick or thinner.** Thickness costs field of view (§4).
5. **Size.** The V squares need 20–30 mm a side (§4). The outer mirrors are
   50 × 50 mm.

---

## 3. Everything found

### 3.1 V squares (the small ones), searched 2026-09-26 and 2026-09-27

The 09-26 search found no one selling 25 × 25 or 25 × 50 mm front-surface
pieces at this price level (the 25 × 25-class pieces found since cost $25–52
each: §3.1, §3.3). The small sizes the Chinese optics sellers stock are 20 × 15, 24 × 20,
25 × 20, 30 × 30, 32.5 × 26 and 35 × 35 mm, which is why the head is drawn for
25 × 20.

| # | where / seller | size × thickness (mm) | pcs | price, delivered | arrives | checked | notes |
|---|---|---|---|---|---|---|---|
| 1 | Amazon, Tuoyibaihuodian, [B0FDB2KL5F](https://www.amazon.com/dp/B0FDB2KL5F) | 25 × 20 × 1.1 | 5 | $21.42 + $1.99 = **$23.41** | Oct 9–22 | live, 09-27 | "Optical Front Surface Mirror … Reflectivity: Above 97%". Ships from China (seller-fulfilled). No reviews yet. 2 mm = B0FDB2NBTK, 3 mm = B0FDB1P8D2, same price. |
| 2 | eBay, explore-space, [158008301027](https://www.ebay.com/itm/158008301027) | 25 × 20 × 1.1 | 5 | **$12.66**, free shipping | not recorded | 09-26 only | Not re-checked since (eBay 403, §5). |
| 3 | eBay, hnpiwxny, [187423766193](https://www.ebay.com/itm/187423766193) | 25 × 20 × 1.1 | 5 | $9.90 + $3.93 = **$13.83** | not recorded | 09-26 only | Listed "Open box", no returns. A 09-27 search snippet showed $9.06–10.87 + $1.93 delivery (snippet only). |
| 4 | Amazon, RUEHALF (PTG Merchant), [B0GB7QSN6M](https://www.amazon.com/dp/B0GB7QSN6M) | 24 × 20 × 1.1 | 2 | **$19.32**, free | Oct 9–22 | live, 09-27 | "Design: First Surface Mirror". No spares, no reviews. |
| 4a | RUEHALF, same listing, other sizes | 20 × 20 × 1.1 ([B0GB8MYP18](https://www.amazon.com/dp/B0GB8MYP18)) | 2 | $18.56, free | | live, 09-27 | |
| 4b | | 30 × 30 × 1.1 ([B0GBF3LXVC](https://www.amazon.com/dp/B0GBF3LXVC)) | 2 | $19.32, free | | live, 09-27 | Fits today's layout (§4). |
| 4c | | 35 × 35 × 1.1 ([B0GB89X1C5](https://www.amazon.com/dp/B0GB89X1C5)) | 2 | $19.97, free | | live, 09-27 | Too long for today's layout (§4). |
| 4d | | 24 × 20 × 2 ([B0GBBNB7BH](https://www.amazon.com/dp/B0GBBNB7BH)), 30 × 30 × 2 ([B0GB8X1R9M](https://www.amazon.com/dp/B0GB8X1R9M)) | 2 | $19.32, $19.74 | | live, 09-27 | 2 mm: loses field. |
| 5 | Spectrum Scientifics, Philadelphia (US stock), [product 7657](https://www.spectrum-scientifics.com/First-Surface-Mirror-30mm-x-30mm-x-2mm-p/7657.htm) | 30 × 30 × **2** | sold singly | 2 × $9.99 + $7.85 USPS Ground Advantage = **$27.83** | ships in 1–2 business days | live, 09-27 (guest-cart estimate) | "has its mirror coating on the front of the glass". The only US-stock option found; the 2 mm glass costs 6° (§4). Free shipping only over $80. |
| 6 | Amazon, JOEBO, [B0DMZNWPGJ](https://www.amazon.com/dp/B0DMZNWPGJ) | 25 × 20 × 2 | 5 | $20.80 + $2.99 = $23.79 | Oct 9–22 | live, 09-27 | No 1.1 mm option actually offered. |
| 7 | Amazon, Tuoyibaihuodian, [B0FY1HL7ND](https://www.amazon.com/dp/B0FY1HL7ND) (1.1 mm: B0FY1J8T74) | 32.5 × 26 × 1.1 | 2 | $24.48 + $1.99 = $26.47 | | live, 09-27 | Too long for today's layout (§4). |
| 8 | Amazon, yajingmaoyishanghang, [B0F6XVX25L](https://www.amazon.com/dp/B0F6XVX25L) | 30 × 30 × 2 | 2 | $25.09 + $1.99 = $27.08 | | live, 09-27 | |
| 9 | Amazon, yajingmaoyishanghang, [B0DNW3L2Q1](https://www.amazon.com/dp/B0DNW3L2Q1) | 20 × 15 × 1.1 | 5 | $28.76 + $1.99 = $30.75 | | live, 09-27 | Fits, but with a smaller shared zone (§4). |
| 10 | Amazon, RUEHALF, [B0GW4ZPPL1](https://www.amazon.com/dp/B0GW4ZPPL1) | 30 × 30 × 2 | 1 | $18.26 each, $36.52 for 2 | | live, 09-27 | Its bullets contradict themselves (they describe a "dielectric mirror sheet"). |
| 11 | eBay, 1_60253, [307103249223](https://www.ebay.com/itm/307103249223) (30 × 30 option) | 30 × 30 × 1.1 | 5 | $12.35 + $3.00 = $15.35 | | 09-26 only | |
| 12 | AliExpress, [1005004590922032](https://www.aliexpress.com/item/1005004590922032.html) | 25 × 20 | 5 | not visible (captcha) | | not verified | |

**Documented, but far over budget:**
- **Edmund Optics** "25 x 25mm Enhanced Aluminum, 4-6λ Mirror"
  ([page](https://www.edmundoptics.com/p/25-x-25mm-enhanced-aluminum-4-6lambda-mirror/5288/)):
  - $31.00 each, per a search snippet (the page itself was blocked);
  - rhodium-coated version, $45.85;
  - full specs published.
- **First Surface Mirror LLC**, Toledo OH, cuts to any size:
  - 25 × 20 mm or 1 × 1 in, in 0.5, 1.0, 1.9 or 3 mm glass;
  - $52.21 for one, $44.38 each for two or more, plus FedEx (public price
    calculator);
  - technical specs published. Its sister site opticalmirror.com quotes the
    same.
- **Knight Optical** MEE1502:
  - 15 × 15 × 3 mm, US$26.97 each plus international shipping;
  - specs published.
- **Thorlabs** ME1S-G01: 1 × 1 in × 3.2 mm, too thick, price not visible.

### 3.2 Outer mirrors (50 × 50), searched 2026-09-26

| # | where / seller | size × thickness (mm) | pcs | price, delivered | arrives | notes |
|---|---|---|---|---|---|---|
| 1 | eBay, hnpiwxny (Nanyang, China), [188927818845](https://www.ebay.com/itm/188927818845) | 50 × 50 × 1.1 (also 1.6/2/3/5) | 5 | $12.64 + $3.70 = **$16.34**, import fees included | Oct 13 – Nov 2 | "Front Surface Projector Reflector Mirror", optical glass. Listed "Open box", no returns. Its item-specifics field is bogus ("8*3.4*10"); the description is right. |
| 2 | eBay, 1_60253 (Nanyang, China), [307103249223](https://www.ebay.com/itm/307103249223) | 50 × 50 × 1.1 option | 5 | $14.25 + $3.00 = **$17.25**, import fees included | Oct 21–27 | "5PCS Front Surface Mirrors". 30-day returns. |
| 3 | Amazon, MPXBM (ships from China), [B0GKG4LC9H](https://www.amazon.com/dp/B0GKG4LC9H) | 50 × 50 × 1.1 | 2 | **$18.22**, free | Oct 8–16 | "Front Surface Mirror … 50x50x1.1mm". |

The two eBay sellers both ship from Nanyang, China.

### 3.3 Looked at and rejected

- **Needs cutting:**
  - Rainbow Art Glass's front-surface 1.1 mm sheet, 140 × 203 mm, $18.98
    delivered from NJ;
  - MPXBM for the V (50 × 50 only);
  - First Surface Mirror LLC's kaleidoscope lot of 24 strips
    (1.125 × 5.5 in × 1 mm, $80.94) and its $9.95 samples (random-size
    offcuts);
  - kaleidoscope suppliers' strips and sheets (KaleidoscopesToYou, Delphi,
    Stained Glass Express, Diamond Tech Crafts, Franklin Art Glass);
  - Surplus Shed's 109 × 61 mm piece.
- **Back-silvered or no front-surface claim:**
  - Hobby Lobby 2 in craft squares (1/16 in, 10 for $1.99);
  - Plymor 2 in (3 mm);
  - Temu and Etsy "1 inch square" mirrors;
  - Eisco plano mirrors;
  - Amazon HENGYANGGUANGXUE [B0F9PKBFJZ](https://www.amazon.com/dp/B0F9PKBFJZ)
    (2.3 mm, sold singly, $21–30 each, never says front surface).
- **Not mirrors:** Surplus Shed's 12.5 and 25 mm squares ("NOT ALUMINIZED").
- **Round, too small, or gone:**
  - Surplus Shed's 17 and 35 mm front-surface mirrors are round, and its
    13 mm square is too small;
  - American Science & Surplus has closed its web store;
  - the 8, 10, 12, 20 × 13 and 15 × 10 mm variants on the RUEHALF and
    yajingmaoyishanghang listings are too small;
  - Amazon's multi-size projector mirrors start at 55 × 50 mm.
- **Unavailable:**
  - Amazon B0DT3DKW81 (25 × 20 × 2), B0F1C7LQZ2 (24 × 20 × 1.1) and
    B0DTJTKGTR (32.5 × 26 × 2) are all "Currently unavailable";
  - anchoroptics.com didn't respond;
  - Advanced Optics (WI) is quote-only.

---

## 4. Which V squares fit (the size study)

`cad/v6/periscope_optics.py --sizes`
(output: [`stereo_vsize_study.txt`](stereo_vsize_study.txt)). The outer
mirrors stay 50 × 50 × 1.1. Each candidate was traced two ways:

- **as laid out today:** the part dropped into the current layout, where only
  the printed pockets change (set `V_TILE`, re-run `head.py`);
- **re-searched:** the layout search re-run for that part, 3 seeds, ±1°.

The head as drawn gives 47° total with 16° shared.

| V square, along the leg × tall × thick (mm) | as laid out today: total / shared | re-searched: total / shared |
|---|---|---|
| **25 × 20 × 1.1** (as drawn) | **47° / 16°** | 46° / 16° |
| 24 × 20 × 1.1 | 47° / 15° | 46° / 16° |
| 20 × 25 × 1.1 | 47° / 13° | 47° / 16° |
| 20 × 20 × 1.1 | 47° / 13° | 46° / 16° |
| 20 × 15 × 1.1 | 47° / 12° | 42° / 16° |
| 15 × 20 × 1.1 | 47° / 9° | 42° / 16° |
| 25 × 25 × 1.1 | 47° / 16° | 48° / 16° |
| 1 × 1 in × 1.0 | 48° / 16° | 49° / 16° |
| 30 × 30 × 1.1 | 47° / 17° | 48° / 16° |
| 26 × 32.5 × 1.1 | 47° / 16° | 49° / 16° |
| 32.5 × 26 × 1.1 | **glass clash** | 49° / 16° (baseline 53 mm) |
| 35 × 35 × 1.1 | **glass clash** | 49° / 16° (baseline 57 mm) |
| 25 × 20 × 1.6 | 44° / 16° | 42° / 16° |
| 25 × 20 × 2 | 41° / 16° | 41° / 16° |
| 30 × 30 × 2 | 41° / 17° | 43° / 16° |
| 25 × 20 × 3 | 36° / 16° | 36° / 16° |

- **Size hardly matters.** 1.1 mm pieces from 20 × 15 to 30 × 30 mm all drop
  into today's layout at 47° total. Pieces 25 × 25 and up keep the full 16°
  shared; smaller ones shrink it.
- **Too long needs a new layout.** Pieces 32.5 mm or more along the leg hit the
  outer mirror's glass as laid out today (`glass_clash`). A re-layout reaches
  49°, but with a shorter baseline and a CAD rework.
- **Thickness is what costs field.** The right eye's square starts one glass
  thickness out from the apex, so extra thickness comes straight off the right
  eye.
  2 mm glass loses about 6° of the total, and 3 mm about 11°. Buy 1.1 mm or
  thinner.

---

## 5. How the listings were checked

- **eBay** was read live on 2026-09-26. From 2026-09-27, eBay returns HTTP 403
  to every automated read (web fetch, and curl with a browser user agent). So
  every eBay fact here is dated 09-26; check price and stock in a browser.
- **Amazon** product pages were read live on 2026-09-27: title, price, stock,
  delivery and variants.
  - Amazon priced them for its default location, Lyons CO 80540, so fees and
    dates may differ at your ZIP.
  - Amazon's search pages are bot-gated, so listings were found through web
    search and product-page carousels. Others may exist.
- **Other sites:**
  - AliExpress showed a captcha, and Edmund a Cloudflare challenge.
  - Spectrum Scientifics' shipping is a guest-cart estimate (nothing was
    ordered).
  - First Surface Mirror LLC's prices come from its public calculator.

---

## 6. When the glass arrives

1. **Check it is front-surface.** Touch a fingertip to the mirror. On a
   front-surface mirror, the fingertip meets its reflection. On a
   back-silvered one, there's a gap the thickness of the glass.
2. **Measure every piece** with calipers: both sides and the thickness.
3. **Update the optics and CAD.**
   - Set `periscope_optics.V_TILE` / `OUTER_TILE` to the measured sizes.
   - Run `.venv/bin/python cad/v6/periscope_optics.py --search`, and update
     `DESIGN` if the layout moves.
   - Run `MUJOCO_GL=cgl .venv/bin/python cad/v6/head.py` (all checks must
     PASS), then print.
4. **Keep the protective film on** the mirror faces until the glue has cured.
   Assembly and alignment are in [`stereo-head.md`](stereo-head.md) §5.
