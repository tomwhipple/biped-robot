# Replacing the GoPro MAX — self-powered camera options

*2026-08-31. Prompted by "recommend a more appropriate camera with a
self-contained power source", after `tools/gopro_live.py` could not reach the
MAX at all (see [gopro-vision-input.md](gopro-vision-input.md) §8).*

**[V]** = verified with a source, **[C]** = community anecdote, **[I]** = inference.

## What the robot actually constrains

Read off this repo, not assumed:

- **The mount is a standard GoPro three-prong finger.** `gopro_base` has
  3.2 mm finger slots and clamps with an M5×20 thumbscrew
  ([assembly.md](assembly.md) §11, [bom-sourced.md](bom-sourced.md) row 13).
  Any camera with GoPro folding fingers drops in with **zero CAD work**, and
  `gopro_base` is deliberately the sacrificial crash fuse, cheap to reprint.
- **The camera is already unpowered by the robot** —
  [hardware-order.md](hardware-order.md): "GoPro unpowered by the robot". So
  self-contained power is the *existing* design assumption, not a new one.
- **Payload mass is trained, not free.** `env_mjx.py` carries a `payload` body;
  the shipped policies were hardened with camera-payload DR over **0–170 g**
  ([DESIGN.md](../DESIGN.md)), `payload_mass=0.154`, `payload_cg_z=0.0945`.
  **Anything lighter than 154 g is strictly inside the trained envelope and
  needs no retrain.** [I] The mass sits at the very top of the tower, which is
  the worst place on a biped to carry it, so lighter is a real win, not a
  cosmetic one.
- **2.4 GHz is contended.** The robot's ESP32 link is 2.4 GHz-only
  (gopro-vision-input.md §7); a camera that also insists on 2.4 GHz competes
  with the 50 Hz command stream.

## The three honest options

| | GoPro **LIT HERO** | **XIAO ESP32S3 Sense** + 1S LiPo | Insta360 **GO Ultra** |
|---|---|---|---|
| Mass | **93 g** [V] | ~10–15 g [I, unweighed] | **52.9 g** [V] |
| Own battery | built-in, ~100 min [V] | LiPo pads + onboard charger [V] | built-in, ~200 min [V] |
| Live video off-robot | Open GoPro preview, ~480p–720p, **~1 s** [C] | MJPEG, **~100 ms** [C] | **none — cannot livestream or act as a webcam** [V] |
| Remote power-on | **yes, BLE** (Open GoPro) [V] | yes, it is your own firmware | no |
| Fits `gopro_base` as-is | **yes** — built-in folding fingers [V] | no — needs a printed adapter [I] | no — magnetic mount [I] |
| Cost | $199 MSRP at launch [V] | ~$25 board + a few $ of cell | ~$450 class |

### Recommendation: **GoPro LIT HERO**, if you want a camera

93 g, built-in non-removable battery (~100 min, USB-C charge), **built-in
folding mounting fingers that work with any GoPro accessory** — so it clamps
straight into the existing `gopro_base` with the M5 thumbscrew you already
have on the BOM. It is 61 g lighter than the MAX at the worst possible
location on the robot, and 93 g is inside the 0–170 g payload DR, so **no
retrain**.

The decisive part is not the mass, it is the API. LIT HERO is on the **Open
GoPro** supported list (alongside MISSION 1, MISSION 1 Pro, MAX 2, HERO13/12/
11 Mini/11/10/9 — the MAX is not, and never will be). That is a
GoPro-maintained, documented BLE + WiFi API, which means **the exact failure
we hit on 2026-08-31 goes away**: BLE control lets mira wake and configure the
camera without anyone walking over to press Connections > Connect Device.
No `_GPHD_` keep-alive reverse-engineering, no archived repos.

What it does **not** fix: preview latency is still ~1 s [C]. That is fine for
a monitoring view and marginal for Tier-3 goal-setting via `cmd`
([sensor-expansion.md](sensor-expansion.md) §2), and still useless for
anything reactive.

**Check before buying:** [I] measure the LIT HERO's finger thickness against
the 3.2 mm slots (bom-sourced.md already flags this for the MAX), and confirm
whether its AP can run on 5 GHz — if it is 2.4 GHz-only it will contend with
the robot's ESP32 link.

### If the real goal is perception in the loop: **XIAO ESP32S3 Sense + a 1S LiPo**

Nothing with a consumer-camera badge closes a 50 Hz loop, LIT HERO included.
The only self-powered option that gets to ~100 ms is the board
gopro-vision-input.md §6 already recommended — and it turns out to satisfy
"self-contained power" for free: the XIAO ESP32-S3 has a **built-in power
management chip with BAT± pads and onboard charging**, so a 3.7 V cell soldered
to the underside makes it fully independent of the robot's electronics [V].

Trade-offs, honestly: OV2640/OV3660 image quality is poor next to any of the
cameras above; charging is slow (**50 mA fast / 3.8 mA trickle** [V]) so charge
it off a bench USB, not between runs; it is 2.4 GHz-only, so it shares the
band with the servo link; and it needs a small printed finger adapter to the
`gopro_base` prongs, which is an hour of build123d, not a redesign.

### Not recommended: Insta360 GO Ultra

52.9 g and a 200-minute battery make it the lightest good-quality recorder
here, and on mass alone it is the best of the three. But Insta360's own manual
states the **GO Ultra does not support livestreaming and cannot be used as a
webcam** [V]. It is a write-only device: excellent glory shots and offline
360 dataset material, zero help with the problem that started this. Given the
MAX already fills the recorder role, buying one is spending ~$450 to save 100 g
and lose the live path entirely.

## Sources

- Open GoPro supported models (LIT HERO, MAX 2, HERO13…): <https://gopro.github.io/OpenGoPro/docs/>
- LIT HERO product page (fingers, 1/4-20, built-in light): <https://gopro.com/en/us/shop/cameras/learn/lit-hero/CHDHF-132-master.html>
- LIT HERO $199 MSRP announcement: <https://investor.gopro.com/press-releases/press-release-details/2024/GoPros-Tiny-4K-Camera-Hits-Global-Shelves-at-199-MSRP/default.aspx>
- LIT HERO 93 g + specs: <https://camerajabber.com/photography-news/gopro-lit-hero-price-specifications-and-availability-announced/>
- GO Ultra 52.9 g / 200 min: <https://www.insta360.com/product/insta360-go-ultra>
- GO Ultra cannot livestream or webcam: <https://onlinemanual.insta360.com/goultra/en-us/faq/functionality/livestream>
- XIAO ESP32S3 battery pads, onboard charger, 50 mA charge: <https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/>
- XIAO ESP32S3 Sense MJPEG streaming: <https://www.makerguides.com/stream-video-with-with-xiao-esp32-s3-sense/>
- Preview-stream resolution by model (480p HERO9, 720p HERO11): <https://github.com/gopro/OpenGoPro/issues/582>

**Prices are launch/MSRP figures from press coverage, not live listings** —
check the current price in a browser before ordering (WebFetch cannot see
Amazon price or stock).
