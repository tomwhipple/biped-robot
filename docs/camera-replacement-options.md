# Camera for navigation + limited perception (processed on mira)

*2026-08-31. Prompted by "recommend a more appropriate camera with a
self-contained power source", then narrowed by the decision that follows.
Supersedes the first cut of this file, which answered the wrong question.*

**[V]** = verified with a source, **[C]** = community anecdote, **[I]** = inference.

## The decision that sets the requirements

> "We want to use the camera for **navigation and limited perception but not in
> a reactive control loop**. That work will happen on a local workstation
> (mira)." — Tom, 2026-08-31

That is exactly the Tier-3 seam [sensor-expansion.md](sensor-expansion.md) §2
already describes: *perception sets the goal; the policy walks.* Perception
never enters the obs frame, so it needs no retrain, no sim change, and no new
obs dimension — it arrives through `cmd`/`ext_cmd` at whatever rate mira can
manage.

The consequence for hardware is blunt: **we need a video link, not a
recorder.** That inverts the earlier recommendation in this file.

What actually matters, in order:

1. **Frame timestamps mira can trust**, syncable to the rest of the robot's
   telemetry. This is the requirement no action camera meets, and it is the
   one that decides everything downstream. [I]
2. **Calibratable, fixed optics** — known intrinsics, so OpenCV can undistort
   and do PnP. A camera that silently re-crops or stabilises is worse than a
   cheap one that does not.
3. **Enough pixels at range** for markers/obstacles: 720p is comfortable, VGA
   is workable for large targets.
4. **Latency ~100–300 ms** is plenty; ~1 s is not, and 50 ms buys nothing.
5. Self-powered, and light — the payload sits at the very top of the tower.

Unchanged repo constraints (read off the tree, not assumed):

- `gopro_base` is a **standard GoPro three-prong finger mount**, 3.2 mm slots,
  M5×20 thumbscrew ([assembly.md](assembly.md) §11), and is the deliberate
  sacrificial crash fuse — cheap to reprint into a different adapter.
- The camera is **already unpowered by the robot**
  ([hardware-order.md](hardware-order.md)).
- Payload DR covers **0–170 g** ([DESIGN.md](../DESIGN.md), `payload_mass=0.154`,
  `payload_cg_z=0.0945`), so anything under 154 g needs **no retrain**. [I]
- The servo command link is **2.4 GHz-only** (gopro-vision-input.md §7), so
  every option below competes with it for air.

## Recommendation: **Raspberry Pi Zero 2 W + Camera Module 3 (Wide) + a LiPo**

Roughly 9 g board [V] + camera module + cell and regulator — call it **45–55 g
assembled** [I, unweighed], a third of the GoPro MAX and comfortably inside the
trained payload envelope.

It wins on the requirement that actually matters. It is a real Linux box, so:

- **libcamera/v4l2 give per-frame timestamps**, and the Pi can NTP-sync to
  mira — you can line camera frames up with BNO055 attitude after the fact.
  Nothing in the action-camera class offers this at all. [I]
- **Standard calibration**: fixed lens, published sensor (IMX708), ordinary
  OpenCV intrinsics. The Wide variant is 102° horizontal / 120° diagonal [V],
  which is the right framing for navigation.
- **Hardware H.264, 1080p30** encode [V] (measured ~19 fps at 1080p on this
  board [C]; 720p streams comfortably [C]) — so bandwidth stays low over
  RTSP/UDP to mira, and the receive side is the pattern `tools/cam_live.py`
  already uses.

Honest costs:

- **2.4 GHz only** [V] — it shares the band with the servo link. Put the video
  on a different channel or a different AP from the robot's command link and
  budget a few Mbps; do not put both on the same channel and hope. [I]
- **Rolling shutter** (IMX708 is a rolling-shutter sensor [V]) on a walking
  biped means jello and skew, worst at foot strike. Keep exposure short and
  expect to reject frames around contact. [I]
- It boots in tens of seconds and wants a clean shutdown or a read-only
  rootfs — it is a computer on the robot, with a computer's failure modes. [I]
- Needs a printed adapter from the board to the `gopro_base` prongs. That is
  an hour of build123d, not a redesign. [I]

**Cheaper start:** [XIAO ESP32S3 Sense](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/)
+ a 1S LiPo — ~$25, ~10–15 g [I], **BAT± pads with an onboard charger** so it
is self-powered for free [V], MJPEG at ~100 ms [C]. If VGA-class marker and
blob detection is enough to start the navigation work this week, buy this
first; it is cheap enough to be wrong about. Its ceilings are real though:
OV2640/OV3660 optics, a hard framerate cliff above VGA, no trustworthy frame
timestamps, and the same 2.4 GHz contention.

**Also considered:** the [FireBeetle 2 ESP32-P4](https://www.dfrobot.com/product-2915.html)
($16.50, 25.4×60 mm) is tempting — 2-channel MIPI-CSI pin-compatible with Pi
cameras, hardware H.264 to 1080p. But its Wi-Fi 6 comes via an **ESP32-C6
co-processor, so it is 2.4 GHz-only** [V]; it buys no relief from the band
contention, has no onboard LiPo charging [V], and has a far thinner streaming
software stack than the Pi. Espressif's P4 + **C5** pairings are dual-band [C]
and would dodge the servo link entirely — worth revisiting if a mature board
appears, but not something to build the first navigation experiment on. [I]

## Not an action camera — correcting the earlier recommendation

The first version of this file recommended the **GoPro LIT HERO** (93 g, Open
GoPro API, drops straight into `gopro_base`). That was the right answer to
"recommend a camera" and is the wrong answer to "a perception sensor for
navigation, processed on mira":

- Its live path is the **~480p–720p preview stream at roughly 1 s** [C] — the
  4K is recorded to the card, not available live. One second is 0.5 m of
  travel at this robot's 0.52–0.63 m/s [V, DESIGN.md]. Fine for watching,
  wrong for navigating.
- No usable per-frame timing, and HyperSmooth stabilisation actively breaks
  the rigid camera↔IMU relationship that pose estimation depends on — you
  would have to turn off the feature you paid for. [I]

**Insta360 GO Ultra** is out for a simpler reason: Insta360's own manual states
it **cannot livestream and cannot act as a webcam** [V]. Write-only.

Both remain good *recorders*. The GoPro MAX already fills that role, and
`tools/gopro_live.py` will talk to it the day someone puts it in AP mode.

## Before buying anything: split "where am I" from "what do I see"

The robot has **no odometry** — the `goal` source in `link/sources.py` is
marked SIM-ONLY for exactly this reason ("the real robot has no odometry ...
closing that gap (encoder dead-reckoning, or the GoPro) is its own piece of
work"), and `stick_to_speed` notes "we have no odometry off" the sim. It is worth noticing that an
on-robot camera is the expensive way to solve the *pose* half:

- **Where am I** — a fixed camera watching the arena plus a marker on the
  robot gives ground-truth pose at 30 Hz with **zero payload, zero battery,
  zero RF, and no rolling-shutter problem**. mira already has two bench
  webcams driven by `tools/cam_live.py` (`/dev/video0` side, `/dev/video2`
  rear). This is days of work, not a purchase. [I]
- **What do I see** — obstacles and goal objects from the robot's own
  viewpoint genuinely need an on-robot camera. That is the half worth
  spending payload on.

Doing the fixed-camera pose channel first would let the `goal` source in
`link/sources.py` come off its SIM-ONLY caveat without touching the robot's
mass at all, and would give a ground-truth reference to check any on-robot
estimate against later. [I]

## Sources

- Pi Zero 2 W 9 g, 2.4 GHz, H.264 1080p30: <https://www.hackster.io/news/raspberry-pi-zero-2-w-review-hands-on-with-the-fastest-zero-ever-b85b155905a5>
- Camera Module 3 / IMX708, rolling shutter, 102° H wide: <https://www.raspberrypi.com/documentation/accessories/camera.html>
- Camera Module 3 product brief: <https://datasheets.raspberrypi.com/camera/camera-module-3-product-brief.pdf>
- XIAO ESP32S3 battery pads + onboard charger: <https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/>
- FireBeetle 2 ESP32-P4 (2.4 GHz via C6, MIPI-CSI, H.264): <https://www.dfrobot.com/product-2915.html>
- ESP32-P4 + C5 dual-band board: <https://www.cnx-software.com/2025/12/17/compact-development-board-features-a-single-esp32-p4-esp32-c5-dual-band-wi-fi-6-module-mipi-display-and-camera-interfaces/>
- Open GoPro supported models: <https://gopro.github.io/OpenGoPro/docs/>
- GoPro preview-stream resolution by model: <https://github.com/gopro/OpenGoPro/issues/582>
- Insta360 GO Ultra cannot livestream or webcam: <https://onlinemanual.insta360.com/goultra/en-us/faq/functionality/livestream>

**Prices are MSRP/listing figures from vendor pages, not live stock** — check
in a browser before ordering.
