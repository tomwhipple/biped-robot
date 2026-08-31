# GoPro MAX (2019) as a Real-Time Vision Input — Research Findings

*Researched 2026-07-11. Camera in question: original GoPro MAX (CHDHZ-201/202, released Oct 2019, 154 g), NOT the MAX2.*

## Summary verdict

**The MAX is not viable as a real-time perception sensor. Keep it as a recorder.**

- **No USB webcam mode** — that starts at HERO8 Black (verified, GoPro official).
- **No Open GoPro API** — that starts at HERO9 Black (verified, GoPro official).
- What the MAX *does* have is the **legacy gpControl WiFi API** (wake, shutter, mode/settings, and a low-res live preview): an MPEG-TS/H.264 stream over UDP port 8554 at roughly **432×240–480p**, with community-measured glass-to-screen latency of **1–4 seconds** (sub-second only with aggressive `ffplay -fflags nobuffer` tuning, and unreliably). In 360 mode the preview is **unstitched dual fisheye**; in Hero mode it is a normal single-lens view.
- **RTMP live streaming** (Hero mode only, 480p/720p/1080p) can point at a local RTMP server on the laptop, but RTMP latency is **2–5 s minimum** — worse than the preview for control purposes.
- For a 50 Hz control loop, anything above ~100–200 ms is useless for reactive perception and marginal even for slow goal-seeking. **1–4 s is 50–200 control ticks of lag.**
- **Recommended perception path:** a dedicated ~5–10 g WiFi camera module — Seeed XIAO ESP32S3 Sense (~5 g, OV2640/OV3660) or classic ESP32-CAM (~10 g) streaming MJPEG to the laptop at ~100 ms latency, or the `hx-esp32-cam-fpv` firmware which achieves a measured **90–110 ms** end-to-end. The GoPro keeps recording 360 footage for offline dataset building (which is genuinely valuable for training vision policies later).

The rest of this document details each question with sources. **[V]** = verified fact with source, **[C]** = community anecdote, **[I]** = my inference.

---

## 1. USB webcam mode

**[V] The MAX does not support webcam mode.** GoPro's official webcam support is HERO8 Black and later (HERO8/9/10/11/12/13); GoPro support staff confirm "the GoPro MAX does not support webcam functions" and that the MAX has no GoPro Connect USB mode — it enumerates as MTP (file transfer) only. It also has no HDMI output, so there is no clean-feed capture-card path either.

- [GoPro Webcam Information and Troubleshooting (official)](https://community.gopro.com/s/article/GoPro-Webcam?language=en_US)
- [GoPro Support Hub: "Max as Webcam" — answered: not supported](https://community.gopro.com/t5/Cameras/Max-as-Webcam/td-p/608175)

The belief "HERO8 and later only" is **confirmed**.

## 2. API support: Open GoPro vs. legacy WiFi API

**[V] Open GoPro does not support the MAX.** The official compatibility list is HERO9 Black and later (HERO9/10/10 Bones/11/11 Mini/12/13). The MAX (and Fusion) are excluded. The belief "HERO9+" is **confirmed**.

- [Open GoPro official docs](https://gopro.github.io/OpenGoPro/)
- [GoPro Open GoPro announcement — HERO9 first supported](https://gopro.com/en/us/news/open-gopro-announce)

**[V] The MAX does support the legacy gpControl WiFi API**, reverse-engineered and documented in KonradIT's `goprowifihack` (repo archived March 2025, but the MAX firmware is frozen too, so the docs remain accurate). The camera runs its own AP; the camera is always `10.5.5.9`. Documented MAX endpoints ([MAX folder of goprowifihack](https://github.com/KonradIT/goprowifihack/tree/master/MAX)):

| Function | Endpoint |
|---|---|
| Power off | `GET http://10.5.5.9/gp/gpControl/command/system/sleep` |
| Power on | Wake-on-LAN magic packet to 10.5.5.9, port 9 (BLE wake also works via the app pairing) |
| Mode: video/photo/multishot | `/gp/gpControl/command/set_mode?p=1000/1001/1002` |
| Shutter start/stop | `/gp/gpControl/command/shutter?p=1` / `?p=0` |
| Lens mode: Hero (single) vs 360 (dual) | `/gp/gpControl/setting/142/0` (single) / `142/1` (dual) |
| Active lens: front/rear | `/gp/gpControl/setting/143/0` / `143/1` |
| Start live preview | `GET http://10.5.5.9/gp/gpControl/execute?p1=gpStream&c1=restart`, then read `udp://10.5.5.9:8554` (MPEG-TS) |

- [goprowifihack MAX/MAX-Commands.md](https://github.com/KonradIT/goprowifihack/blob/master/MAX/MAX-Commands.md)
- [goprowifihack MAX/Livestreaming.md](https://github.com/KonradIT/goprowifihack/blob/master/MAX/Livestreaming.md)

### Preview stream: format, resolution, latency

- **[V] Format:** MPEG-TS over UDP port 8554, H.264 Main profile video + AAC audio. ([matmoi/gopro-stream](https://github.com/matmoi/gopro-stream), [Konrad Iturbe's streaming guide](https://medium.com/@konrad_it/how-to-stream-from-a-gopro-camera-f4a164150797))
- **[V] Resolution:** the legacy GoPro preview is low-res — 432×240 @ 30 fps on most gpControl-era cameras (some report 640×480; HERO9 raised it to 480p, HERO11 to 720p — both irrelevant to MAX). No MAX-specific measurement was found; expect ~240p–480p. **[I]**
- **[C] Latency:** community reports on gpControl preview across models are consistently **3–4 s initially, reducible to ~1 s or less** by forcing a stream restart and using `ffplay -fflags nobuffer -flags low_delay -probesize 8192 -f:v mpegts udp://:8554`. Nobody credibly reports sub-300 ms glass-to-screen on this stream. ([goprowifihack issue #283 — stream latency](https://github.com/KonradIT/goprowifihack/issues/283), [OpenCV forum thread on GoPro WiFi streaming](https://answers.opencv.org/question/68387/open-gopro-hero-wifi-live-streaming-with-vs-c/))
- **[C] Keep-alive required:** the preview dies within seconds unless a keep-alive datagram `_GPHD_:0:0:2:0.000000\n` is sent to `10.5.5.9:8554` every ~2.5 s (KEEP_ALIVE_PERIOD = 2500 ms in [KonradIT/GoProStream](https://github.com/KonradIT/GoProStream/blob/master/GoProStream.py)). A MAX owner reporting the UDP stream "freezing after 10 seconds" ([goprowifihack issue #274](https://github.com/KonradIT/goprowifihack/issues/274)) matches the no-keep-alive failure signature. **[I]**

### 360 mode vs Hero mode preview

- **[C/V] In 360 mode the preview stream is raw, unstitched dual fisheye** — two circular fisheye images side by side, no in-camera stitching over WiFi ([goprowifihack issue #170 — "2 fisheye instead of 360 live streaming"](https://github.com/KonradIT/goprowifihack/issues/170)). Software stitching on the laptop is possible in principle but adds latency and calibration work on top of an already-slow stream. **[I]**
- **[I] In Hero mode (setting 142/0, lens select 143/x) the preview is a normal single-lens view** — this is how the GoPro app shows Hero-mode framing, and it is the only mode GoPro supports for its own live streaming (see §3), which strongly implies the single-lens pipeline is the supported preview path.

## 3. RTMP live streaming

- **[V] Supported, Hero mode only.** GoPro's official position: "live streaming on the MAX is only available in HERO mode — 360 mode is not supported" ([GoPro Support Hub: 360 Live Stream support](https://community.gopro.com/t5/Cameras/360-Live-Stream-support/td-p/542042)).
- **[V] Resolutions 480p / 720p / 1080p**, auto bitrate ~2.5 Mbps @ 720p, ~5 Mbps @ 1080p ([GoPro live streaming support docs](https://community.gopro.com/s/article/How-To-Live-Stream-From-Your-GoPro?language=en_US)).
- **[V] Custom RTMP/RTMPS URLs are supported** via the GoPro Quik app's live-stream setup, so a local server (e.g. `rtmp://<laptop-ip>/live` on nginx-rtmp or MediaMTX) works — the "platform" doesn't have to be YouTube/Twitch ([Bliksund guide to GoPro RTMPS streaming](https://bliksund.com/blog-and-news/stream-live-rtmps-video-from-your-gopro), [GoPro Support Hub: Streaming (RTMP)](https://community.gopro.com/t5/Cameras/Streaming-RTMP/td-p/607667)). Setup requires the phone app each time the stream is started — there is no documented HTTP endpoint on the MAX to start an RTMP stream headlessly. **[I — caveat]** (GoPro Labs adds QR-code RTMP start, but [Labs RTMP control](https://gopro.github.io/labs/control/rtmp/) lists "HERO8–13 and BONES" only — MAX has Labs firmware but not the RTMP feature.)
- **[C] Latency: 2–5 s minimum** end-to-end even on a LAN (RTMP is TCP with encoder buffering), and >10 s if re-wrapped to HLS. Worse than the UDP preview for control purposes.
- **[V] Topology note:** for RTMP streaming the MAX operates as a WiFi *client* joining a hotspot/home network — the opposite of the preview API, where the MAX is the AP.

## 4. Practical experience reports (robotics / CV use)

- **[V] Library support:** [KonradIT/gopro-py-api (`pip install goprocam`)](https://github.com/KonradIT/gopro-py-api) explicitly lists MAX as supported for "WiFi + some BLE: controls, media management, status, live preview." `GoProCamera.GoPro().livestream("start")` wraps the `gpStream restart` call; `gopro.stream("udp://127.0.0.1:10000")` re-serves the feed locally via ffmpeg for OpenCV: `cv2.VideoCapture("udp://127.0.0.1:10000", cv2.CAP_FFMPEG)` ([example issue thread](https://github.com/KonradIT/gopro-py-api/issues/37)). [KonradIT/GoProStream](https://github.com/KonradIT/GoProStream) is the reference implementation of the keep-alive + viewer loop. Both projects are now archived/low-maintenance; there is no Open GoPro alternative for pre-HERO9 cameras.
- **[C] Reliability issues reported across users:**
  - Stream freezes without the 2.5 s keep-alive (see §2); some cameras also need periodic `gpStream restart` to recover.
  - Stream drops when the phone app reconnects (only one preview consumer at a time). **[C]**
  - Latency drifts upward over minutes as buffers grow; consumers typically restart the stream to re-sync. ([goprowifihack #283](https://github.com/KonradIT/goprowifihack/issues/283))
  - **Overheating:** MAX users report thermal shutdown in 16–30 min of continuous recording/streaming, faster in sun ([GoPro Support Hub: Max overheat issue](https://community.gopro.com/t5/Cameras/Max-overheat-issue/td-p/501793), [Go Pro Max overheating/battery issues](https://community.gopro.com/t5/Cameras/Go-Pro-Max-overheating-battery-issues/td-p/999158)). A torso-mounted camera on a walking robot has no airflow advantage; continuous preview + WiFi is a sustained thermal load. **[I]**
  - WiFi contention: the MAX AP on 2.4 GHz will fight the robot ESP32's own 2.4 GHz link (see §7). **[I]**
- I found **no** published robotics project that closed a control loop on a GoPro WiFi preview; the documented uses are FPV-style monitoring and framing (e.g. drone/QGroundControl viewers), where seconds of latency are tolerable. **[I from absence]**

## 5. 360 vs Hero mode

- **[V] Hero mode makes the MAX a single-lens camera** (front or rear selectable via setting 143). Video is 1440p 4:3 or 1080p 16:9, with digital lenses Narrow / Linear / Wide / **Max SuperView** — Max SuperView is the widest FOV GoPro shipped at the time, an ultra-wide ~ upscaled from the fisheye (GoPro doesn't publish exact degrees for MAX; HERO SuperView is ~ 16:9 from 4:3 sensor; community estimates Max SuperView at roughly 140–150° horizontal). ([GoPro: MAX Digital Lenses](https://community.gopro.com/s/article/MAX-Digital-Lenses-formerly-known-as-FOV?language=en_US), [MAX user manual video settings](https://usermanual.com/support/gopro/web-manual/max/video-settings))
- **[I] The preview stream in Hero mode is the usable one** for any vision purpose: single rectilinear-ish view, and it's the same pipeline GoPro certifies for live streaming. A wide digital lens at 240–480p, however, spends very few pixels per degree — fine for "is there a big red goal marker ahead," useless for fine features.
- 360 mode preview = unstitched dual fisheye (§2); stitching at 240p yields a very low-res equirect and adds latency. Not worth it in real time; extremely worth it *offline* (360 recordings give you every heading for free when building nav datasets). **[I]**

## 6. Verdict: perception sensor architecture

**[I] Numbers against the MAX as the real-time eye:**

| Path | Resolution | Real latency | Headless start? | Mode |
|---|---|---|---|---|
| MAX WiFi preview (UDP 8554) | ~432×240–480p | 1–4 s (best ~1 s) | Yes (HTTP + keep-alive) | Hero or 360 (unstitched) |
| MAX RTMP → local server | up to 1080p | 2–5 s | No (needs phone app) | Hero only |
| MAX USB webcam | — | — | — | Not supported |

At a 50 Hz control loop, even the best case (~1 s) is ~50 ticks of lag; a walking biped moves several body-lengths of foot placement in that time. The preview *is* adequate for slow, high-level goal selection ("waypoint is roughly left"), but the keep-alive fragility, single-consumer limit, thermal ceiling (~20–30 min), and battery drain make it a poor foundation.

**[I] Recommended: MAX stays a recorder; add a dedicated ~5–10 g camera for perception.** Options, in rough order of fit:

1. **Seeed XIAO ESP32S3 Sense** (~5 g with camera, 21×17.8 mm, OV2640/OV3660): streams MJPEG over WiFi; stock esp32-camera MJPEG latency is ~one frame period (~100 ms at 10 fps, less at higher fps). Runs as a *separate* module from the Waveshare servo ESP32 (that board has no camera DVP interface), powered from the robot's 5 V rail (<300 mA·5 V ≈ 1.5 W worst case — small next to 8×55 g servos on 3S 850 mAh). ([Seeed product page](https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html), [ESP-FAQ camera application notes](https://docs.espressif.com/projects/esp-faq/en/latest/application-solution/camera-application.html))
2. **Classic ESP32-CAM (AI-Thinker, OV2640, ~10 g):** same idea, cheaper, bulkier, worse antenna. ([Waveshare ESP32-CAM](https://www.waveshare.com/esp32-cam.htm))
3. **`hx-esp32-cam-fpv` firmware** on either board: measured **90–110 ms** end-to-end at 640×360@30 or 720p@12–30, MJPEG over raw WiFi packet injection with FEC — but the receiver then needs an RTL8812AU-class USB adapter in monitor mode on the laptop. Best latency of the WiFi options. ([RomanLut/hx-esp32-cam-fpv](https://github.com/RomanLut/hx-esp32-cam-fpv))
4. **Analog FPV cam + 5.8 GHz VTX + USB receiver on the laptop:** ~10–40 ms latency, ~5–15 g, but adds a battery-hungry transmitter, a second RF system, and NTSC-quality images — the classic choice when latency is everything; probably overkill for goal-seeking. ([Oscar Liang FPV camera latency testing](https://oscarliang.com/fpv-camera-latency/))
5. **Small USB webcam wired to... nothing** — there is no onboard computer; the laptop is remote, so USB is not an option without adding an SBC (a Pi Zero 2 W + camera ≈ 16 g + 11 g is the step-change option if onboard inference is ever wanted). **[I]**

Division of labor: ESP32-S3 camera → laptop (perception, goal inference) → policy commands to the Waveshare ESP32 at 50 Hz; GoPro records 360 for offline analysis, dataset building, and glory shots. Optionally keep goprocam wake/shutter control so the robot can start/stop recordings autonomously — the *control* API is low-bandwidth and reliable even though the *video* path is slow.

## 7. WiFi topology

Constraints: the MAX preview API only works with the MAX as its own AP (10.5.5.9); the Waveshare ESP32 is 2.4 GHz-only; the laptop must talk to both.

**[V] Key fact that makes this easy: the MAX AP can run on 5 GHz** (settable on-camera; in the US it uses channel 161). ([GoPro: Camera Wi-Fi Information](https://community.gopro.com/s/article/What-is-the-Camera-Wi-Fi-Frequency?language=en_US), [Have Camera Will Travel: GoPro WiFi band frequency](https://havecamerawilltravel.com/action/gopro-wifi-band-frequency/))

**Recommended topology [I]:**

- **Robot ESP32 → STA mode on the house 2.4 GHz network** (or its own AP if in the field). Pin the house AP to channel 1, 6, or 11 and note which.
- **GoPro MAX AP on 5 GHz (ch 161)** — zero spectral overlap with the ESP32's 2.4 GHz link. This alone removes the worst failure mode (preview traffic starving the 50 Hz command stream).
- **Laptop with two WiFi interfaces:** internal card joins the GoPro's 5 GHz AP (it assigns the laptop 10.5.5.x); a cheap USB WiFi dongle (or Ethernet to the house router) carries the ESP32/control network. macOS/Linux both handle two WLAN interfaces fine as long as only one has a default route — the GoPro side needs only the 10.5.5.0/24 route.
- If the MAX must stay on 2.4 GHz (region-locked units hide the 5 GHz option outside US/Japan — US unit should be fine): put the GoPro AP and the ESP32 network on channels 1 vs 11, and accept some contention. **[C/I]**
- The future ESP32-S3 perception camera joins the same house/robot 2.4 GHz network as the servo ESP32 — no third network needed; budget ~2–4 Mbps for MJPEG at 640×480@15. **[I]**
- For MAX RTMP streaming (the one case where the MAX is a client): point it at the laptop's hotspot or the house network and run MediaMTX/nginx-rtmp on the laptop. This cannot coexist with the preview-API topology at the same time — the camera is either an AP (preview) or a client (RTMP). **[V/I]**

---

## 8. Talking to it from mira — `tools/gopro_live.py`

Written and tested 2026-08-31. Implements §2's control path: gpControl probe,
`gpStream restart`, the 2.5 s `_GPHD_` keep-alive, and ffmpeg capture off
UDP 8554.

```
.venv/bin/python tools/gopro_live.py --probe                  # reachable? what is it?
.venv/bin/python tools/gopro_live.py --snapshot /tmp/gp.jpg   # cheapest proof of life
.venv/bin/python tools/gopro_live.py --record 10 --out /tmp/gp.mp4
.venv/bin/python tools/gopro_live.py --serve 8647             # http://mira:8647/
```

Verified against a stub camera on loopback (gpControl HTTP + a synthetic
432x240 MPEG-TS feed on UDP 8554), which is the same shape as the real
preview. `--probe` printed the identity, `--snapshot` produced a 432x240
JPEG, `--record 4` produced a 4.000 s MP4 (120 frames) and a 4.000 s TS.
Not yet run against the physical camera — see the two blockers below.

Three things cost real time getting there; they are encoded in the tool and
worth not rediscovering:

- **`-t SEC` does not work on this stream.** We always join an
  already-running preview, whose MPEG-TS timestamps start far from zero, so
  `-t` is satisfied immediately and ffmpeg writes an empty file (measured: a
  262-byte MP4 of 0.000000 s). Count frames instead — `-frames:v` is
  timestamp-independent — and rebase with `setpts=PTS-STARTPTS` so the
  duration reads true.
- **`-c copy` into MP4 does not work either.** Joining mid-GOP there is no
  leading SPS/PPS, so the MP4 muxer cannot build its extradata. MPEG-TS
  tolerates it; anything else has to be re-encoded (trivial at 432x240).
- **`-probesize 32768 -analyzeduration 0` breaks lock-on.** Those are the
  flags the community quotes for low latency, but combined with a mid-stream
  join ffmpeg misses the SPS/PPS and floods `non-existing PPS 0 referenced`
  until the next keyframe. `-fflags nobuffer -flags low_delay` alone locks on
  cleanly. The tool counts and suppresses the remaining lock-on lines.

### Two blockers before this can run for real

1. **The camera has to be put into AP mode by hand.** Power on, then
   Preferences > Connections > Connect Device > GoPro App. There is no remote
   way in: the MAX has no BLE control API (§2), and a 45 s BLE scan from mira
   on 2026-08-31 saw no GoPro advertising at all. Wake-on-LAN (`--wake MAC`)
   only helps once the camera has been paired and is merely asleep.
2. **mira's WiFi is soft-blocked and `claw` cannot unblock it.** `rfkill list`
   shows `phy0: Soft blocked: yes`; `rfkill unblock wifi` and
   `nmcli radio wifi on` both need root, and NetworkManager reports
   `enable-disable-wifi: no` / `wifi.scan: auth` for this user. mira reaches
   the LAN over USB ethernet (`enxf8e43b5e358e`, 192.168.2.5), so nothing
   depends on the WiFi card today.

Once someone with root has run:

```
sudo rfkill unblock wifi && sudo nmcli radio wifi on
nmcli device wifi list                                  # find the GPxxxxxxx SSID
sudo nmcli device wifi connect GPxxxxxxx password <pw> ifname wlp14s0
sudo nmcli connection modify GPxxxxxxx ipv4.never-default yes ipv6.never-default yes
```

...mira keeps its LAN default route over ethernet and only routes 10.5.5.0/24
over WiFi, and `--probe` should answer from 10.5.5.9. The AP SSID and password
are shown on the camera under the same Connections menu.

---

### Source quality note

`goprowifihack`, `gopro-py-api`, and `GoProStream` are reverse-engineered community projects (now archived) — authoritative in practice for legacy cameras since GoPro never documented this API, but not vendor-supported. GoPro official sources are used for webcam/Open GoPro/RTMP/band facts. Latency figures are community measurements, not benchmarks I ran; expect ±50% variation with RF conditions.
