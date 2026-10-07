# Print list

What to print for the robot, how to orient each part, what supports each one
needs, and how to slice and send it. The CAD is `cad/v6/`; its code map is
[README.md](README.md). What to buy is [docs/bom.md](../docs/bom.md), and how it
goes together is [docs/assembly.md](../docs/assembly.md).

**The code's default build is the robot to print**: arms on, the pelvis with
the girdle's deck pilots, 17 × STS3215 in the mass line. The `ARMS=0` variant
(`neck_collar` for the neck, no girdle, no arm links, no pilots) exists for
the CAD checks and is **not a print target**: the collar's flange screws have
nothing to bite (their holes land in the Pi slide slot and the General Driver
lead slot, and there is no deck under the rest of the flange).

## Status

| can print now | provisional |
|---|---|
| `foot_L`, `foot_R`, `sole_tpu_L`, `sole_tpu_R`, `ankle_link` ×2, `leg_link_v6` ×4, `hip_yoke_v6` ×2, `head_shell`, `head_face`, `neck_floor`, `shoulder_girdle_v6`, `arm_upper_v6_L/_R`, `arm_fore_v6_L/_R` | **`pelvis_v7` and the two yaw carriers are provisional, waiting on #75** (the hip-yaw bearing option). The code exports option C as a placeholder; option E adds a cap and a retainer per hip. |

**Fit check first (#79).** Before printing the full set, print one
`leg_link_v6` and one `hip_yoke_v6`. Fit them to a real servo, and check:

- the grip channel slides on;
- the disc screws engage;
- the supported horn-seating face sits flat on the disc.

## The parts

**How the masses are computed:** build123d volume × PETG 1.27 g/cm³ × 0.90
print factor (`parts_v6.mass_g`); TPU is 1.21 g/cm³ with the same factor. They
are the rollup's, `docs/design-v6/parts_v6_rollup.txt`
(`cad/v6/parts_v6.py --no-export`).

**Bounding boxes** are in the model frame (mm). **Supports** are what to tell
the slicer; see [Supports](#supports-are-the-slicers-job).

| part | qty | g each | bbox | orientation on the bed | supports | notes |
|---|---|---|---|---|---|---|
| `pelvis_v7` | 1 | 194 (C) · 190 (A) · 193 (E) | 162 × 105 × 83 (C) | deck top down | **yes:** the two yaw-cell ceilings; the skin-window roofs near the housing floor | one print, the whole torso, with the girdle's ten deck pilots. **Provisional, waits on #75.** The A / E versions export as `pelvis_v7_optA` / `pelvis_v7_optE` |
| yaw carrier (`yaw_carrier_v6` for C, `yaw_carrier_v6_optA` for A, `yaw_carrier_v6_optE` for E) | 2 | 29.9 (C) · 23.8 (A) · 24.5 (E) | 55 × 55 × 44 | horn-plate face down, bay walls rising | **yes:** the rear-wall connector window's ceiling, a 1 mm bar spanning 22.8 mm | carries the hip-roll servo in its bay. **Provisional, waits on #75** |
| *option E:* `yaw_cap_optE`, `yaw_retainer_optE` | 2 + 2 | 1.8, 2.7 | Ø57 × 1.0; 78 × 75 × 2.4 | flat | no | screwed rings that retain the bearing races; one retainer part serves both hips (turned 180°). **Provisional, waits on #75** |
| `hip_yoke_v6` | 2 | 25.4 | 74 × 48 × 44 | on edge, model −Y on the bed, so the roll arms print as walls | **yes + 5 mm brim** | roll and pitch clevis in one print; see its notes below |
| `leg_link_v6` | 4 | 27.1 | 117 × 44 × 31 | standing on the lower fork end | no; brim recommended | thigh and shin are the same part |
| `ankle_link` | 2 | 12.1 | 65 × 44 × 44 | on its back (model +X up) | **yes** (ceiling/island class) | grips the ankle-pitch servo, forks onto the ankle-roll servo |
| `foot_L`, `foot_R` | 1 + 1 | 52.0 | 130 × 84 × 30 | sole down | no | a mirrored pair |
| `sole_tpu_L`, `sole_tpu_R` | 1 + 1 | 23.2 | 130 × 84 × 2 | flat | no | **TPU 95A**; glued to the foot plate's underside |
| `head_shell` | 1 | 26.1 | 62 × 60 × 50 | base (horn plate) down; the dome is ≥ 45° | none expected | from `head.py` |
| `head_face` | 1 | 8.6 | 62 × 56 × 6 | flat, camera bosses up | no | |
| `neck_floor` | 1 | 4.0 | 46 × 45 × 5 | flat, bottom face down, pads and lugs up | no | the neck servo's seat (#90): screwed up into the neck tube's four bosses; the stator screws go up through it |
| `shoulder_girdle_v6` | 1 | 77.1 | 200 × 58 × 33 | base down | **yes**, build plate only | carries both shoulder servos and the neck tube (with the four bosses for `neck_floor`) |
| `arm_upper_v6_L`, `_R` | 1 + 1 | 33.1 | 182 × 44 × 29 | on its back, web face down | **yes**, build plate only: the two elbow-pad undersides start 5 mm off the bed | a mirror pair; the front wall prints as a 6.4 mm bridge |
| `arm_fore_v6_L`, `_R` | 1 + 1 | 33.7 | 169 × 38 × 28 | on its back | no | a mirror pair; grips the elbow servo; 12 mm hand knuckle |

**Part notes:**

- **`pelvis_v7`**
  - The yaw-cell ceilings need support because the servo fills its cell to
    0.3 mm, so no support-free ceiling geometry exists.
  - Check the slice preview at the window roofs and at the deck-slot edges.
  - The deck does not seat the neck servo: its battery aperture runs under the
    whole servo. The seat is `neck_floor`.
- **Yaw carrier**
  - The carrier is not covered by the robot's printability audits.
  - The CAD models no break-away support under the connector window's ceiling
    bar, so the slicer has to support it.
- **`hip_yoke_v6`**
  - The pitch horn arm is a slab over the servo void, and the roll pad rims and
    arm plates start in mid-air.
  - The underside of the pitch horn arm is a support-interface face that clamps
    onto the horn disc. Check that it seats flat.
- **`leg_link_v6`:** the brim is there because the round pads meet the bed on a
  line.
- **`head_shell`:** the printability audit flags a ceiling at the dome seam. It
  is believed to be a boolean artifact; confirm in the slice preview.
- **`neck_floor`:** a separate print because it hangs 5 mm below the girdle's
  print plane. Its counterbores and countersinks open onto the bed; the audit
  finds nothing to support.
- **`shoulder_girdle_v6`**
  - It is 200 mm long, so **centre it on the bed**.
  - Supports go under the trapezius-web window tops (34 mm spans), the
    grip-plate rib-relief roofs, and the bay disc-relief tops.

**Totals:** 23 prints (21 PETG, 2 TPU) from 17 STLs with bearing C, before
supports and brims (option E adds 4 prints from 2 STLs):

- PETG: ≈ 787 g with bearing option C, ≈ 771 g with A, ≈ 784 g with E;
- TPU: 46 g.

**Not printed:**

- `head.stl`: the shell and face fused, used for mass, the assembly checks and
  the sim plant.
- `neck_collar.stl` and anything built with `ARMS=0` (see the top of this
  page).
- Whichever yaw carrier and pelvis the chosen bearing option does not name.

## Regenerating

```bash
.venv/bin/python cad/v6/parts_v6.py                    # every part -> cad/v6/stl + cad/v6/step, bed check, mass rollup
.venv/bin/python cad/v6/parts_v6.py --only foot_L foot_R
.venv/bin/python cad/v6/parts_v6.py --no-export        # the rollup alone
YAW_BEARING_VARIANT=E .venv/bin/python cad/v6/parts_v6.py --only pelvis_v7_optE yaw_carrier_v6_optE yaw_cap_optE yaw_retainer_optE
.venv/bin/python cad/v6/head.py                        # head_shell + head_face (+ the fused head), audits
```

- Each part module's `__main__` exports that part and runs its own audits.
  `leg_link_v6`, `hip_yoke_v6`, `shoulder_girdle_v6`, `neck_floor` and `arm_v6`
  rewrite their STL as they go.
- Run `sh cad/run_checks.sh` after any CAD change and before printing (see
  [README.md](README.md#the-gate)).

## Global print settings

**PETG for every structural part; TPU 95A for the soles.**

- **Base settings:** 0.4 mm nozzle, 0.2 mm layers, 3 perimeters (every wall
  ≥ 2.4 mm is then perimeter-only), 30–40 % infill.
- **The profile that printed the prototype's PETG foot cleanly:** nozzle
  255 °C, bed 85 °C, 0.2 mm layers, 35 % infill.
- **Bed prep:** the FlashForge glue is a release layer on the PEI plate (PETG
  bonds hard to bare PEI). Degrease the plate with IPA.
- **Fits:** drop-in fits are `FIT = 0.30` mm (`cad/dimensions.py`). PETG runs
  hotter and strings more than PLA. On the first parts, check the snug
  interfaces: the grip channels on a servo case, the foot cradle, the pads on
  the discs. If anything binds, tune the flow or print a size test.
- **Filament:** keep PETG and TPU dry.
- **TPU 95A soles:** they print flat, 2 mm thick. No tuned profile is recorded
  yet, so use the slicer's TPU preset and print slowly.
- **Bores:** small horizontal bores are teardropped toward each part's print-up
  direction, so their tops print without support. That is designed in;
  supports are not.

## Supports are the slicer's job

**Supports are never modelled into the STL.** Modelled fins put their anchor
tabs below the part, so the real first layer became disconnected stamps, and
the tabs fused into the part. The parts that need supports say so in the table
above, in their module's `SUPPORT_NOTE`, and in `check_printability.SUPPORTED`.

OrcaSlicer settings that work:

| setting | value |
|---|---|
| Enable support | on |
| **Support on build plate only** | **on**: otherwise every horizontal bore fills with support |
| Type | `normal(auto)`; `tree(auto)` also works and is gentler on round pads |
| Threshold angle | 30° |
| Top Z distance | **0.2 mm** (PETG welds to support at 0.1) |
| Support/object XY distance | 0.35 mm |
| Brim | outer only, **5 mm**, for tall narrow parts: `hip_yoke_v6`, and recommended for `leg_link_v6` |

- Put the supported parts on their own plate so the setting does not leak onto
  the others.
- Read each supported part's audit findings (the `**` lines) against the slice
  preview.

**The printability audits** use `cad/check_printability.py`'s engine, run by
`cad/v6/audit_*.py` and by each module's `__main__`. They check each STL in its
print orientation for bridges, ceilings, ledges, islands, first-layer contact
and thin walls. The rules:

- PETG bridges 8 mm.
- A bridge is judged across its short side only when both ends are anchored. A
  window cut through a wall is a beam over its long span.
- The knife-edge threshold is 3.0 mm².

## Slicing

Parts can be sliced with the OrcaSlicer CLI on **mira** (the GPU box),
headless, through `cad/slice.py`, or in the OrcaSlicer GUI.

```bash
python3 cad/slice.py foot --filament PETG               # -> cad/gcode/foot.gcode (gitignored)
python3 cad/slice.py --all --filament PETG
python3 cad/slice.py foot --nozzle 0.6 --layer 0.28 --filament PETG
```

- **Defaults** are a 0.4 mm nozzle, 0.20 mm layers and **PLA**, so pass
  `--filament PETG`. For each part it prints the time, grams, layers, bed
  footprint and centre, and flags a part that lands off the bed. The footprint
  excludes the start gcode's purge line.
- **It only sees `cad/stl`, the prototype's parts.**
  - `STL_DIR` is `cad/stl`.
  - The flatpak sandbox exposes only `cad/stl` (read-only),
    `cad/print_profiles` (read-only) and `cad/gcode`, so passing a
    `cad/v6/stl/...` path does not help.
  - Slicing the robot's parts this way needs `STL_DIR` and the sandbox widened
    to `cad/v6/stl` (open). Until then, slice them in the GUI.
- **How it is installed.** OrcaSlicer 2.4.2 is a user-scope Flathub flatpak,
  `com.orcaslicer.OrcaSlicer`. The upstream AppImage needs glibc 2.38; mira
  runs Pop!_OS 22.04 with 2.35. Flathub serves signed commits, whereas the
  GitHub release publishes no checksum.
- **Headless.** The slicer runs under `xvfb-run`, because no X display is
  available to the user on mira.
- **Sandbox.** Without it the slicer would see `/dev/ttyUSB0` (the live servo
  bus) and `~/.ssh`. So it gets no devices, no network and no home directory:

  ```bash
  flatpak override --user \
    --nodevice=all --unshare=network \
    --nofilesystem=home --nofilesystem=/media --nofilesystem=/run/media \
    --nofilesystem=/mnt --nofilesystem=xdg-run/gvfs \
    --system-no-talk-name=org.freedesktop.UDisks2 \
    --filesystem=<repo>/cad/stl:ro \
    --filesystem=<repo>/cad/print_profiles:ro \
    --filesystem=<repo>/cad/gcode \
    com.orcaslicer.OrcaSlicer
  ```

  Verify with `flatpak info --show-permissions com.orcaslicer.OrcaSlicer`.
- **Two upstream quirks the wrapper works around:**
  - The CLI's `--load-settings` does not resolve a preset's `inherits` chain
    from a file path. `cad/orca_profile.py` flattens the vendor chain into a
    standalone preset (the `from` key must survive). A thin user filament preset
    loaded by path otherwise falls back to PLA without a word.
  - Upstream's AD5M preset sets relative extrusion but has no `G92 E0` in its
    layer-change gcode, so upstream's own validator rejects it (return code
    −51). `slice.py` appends `G92 E0`.

### The printer

The printer is a **FlashForge Adventurer 5M Pro** at **`<printer-ip>`**, on
DHCP.

- **Fixed address:** if one is wanted, use a router DHCP reservation. Never set
  a static IP on the printer's panel: it ends up with two live addresses that
  flap.
- **LAN mode:** the working configuration has LAN mode off.
- **The bed is centre-origin, −110 … +110 mm in X and Y: centre every part on
  (0, 0).**
  - `slice.py` reports each part's measured centre.
  - A profile that emits corner-origin (0 … 220) coordinates puts the part in
    the +X/+Y corner, partly off the bed. If the centre is not (0, 0), shift
    X/Y by the part's own bounding-box centre before sending, and check the
    result stays inside ±110.
- **Upload over the LAN from OrcaSlicer.** Set these keys in the machine
  profile:
  - `host_type: flashforge`
  - `print_host: <printer-ip>` (bare IP; Orca adds port 8898 itself)
  - the printer's serial number, and its access code in `printhost_apikey`.
    Both are on the printer's panel under Settings → Network. **Keep them out
    of the repo.**
- **If an upload fails, check `host_type` before the network.** Without it,
  Orca falls back to plain HTTP on port 80, which this printer refuses.
- **Ports:**
  - 8898 is the printer's JSON API. `POST /detail` returns its status and its
    own `ipAddr`, which is the authoritative address.
  - 8899 is the raw `~M` command channel, not HTTP.
- **Sending from mira** is a separate host-side step, because the slicer's
  sandbox has no network.
