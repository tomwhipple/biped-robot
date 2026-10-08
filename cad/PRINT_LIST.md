# Print list

What to print for the robot, how to orient each part, what supports each one
needs, and how to slice and send it. The CAD is `cad/v6/`; its code map is
[README.md](README.md). What to buy is [docs/bom.md](../docs/bom.md), and how it
goes together is [docs/assembly.md](../docs/assembly.md).

**The code's default build is the robot to print**: arms on, the pelvis with
the girdle's deck pilots, 15 × STS3215 + 2 × STS3250 (the hip rolls) in the
mass line. The `ARMS=0` variant
(`neck_collar` for the neck, no girdle, no arm links, no pilots) exists for
the CAD checks and is **not a print target**: the collar's flange screws have
nothing to bite (their holes land in the Pi slide slot and the General Driver
lead slot, and there is no deck under the rest of the flange).

## Status

| can print now | wait |
|---|---|
| `foot_L`, `foot_R`, `ankle_link` ×2, `leg_link_v6` ×4, `hip_yoke_v6` ×2, `pelvis_v7`, `yaw_carrier_v6` ×2, `head_shell`, `head_face`, `neck_floor`, `shoulder_girdle_v6`, `arm_upper_v6_L/_R`, `arm_fore_v6_L/_R` | **`yaw_bearing_housing`**: its shoulder is sized to SKF's published ring dimensions; measure the generic bearings first ([docs/bom.md §5](../docs/bom.md)). |

**Fit check first (#79).** Before printing the full set, print one
`leg_link_v6` and one `hip_yoke_v6`. Fit them to a real servo, and check:

- the grip channel slides on;
- the disc screws engage;
- the supported horn-seating face sits flat on the disc.

## The parts

**How the masses are computed:** build123d volume × PETG 1.27 g/cm³ × 0.90
print factor (`parts_v6.mass_g`); the cut silicone soles are solid sheet,
estimated at 1.21 g/cm³ (no print factor) until weighed. They
are the rollup's, `docs/design-v6/parts_v6_rollup.txt`
(`cad/v6/parts_v6.py --no-export`).

**Bounding boxes** are in the model frame (mm). **Supports** are what to tell
the slicer; see [Supports](#supports-are-the-slicers-job).

| part | qty | g each | bbox | orientation on the bed | supports | notes |
|---|---|---|---|---|---|---|
| `pelvis_v7` | 1 | 173.8 | 99 × 124 × 72 | deck top down | **yes:** the two yaw-cell ceilings; the skin-window roofs near the housing floor | one print, the whole torso, with the girdle's ten deck pilots and the bearing housing's five pilot bosses; each yaw cell's ceiling has a hole for the servo's free-hub post and a relief for its back-cover platform |
| `yaw_bearing_housing` | 1 | 24.5 | 80 × 155 × 9.3 | the face that goes against the pelvis down | no | both hips' outer-race seats (Ø64.96 recess, −0.04 mm on the 6810-2RS, under a Ø62.6 shoulder), joined by a bridge, with a rear bar; five M2.5 × 8 flat-heads up into the pelvis |
| `yaw_carrier_v6` | 2 | 25.8 | 50 × 50 × 49 | horn-plate face down, bay walls rising | **yes:** the rear-wall connector window's ceiling, a 1 mm bar spanning 22.8 mm | carries the hip-roll servo in its bay, 5 mm below the horn plate; its round hub (Ø50.08, +0.08 mm) takes the bearing's inner race |
| `hip_yoke_v6` | 2 | 25.4 | 74 × 48 × 44 | on edge, model −Y on the bed, so the roll arms print as walls | **yes + 5 mm brim** | roll and pitch clevis in one print; see its notes below |
| `leg_link_v6` | 4 | 28.6 | 117 × 44 × 28 | on its back: web face on the bed | **yes**, build plate only: under the two round pad ends | thigh and shin are the same part |
| `ankle_link` | 2 | 13.6 | 65 × 43 × 38 | on its back (model +X up): the rear tine on the bed | **yes**: under the web (4.6 mm, from the plate) and under the front tine below the floor, standing on the rear tine's inner face, which seats the idler disc: clean it flat. The front wall bridges between the side walls | grips the ankle-pitch servo in a box around its base, forks onto the ankle-roll servo |
| `foot_L`, `foot_R` | 1 + 1 | 54.4 | 130 × 84 × 30 | sole down | no | a mirrored pair; each retention tab is braced by two fins on its outer face, sloped tops, support-free; a short near-row screw boss beside each disc |
| `head_shell` | 1 | 26.1 | 62 × 60 × 50 | base (horn plate) down; the dome is ≥ 45° | none expected | from `head.py` |
| `head_face` | 1 | 8.6 | 62 × 56 × 6 | flat, camera bosses up | no | |
| `neck_floor` | 1 | 4.0 | 46 × 45 × 5 | flat, bottom face down, pads and lugs up | no | the neck servo's seat (#90): screwed up into the neck tube's four bosses; the stator screws go up through it |
| `shoulder_girdle_v6` | 1 | 77.1 | 200 × 58 × 33 | base down | **yes**, build plate only | carries both shoulder servos and the neck tube (with the four bosses for `neck_floor`) |
| `arm_upper_v6_L`, `_R` | 1 + 1 | 27.8 | 169 × 38 × 29 | on its back, web face down | no | a mirror pair; open-front U with two end walls; grips the elbow servo |
| `arm_fore_v6_L`, `_R` | 1 + 1 | 41.5 | 182 × 44 × 27 | on its back, web face down | **yes**, build plate only: the two elbow-pad undersides start 5 mm off the bed | a mirror pair; the elbow fork, an end wall, 12 mm hand knuckle |

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
- **`leg_link_v6`:** the back web and both side walls sit on the bed. Only the
  lower-rear quarter of each round pad end overhangs, 5 mm up; support it from
  the plate. The knee pocket in the idler side wall prints with a 6.6 mm roof.
- **`head_shell`:** the printability audit flags a ceiling at the dome seam. It
  is believed to be a boolean artifact; confirm in the slice preview.
- **`neck_floor`:** a separate print because it hangs 5 mm below the girdle's
  print plane. Its counterbores and countersinks open onto the bed; the audit
  finds nothing to support.
- **`shoulder_girdle_v6`**
  - It is 200 mm long, so **centre it on the bed**.
  - Supports go under the trapezius-web window tops (34 mm spans), the
    grip-plate rib-relief roofs, and the bay disc-relief tops.

**Totals:** 22 prints, all PETG, from 16 STLs: ≈ 797 g before supports and
brims.

**Not printed:**

- `sole_tpu_L/R`: the soles are cut from 2 mm silicone rubber sheet
  ([docs/bom.md §7](../docs/bom.md)), not printed. Each is the foot plate's
  outline: 130 × 84 mm with 14 mm corner radii, the same for left and right.
  The STLs are that outline.
- `head.stl`: the shell and face fused, used for mass, the assembly checks and
  the sim plant.
- `neck_collar.stl` and anything built with `ARMS=0` (see the top of this
  page).

## Regenerating

```bash
.venv/bin/python cad/v6/parts_v6.py                    # every part -> cad/v6/stl + cad/v6/step, bed check, mass rollup
.venv/bin/python cad/v6/parts_v6.py --only foot_L foot_R
.venv/bin/python cad/v6/parts_v6.py --no-export        # the rollup alone
.venv/bin/python cad/v6/head.py                        # head_shell + head_face (+ the fused head), audits
```

- Each part module's `__main__` exports that part and runs its own audits.
  `leg_link_v6`, `hip_yoke_v6`, `shoulder_girdle_v6`, `neck_floor` and `arm_v6`
  rewrite their STL as they go.
- Run `sh cad/run_checks.sh` after any CAD change and before printing (see
  [README.md](README.md#the-gate)).

## Global print settings

**PETG for every part.**

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
- **Filament:** keep PETG dry.
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
| Brim | outer only, **5 mm**, for tall narrow parts: `hip_yoke_v6` |

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

`cad/slice.py` slices a part, or a **plate** of parts printed together, into an
**OrcaSlicer project**, each part in its print orientation with the settings
above. Open the project in the GUI, check it, and print it from there.

```bash
python3 cad/slice.py leg_link_v6             # -> cad/gcode/leg_link_v6.3mf + .gcode (gitignored)
python3 cad/slice.py leg_link_v6 --copies 4  # four on the plate, this run only
python3 cad/slice.py feet                    # both feet, one plate -> cad/gcode/feet.3mf
python3 cad/slice.py --all                   # every plate, and every part on no plate
python3 cad/slice.py --harvest               # only read saved projects back
```

- **Open `cad/gcode/<part or plate>.3mf`** with File → Open Project.
  - The printer is the GUI's own printer preset, the one that holds the print
    host, so the plate prints from the GUI.
  - The process and filament are Flashforge's "0.20mm Standard" and "Generic
    PETG", marked as modified. The modified values are this file's spec.
  - The plate is already sliced: check the preview, then print it.
- **The settings live in `cad/v6/print_settings.json`.** They are layered:
  1. Flashforge's presets.
  2. The spec on top, from Global print settings and Supports above.
  3. One entry per part, with:
     - the orientation: `IDENT`, `RX180`, `RY_XUP`, `RY_XDOWN`, `RY_ROLL_WALL`,
       or a 3×3 matrix whose rows are the print axes in model coordinates;
     - supports, and copies;
     - overrides, in OrcaSlicer's own keys.
  4. Plates: parts printed together. **`feet`** is `foot_L` + `foot_R`. One
     process slices a plate, so its parts must agree on supports and overrides
     (`slice.py` refuses a plate whose parts don't). A plate's copies are its
     own: `--copies 2` puts two of each foot on it.
- **Changes made in the GUI carry forward.** Save the project (Cmd+S). The next
  `slice.py` run on that part or plate reads the saved project's changes back
  into `print_settings.json`:
  - any process or filament setting, including per-object settings (on a
    plate, for every part on it);
  - a part tipped onto another face (on a plate, that part only);
  - the number of copies.

  It prints what it changed, and keeps the saved project under
  `cad/gcode/.work/saved/`. Commit the JSON.
  - **Printer settings are not carried.** Change those in the GUI's printer
    preset.
  - **Painted supports or seams, modifier volumes and height-range modifiers
    cannot be carried.** `slice.py` then leaves that project alone and says so.
- **For each part it prints** the time, grams, height, supports, bed footprint
  and centre, and flags a part that lands off the bed. The footprint excludes the
  start gcode's purge line.
- **Where it runs.** On the Mac it drives `/Applications/OrcaSlicer.app`
  (`ORCA_APP` overrides the path). Elsewhere it runs the Flathub flatpak
  headless, as on mira. How it works and the upstream quirks it works around are
  in [docs/slicing.md](../docs/slicing.md).
- **One part departs from the table above** in `print_settings.json`:
  **`head_face`** is built in the assembly frame. Printed flat, outer face down
  and bosses up, its outer-face pocket becomes a ceiling of about 50 × 44 mm,
  1.8 mm above the bed. It is sliced with supports. Open: whether to keep the
  pocket and the supports.

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
