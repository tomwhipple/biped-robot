# CAD

The robot's parametric CAD, written in Python with
[build123d](https://build123d.readthedocs.io/). It lives in **`cad/v6/`**.

- **Dimensions:** every dimension comes from `cad/v6/dimensions_v6.py`. That file
  takes every servo-interface number from `cad/dimensions.py`, which is measured
  from the servo vendor's STEP model.
- **Related docs:** the design is [DESIGN.md §5](../DESIGN.md); what to print and
  how is [PRINT_LIST.md](PRINT_LIST.md); assembly is
  [docs/assembly.md](../docs/assembly.md).

```
cad/
├── v6/                     the robot: part modules, assembly, gates (map below)
│   ├── stl/  step/         exported parts and assemblies (generated)
│   └── renders/            part renders, assembly views, fly-in filmstrips
├── dimensions.py           servo ground truth and shared constants
├── parts.py                shared solid helpers; the prototype's part builders
├── check_assembly.py       servo mocks (used by cad/v6); the prototype's interference check
├── check_printability.py   the printability audit engine
├── fasteners.py            screws as solids (prototype); the hip interfaces' screw sets
├── run_checks.sh           the CAD gate
├── slice.py, orca_profile.py        OrcaSlicer CLI wrapper (PRINT_LIST.md, Slicing)
├── render_part.py, render_assembly.py, animate_assembly.py   renders (below)
├── vendor/ST3215.step      the servo's vendor STEP
└── stl/  step/  renders/   the prototype's parts and renders
```

The prototype's part builders in `parts.py`, and its `cad/stl` and `cad/step`,
exist because the 10-joint training plant (`sim/bimo_biped_v5body.xml`) meshes
`cad/stl`.

## `cad/v6` module map

| module | what it is |
|---|---|
| `dimensions_v6.py` | every dimension the robot adds or changes: kinematics, feet, torso, pack, Pi, neck and head, the hip-yaw bearing (`YAW_BRG_*`, a 6810-2RS), arms and girdle. It re-exports `cad/dimensions.py` as `D`, and `SIM_EXPECT` is the set of numbers the plant must share (pinned by `tests/test_v6_design_gates.py`) |
| `parts_v6.py` | exports every printed part to `stl/` and `step/`, with a bed check and the mass rollup `docs/design-v6/parts_v6_rollup.txt` (`--only <names>`, `--no-export` for the rollup alone) |
| `assembly_v6.py` | the articulated robot: one kinematic chain of parts, servo mocks and electronics mocks, posable (`--pose knee=-60,...`). Writes `step/assembly_v6_arms.step` (`assembly_v6.step` with `ARMS=0`) and `renders/assembly_v6_arms.png`. The checker, the fly-in and the step figures read this chain |
| `check_assembly_v6.py` | the CAD gate: every pair of pieces that move relative to each other, at the ROM extremes plus interior samples; the relatively fixed pairs that must keep their distance (`--joint static`); every disc screw's thread engagement (`--joint screws`). Options: `--joint`, `--samples`, `--side` |
| `animate_v6.py` | fly-in along each part's insertion path. Writes `renders/assembly_v6_flyin*.mp4` (gitignored) and a committed `_strip.png` |
| `render_steps_v6.py` | the assembly guide's step figures, `docs/assembly/v6_*.png`: bench order, the incoming part orange along its insertion path |
| `pelvis_v7.py` | the torso, one print: `pelvis_v7(arm_mounts)` with the hip-yaw bearings' outer-race seats, its `SCREWS()`, a driver-access check and the board slide checks |
| `yaw_carrier_v6.py` | the prototype's carrier with a round hub for the 6810-2RS's inner race, and its `SCREWS()` |
| `hip_yoke_v6.py` | the hip roll and pitch clevises fused into one print |
| `yoke_pitch_v6.py` | the pitch clevis alone. `hip_yoke_v6` is built from it; it is printed only under `HIP_YOKE_VARIANT=split` |
| `leg_link_v6.py` | thigh and shin (one part): a 110 mm box section |
| `ankle_link.py` | grips the ankle-pitch servo and forks onto the ankle-roll servo |
| `foot_v6.py` | `foot_L`/`foot_R` and the TPU soles `sole_tpu_L`/`_R` |
| `head.py` | `head_shell`, `head_face`, and `head` (the two fused, for checks) |
| `neck_floor.py` | the neck servo's seat: a plate screwed up into the neck tube's bosses, in the deck's battery aperture (#90); its checks and `tube_bosses()` |
| `neck_collar.py` | the `ARMS=0` variant's neck tube (not a print target: its flange pilots miss the deck) |
| `shoulder_girdle_v6.py` | the girdle: both shoulder pods and the neck tube in one print |
| `arm_v6.py` | upper arm and forearm (mirror pairs), the elbow-servo mock, and the elbow ROM check |
| `audit_torso.py` | printability report for `pelvis_v7`, the head parts and `neck_collar` |
| `audit_ankle_foot.py` | printability report for `ankle_link` and `foot_L/R` |
| `audit_leg_link.py` | printability and interface audit for `leg_link_v6` |
| `check_hip_yoke_clearance.py` | clearance A/B between the hip yoke's raw union and the styled part |

## Shared builders in `cad/`

- **`dimensions.py`** is the single source of servo truth. It holds:
  - the case, the horn and idler discs and their tapped flanges, the screw rows,
    the rib and platform detents, the idler face, the connector trench and the
    countersinks;
  - the walls and fits (`FIT = 0.30`), the filament density and print factor,
    and the bed size.

  Every `cad/v6` module takes these from here and never retypes them.
- **`parts.py`** provides two things:
  - the solid helpers every `cad/v6` module uses: `box`, `cyl_x/y/z`, `teardrop_x/y`,
    `wedge_x/y/z`, `csk_x/y/z`, `bcd_x`;
  - the prototype parts the robot reuses: `yaw_carrier` (inside `yaw_carrier_v6`), `yoke_roll` and `yoke_pitch` (inside `hip_yoke_v6`), and
    `leg_link` (the basis of `leg_link_v6`).

  Its `main()` exports the prototype's parts to `cad/stl` and `cad/step`.
- **`check_assembly.py`** provides `servo_mock()`: the STS3215 in one canonical
  frame, built from the vendor STEP. `servo_mock_x/_y/_z` are rotations of it,
  so the three orientations cannot drift apart. It also has the board mocks.
  The `cad/v6` assembly and part modules use these. Its `main()` is the prototype's
  own interference check, and it holds the disc-screw engagement rule
  ([docs/assembly.md §1](../docs/assembly.md#1-the-joint-the-servo-is-the-axle)).
- **`check_printability.py`** is the audit engine.
  - It rotates each STL into its print orientation and classifies down-facing
    facets as CEILING, LEDGE, ISLAND, BEAM, first-layer CONTACT or thin wall.
  - Parts listed in `SUPPORTED` are expected to need slicer supports, so their
    overhang findings are reported as support needs, not failures.
  - The v6 audits point its `STL` directory and `ORIENT` table at `cad/v6` at
    runtime.
- **`fasteners.py`** models the prototype's screws as solids. The robot's fastener
  counts take the hip interfaces from it:
  - `disc_screws_x` / `disc_screws_y`, for the hip yoke's roll and pitch
    clevises;
  - `yaw_horn_screws` + `yaw_wall_screws`, for the carrier.

## Regenerate

```bash
.venv/bin/python cad/v6/parts_v6.py                      # the robot: every printed part -> cad/v6/stl + cad/v6/step, bed check, rollup
.venv/bin/python cad/v6/parts_v6.py --no-export          # the rollup alone
.venv/bin/python cad/v6/head.py                          # head_shell, head_face (+ the fused head), audits, render
.venv/bin/python cad/v6/assembly_v6.py                   # step/assembly_v6_arms.step + renders/assembly_v6_arms.png
.venv/bin/python cad/v6/assembly_v6.py --pose hip_pitch=-60,knee=90 --step /tmp/posed.step --png /tmp/posed.png
.venv/bin/python cad/v6/animate_v6.py                    # fly-in mp4 (gitignored) + filmstrip
.venv/bin/python cad/v6/render_steps_v6.py               # docs/assembly/v6_*.png
.venv/bin/python cad/v6/<module>.py                      # one part: STL + STEP + its own audits + render
.venv/bin/python sim/build_v6_inertia.py --write         # the plant from the exported STLs (sim/bimo_biped_v6ar.xml)
.venv/bin/python cad/parts.py                            # the prototype's parts -> cad/stl, cad/step (the training plant meshes them)
```

- **Each part module's `__main__`** exports that part and runs its own audits.
- **Renders on the Mac need `MUJOCO_GL=cgl`.** The scripts default it to `egl`
  (Linux), and on macOS they then skip the render.
- **The exports are generated files:** regenerate them, never edit them. See
  [AGENTS.md](../AGENTS.md). The `ARMS=0` pelvis exports as
  `pelvis_v7_armless`, so it never overwrites the robot's `pelvis_v7`.

**Environment flags** (the defaults are the robot to print):

| flag | default | read by | effect |
|---|---|---|---|
| `ARMS` | `1` | `parts_v6.py`, `assembly_v6.py` (and so `check_assembly_v6.py`, `animate_v6.py`, `render_steps_v6.py`) | the girdle, the four arm links, and the pelvis with the ten girdle pilots. `0` is the armless variant: `neck_collar`, no pilots, no arms |
| `HIP_YOKE_VARIANT` | `single` | `parts_v6.py`, `assembly_v6.py` | `split` builds the bolted `yoke_roll` + `yoke_pitch_v6` pair instead of `hip_yoke_v6` |
| `SERVO_PLAN` | `hips` | `parts_v6.py` (`sim/build_v6_inertia.py --servo-plan`) | `hips`: STS3250s at the two hip rolls, STS3215s elsewhere; `B`: 17 × STS3215; `3250`: STS3250s at the six roll and knee joints (masses only; same case) |
| `MUJOCO_GL` | `egl` (set by default) | every render | set `cgl` on macOS |

## The gate

```bash
sh cad/run_checks.sh          # exit 0 = clear; "CHECKS FAILED -- do not print" otherwise
```

It runs four things with `.venv/bin/python`:

1. **`check_assembly_v6.py` on the default build (the robot, with arms).**
   Every relatively-moving pair is posed at its joint's ROM extremes and
   interior samples. The hip-yaw bearing is in the chain as its two races,
   split at the estimated ball gap: the turning side must clear the fixed
   race through the yaw sweep, and the leg must clear both.
   - Each pair must show zero intersection and at least `SWEEP_BUFFER` of
     clearance. Designed contacts (pads on discs, seats) are checked for
     volume only, by name.
   - The relatively fixed pairs (the neck floor over the pack, off the pelvis
     and the boards) are checked at the standing pose, each against its own
     minimum distance. The bearing's two press fits must overlap by exactly
     their designed interference volume.
   - Every disc screw's thread engagement is checked against its flange.
   - The pair list is printed with the result, because a pair that is not listed
     was never checked.
   - Results are recorded in `docs/design-v6/cad_rom_check.txt`.
2. **The same check with `ARMS=0`** (the armless variant).
3. **`audit_torso.py`**, a printability report on `cad/v6/stl`.
4. **`audit_ankle_foot.py`**, likewise.

The two printability reports do not fail the run: `pelvis_v7` and `ankle_link`
carry ceilings and islands by design, and supports are the slicer's job. Read
their `**` lines against the slice preview.

**What the gate does not cover:**

- **The other printed parts** audit themselves from their module's `__main__`
  (`leg_link_v6`, `hip_yoke_v6`, `shoulder_girdle_v6`, `arm_v6`). Those rewrite
  their STL as they run, so run them when that part changes.
- **Insertion paths:** on a layout change, `animate_v6.py` proves that the parts
  fly in along them.

`run_checks.sh` is not in the pre-push hook. Run it for any CAD change
([AGENTS.md](../AGENTS.md)).

## Viewing in FreeCAD

Open the STEP exports:

- **the robot:** `cad/v6/step/assembly_v6_arms.step` (`assembly_v6.step` is the
  armless variant);
- **each part:** `cad/v6/step/<part>.step`.

A posed robot is `assembly_v6.py --pose ... --step <file>`. Use the CAD, not
the PNGs, to judge a design: the renders are for documentation.

**Optional: the FreeCAD MCP addon** lets an agent drive a running FreeCAD.

- It is installed in FreeCAD's `v1-1/Mod` and registered user-scope with
  `uvx --with 'mcp[cli]<2' freecad-mcp`.
- It needs the FreeCAD GUI running; the addon's RPC listens on port 9875.

## Servo geometry: the ground truth

**The sources:**

- `cad/vendor/ST3215.step`, Waveshare's STEP model;
- the dimensioned drawing and the Feetech datasheet in
  `docs/datasheets/st3215/`.

`cad/dimensions.py` measures from these programmatically; nothing comes from
photos. The STS3235 and STS3250 share this case, horn and disc pattern, so a
servo swap changes no geometry.

| feature | value |
|---|---|
| case | 45.22 × 24.72 mm, 34.7 across the output axis (35 with the boss); output axis 10.11 from the output end |
| discs | metal horn and free-spinning idler, both Ø19.2 with 4 × M3 on Ø14, tapped through a 2.5 mm (horn) and 2.1 mm (idler) flange |
| case holes | M2.5 at rows 8.30 / 29.00 behind the axis (horn face) and 8.30 / 32.75 (idler face), ±10.25 across |
| idler face | the disc and its hub rotate and stand proud; the stator screw bosses sit 1.78 below the cover slab (mounts land on pads); a horn-side rib sits on the cable half |
| **connector trench, on the idler face** | the two bus ports side by side across the width, in a trench 11.75–16.35 mm from the axis toward the cable end, ±10.9 wide, floor 4.78 below the slab (`SV_CONN_*`) |

**Lead openings belong over the connector trench.** The ports are not on the
cable-end face.

## Renders of the prototype plant

These three scripts render the prototype's parts in `cad/stl`. The training
report (`sim/build_report.py`) reads `cad/renders/assembly_mujoco.png` and
`assembly_flyin.gif`.

| script | output |
|---|---|
| `render_part.py <name or path.stl>` | one STL as a strip of views: `cad/renders/<name>.png`. It takes any STL path, so it works on `cad/v6/stl` parts too |
| `render_assembly.py` | `cad/renders/assembly_mujoco.png` |
| `animate_assembly.py` | `cad/renders/assembly_flyin.gif` and `.mov`, both gitignored |
