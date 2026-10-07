# Slicing with the OrcaSlicer CLI

`cad/slice.py` turns a v6 part into an OrcaSlicer project for the **FlashForge
Adventurer 5M Pro**: the part in its print orientation, the print spec's
settings, the plate already sliced. The workflow (open, check, print, save) is
in [cad/PRINT_LIST.md](../cad/PRINT_LIST.md), Slicing; this page is how it
works.

```bash
python3 cad/slice.py leg_link_v6       # -> cad/gcode/leg_link_v6.3mf + leg_link_v6.gcode
python3 cad/slice.py --all
python3 cad/slice.py --harvest
```

Output goes to `cad/gcode/` (gitignored: regenerate, don't commit). Its
`.work/` holds the staged STLs and presets, OrcaSlicer's data dirs, each
project's state, and copies of projects saved from the GUI (`.work/saved/`).
Parts slice in parallel (`--jobs`, default half the cores).

## What one slice does

1. **Orient.** The part's STL from `cad/v6/stl` is rotated into its print
   orientation (`cad/v6/print_settings.json`) and written to `.work/<part>/`.
2. **Build the presets.** `cad/orca_profile.py` flattens three presets by name,
   from the GUI's user presets first, then the bundled Flashforge profiles:
   - the printer: the GUI's own printer preset if exactly one inherits the
     spec's printer and has a print host, else the system preset (`--printer`
     picks one);
   - the process and the filament: Flashforge's system presets.

   The spec and the part's overrides go on top. The print host's access code is
   dropped from what the CLI sees.
3. **Slice and export.** The CLI slices and writes the project
   (`--export-3mf`) and the plate's gcode.
4. **Fill in `different_settings_to_system`** in the project's
   `Metadata/project_settings.config` (below).
5. **Record the project's state**: its sha256, the orientation and instance
   rotation it was generated with, its copies, and every process and filament
   value. The next run compares a saved project against this.

## Upstream quirks it works around

**1. `inherits` is not resolved from a file path.** The CLI's `--load-settings`
loads exactly the keys in the file you hand it and fills the rest from built-in
defaults. A thin filament preset silently becomes PLA. `orca_profile.Library`
walks the chain within the Flashforge vendor directory and the user's presets
and hands the CLI flat presets. Two details:
- `from` must survive flattening, or the loader fails with `from unsupported`.
- A flattened GUI printer preset keeps `inherits` pointing at its system
  parent, because the CLI checks process compatibility through it. Without it
  the CLI rejects the process (exit 239).

**2. The CLI's normative check rejects Flashforge's own printer preset.**
`fdm_adventurer5m_common` sets relative extrusion but its `layer_change_gcode`
has no `G92 E0`, and the CLI's check aborts on that. The GUI slices the same
preset without complaint. `slice.py` passes `--normative-check=0`
(`--normative-check 0` with a space reads the `0` as a model file). So the
project's printer settings are exactly the GUI preset's.

**3. The GUI resets a project's settings unless they are listed.** On opening a
project, OrcaSlicer resets every value that differs from the selected system
preset to the system value, except keys listed in `different_settings_to_system`
(`Preset.cpp`, `load_external_preset`; `PrintConfig.cpp`,
`update_non_diff_values_to_base_config`). The CLI writes that list empty, so an
unpatched project opens with Flashforge's 2 walls and 15 % infill and shows no
change.

`slice.py` writes the list the way the GUI does when it saves a project:
- one string per preset, in the order print, filament, printer;
- each string the keys joined by `;`;
- each list holding every key that differs from the system preset, plus every
  key the spec, the part or the GUI printer preset sets. Over-listing is
  harmless; a missing key is silently reset.

## Reading a saved project back

A project whose sha256 no longer matches its state was saved from the GUI.
Before slicing that part again, `slice.py` copies the project to
`.work/saved/` and folds what changed into `print_settings.json`:

- **Settings.** Every process and filament value in
  `project_settings.config` that differs from the generated one, and every
  per-object or per-part `<metadata>` in `Metadata/model_settings.config`,
  becomes a part override. If the value equals the spec's, any override is
  dropped instead. Printer keys are reported and ignored.
- **Orientation.** The project's mesh is the part already oriented, so the
  instance rotation (`3D/3dmodel.model`: item transform × component transform)
  acts on top of the orientation it was generated with. The new orientation is
  that rotation times the old one.
  - Only a change in the rotation's bottom row, the up direction, counts. A
    spin on the bed does not.
  - 3MF transforms are row-vector matrices, `m00 m01 m02 m10 … m32`, so the
    rotation as `print = R · model` is the transpose.
- **Copies.** The number of build items.

Painted supports, seams or colour, modifier and support-blocker volumes,
height-range modifiers, and objects other than the part cannot be written back.
For those, `slice.py` leaves the project in place, says why, and does not
re-slice that part. A project it cannot read is treated the same way.

## mira: the flatpak

OrcaSlicer 2.4.2 is installed on **mira** as a user-scope flatpak, for headless
slicing.

### Why it is a flatpak and not the AppImage

Upstream's Linux AppImage is built on Ubuntu 24.04 and needs `GLIBC_2.38` +
`GLIBCXX_3.4.32`. mira is Pop!_OS 22.04 with `2.35` / `3.4.30`, so the AppImage
cannot run and no 22.04 AppImage is published for any release. The flatpak ships
its own `org.gnome.Platform/50` runtime, so the host glibc is irrelevant.

It is installed **from Flathub rather than the GitHub release bundle** because
Flathub serves GPG-signed OSTree commits; the GitHub release publishes no
checksum or signature of any kind. Same app ID, same version 2.4.2.

### Sandbox

The flatpak ships with `devices=all`, `filesystems=home` and `shared=network`.
On mira that would expose `/dev/ttyUSB0` (the live servo bus),
`~/.config/gh/hosts.yml`, `~/.hermes/`, and `~/.ssh/`. It is locked down to what
a slicer actually needs. `slice.py` stages every file the slicer reads in
`cad/gcode/.work`, so `cad/gcode` is the only directory it needs:

```bash
flatpak override --user \
  --nodevice=all --unshare=network \
  --nofilesystem=home --nofilesystem=/media --nofilesystem=/run/media \
  --nofilesystem=/mnt --nofilesystem=xdg-run/gvfs \
  --system-no-talk-name=org.freedesktop.UDisks2 \
  --filesystem=$HOME/code/robot/cad/gcode \
  com.orcaslicer.OrcaSlicer
```

Verify with `flatpak info --show-permissions com.orcaslicer.OrcaSlicer`.

Slicing needs **no device access at all** — tested, works with `--nodevice=all`,
so not even `--device=dri` is granted. Network is off: slicing is pure geometry,
and pushing gcode to the printer at `<printer-ip>` is a separate host-side step
(see [flashforge notes](printer-order.md)), so the slicer needs no egress.

Reviewed by Percy under kanban `t_19e6d082` (approved with conditions; all
conditions applied).

### Headless

OrcaSlicer is GUI-first and has no headless flag, so `cad/slice.py` runs it under
`xvfb-run`. There is no usable X display for the `claw` user on mira (the running
Xorg belongs to gdm), which is also why the FreeCAD GUI cannot be opened here.

## Bed geometry

The 5M Pro bed is **center-origin**, `printable_area` `-110..110` in X and Y.
Parts sliced through a real machine profile are placed centred on `(0, 0)`
automatically, which satisfies the centring rule that the direct-to-printer
gcode path has to handle manually. `cad/slice.py` reports the measured centre so
this stays visible.

Measured footprints exclude the start gcode's purge line (X55 -> X-55 at the
front edge), which is wrapped in `;TYPE:Custom`; counting it reports a flat
110 mm width for every part.
