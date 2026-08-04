# Slicing on mira (OrcaSlicer CLI)

OrcaSlicer 2.4.2 is installed on **mira** as a user-scope flatpak, for headless
slicing of `cad/stl/*.stl` for the **FlashForge Adventurer 5M Pro**.

```bash
python3 cad/slice.py foot                  # -> cad/gcode/foot.gcode
python3 cad/slice.py foot tower yoke_roll
python3 cad/slice.py --all
python3 cad/slice.py foot --nozzle 0.6 --layer 0.28 --filament PETG
```

Output goes to `cad/gcode/` (gitignored — regenerate, don't commit). Each part
prints its estimated time, filament mass, layer count, bed footprint, and centre
position, and is flagged if it lands off the bed.

## Why it is a flatpak and not the AppImage

Upstream's Linux AppImage is built on Ubuntu 24.04 and needs `GLIBC_2.38` +
`GLIBCXX_3.4.32`. mira is Pop!_OS 22.04 with `2.35` / `3.4.30`, so the AppImage
cannot run and no 22.04 AppImage is published for any release. The flatpak ships
its own `org.gnome.Platform/50` runtime, so the host glibc is irrelevant.

It is installed **from Flathub rather than the GitHub release bundle** because
Flathub serves GPG-signed OSTree commits; the GitHub release publishes no
checksum or signature of any kind. Same app ID, same version 2.4.2.

## Sandbox

The flatpak ships with `devices=all`, `filesystems=home` and `shared=network`.
On this machine that would expose `/dev/ttyUSB0` (the live servo bus),
`~/.config/gh/hosts.yml`, `~/.hermes/`, and `~/.ssh/`. It is locked down to what
a slicer actually needs:

```bash
flatpak override --user \
  --nodevice=all --unshare=network \
  --nofilesystem=home --nofilesystem=/media --nofilesystem=/run/media \
  --nofilesystem=/mnt --nofilesystem=xdg-run/gvfs \
  --system-no-talk-name=org.freedesktop.UDisks2 \
  --filesystem=/home/claw/code/robot/cad/stl:ro \
  --filesystem=/home/claw/code/robot/cad/print_profiles:ro \
  --filesystem=/home/claw/code/robot/cad/gcode \
  com.orcaslicer.OrcaSlicer
```

Verify with `flatpak info --show-permissions com.orcaslicer.OrcaSlicer`.

Slicing needs **no device access at all** — tested, works with `--nodevice=all`,
so not even `--device=dri` is granted. Network is off: slicing is pure geometry,
and pushing gcode to the printer at `192.168.2.116` is a separate host-side step
(see [flashforge notes](printer-order.md)), so the slicer needs no egress.

Reviewed by Percy under kanban `t_19e6d082` (approved with conditions; all
conditions applied).

## Two upstream quirks the wrapper works around

**1. `inherits` is not resolved from a file path.** The CLI's `--load-settings`
loads exactly the keys in the file you hand it and fills the rest from built-in
defaults. `Flashforge Adventurer 5M Pro 0.4 Nozzle.json` holds only 14 keys and
inherits the other ~55 through
`fdm_adventurer5m_common -> fdm_flashforge_common -> fdm_machine_common`.
`cad/orca_profile.py` walks that chain and writes a flattened preset.
Note `from` must survive flattening — without it the loader fails with
`from unsupported`.

**2. Upstream's own 5M Pro preset fails upstream's own validator.**
`fdm_adventurer5m_common` sets `use_relative_e_distances = 1` but ships a
`layer_change_gcode` with no `G92 E0`, so slicing aborts with return code `-51`
("Relative extruder addressing requires resetting the extruder position at each
layer"). `cad/slice.py` appends `G92 E0` — exactly what the validator asks for,
and a no-op for positioning under the `M83` relative extrusion the start gcode
already sets.

## Headless

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
