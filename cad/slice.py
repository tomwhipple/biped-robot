#!/usr/bin/env python3
"""Slice printable parts to gcode for the FlashForge Adventurer 5M Pro.

Wraps the OrcaSlicer 2.4.2 flatpak CLI. OrcaSlicer is GUI-first, so it needs a
display even to slice headlessly -- hence Xvfb. It also can't resolve a vendor
preset's ``inherits`` chain from a file path, so presets are flattened first
(see cad/orca_profile.py).

    python3 cad/slice.py foot
    python3 cad/slice.py foot tower --layer 0.12
    python3 cad/slice.py --all

Gcode lands in cad/gcode/<part>.gcode. That directory is gitignored -- gcode is
a build artifact, regenerate it rather than committing it.
"""

import argparse
import glob
import json
import os
import re
import subprocess
import sys

import orca_profile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STL_DIR = os.path.join(REPO, "cad", "stl")
GCODE_DIR = os.path.join(REPO, "cad", "gcode")
PROFILE_DIR = os.path.join(REPO, "cad", "print_profiles")

FLATPAK_ID = "com.orcaslicer.OrcaSlicer"
VENDOR = os.path.join(orca_profile.FLATPAK_PROFILES, "Flashforge")

# The sandbox is locked down (no network, no devices, only cad/stl,
# cad/gcode and cad/print_profiles are visible) -- see cad/PRINT_LIST.md, Slicing.
SANDBOX_ENV = {
    "XDG_RUNTIME_DIR": f"/run/user/{os.getuid()}",
    "DBUS_SESSION_BUS_ADDRESS": f"unix:path=/run/user/{os.getuid()}/bus",
}


def build_profiles(nozzle, layer, filament, quiet=False):
    """Flatten the three vendor presets we need into PROFILE_DIR."""
    wanted = {
        "machine": f"machine/Flashforge Adventurer 5M Pro {nozzle} Nozzle.json",
        "process": f"process/{layer}mm Standard @Flashforge AD5M Pro {nozzle} Nozzle.json",
        "filament": f"filament/Flashforge Generic {filament}.json",
    }
    out = {}
    for kind, rel in wanted.items():
        src = os.path.join(VENDOR, rel)
        if not os.path.exists(src):
            available = sorted(
                os.path.basename(p)
                for p in glob.glob(os.path.join(VENDOR, kind, "*.json"))
            )
            raise SystemExit(
                f"no {kind} preset at {rel}\navailable:\n  "
                + "\n  ".join(available[:40])
            )
        dst = os.path.join(PROFILE_DIR, f"ad5m_pro_{nozzle}_{kind}.json")
        merged = orca_profile.resolve(src)

        if kind == "machine":
            # Upstream's fdm_adventurer5m_common sets use_relative_e_distances=1
            # but ships a layer_change_gcode with no G92 E0, which OrcaSlicer's
            # own validator then rejects (return -51). Add exactly what the
            # validator asks for. Harmless under M83 relative extrusion.
            lc = merged.get("layer_change_gcode", "")
            if merged.get("use_relative_e_distances") in ("1", 1, True) \
                    and "G92 E0" not in lc:
                merged["layer_change_gcode"] = lc.rstrip("\n") + "\nG92 E0"

        os.makedirs(PROFILE_DIR, exist_ok=True)
        with open(dst, "w") as fh:
            json.dump(merged, fh, indent=2)
        out[kind] = dst
        if not quiet:
            print(f"  profile {kind:8s} {len(merged):4d} keys  {os.path.basename(dst)}")
    return out


def slice_part(stl, profiles, extra_args=()):
    """Run the slicer once; return (gcode_path, stats dict)."""
    name = os.path.splitext(os.path.basename(stl))[0]
    os.makedirs(GCODE_DIR, exist_ok=True)

    cmd = [
        "xvfb-run", "-a", "flatpak", "run", FLATPAK_ID,
        "--load-settings", f"{profiles['machine']};{profiles['process']}",
        "--load-filaments", profiles["filament"],
        "--slice", "0", "--ensure-on-bed",
        "--outputdir", GCODE_DIR,
        *extra_args,
        stl,
    ]
    env = dict(os.environ, **SANDBOX_ENV)
    proc = subprocess.run(cmd, capture_output=True, text=True, env=env)

    result_path = os.path.join(GCODE_DIR, "result.json")
    result = {}
    if os.path.exists(result_path):
        with open(result_path) as fh:
            result = json.load(fh)

    if proc.returncode != 0 or result.get("return_code", 0) != 0:
        detail = result.get("error_string") or proc.stdout[-600:] or proc.stderr[-600:]
        raise SystemExit(f"slice failed for {name}: {detail.strip()}")

    produced = os.path.join(GCODE_DIR, "plate_1.gcode")
    final = os.path.join(GCODE_DIR, f"{name}.gcode")
    os.replace(produced, final)
    os.remove(result_path)
    return final, gcode_stats(final)


def gcode_stats(path):
    """Pull the header numbers worth printing, plus the real bed footprint."""
    stats = {}
    patterns = {
        "grams": r"^; filament used \[g\] = ([\d.]+)",
        "time": r"^; estimated printing time \(normal mode\) = (.+)",
        "layers": r"^; total layer number: (\d+)",
        "height": r"^; max_z_height: ([\d.]+)",
    }
    xs, ys = [], []
    # machine_start_gcode draws a purge line across X55..X-55 at the front of
    # the bed. It is extrusion, but it is not the part -- counting it puts the
    # footprint at a flat 110mm wide for every model. The custom start/end
    # blocks are wrapped in ";TYPE:Custom", so track the role and only measure
    # while inside a real extrusion role.
    in_part = False
    with open(path, errors="ignore") as fh:
        for line in fh:
            if line.startswith(";TYPE:"):
                in_part = line[len(";TYPE:"):].strip() != "Custom"
            if line.startswith("; "):
                for key, pat in patterns.items():
                    if key not in stats:
                        m = re.match(pat, line)
                        if m:
                            stats[key] = m.group(1).strip()
            elif in_part and line.startswith("G1 ") and " E" in line:
                mx = re.search(r"X(-?\d+\.?\d*)", line)
                my = re.search(r"Y(-?\d+\.?\d*)", line)
                if mx:
                    xs.append(float(mx.group(1)))
                if my:
                    ys.append(float(my.group(1)))
    if xs and ys:
        stats["footprint"] = f"{max(xs)-min(xs):.1f} x {max(ys)-min(ys):.1f} mm"
        # The 5M Pro bed is center-origin (-110..110); a part centred on (0,0)
        # is what we want to send. See the "always center prints" rule.
        stats["centre"] = f"({(min(xs)+max(xs))/2:+.1f}, {(min(ys)+max(ys))/2:+.1f})"
        stats["in_bounds"] = (min(xs) >= -110 and max(xs) <= 110
                              and min(ys) >= -110 and max(ys) <= 110)
    return stats


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("parts", nargs="*", help="part names (foot) or stl paths")
    ap.add_argument("--all", action="store_true", help="slice every stl in cad/stl")
    ap.add_argument("--nozzle", default="0.4", help="nozzle diameter (default 0.4)")
    ap.add_argument("--layer", default="0.20", help="layer height (default 0.20)")
    ap.add_argument("--filament", default="PLA", help="e.g. PLA, PETG, ABS")
    ap.add_argument("--", dest="passthrough", nargs=argparse.REMAINDER, default=[],
                    help="extra flags passed straight to OrcaSlicer")
    args = ap.parse_args()

    if args.all:
        stls = sorted(glob.glob(os.path.join(STL_DIR, "*.stl")))
    elif args.parts:
        stls = []
        for part in args.parts:
            path = part if os.path.exists(part) else os.path.join(STL_DIR, f"{part}.stl")
            if not os.path.exists(path):
                raise SystemExit(f"no such stl: {path}")
            stls.append(path)
    else:
        ap.error("name at least one part, or pass --all")

    print(f"AD5M Pro | {args.nozzle}mm nozzle | {args.layer}mm layers | {args.filament}")
    profiles = build_profiles(args.nozzle, args.layer, args.filament)

    failures = 0
    for stl in stls:
        name = os.path.splitext(os.path.basename(stl))[0]
        try:
            path, st = slice_part(stl, profiles, args.passthrough)
        except SystemExit as exc:
            print(f"  {name:22s} FAILED: {exc}")
            failures += 1
            continue
        flag = "" if st.get("in_bounds", True) else "  !! OFF BED"
        print(f"  {name:22s} {st.get('time','?'):>10s}  {st.get('grams','?'):>6s} g  "
              f"{st.get('layers','?'):>4s} layers  {st.get('footprint','?'):>16s}  "
              f"centre {st.get('centre','?')}{flag}  -> {os.path.relpath(path, REPO)}")

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
