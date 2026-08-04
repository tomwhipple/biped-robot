#!/usr/bin/env python3
"""Flatten OrcaSlicer vendor presets into self-contained JSON.

OrcaSlicer's GUI resolves a preset's ``inherits`` chain against its bundled
profile tree. The CLI does *not* do this when you hand it a file path -- it
loads exactly the keys in that one file and fills the rest from built-in
defaults. For the Adventurer 5M Pro that silently yields an inconsistent
machine (relative E addressing with no ``G92 E0`` in layer_gcode), and slicing
aborts with return code -51.

This walks the chain and writes a flattened preset the CLI can consume.
"""

import argparse
import json
import os
import sys

FLATPAK_PROFILES = os.path.expanduser(
    "~/.local/share/flatpak/app/com.orcaslicer.OrcaSlicer/"
    "current/active/files/share/OrcaSlicer/profiles"
)

# A preset's parent may live in any of the vendor's category dirs, not just the
# one the child came from -- machine presets in particular reach into shared
# "fdm_*_common" files that sit alongside them.
CATEGORIES = ("machine", "process", "filament")


def _candidates(root, vendor, name):
    """Every path a preset called ``name`` might live at."""
    leaf = name if name.endswith(".json") else name + ".json"
    yield os.path.join(root, vendor, leaf)
    for category in CATEGORIES:
        yield os.path.join(root, vendor, category, leaf)
    yield os.path.join(root, leaf)


def _find(root, vendor, name):
    for path in _candidates(root, vendor, name):
        if os.path.exists(path):
            return path
    return None


def resolve(path, root=FLATPAK_PROFILES, _seen=None):
    """Return the fully merged preset dict for the preset at ``path``."""
    path = os.path.abspath(path)
    _seen = _seen or []
    if path in _seen:
        raise SystemExit(f"inherits cycle: {' -> '.join(_seen + [path])}")

    with open(path) as fh:
        child = json.load(fh)

    parent_name = child.get("inherits")
    if not parent_name:
        return child

    # Vendor dir is the one holding the category dirs, e.g. profiles/Flashforge.
    vendor_dir = os.path.dirname(os.path.dirname(path))
    vendor = os.path.basename(vendor_dir)
    parent_path = _find(root, vendor, parent_name)
    if parent_path is None:
        raise SystemExit(
            f"cannot resolve inherits={parent_name!r} referenced by {path}"
        )

    merged = resolve(parent_path, root, _seen + [path])
    merged.update(child)
    # The chain is baked in now, so the pointer to the parent has to go -- but
    # "from" must survive: the loader rejects a preset without it ("from
    # unsupported"), and an intermediate in the chain is marked
    # instantiation=false, which the child overrides back to true.
    merged.pop("inherits", None)
    return merged


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("preset", help="path to a vendor preset json")
    ap.add_argument("-o", "--out", required=True, help="flattened output path")
    ap.add_argument("--root", default=FLATPAK_PROFILES)
    ap.add_argument("--show", action="store_true",
                    help="print a few resolved keys for sanity")
    args = ap.parse_args()

    merged = resolve(args.preset, args.root)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as fh:
        json.dump(merged, fh, indent=2)

    print(f"{os.path.basename(args.preset)} -> {args.out}  ({len(merged)} keys)")
    if args.show:
        for key in ("name", "layer_gcode", "use_relative_e_distances",
                    "printable_area", "nozzle_diameter"):
            if key in merged:
                print(f"  {key} = {json.dumps(merged[key])[:110]}")


if __name__ == "__main__":
    sys.exit(main())
