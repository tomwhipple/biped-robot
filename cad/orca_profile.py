#!/usr/bin/env python3
"""Flatten OrcaSlicer presets into self-contained JSON.

OrcaSlicer's GUI resolves a preset's ``inherits`` chain against its bundled
profile tree. The CLI does *not* do this when you hand it a file path -- it
loads exactly the keys in that one file and fills the rest from built-in
defaults. A thin preset therefore slices with silently wrong settings (a thin
filament preset falls back to PLA), so every preset handed to the CLI is
flattened first.

Presets are looked up by name, first in the user's preset directory (the GUI's
own presets, e.g. the printer preset that holds the print host), then in the
bundled vendor profiles.

    python3 cad/orca_profile.py process "0.20mm Standard @Flashforge AD5M Pro 0.4 Nozzle" -o p.json
"""

import argparse
import json
import os
import sys

FLATPAK_PROFILES = os.path.expanduser(
    "~/.local/share/flatpak/app/com.orcaslicer.OrcaSlicer/"
    "current/active/files/share/OrcaSlicer/profiles"
)
MAC_APP = os.environ.get("ORCA_APP", "/Applications/OrcaSlicer.app")
MAC_PROFILES = os.path.join(MAC_APP, "Contents", "Resources", "profiles")
MAC_USER = os.path.expanduser("~/Library/Application Support/OrcaSlicer/user/default")
FLATPAK_USER = os.path.expanduser(
    "~/.var/app/com.orcaslicer.OrcaSlicer/config/OrcaSlicer/user/default")

VENDOR = "Flashforge"
KINDS = ("machine", "process", "filament")


def default_roots():
    """(vendor profile dir, user preset dir) for the installed OrcaSlicer."""
    if sys.platform == "darwin":
        return os.path.join(MAC_PROFILES, VENDOR), MAC_USER
    return os.path.join(FLATPAK_PROFILES, VENDOR), FLATPAK_USER


class Library:
    """Every preset under the vendor and user dirs, indexed by name."""

    def __init__(self, vendor_dir=None, user_dir=None):
        vd, ud = default_roots()
        self.vendor_dir = vendor_dir or vd
        self.user_dir = user_dir if user_dir is not None else ud
        if not os.path.isdir(self.vendor_dir):
            raise SystemExit(f"no OrcaSlicer vendor profiles at {self.vendor_dir}")
        self.presets = {k: {} for k in KINDS}   # kind -> name -> (path, dict)
        # Vendor first, user second: a user preset may shadow a vendor name.
        for root in (self.vendor_dir, self.user_dir):
            for kind in KINDS:
                d = os.path.join(root, kind)
                if not os.path.isdir(d):
                    continue
                for fn in sorted(os.listdir(d)):
                    if not fn.endswith(".json"):
                        continue
                    path = os.path.join(d, fn)
                    try:
                        with open(path) as fh:
                            data = json.load(fh)
                    except (OSError, ValueError):
                        continue
                    if not isinstance(data, dict):
                        continue
                    # The GUI shows a user preset under its file stem.
                    for name in {data.get("name"), fn[:-5]} - {None}:
                        self.presets[kind][name] = (path, data)

    def get(self, kind, name):
        # A parent can sit in another category's dir (the shared fdm_* bases).
        for k in (kind,) + tuple(x for x in KINDS if x != kind):
            if name in self.presets[k]:
                return self.presets[k][name][1]
        raise SystemExit(f"no {kind} preset named {name!r}")

    def resolve(self, kind, name, _seen=()):
        """The fully merged preset: parents first, then each child's keys.

        ``inherits`` is dropped -- the chain is baked in. ``from`` survives (the
        loader rejects a preset without it).
        """
        if name in _seen:
            raise SystemExit(f"inherits cycle: {' -> '.join(_seen + (name,))}")
        data = self.get(kind, name)
        parent = data.get("inherits")
        merged = self.resolve(kind, parent, _seen + (name,)) if parent else {}
        merged.update(data)
        merged.pop("inherits", None)
        return merged

    def user_printers(self, system_name):
        """User printer presets that inherit ``system_name`` and have a print host."""
        out = []
        for name, (path, data) in self.presets["machine"].items():
            if not path.startswith(self.user_dir) or name != data.get("name"):
                continue
            if data.get("inherits") == system_name and data.get("host_type"):
                out.append(name)
        return sorted(set(out))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("kind", choices=KINDS)
    ap.add_argument("name", help="preset name, as shown in the GUI")
    ap.add_argument("-o", "--out", required=True, help="flattened output path")
    args = ap.parse_args()

    merged = Library().resolve(args.kind, args.name)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as fh:
        json.dump(merged, fh, indent=2)
    print(f"{args.name} -> {args.out}  ({len(merged)} keys)")


if __name__ == "__main__":
    sys.exit(main())
