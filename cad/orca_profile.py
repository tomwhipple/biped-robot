#!/usr/bin/env python3
"""Flatten OrcaSlicer presets into self-contained JSON.

OrcaSlicer's GUI resolves a preset's ``inherits`` chain against its bundled
profile tree. The CLI does *not* do this when you hand it a file path -- it
loads exactly the keys in that one file and fills the rest from built-in
defaults. A thin preset therefore slices with silently wrong settings (a thin
filament preset falls back to PLA), so every preset handed to the CLI is
flattened first.

The system presets the print spec names (cad/v6/print_settings.json: printer,
process, filament) come from a snapshot in the repo, cad/orca/<kind>.json:
each one flattened from OrcaSlicer's published profile JSONs at a release tag,
so slicing never reads the installed app's presets. The app ships those only
as binary caches (<vendor>.opc, OrcaSlicer docs/HLSD/preset-cache.md), and
their format is not an interface. The GUI's own presets (the user preset dir,
e.g. the printer preset that holds the print host) are layered on top of the
snapshot by name.

    python3 cad/orca_profile.py process "0.20mm Standard @Flashforge AD5M Pro 0.4 Nozzle" -o p.json
    python3 cad/orca_profile.py snapshot                 # at the installed app's release
    python3 cad/orca_profile.py snapshot --ref v2.5.1
"""

import argparse
import json
import os
import plistlib
import subprocess
import sys
import urllib.parse
import urllib.request

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SNAPSHOT_DIR = os.path.join(REPO, "cad", "orca")
SETTINGS = os.path.join(REPO, "cad", "v6", "print_settings.json")

FLATPAK_ID = "com.orcaslicer.OrcaSlicer"
MAC_APP = os.environ.get("ORCA_APP", "/Applications/OrcaSlicer.app")
MAC_USER = os.path.expanduser("~/Library/Application Support/OrcaSlicer/user/default")
FLATPAK_USER = os.path.expanduser(
    f"~/.var/app/{FLATPAK_ID}/config/OrcaSlicer/user/default")

# The vendor whose presets the spec names, then the shared filament library
# any vendor's filament may inherit from.
VENDORS = ("Flashforge", "OrcaFilamentLibrary")
KINDS = ("machine", "process", "filament")
UPSTREAM = "https://raw.githubusercontent.com/OrcaSlicer/OrcaSlicer"


def default_user_dir():
    return MAC_USER if sys.platform == "darwin" else FLATPAK_USER


class Library:
    """The snapshot's system presets and the GUI's user presets, indexed by name."""

    def __init__(self, snapshot_dir=SNAPSHOT_DIR, user_dir=None):
        self.snapshot_dir = snapshot_dir
        self.user_dir = user_dir if user_dir is not None else default_user_dir()
        self.presets = {k: {} for k in KINDS}   # kind -> name -> (path, dict)
        for kind in KINDS:
            path = os.path.join(snapshot_dir, f"{kind}.json")
            if not os.path.exists(path):
                raise SystemExit(f"no {kind} preset snapshot at {path} "
                                 "(python3 cad/orca_profile.py snapshot)")
            with open(path) as fh:
                data = json.load(fh)
            self.presets[kind][data["name"]] = (path, data)
        # User presets second: one may shadow a system name.
        for kind in KINDS:
            d = os.path.join(self.user_dir, kind)
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
        if name in self.presets[kind]:
            return self.presets[kind][name][1]
        raise SystemExit(f"no {kind} preset named {name!r} "
                         f"(the snapshot in {self.snapshot_dir} has "
                         f"{sorted(n for n, (p, _) in self.presets[kind].items() if p.startswith(self.snapshot_dir))})")

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


# --------------------------------------------------------------------------
# snapshot

def installed_version():
    """The installed OrcaSlicer's version, e.g. '2.5.1', or None."""
    if sys.platform == "darwin":
        try:
            with open(os.path.join(MAC_APP, "Contents", "Info.plist"), "rb") as fh:
                return plistlib.load(fh)["CFBundleShortVersionString"]
        except (OSError, KeyError):
            return None
    try:
        out = subprocess.run(["flatpak", "info", FLATPAK_ID], capture_output=True,
                             text=True, check=True).stdout
    except (OSError, subprocess.CalledProcessError):
        return None
    for line in out.splitlines():
        key, _, value = line.strip().partition(":")
        if key == "Version":
            return value.strip()
    return None


def fetch_json(ref, path):
    url = f"{UPSTREAM}/{urllib.parse.quote(ref)}/resources/profiles/{urllib.parse.quote(path)}"
    with urllib.request.urlopen(url, timeout=60) as resp:
        return json.load(resp)


class Upstream:
    """OrcaSlicer's vendor profile JSONs at one git ref, fetched on demand."""

    def __init__(self, ref):
        self.ref = ref
        self.versions = {}
        self.where = {k: {} for k in KINDS}    # kind -> name -> (vendor, sub_path)
        for vendor in VENDORS:
            index = fetch_json(ref, f"{vendor}.json")
            self.versions[vendor] = index["version"]
            for kind in KINDS:
                for item in index.get(f"{kind}_list", []):
                    self.where[kind].setdefault(item["name"], (vendor, item["sub_path"]))

    def resolve(self, kind, name, _seen=()):
        if name in _seen:
            raise SystemExit(f"inherits cycle: {' -> '.join(_seen + (name,))}")
        if name not in self.where[kind]:
            raise SystemExit(f"no {kind} preset named {name!r} in {', '.join(VENDORS)} at {self.ref}")
        vendor, sub_path = self.where[kind][name]
        data = fetch_json(self.ref, f"{vendor}/{sub_path}")
        if data.get("include"):
            # Upstream layers an include between the parent and the preset's own keys;
            # no preset the spec names uses one yet.
            raise SystemExit(f"{name!r} uses include, which the snapshot does not resolve")
        parent = data.get("inherits")
        merged = self.resolve(kind, parent, _seen + (name,)) if parent else {}
        merged.update(data)
        merged.pop("inherits", None)
        return merged


def snapshot(ref, outdir=SNAPSHOT_DIR, settings=SETTINGS):
    with open(settings) as fh:
        spec = json.load(fh)
    names = {"machine": spec["printer"], "process": spec["process"]["preset"],
             "filament": spec["filament"]["preset"]}
    up = Upstream(ref)
    os.makedirs(outdir, exist_ok=True)
    for kind, name in names.items():
        with open(os.path.join(outdir, f"{kind}.json"), "w") as fh:
            json.dump(up.resolve(kind, name), fh, indent=2, sort_keys=True)
            fh.write("\n")
    with open(os.path.join(outdir, "source.json"), "w") as fh:
        json.dump({"generated_by": "cad/orca_profile.py snapshot -- do not edit",
                   "upstream": f"{UPSTREAM}/{ref}/resources/profiles",
                   "ref": ref, "vendor_versions": up.versions, "presets": names},
                  fh, indent=2)
        fh.write("\n")
    print(f"{ref}: {', '.join(f'{k} {n!r}' for k, n in names.items())} -> {outdir}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd")
    snap = sub.add_parser("snapshot", help="rewrite cad/orca/ from OrcaSlicer's profile JSONs")
    snap.add_argument("--ref", help="OrcaSlicer git ref (default: v<installed version>)")
    for kind in KINDS:
        p = sub.add_parser(kind, help=f"flatten a {kind} preset")
        p.add_argument("name", help="preset name, as shown in the GUI")
        p.add_argument("-o", "--out", required=True, help="flattened output path")
    args = ap.parse_args()

    if args.cmd == "snapshot":
        ref = args.ref
        if not ref:
            version = installed_version()
            if not version:
                raise SystemExit("no installed OrcaSlicer found; pass --ref")
            ref = f"v{version}"
        snapshot(ref)
    elif args.cmd in KINDS:
        merged = Library().resolve(args.cmd, args.name)
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, "w") as fh:
            json.dump(merged, fh, indent=2)
        print(f"{args.name} -> {args.out}  ({len(merged)} keys)")
    else:
        ap.print_help()
        return 2


if __name__ == "__main__":
    sys.exit(main())
