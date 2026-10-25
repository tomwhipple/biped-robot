"""The board's connectivity, exported from the KiCad schematic by kicad-cli.

The whole-board simulation is built from this netlist, not from a hand
transcription, so a schematic edit changes what is simulated. kicad-cli does
the connectivity (wires, junctions, labels, power symbols, pin rotation),
which is not worth re-implementing; without it the whole-board tests skip and
say why. The export is cached in ``build/power-board/`` keyed by the
schematic's hash.
"""
from __future__ import annotations

import functools
import hashlib
import os
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

BOARD_DIR = Path(__file__).resolve().parents[2]
SCHEMATIC = BOARD_DIR / "power-board.kicad_sch"
REPO = BOARD_DIR.parents[1]
CACHE = REPO / "build" / "power-board"
KICAD_CLI_CANDIDATES = (
    "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli",
    "/usr/bin/kicad-cli",
    "/usr/local/bin/kicad-cli",
)


KICAD_MAJOR = 10      # the schematic's format; KiCad 9 cannot read it


@functools.cache
def find_kicad_cli() -> str | None:
    """A kicad-cli from KiCad 10 or later, or None."""
    env = os.environ.get("KICAD_CLI")
    cands = [env] if env else [c for c in KICAD_CLI_CANDIDATES if Path(c).exists()]
    if not cands and shutil.which("kicad-cli"):
        cands = [shutil.which("kicad-cli")]
    for c in cands:
        try:
            v = subprocess.run([c, "version"], capture_output=True, text=True,
                               timeout=30).stdout.strip()
        except OSError:
            continue
        m = re.match(r"(\d+)\.", v)
        if m and int(m.group(1)) >= KICAD_MAJOR:
            return c
    return None


@dataclass
class Part:
    ref: str
    value: str
    mpn: str
    footprint: str
    pins: dict[str, str] = field(default_factory=dict)   # pin number -> net
    pinfn: dict[str, str] = field(default_factory=dict)  # pin number -> function


@dataclass
class Netlist:
    parts: dict[str, Part]
    nets: dict[str, list[tuple[str, str]]]               # net -> [(ref, pin)]
    sch_sha256: str

    def net(self, ref: str, pin: str | int) -> str:
        return self.parts[ref].pins[str(pin)]

    def pins_on(self, net: str) -> list[tuple[str, str]]:
        return self.nets[net]


def _export(sch: Path, out: Path) -> None:
    cli = find_kicad_cli()
    if cli is None:
        raise FileNotFoundError(
            f"no kicad-cli from KiCad {KICAD_MAJOR} or later (install it, or set "
            "KICAD_CLI); the whole-board simulation reads the schematic through it")
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(".tmp")
    subprocess.run([cli, "sch", "export", "netlist", "--format", "kicadxml",
                    "-o", str(tmp), str(sch)], check=True,
                   capture_output=True, text=True)
    tmp.replace(out)


def load(sch: Path = SCHEMATIC) -> Netlist:
    digest = hashlib.sha256(sch.read_bytes()).hexdigest()
    xml = CACHE / f"power-board-{digest[:16]}.net.xml"
    if not xml.exists():
        _export(sch, xml)
    root = ET.parse(xml).getroot()
    parts: dict[str, Part] = {}
    for c in root.iter("comp"):
        fields = {f.get("name"): (f.text or "") for f in c.iter("field")}
        parts[c.get("ref")] = Part(c.get("ref"), c.findtext("value") or "",
                                   fields.get("MPN", ""),
                                   c.findtext("footprint") or "")
    nets: dict[str, list[tuple[str, str]]] = {}
    for n in root.iter("net"):
        name = n.get("name")
        nodes = []
        for x in n.iter("node"):
            ref, pin = x.get("ref"), x.get("pin")
            nodes.append((ref, pin))
            if ref in parts:
                parts[ref].pins[pin] = name
                fn = (x.get("pinfunction") or "").rsplit("_", 1)[0]
                parts[ref].pinfn[pin] = fn
        nets[name] = nodes
    return Netlist(parts, nets, digest)


_SI = {"p": 1e-12, "n": 1e-9, "u": 1e-6, "µ": 1e-6, "m": 1e-3, "": 1.0,
       "k": 1e3, "K": 1e3, "M": 1e6, "G": 1e9}


def si(value: str) -> float:
    """KiCad value text → number. KiCad's 'M' is mega and 'm' milli (SPICE's
    'M' is milli, so values are always emitted as plain numbers)."""
    tok = value.split()[0]
    m = re.fullmatch(r"([0-9.]+)\s*([pnuµmkKMG]?)[A-Za-zΩ]*", tok)
    if not m:
        m = re.fullmatch(r"([0-9]+)([pnuµmkKMGR])([0-9]+)", tok)
        if not m:
            raise ValueError(f"cannot parse value {value!r}")
        mult = 1.0 if m.group(2) == "R" else _SI[m.group(2)]
        return float(f"{m.group(1)}.{m.group(3)}") * mult
    return float(m.group(1)) * _SI[m.group(2)]


def node_name(net: str) -> str:
    """A SPICE-safe node name for a KiCad net; GND is the reference."""
    if net == "GND":
        return "0"
    s = net.strip("/")
    s = s.replace("+", "p").replace("-", "n")
    s = re.sub(r"[^A-Za-z0-9_]", "_", s).strip("_").lower()
    s = re.sub(r"_+", "_", s)
    return "n_" + s
