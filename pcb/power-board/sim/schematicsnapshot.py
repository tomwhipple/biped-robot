"""Read-only parse of ``pcb/power-board/power-board.kicad_sch``.

The verification record (C3) states the component values as facts. Tests in
this suite must never re-type those facts by hand: every threshold in the
behavioural models is derived from the schematic's *actual* parsed values
(R24 ohmic value, R30/R31 divider, C16/C17, C19...) so a design revision that
touches the schematic without updating these tests fails immediately.

We pull four things out of the S-expression schematic:

- ``components``  ``{ref: {lib, value, at(x, y, rot), pins[..]}}`` for every
  instantiation. ``lib`` is e.g. ``Device:R``, ``power-board:BQ76922``.
- ``local_labels`` ``set`` of named ``(label ...)`` tokens (the intra-sheet
  named nets: ``BQ_CHG``, ``BQ_DSG``, ``BQ_PACK``, ``BQ_LD``, ``ESTOP_RET``,
  ``SERVO_EN``, ``SERVO_IMON``, ``SERVO_FLT_N``, ``RAIL_V``, ``ESTOP_N``,
  ``KILL``, ``PWR_INT_N``, ``SCL``, ``SDA``).
- ``power_names``  set of named power-symbol values (``GND``, ``BAT-``,
  ``PACK+``, ``SERVO+``, ``+BATT``, ``+3V3_HOST``, ``PWR_FLAG``, ``VCC``...).
- ``wires``  segment list — used by the parse test to know the schematic is a
  *wired* one, not a set of labels alone; we do not attempt to build a
  physical adjacency graph from geometry.

KiCad's file format is stable across revs in the attributes this module
touches (``lib_id``/``property``/``pin``/``at``/``label``/``wire``).
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

SCHEMATIC = (
    Path(__file__).resolve().parent.parent / "power-board.kicad_sch"
)


# ---------------- s-expression walker ---------------------------------------


def _tokenize(s: str):
    toks: list[str] = []
    i, n = 0, len(s)
    while i < n:
        c = s[i]
        if c in " \t\r\n":
            i += 1
        elif c == "(":
            toks.append("(")
            i += 1
        elif c == ")":
            toks.append(")")
            i += 1
        elif c == '"':
            j = s.index('"', i + 1)
            while s[j - 1] == "\\":
                j = s.index('"', j + 1)
            toks.append(s[i:j + 1])
            i = j + 1
        else:
            j = i
            while j < n and s[j] not in "() \t\r\n":
                j += 1
            toks.append(s[i:j])
            i = j
    return toks


def _parse(toks: list[str]):
    root: list[Any] = []
    stack = [root]
    for t in toks:
        if t == "(":
            lst: list[Any] = []
            stack[-1].append(lst)
            stack.append(lst)
        elif t == ")":
            stack.pop()
        else:
            if t.startswith('"'):
                t = t[1:-1]
            else:
                try:
                    t = float(t) if "." in t else int(t)  # type: ignore[assignment]
                except ValueError:
                    pass
            stack[-1].append(t)
    return root[0]


def _walk(node):
    yield node
    if isinstance(node, list):
        for child in node[1:]:
            yield from _walk(child)


# ---------------- public view -----------------------------------------------


@dataclass(frozen=True)
class Component:
    lib: str
    ref: str
    value: str
    at: tuple[float, float, float]  # x, y, rot_deg
    pins: tuple[str, ...] = ()
    properties: dict[str, str] = field(default_factory=dict)


@dataclass
class Schematic:
    components: dict[str, Component]            # keyed by ref (skip #* power syms)
    ref_libs: dict[str, str]                    # ref -> lib_id
    values: dict[str, str]                      # ref -> Value property
    local_labels: set[str]
    power_names: set[str]
    wires: list[tuple[tuple[float, float], tuple[float, float]]]
    text_notes: list[str]


_SUFFIX_SCALE = {
    "p": 1e-12, "n": 1e-9, "u": 1e-6, "µ": 1e-6,
    "m": 1e-3, "k": 1e3, "M": 1e6, "G": 1e9,
}


def parse_si_value(s: str) -> float:
    """Best-effort SI → float for component Value strings.

    Accepts: ``"2m"`` → 2e-3, ``"4.7k"`` → 4700, ``"100n 25V"`` → 1e-7,
    ``"1000u 25V"`` → 1e-3, ``"15A"`` → 15 (suffix after digits is the unit).
    Raises ``ValueError`` on anything it cannot parse — the caller decides
    whether that is a failure (test_schematic_parse asserts every R/C/F/L is
    parseable).
    """
    # drop trailing unit / voltage rating text
    first_token = s.split()[0]
    # match NNN[.NNN][suffix]
    m = re.fullmatch(r"([0-9]+(?:\.[0-9]+)?)([a-zA-Zµ]*)", first_token)
    if not m:
        raise ValueError(f"cannot parse value token {s!r}")
    base = float(m.group(1))
    suf = m.group(2)
    if suf not in _SUFFIX_SCALE:
        # bare number OR suffix like "V"/"A" that we treat as multiplier 1
        return base
    return base * _SUFFIX_SCALE[suf]


def load(path: Path = SCHEMATIC) -> Schematic:
    text = path.read_text()
    tree = _parse(_tokenize(text))

    components: dict[str, Component] = {}
    ref_libs: dict[str, str] = {}
    values: dict[str, str] = {}
    local_labels: set[str] = set()
    power_names: set[str] = set()
    wires = []
    text_notes: list[str] = []

    for node in _walk(tree):
        if not (isinstance(node, list) and node):
            continue
        tag = node[0]
        if tag == "symbol":
            lib = None
            ref = None
            value = None
            at = None
            pins: list[str] = []
            props: dict[str, str] = {}
            for sub in node[1:]:
                if not (isinstance(sub, list) and sub):
                    continue
                if sub[0] == "lib_id" and len(sub) > 1:
                    lib = sub[1]
                elif sub[0] == "at" and len(sub) > 2:
                    at = (float(sub[1]), float(sub[2]),
                          float(sub[3]) if len(sub) > 3 else 0.0)
                elif sub[0] == "pin" and len(sub) > 1:
                    pins.append(str(sub[1]))
                elif sub[0] == "property" and len(sub) > 2:
                    key, val = sub[1], sub[2]
                    props[key] = val
                    if key == "Reference":
                        ref = val
                    elif key == "Value":
                        value = val
            if lib is None or ref is None:
                continue
            comp = Component(lib=lib, ref=ref, value=value or "",
                             at=at or (0.0, 0.0, 0.0),
                             pins=tuple(pins), properties=props)
            # power symbols carry ``#PWRxxx`` refs; connectors and parts get
            # real refs. Keep both but make a power-name view.
            if ref.startswith("#"):
                # lib_id like "power:GND"; the value is the net name
                power_names.add(value or lib.split(":", 1)[-1])
            else:
                components[ref] = comp
                ref_libs[ref] = lib
                values[ref] = value or ""
        elif tag == "label" and len(node) > 1:
            local_labels.add(node[1])
        elif tag == "wire":
            pts = [s for s in node[1:] if isinstance(s, list) and s[0] == "pts"]
            if pts:
                xy = [s[1:] for s in pts[0][1:] if isinstance(s, list)]
                if len(xy) >= 2:
                    wires.append(
                        ((float(xy[0][0]), float(xy[0][1])),
                         (float(xy[1][0]), float(xy[1][1])))
                    )
        elif tag == "text" and len(node) > 1:
            text_notes.append(node[1])

    return Schematic(
        components=components,
        ref_libs=ref_libs,
        values=values,
        local_labels=local_labels,
        power_names=power_names,
        wires=wires,
        text_notes=text_notes,
    )
