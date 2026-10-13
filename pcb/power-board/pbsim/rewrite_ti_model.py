"""Audit script: describe the difference between
`vendor/CSD17556Q5B_TI_ORIGINAL.lib` (byte-identical to TI's released
SLPM066) and `vendor/csd17556q5b_ngspice.lib` (the rewrite that Debian
ngspice-42 parses). Kept for auditability (P3-6).

What it reports:

  1. The set of `.PARAM` constants TI declares at the top of the subckt
     (these are the six values the rewrite inlines as literals: ptrc1,
     ptrc2, pwidthP, pwidth, perimP, perim).
  2. The `.MODEL` keyword deltas between the two files — the only
     intentional differences (LEVEL 7 → 8, VERSION 3.2 → 3.3).
  3. The sha256 of each file, for comparison against `fetmodel.py`'s
     pinned values.

It is NOT a code generator: producing the rewrite required manual review
(to confirm the six constants were the only `.PARAM` lines and that the
two keyword pairs were the only model changes). Regenerating it
mechanically from TI's release on a new ngspice version is a re-do of
that review, not a sed pipeline.
"""
from __future__ import annotations

import re
import sys
from hashlib import sha256
from pathlib import Path

HERE = Path(__file__).resolve().parent
TI_ORIGINAL = HERE / "vendor" / "CSD17556Q5B_TI_ORIGINAL.lib"
REWRITE = HERE / "vendor" / "csd17556q5b_ngspice.lib"


def main() -> int:
    ti_text = TI_ORIGINAL.read_text()
    rw_text = REWRITE.read_text()

    print(f"TI original sha256:  {sha256(ti_text.encode()).hexdigest()}")
    print(f"ngspice rewrite sha: {sha256(rw_text.encode()).hexdigest()}")
    print()

    # 1. .PARAM constants TI declares (must be the six the rewrite inlines)
    # TI's format is ` .PARAM  name    value` (possibly multi-line).
    params = re.findall(r"(?im)^\s*\.PARAM\s+(\w+)\s*=", ti_text)
    if not params:
        # TI sometimes writes `.PARAM name value` without `=`; catch both
        params = re.findall(r"(?im)^\s*\.PARAM\s+(\w+)\s+\S", ti_text)
    print(f".PARAM constants in TI original ({len(params)}):")
    for p in params:
        print(f"  {p}")
    expected = {"ptrc1", "ptrc2", "pwidthP", "pwidth", "perimP", "perim"}
    if set(params) != expected:
        print(f"WARN: expected {sorted(expected)}; diff above is review-relevant")
    print()

    # 2. .MODEL keyword deltas
    ti_models = re.findall(r"(?m)^\s*\.MODEL.*$", ti_text)
    rw_models = re.findall(r"(?m)^\s*\.MODEL.*$", rw_text)
    print(f".MODEL lines: TI {len(ti_models)}, rewrite {len(rw_models)}")
    for a, b in zip(ti_models, rw_models):
        if a != b:
            print(f"  TI:      {a.strip()}")
            print(f"  rewrite: {b.strip()}")

    # 3. LEVEL / VERSION specifically (the known keyword changes)
    ti_lv = re.findall(r"LEVEL\s*=\s*(\d+).*?VERSION\s*=\s*([\d.]+)", ti_text)
    rw_lv = re.findall(r"LEVEL\s*=\s*(\d+).*?VERSION\s*=\s*([\d.]+)", rw_text)
    print()
    print(f"TI      LEVEL/VERSION pairs: {ti_lv}")
    print(f"rewrite LEVEL/VERSION pairs: {rw_lv}")
    print()
    print("Intentional deltas: LEVEL 7 -> 8, VERSION 3.2 -> 3.3; every "
          "other parameter must match.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
