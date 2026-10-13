"""Paths to the vendored TI CSD17556Q5B ngspice model + rewrite provenance.

``csd17556q5b_ngspice.lib`` is a rewrite of TI's released ``CSD17556Q5B.lib``
(SLPM066, downloaded 2026-10-12) that Debian ngspice-42 will actually parse:

1. TI puts a `.PARAM` block at the top of the subckt body. Debian ngspice's
   parser evaluates `.PARAM` at parse time and reports the constants as
   undefined when the subckt is *instantiated* later. The rewrite inlines
   TI's six constants as literal numbers: `ptrc1=5.50e-3`, `ptrc2=12e-6`,
   `pwidthP=9.35`, `pwidth=9.35e6`, `perimP=19.53` (= 2.1 × 9.35),
   `perim=19.53e6`.
2. TI's `.MODEL NMOS NMOS (LEVEL=7 VERSION=3.2 ...)` is SPICE3 BSIM3 3.2.
   Debian ngspice-42 registers BSIM3v3 at LEVEL=8; the rewritten model
   reads `LEVEL=8 VERSION=3.3`. Every explicit parameter TI sets is carried
   over textually; BSIM3v3 3.3 differs from 3.2 only in parameter
   *defaults* for fields TI sets anyway.

The TI original is vendored alongside as `CSD17556Q5B_TI_ORIGINAL.lib`;
``test_ngsmoke.py`` asserts the rewrite's semiconductor parameter block is
character-identical to the original's (modulo whitespace and the two noted
keywords) so the rewrite cannot silently drift from TI's physics.

The rewrite was produced by a one-shot script (kept in vendor/README.md);
that script is not part of the test suite and is not re-run.
"""
from __future__ import annotations

from hashlib import sha256
from pathlib import Path

VENDOR = Path(__file__).resolve().parent / "vendor"
REWRITE_LIB = VENDOR / "csd17556q5b_ngspice.lib"
TI_ORIGINAL_LIB = VENDOR / "CSD17556Q5B_TI_ORIGINAL.lib"

# Pinned by inspection; tests assert them so a silent file swap is loud.
SHA256_TI_ORIGINAL = "40dbf08721b84b5fd8427aa3fae0ed2c4d101a4306391fb71a3b67027614cf27"
SHA256_REWRITE     = "bc28a017fab73c5d6b42058596c9921b1dbd7bd859511f50431534162d5a0beb"
SHA256_NGSPICE_SO  = "e69c2f217188e6ec8537b48983492950022e85a4c4adbd72a4faf4060445e3a2"


def check_hashes() -> dict[str, str]:
    """Return sha256 → status for each vendored artifact."""
    out = {}
    for name, want, path in (
        ("TI original .lib", SHA256_TI_ORIGINAL, TI_ORIGINAL_LIB),
        ("ngspice-suitable .lib", SHA256_REWRITE, REWRITE_LIB),
        ("libngspice.so.0.0.9", SHA256_NGSPICE_SO,
         VENDOR / "ngspice" / "libngspice.so.0.0.9"),
    ):
        if not path.exists():
            out[name] = f"missing: {path}"
            continue
        got = sha256(path.read_bytes()).hexdigest()
        out[name] = "ok" if got == want else f"MISMATCH got={got} want={want}"
    return out
