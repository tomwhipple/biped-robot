"""Per-part STEP exports.

DEPRECATED as a separate step since 2026-07-27: `parts.py` now writes the STEP
beside every STL, from the same solid in the same run. Keeping a second export
path is exactly how the STEPs went three days stale while the STLs were current
-- so this script just delegates rather than re-implementing the loop.

Run:  .venv/bin/python cad/parts.py     (exports cad/stl/*.stl AND cad/step/*.step)
"""
import parts

if __name__ == "__main__":
    print(__doc__.strip().splitlines()[2])
    print("delegating to parts.main() ...\n")
    parts.main()
