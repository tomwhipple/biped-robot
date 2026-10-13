# vendored ngspice + TI CSD17556Q5B model

`ngspice/libngspice.so.0.0.9` is the binary from the Pop!_OS 24.04
`libngspice0` apt package (upstream Debian build of ngspice 42, released
2024-03-31, compiled with KLU). Vendored so the suite runs on a machine that
cannot `sudo apt install ngspice` (the task brief calls out that ngspice and
PySpice are not installed, and sudo needs a password this session does not
have).

The shared library's *device plugins* (xtradev, xtraevt, spice2poly, table,
analog, digital) are NOT needed by the suite: the only devices used are
R, C, L, D, V, I, the MOSFET (BSIM3 via `LEVEL=8`), and subckts, all of
which ngspice-42 links into the main .so. Verified by the smoke test
(see `tests/test_ngsmoke.py`).

`CSD17556Q5B_TI_ORIGINAL.lib` is byte-identical to TI's
https://www.ti.com/lit/zip/slpm066 (fetch and unzip to verify). ngspice-42
as shipped on Debian refuses TI's syntax for two reasons:

1. TI uses a `.PARAM` block at the top of the subckt. ngspice parses
   `.PARAM` at parse time and reports `Undefined parameter [pwidthp]`.
   Debian ngspice's compatibility-mode selector (`set compat=ps`) is
   broken in this build — the in-circuit directive is silently ignored.
2. TI's `.MODEL NMOS NMOS ( LEVEL = 7 VERSION = 3.2 ...)` uses LEVEL=7,
   which is the SPICE3 spelling of BSIM3 3.2; Debian ngspice-42 registers
   BSIM3 at LEVEL=8 instead.

`csd17556q5b_ngspice.lib` is the mechanically rewritten form:

- every TI `.PARAM` constant flattened to the literal numeric value TI
  shipped (no formula change);
- `LEVEL = 7 VERSION = 3.2` → `LEVEL = 8 VERSION = 3.3` (BSIM3v3 3.3 is a
  strict superset of 3.2 defaults as far as TI's explicit parameter set is
  concerned — every parameter TI sets is set in both versions);
- TI's device names (`NMOS`, `PMOSd`, `DBD`, `DD`, `DDx`, `RTEMP`) kept.

Regenerate with `python pcb/power-board/pbsim/rewrite_ti_model.py` (kept
here as the script that produced the rewrite, for auditability.
`test_schematic_intact.py` pins the SHA-256 of both files so the rewrite
cannot drift silently.
