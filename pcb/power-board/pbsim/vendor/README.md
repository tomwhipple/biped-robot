# vendored ngspice + TI CSD17556Q5B model

## libngspice

`ngspice/libngspice.so.0.0.9` is the shared library from Ubuntu 24.04
"noble" (universe), package `libngspice0` version `42+ds-3build1`, amd64
(the Debian build of ngspice 42, compiled with KLU; Pop!_OS 24.04 serves
the same package). Vendored so the suite runs on a Linux machine that
cannot `sudo apt install ngspice`. It is an x86-64 ELF and loads on
linux-x86_64 only; elsewhere `ngsim.find_lib()` loads a native libngspice
(macOS: `brew install libngspice`, ngspice 47 — the version the
verification record's C4–C6 runs used) or `$PBSIM_NGSPICE`, and the spice
tests skip with the reason when there is none.

Provenance, checkable without trusting the committer: the archive's
signed index publishes the package's hash, and the package's own
`md5sums` lists the library's.

| | value |
|---|---|
| package | http://archive.ubuntu.com/ubuntu/pool/universe/n/ngspice/libngspice0_42+ds-3build1_amd64.deb |
| index | http://archive.ubuntu.com/ubuntu/dists/noble/universe/binary-amd64/Packages.gz |
| index stanza | `Size: 2531432`, `SHA256: 967d5c0c83ab264f344504fa40d8475b7bc4e80abac4395e43ac04fec1b0de98` |
| `md5sums` line | `4b330a99bff3353171689851b2561588  usr/lib/x86_64-linux-gnu/libngspice.so.0.0.9` |
| library SHA-256 | `e69c2f217188e6ec8537b48983492950022e85a4c4adbd72a4faf4060445e3a2` (pinned in `fetmodel.py`) |

Checked 2026-10-14: the downloaded `.deb` matches the index stanza, and the
library extracted from it matches the vendored file byte for byte. To
repeat (on Linux use `sha256sum` / `md5sum`; `tar` must read zstd):

```sh
mkdir /tmp/ngv && cd /tmp/ngv
B=http://archive.ubuntu.com/ubuntu
D=libngspice0_42+ds-3build1_amd64.deb
curl -fsSLO $B/pool/universe/n/ngspice/$D
curl -fsSL $B/dists/noble/universe/binary-amd64/Packages.gz | gzip -dc \
  | awk -v RS= '/^Package: libngspice0\n/' | grep -E '^(Version|Size|SHA256):'
shasum -a 256 $D                      # = the SHA256 line above
ar x $D && mkdir d c && tar -xf data.tar.zst -C d && tar -xf control.tar.zst -C c
grep libngspice.so c/md5sums
shasum -a 256 d/usr/lib/x86_64-linux-gnu/libngspice.so.0.0.9   # = e69c2f21…e3a2
```

The index itself is trusted here over HTTP; for the full chain, verify
`dists/noble/InRelease` with `gpgv` against Ubuntu's archive key and check
`Packages.gz` against the hash it lists.

The library's device plugins (xtradev, xtraevt, spice2poly, table, analog,
digital) are NOT needed: the suite uses only R, C, L, D, V, I, the MOSFET
(BSIM3 via `LEVEL=8`) and subckts, all of which ngspice links into the
main library.

## TI CSD17556Q5B model

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

`csd17556q5b_ngspice.lib` is the hand-written rewrite:

- TI's `.PARAM` block removed, and each `{expression}` that used it
  replaced by its value (`W=9.35`, `PS=PD=19.635`, `TC1=5.50e-3`,
  `TC2=12.00e-6`);
- `LEVEL = 7 VERSION = 3.2` → `LEVEL = 8 VERSION = 3.3` (BSIM3v3 3.3 is a
  strict superset of 3.2 defaults as far as TI's explicit parameter set is
  concerned — every parameter TI sets is set in both versions);
- TI's device names (`NMOS`, `PMOSd`, `DBD`, `DD`, `DDx`, `RTEMP`) kept.

`../rewrite_ti_model.py` audits it: it evaluates TI's expressions,
compares the two files token by token, and prints every difference;
`test_ngsmoke.py` asserts the differences are exactly the two keywords,
that the model tracks TI's transfer curve (datasheet Figure 3), and that
the SHA-256 pins in `fetmodel.py` hold for all three files.
