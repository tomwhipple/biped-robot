"""Compare a KiCad netlist (kicad-cli sch export netlist) with tools/parts.py, pin by pin.
usage: python3 tools/netcheck.py <netlist>   exit 1 on any mismatch"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import parts as PD  # noqa: E402
import sexpr as S  # noqa: E402

root = S.parse(open(sys.argv[1]).read())[0]
got = {}
for n in S.find_all(S.find(root, "nets"), "net"):
    name = S.find(n, "name")[1].lstrip("/")
    for node in S.find_all(n, "node"):
        got[(S.find(node, "ref")[1], S.find(node, "pin")[1])] = name
bad = 0
nets = set()
for ref, d in PD.parts.items():
    for pin, want in d["nets"].items():
        have = got.get((ref, pin))
        if want is None:
            if have and not have.startswith("unconnected-"):
                print(f"MISMATCH {ref}.{pin}: want no connection, netlist has {have}")
                bad += 1
            continue
        nets.add(want)
        if have != want:
            print(f"MISMATCH {ref}.{pin}: want {want}, netlist has {have}")
            bad += 1
print(f"{len(PD.parts)} parts, {len(nets)} nets, {bad} mismatches")
sys.exit(1 if bad else 0)
