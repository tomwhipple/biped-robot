"""Check the design against tools/parts.py.

usage: python3 tools/netcheck.py <netlist>          a KiCad netlist (kicad-cli sch export netlist), pin by pin
       python3 tools/netcheck.py --keys <board>      the servo headers' keys on the board (below)
exit 1 on any mismatch.

--keys: a stock servo lead joins the bevelled corners of its two housings, so on each of J4-J8 the pads, read
from the bevelled corner of the silkscreen outline, must carry the General Driver's H5 nets in H5's order from
its own bevelled corner (parts.H5_FROM_BEVEL). It also checks that each header's body (its F.Fab outline)
clears the mounting hardware in the holes beside it.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import parts as PD  # noqa: E402
import sexpr as S  # noqa: E402

# the hardware in the mounting holes (docs/assembly.md, 9c), as a radius around the hole: M2.5 x 6 button heads
# in H2 and H4 (ISO 7380, 4.7 mm across); M2.5 x 15 male-female standoffs in H1 and H3 (5 mm hex, 2.89 mm to a
# corner)
HARDWARE_R = {"H1": 2.89, "H2": 2.35, "H3": 2.89, "H4": 2.35}
MIN_CLEARANCE = 0.2   # mm, a header's body to that hardware


def check_netlist(path):
    root = S.parse(open(path).read())[0]
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
    return bad


def footprints(board):
    """ref -> footprint node, for every footprint on a parsed .kicad_pcb"""
    out = {}
    for fp in S.find_all(board, "footprint"):
        for p in S.find_all(fp, "property"):
            if p[1] == "Reference":
                out[p[2]] = fp
    return out


def lines(fp, layer):
    """the footprint's fp_line segments on <layer>, in footprint coordinates"""
    out = []
    for g in S.find_all(fp, "fp_line"):
        if S.find(g, "layer")[1] == layer:
            s, e = S.find(g, "start"), S.find(g, "end")
            out.append(((float(s[1]), float(s[2])), (float(e[1]), float(e[2]))))
    return out


def pads(fp):
    """pad number -> ((x, y) in footprint coordinates, net name or None)"""
    out = {}
    for p in S.find_all(fp, "pad"):
        at, net = S.find(p, "at"), S.find(p, "net")
        name = None
        if net:
            name = net[-1].lstrip("/")
            if name.startswith("unconnected-") or not name:
                name = None
        out[p[1]] = ((float(at[1]), float(at[2])), name)
    return out


def to_board(fp, pt):
    """footprint coordinates -> board coordinates (KiCad: y down, rotation counter-clockwise as seen)"""
    at = S.find(fp, "at")
    x0, y0 = float(at[1]), float(at[2])
    t = math.radians(float(at[3]) if len(at) > 3 else 0.0)
    x, y = pt
    return x0 + x * math.cos(t) + y * math.sin(t), y0 - x * math.sin(t) + y * math.cos(t)


def seg_dist(p, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    u = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(p[0] - ax - u * dx, p[1] - ay - u * dy)


def check_keys(board_text):
    """Problems with J4-J8's orientation against H5 and their clearance to the mounting hardware, as strings;
    an empty list passes."""
    fps = footprints(S.parse(board_text)[0])
    problems = []
    for ref in PD.SERVO_HEADERS:
        fp = fps[ref]
        bevel = [(a, b) for a, b in lines(fp, "F.SilkS") if abs(a[0] - b[0]) > 0.01 and abs(a[1] - b[1]) > 0.01]
        if not bevel:
            problems.append(f"{ref}: no bevelled corner on the silkscreen outline")
            continue
        ends = [pt for seg in bevel for pt in seg]
        mid = (sum(x for x, _ in ends) / len(ends), sum(y for _, y in ends) / len(ends))
        pd = pads(fp)
        order = sorted(pd, key=lambda n: math.dist(pd[n][0], mid))
        spec = PD.parts[ref]["nets"]
        for n, want in zip(order, PD.H5_FROM_BEVEL):
            have = pd[n][1]
            if have != want and not (have is None and spec.get(n) is None):
                problems.append(f"{ref}: pad {n}, {order.index(n) + 1} from the bevelled corner, is {have}; "
                                f"a stock lead from H5 puts H5's {want} there")
        fab = [(to_board(fp, a), to_board(fp, b)) for a, b in lines(fp, "F.Fab")]
        for hole, r in HARDWARE_R.items():
            if hole in fps:
                c = to_board(fps[hole], (0.0, 0.0))
                gap = min(seg_dist(c, a, b) for a, b in fab) - r
                if gap < MIN_CLEARANCE:
                    problems.append(f"{ref}: its body is {gap:.2f} mm from {hole}'s hardware "
                                    f"(want {MIN_CLEARANCE} mm or more)")
    return problems


def main():
    if sys.argv[1] == "--keys":
        problems = check_keys(open(sys.argv[2]).read())
        for p in problems:
            print("MISMATCH", p)
        print(f"servo headers {', '.join(PD.SERVO_HEADERS)}: keyed to H5 ({' '.join(PD.H5_FROM_BEVEL)} from the "
              f"bevelled corner), {len(problems)} problems")
        return 1 if problems else 0
    return 1 if check_netlist(sys.argv[1]) else 0


if __name__ == "__main__":
    sys.exit(main())
