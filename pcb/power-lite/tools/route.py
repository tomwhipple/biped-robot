"""schematic netlist -> place (gen_pcb) -> DSN -> Freerouting -> session import + pours -> DRC.
usage: python3 tools/route.py <work dir>   (from pcb/power-lite; KiCad 10 and Freerouting 2.x)
The Freerouting jar is taken from $FREEROUTING_JAR."""
import collections
import os
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROJ = HERE.parent
WORK = Path(sys.argv[1])
WORK.mkdir(parents=True, exist_ok=True)
KPY = "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3"
KCLI = "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli"
JAVA = os.environ.get("JAVA", "/opt/homebrew/opt/openjdk/bin/java")
JAR = os.environ["FREEROUTING_JAR"]
BOARD = str(PROJ / "power-lite.kicad_pcb")


def run(cmd, log=None):
    r = subprocess.run([str(c) for c in cmd], capture_output=True, text=True)
    out = "\n".join(line for line in (r.stdout + r.stderr).splitlines()
                    if "assert" not in line.lower() and "Fontconfig" not in line)
    if log:
        Path(log).write_text(out)
    if r.returncode:
        sys.exit(f"FAILED {cmd[:2]}\n{out[-3000:]}")
    return out


net = WORK / "power-lite.net"
run([KCLI, "sch", "export", "netlist", "-o", net, PROJ / "power-lite.kicad_sch"])
print(run([sys.executable, HERE / "netcheck.py", net]))
print(run([KPY, HERE / "gen_pcb.py", net]))
print(run([KPY, HERE / "export_dsn.py", BOARD, WORK / "power-lite.dsn"]))
best = None
for i, extra in enumerate((["-mp", "60"], ["-mp", "100", "-is", "random"], ["-mp", "100", "-is", "sequential"])):
    ses = WORK / f"try{i}.ses"
    fr = run([JAVA, "-Djava.awt.headless=true", "-jar", JAR, "-de", WORK / "power-lite.dsn", "-do", ses,
              "--gui.enabled=false"] + extra, log=WORK / f"try{i}.log")
    unrouted = sorted({ln.strip() for ln in fr.splitlines() if "->" in ln and ln.strip().startswith("-")})
    print(f"freerouting try {i} {' '.join(extra)}: unrouted {unrouted or 'none'}")
    if best is None or len(unrouted) < len(best[1]):
        best = (ses, unrouted)
    if not unrouted:
        break
print(run([KPY, HERE / "post_pcb.py", BOARD, best[0]]))
run([KCLI, "pcb", "drc", "--severity-all", "--schematic-parity", "-o", WORK / "drc.rpt", BOARD])
rpt = (WORK / "drc.rpt").read_text()
print("DRC:", dict(collections.Counter(re.findall(r"^\[(\w+)\]", rpt, re.M))))
print("\n".join(line for line in rpt.splitlines() if "**" in line))
