"""Run fem_yoke.py for every (tag, case), 3 at a time:  python run_cases.py <tag> [<tag> ...]"""
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                       "build", "experiments", "hip-yoke-strength")
HERE = os.path.dirname(os.path.abspath(__file__))
CASES = ("roll", "pitch", "yaw", "fz", "fx")


def one(job):
    tag, case = job
    os.makedirs(os.path.join(OUT_DIR, "fem"), exist_ok=True)
    log = os.path.join(OUT_DIR, "fem", f"log_{tag}_{case}.txt")
    with open(log, "w") as fh:
        r = subprocess.run([sys.executable, "-I", os.path.join(HERE, "fem_yoke.py"), tag], cwd=HERE,
                           env=dict(os.environ, FEM_CASE=case), stdout=fh, stderr=subprocess.STDOUT)
    return tag, case, r.returncode


if __name__ == "__main__":
    jobs = [(t, c) for t in sys.argv[1:] for c in CASES]
    with ThreadPoolExecutor(max_workers=3) as ex:
        for tag, case, rc in ex.map(one, jobs):
            print(tag, case, "ok" if rc == 0 else f"FAILED rc={rc}")
