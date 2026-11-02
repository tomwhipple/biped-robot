"""Run a set of whole-board cases on a process pool and save the results.

    python -m pbsim.board.campaign truth  [--workers N] [--limit K]
    python -m pbsim.board.campaign seq    [--workers N]
    python -m pbsim.board.campaign sweep  [--workers N]

(from ``pcb/power-board``; ``make pbfull`` from the repo root runs all three).
Results go to ``build/power-board/fullsim/<suite>.json``. Each worker process
loads its own libngspice (ngspice's shared mode is one engine per process).
"""
from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import os
import sys
import time
import traceback
from concurrent.futures import ProcessPoolExecutor, as_completed

from .netlist import CACHE, load

OUT = CACHE / "fullsim"


def _init():
    os.environ.setdefault("OMP_NUM_THREADS", "1")


def _call(job):
    mod, fn, arg = job
    import importlib
    t0 = time.time()
    try:
        res = getattr(importlib.import_module(mod), fn)(arg)
        res["wall_s"] = time.time() - t0
        return res
    except Exception as e:  # noqa: BLE001
        return {"name": getattr(arg, "name", str(arg)), "error": repr(e),
                "trace": traceback.format_exc(), "fail": [f"error: {e!r}"],
                "wall_s": time.time() - t0}


def run(jobs, workers: int, label: str) -> list[dict]:
    load()      # export the netlist once, before the workers race for it
    out = []
    t0 = time.time()
    ctx = mp.get_context("spawn")
    with ProcessPoolExecutor(max_workers=workers, mp_context=ctx,
                             initializer=_init) as ex:
        futs = [ex.submit(_call, j) for j in jobs]
        for k, f in enumerate(as_completed(futs), 1):
            r = f.result()
            out.append(r)
            if len(futs) <= 300:
                print(f"[{label}]   {r.get('name')}: {r.get('wall_s', 0):.1f} s"
                      f"{' FAIL' if r.get('fail') else ''}", flush=True)
            if k % 25 == 0 or k == len(futs):
                nfail = sum(1 for x in out if x.get("fail"))
                print(f"[{label}] {k}/{len(futs)} done, {nfail} failing, "
                      f"{time.time() - t0:.0f} s", flush=True)
    out.sort(key=lambda r: r.get("name", ""))
    return out


def save(name: str, results: list[dict]):
    OUT.mkdir(parents=True, exist_ok=True)
    p = OUT / f"{name}.json"
    p.write_text(json.dumps(results, indent=1, default=str))
    print(f"wrote {p}")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("suite", choices=("truth", "seq", "sweep"))
    ap.add_argument("--workers", type=int,
                    default=max(1, min(12, (os.cpu_count() or 4) - 4)))
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--only", default="")
    a = ap.parse_args(argv)
    if a.suite == "truth":
        from .truth import all_cases
        cases = all_cases()
        if a.only:
            cases = [c for c in cases if a.only in c.name]
        if a.limit:
            cases = cases[:a.limit]
        jobs = [("pbsim.board.truth", "run_case", c) for c in cases]
    elif a.suite == "seq":
        from .sequences import all_sequences
        names = [n for n in all_sequences() if a.only in n]
        jobs = [("pbsim.board.sequences", "run_named", n) for n in names]
    else:
        from .sweeps import all_sweeps
        names = [n for n in all_sweeps() if a.only in n]
        jobs = [("pbsim.board.sweeps", "run_named", n) for n in names]
    res = run(jobs, a.workers, a.suite)
    save(a.suite if not a.only else f"{a.suite}-{a.only}", res)
    nfail = sum(1 for r in res if r.get("fail"))
    print(f"{a.suite}: {len(res)} cases, {nfail} failing")
    return 0


if __name__ == "__main__":
    sys.exit(main())
