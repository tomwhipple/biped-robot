"""Where a run's config.json is found (#236).

A run's own config.json lives in sim/runs/<run>/, which is gitignored: a
clean clone has none. The deployed run's is also tracked beside its weight
sidecar, as sim/sil/weights/<run>.config.json (a verbatim copy of the
training box's), so the twin, the console's Sim tab and the SIL suite can run
the deployed policy from a clean clone -- its weights are then the ones
compiled into the firmware (policy/weights.h, exported from the same
params.pkl; eval_precision.sil_setup).

The run directory's copy wins when both exist. Dependency-free on purpose:
sim/sil_twin.py reads the config before sim/sil/harness.py may be imported
(the harness sizes its ABI from $BIMO_POLICY at import).
"""
import os

SIM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RUNS_DIR = os.path.join(SIM, "runs")
WEIGHTS_DIR = os.path.join(SIM, "sil", "weights")


def tracked_config(run):
    """The tracked copy's path, sim/sil/weights/<run>.config.json."""
    return os.path.join(WEIGHTS_DIR, f"{run}.config.json")


def config_path(run, run_dir=None):
    """The run's config.json: <run_dir>/config.json (run_dir defaults to
    sim/runs/<run>) when it exists, else the tracked copy when that exists,
    else the run directory's path, so an error names the usual place."""
    own = os.path.join(run_dir or os.path.join(RUNS_DIR, run), "config.json")
    if os.path.exists(own):
        return own
    tracked = tracked_config(run)
    return tracked if os.path.exists(tracked) else own
