"""Where a run's config.json is found (#236).

A run's own config.json lives in sim/runs/<run>/, which is gitignored: a
clean clone has none. A deployed run's is also tracked, with the inputs the
firmware's policy sets are generated from: firmware/deployed/runs/<run>/
config.json (tools/policy_profile.py), so the twin, the console's Sim tab and
the SIL suite can run the deployed policy from a clean clone -- its weights
are then the ones compiled into the firmware (policy/weights.h, generated
from the same params.pkl; eval_precision.sil_setup).

The run directory's copy wins when both exist. Dependency-free on purpose:
sim/sil_twin.py reads the config before sim/sil/harness.py may be imported
(the harness sizes its ABI from $BIMO_POLICY at import).
"""
import os

SIM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RUNS_DIR = os.path.join(SIM, "runs")
DEPLOYED_RUNS_DIR = os.path.join(os.path.dirname(SIM), "firmware", "deployed", "runs")


def tracked_config(run):
    """The tracked copy's path, firmware/deployed/runs/<run>/config.json."""
    return os.path.join(DEPLOYED_RUNS_DIR, run, "config.json")


def config_path(run, run_dir=None):
    """The run's config.json: <run_dir>/config.json (run_dir defaults to
    sim/runs/<run>) when it exists, else the tracked copy when that exists,
    else the run directory's path, so an error names the usual place."""
    own = os.path.join(run_dir or os.path.join(RUNS_DIR, run), "config.json")
    if os.path.exists(own):
        return own
    tracked = tracked_config(run)
    return tracked if os.path.exists(tracked) else own
