"""The walking probes under the loop gain (planner 2026-11-25).

grade.py's `x1.5` applies the robot's measured ~1.5x loop gain (2026-11-02, a stand) to a 20 s stand only. This
wrapper applies the same law (out_gain: the action's deviation from its 0.5 s mean x G) to the six `dir *` probes
and `walk 0.3 -> stop`, so a student is also graded walking under the gain. Everything else is grade.py as is.

    OUT_GAIN=1.5 RUNS=<students> ONLY="walk 0.3 -> stop,dir fwd 0.3,dir back -0.25,dir side L 0.15,dir side R -0.15,dir turn L 0.5,dir turn R -0.5" \
    SHAPE_HZ=8 GRADE_XML=bimo_biped_v6legs_meas_sole.xml GRADE_PLANT_TERMS="sag_L=6.4,sag_R=2.9" \
    JAX_PLATFORMS=cpu CUDA_VISIBLE_DEVICES= WORKERS=12 python experiments/policy/x15-margin/grade_gain.py 8 out.json runs

First use (ledger "Night of 2026-11-24"): at G=1.5, M1 (prec97) fell 56/56 on all three seeds, L2 (lg13) 38-49/56,
L1 (lg16) 0-6/56.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "shaper-train"))
import grade  # noqa: E402

G = float(os.environ["OUT_GAIN"])
for _k in list(grade.DIRS) + ["walk 0.3 -> stop"]:
    grade.PROBES[_k] = dict(grade.PROBES[_k], out_gain=G)

if __name__ == "__main__":   # the pool may spawn: keep main() behind the guard
    sys.argv = ["grade.py"] + sys.argv[1:]
    grade.main()
