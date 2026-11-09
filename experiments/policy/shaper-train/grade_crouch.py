"""grade.py at a crouch command: every probe's cmd[3] = 1.0 becomes CROUCH (the crouch-channel policies,
--cmd-crouch-range; Tom 11-05 "walk with a slight bend to the knee"). The walk16 reference rows are dropped.
  CROUCH=0.9 RUNS=<student> SHAPE_HZ=8 GRADE_XML=... python grade_crouch.py 8 <out.json> runs
"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
CROUCH = float(os.environ["CROUCH"])
import grade as G
import walker_env as WE
_sc0 = WE.BimoWalkerEnv.set_command
def _sc(self, *ch):
    ch = list(ch)
    if len(ch) >= 4 and ch[3] == 1.0:
        ch[3] = CROUCH
    return _sc0(self, *ch)
WE.BimoWalkerEnv.set_command = _sc
_rs0 = G.runs_set
G.runs_set = lambda runs, hz: {k: v for k, v in _rs0(runs, hz).items() if "walk16" not in k}
if __name__ == "__main__":
    G.main()
