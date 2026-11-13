import os, sys, json, itertools
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from concurrent.futures import ProcessPoolExecutor
import sway as W, testbed as T, laws as Lw
PL = {"fit": T.BOARD, "lo": "grade+play_L_hip=4,play_R_hip=6,play_L_ankle=2,play_R_ankle=0",
      "hi": "grade+play_L_hip=10,play_R_hip=10,play_L_ankle=3,play_R_ankle=2", "hard": T.BOARD + ",floor_tc=0.01"}
DIRS = {"march": {}, "fwd": dict(dx=0.02), "back": dict(dx=-0.02), "sideL": dict(dy=0.015), "sideR": dict(dy=-0.015),
        "turn": dict(yaw=0.15)}

def one(a):
    Tst, k, ss, plant, d, delay = a
    try:
        tr = W.Sway(T=Tst, n_steps=8, k=k, ss=ss, h=0.04, kL=1.35, kR=1.1, **DIRS[d])
        law = Lw.Trim(kp=0.25, ki=1.0, t0={'L': 8.0, 'R': -3.0})
        o = T.run(tr, law, plant=PL[plant], extra_ms=delay)
        return dict(T=Tst, k=k, ss=ss, plant=plant, dir=d, delay=delay, lifts=o['lifts'], n=o['n_phases'],
                    fell=o['fell'], fdir=o['fall_dir'], peak=round(o['peak_deg'], 1))
    except Exception as e:
        return dict(T=Tst, error=repr(e)[:200])

if __name__ == '__main__':
    jobs = list(itertools.product([2.5, 2.0, 1.75, 1.5], [1.2], [0.5], list(PL), list(DIRS), [0.0, 60.0]))
    with ProcessPoolExecutor(14) as ex:
        rows = list(ex.map(one, jobs))
    json.dump(rows, open(sys.argv[1], 'w'))
    print(len(rows), 'rows')
