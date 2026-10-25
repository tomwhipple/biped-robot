"""Generic parallel scan helper: python scan.py <name>  (see the SCANS dict)."""
import itertools, json, multiprocessing as mp, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import swing_sim as S
import numpy as np

HW = dict(rms=[4.04, 2.00, 0.75], err_peak=2.35, f_hz=2.0)

def job(a):
    cfg, ctl, sw = a
    try:
        r = S.run(cfg, ctl, sw, keep_log=False)
    except Exception as e:
        return dict(cfg=cfg, ctl=ctl, sw=sw, error=repr(e), fell=True, tripped=True, rms=[99.0] * 3, err_peak=99.0, roll_pp=99.0,
                    f_hz=float("nan"), clr_min=-99.0, clr_med=-99.0, gap_min=-99.0, contact_frac=1.0, selfc=0, usw_max=0.0, u_max=0.0, t_end=0.0)
    return r

def pmap(jobs, procs=12):
    with mp.get_context('fork').Pool(procs) as p:
        return p.map(job, jobs, chunksize=1)

def fmt(r):
    return "rms %s err %.2f pp %.2f f %.2f clr %.1f gap %.1f fell %d" % (
        "/".join("%.2f" % x for x in r['rms']), r['err_peak'], r['roll_pp'], r['f_hz'], r['clr_min'], r['gap_min'], r['fell'])

if __name__ == "__main__":
    grid = dict(roll_lash=[1.0, 2.6, 4.5], play=[0.0, 3.0], kp_scale=[1.0, 0.5], pad_solref=[None, (0.03, 0.2), (0.06, 0.1)], delay=[1, 3])
    keys = list(grid)
    jobs = [(dict(zip(keys, v)), None, None) for v in itertools.product(*grid.values())]
    res = pmap(jobs)
    for r in res:
        if 'error' in r: print(r['cfg'], r['error']); continue
        c = {k: r['cfg'][k] for k in keys}
        print(c, fmt(r))
