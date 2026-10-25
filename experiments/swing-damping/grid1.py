import sys,json,itertools; sys.path.insert(0,'.')
import numpy as np, swing_eval as E, scan
SEEDS=3
def agg(res):
    rr=[r for r in res if 'error' not in r]
    rm=np.array([r['rms'] for r in rr]); 
    return dict(rms=np.exp(np.log(np.maximum(rm,.05)).mean(0)).round(2).tolist(), err=round(float(np.mean([r['err_peak'] for r in rr])),2),
      clr=round(min(r['clr_min'] for r in rr),1), gap=round(min(r['gap_min'] for r in rr),2), fell=sum(r['fell'] for r in rr), usw=round(max(r['usw_max'] for r in rr),1))
variants={'hip':(1,0),'ankle':(0,1),'both+':(1,1),'both-':(1,-1)}
gains=[('sr',v) for v in (-0.4,-0.2,-0.1,-0.05,0.05,0.1,0.2,0.4)]+[('se',v) for v in (-1.0,-0.6,-0.3,-0.1,0.1,0.3,0.6,1.0)]
out=[]
for kd in (0.03,0.05):
    base=agg(scan.pmap([(E.cal(seed=s),dict(kd=kd),None) for s in range(SEEDS)]))
    print("kd",kd,"no swing",base,flush=True)
    jobs=[]; meta=[]
    for name,(sh,sa) in variants.items():
        for g,v in gains:
            for s in range(SEEDS):
                jobs.append((E.cal(seed=s),dict(kd=kd),{g:v,'sh':sh,'sa':sa})); meta.append((kd,name,g,v))
    res=scan.pmap(jobs)
    for i in range(0,len(res),SEEDS):
        a=agg(res[i:i+SEEDS]); m=meta[i]; out.append((m,a)); 
        print(m,a,flush=True)
json.dump(out,open('/home/claw/code/robot-swing-damp/build/swing/grid1.json','w'),default=str)
