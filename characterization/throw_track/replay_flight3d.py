"""Causal before/after on every saved clip; does not require or invent gestures.

Old clips use their session's tuned colour configuration (not a gesture lock).
loc3 may be excluded with --training, then run once after freezing parameters.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'ball_view'))
sys.path.insert(0,str(HERE.parents[1]/'ros/src/omni_catch'))
sys.path.insert(0,str(HERE.parent/'src/omni_catch'))
import session as S
from flight3d import Config,FlightDetector,public_result
from bucket_vision import observe
from omni_catch.fast_detector import FastDetector
from omni_catch.detector import DetectorConfig
from omni_catch.frames import ArrayFrame


def run(sd,td,cfg,cache):
    sess=S.load_session(sd)
    th=S.load_throw(td,sess)
    rs=sess['rs']
    intr=tuple(rs['streams']['color']['intrinsics'][k] for k in ('fx','fy','ppx','ppy'))
    di=tuple(rs['streams']['depth']['intrinsics'][k] for k in ('fx','fy','ppx','ppy'))
    ext=rs.get('extrinsics',{}).get('depth_to_color')
    R=np.array(ext['R']).reshape(3,3,order='F') if ext else np.eye(3)
    tr=np.array(ext['t']) if ext else np.zeros(3)
    key=cache/f'{sd.name}_{td.name}.json'
    if key.exists():
        observations=json.loads(key.read_text())
    else:
        det=FastDetector(DetectorConfig(**sess['detector']))
        dep=th.stream('depth')
        observations=[]
        for i,t in enumerate(th.t):
            cs=det.candidates(ArrayFrame(np.asarray(th.frames[i])),8)
            j=int(np.searchsorted(dep.stamps[:,0],th.stamps[i,0],side='right')-1)
            depth=dep.frames[j] if j>=0 and th.stamps[i,0]-dep.stamps[j,0]<=40_000_000 else None
            obs=[observe(float(t),c,intr,cfg.radius,depth,rs.get('depth_scale',.001),di,R,tr) for c in cs]
            observations.append([vars(o) for o in obs if o is not None])
        key.write_text(json.dumps(observations))
    from flight3d import Observation
    detector=FlightDetector(intr,cfg)
    confirmations=[]
    checks=[]
    ms=[]
    cpu=[]
    for i,(t,os) in enumerate(zip(th.t,observations)):
        start=time.perf_counter()
        cpu_start=time.process_time()
        hit=detector.update(float(t),[Observation(**o) for o in os])
        ms.append((time.perf_counter()-start)*1000)
        cpu.append((time.process_time()-cpu_start)*1000)
        if hit:
            confirmations.append(dict(frame=i,t=float(t),track=hit.id,**public_result(hit.result)))
        if detector.best:
            checks.append(dict(frame=i,t=float(t),track=detector.best.id,**public_result(detector.best.result)))
    return dict(session=sd.name,clip=td.name,confirmed=bool(confirmations),confirmations=confirmations,
                best_checks=checks,ms_p50_95_99=np.percentile(ms,[50,95,99]).tolist(),
                gate_cpu_ms_mean=float(np.mean(cpu)),gate_core_equivalent_60fps=float(np.mean(cpu)*.06),
                review=th.meta.get('review'))


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('root',type=Path)
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--training',action='store_true')
    ap.add_argument('--heldout',action='store_true')
    ap.add_argument('--session',default='')
    a=ap.parse_args()
    a.out.parent.mkdir(parents=True,exist_ok=True)
    cache=a.out.parent/'measurements'
    cache.mkdir(exist_ok=True)
    cfg=Config()
    result=dict(config=vars(cfg),limits=__doc__,clips=[],
                code_sha256=hashlib.sha256((HERE.parent/'ball_view/flight3d.py').read_bytes()).hexdigest())
    for sd in sorted(a.root.iterdir()):
        if not (sd/'session.json').exists() or a.session not in sd.name:
            continue
        if (a.training and sd.name.endswith('loc3')) or (a.heldout and not sd.name.endswith('loc3')):
            continue
        for td in S.throw_dirs(sd):
            row=run(sd,td,cfg,cache)
            result['clips'].append(row)
            a.out.write_text(json.dumps(result,indent=2))
            print(sd.name,td.name,row['confirmed'],len(row['confirmations']),
                  row['confirmations'][0]['t'] if row['confirmations'] else '',
                  row['ms_p50_95_99'],flush=True)


if __name__=='__main__':
    main()
