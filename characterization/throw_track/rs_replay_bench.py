"""Camera-free Pi benchmark: native colour candidates + causal arc/depth gate.

numpy only; loads one clip's colour into RAM before timing. No camera, recorder
writes, ROS, prediction or rendering. Replays every frame without pacing, twice.
The depth gate uses the latest preceding depth frame (<=40 ms), never a future
frame. These are per-clip resets with no learning or continuous re-arm state;
trigger counts are diagnostics, not independently labelled throw recall.
"""
import argparse
import hashlib
import json
import platform
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
for root in (HERE.parents[1]/'ros/src/omni_catch', HERE.parent/'src/omni_catch'):
    if root.exists():
        sys.path.insert(0, str(root))
from omni_catch.detector import DetectorConfig
from omni_catch.fast_detector import FastDetector, available, SRC
from omni_catch.frames import ArrayFrame
import arc2d
import session as S


def run(td, sess, speed):
    th = S.load_throw(td, sess)
    frames = np.array(th.frames)  # IO/copy explicitly outside timing
    cfg = DetectorConfig(**sess['detector'])
    det = FastDetector(cfg)
    radius = sess.get('ball',{}).get('radius',.08034)
    rs = sess['rs']
    ci = rs['streams']['color']['intrinsics']
    ci = tuple(ci[k] for k in ('fx','fy','ppx','ppy'))
    dp = th.stream('depth') if 'depth' in th.names else None
    if dp is not None:
        di = rs['streams']['depth']['intrinsics']
        di = tuple(di[k] for k in ('fx','fy','ppx','ppy'))
        ext = rs.get('extrinsics',{}).get('depth_to_color')
        R = np.array(ext['R']).reshape(3,3,order='F') if ext else np.eye(3)
        tr = np.array(ext['t']) if ext else np.zeros(3)
        depth = np.array(dp.frames)
    det.candidates(ArrayFrame(frames[0]),6)  # warm native code
    ms, cpu, detector_ms, hit_frames = [], [], [], []
    for rep in range(2):
        arc = arc2d.ArcTracker(arc2d.ArcCfg(ball_radius=radius,min_speed=speed))
        hits = []
        for i, frame in enumerate(frames):
            start, cp = time.perf_counter_ns(), time.process_time_ns()
            cs = det.candidates(ArrayFrame(frame),6)
            mid = time.perf_counter_ns()
            hit = arc.update(float(th.t[i]), cs, i)
            if hit and dp is not None:
                j = int(np.searchsorted(dp.stamps[:,0],th.stamps[i,0],side='right')-1)
                if j >= 0 and th.stamps[i,0]-dp.stamps[j,0] <= 40_000_000:
                    if arc2d.depth_in_front(hit['u'],hit['v'],hit['r_px'],depth[j],rs.get('depth_scale',.001),ci,di,R,tr,
                                          ci[0]*radius/max(hit['r_px'],.5)):
                        hit = None
            cpu.append((time.process_time_ns()-cp)*1e-6)
            ms.append((time.perf_counter_ns()-start)*1e-6)
            detector_ms.append((mid-start)*1e-6)
            if hit:
                hits.append(i)
        hit_frames.append(hits)
    return dict(session=td.parent.name,clip=td.name,min_speed=speed,frames=len(frames),
                repeats=2,wall_ms_p50_95_99=np.percentile(ms,[50,95,99]).tolist(),
                max_ms=max(ms),cpu_ms_mean=float(np.mean(cpu)),
                core_equivalent_60fps=float(np.mean(cpu)*.06),
                detector_ms_p50_95_99=np.percentile(detector_ms,[50,95,99]).tolist(),
                over_16_8ms=sum(v>16.8 for v in ms),hit_frames=hit_frames,
                review=th.meta.get('review'))


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('root',type=Path)
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--speed',type=float,default=1.2)
    a=ap.parse_args()
    a.out.parent.mkdir(parents=True, exist_ok=True)
    if not available():
        raise RuntimeError('Native detector required; refusing to benchmark a fallback')
    result=dict(host=platform.node(),machine=platform.machine(),python=sys.version,
                native_sha256=hashlib.sha256(SRC.read_bytes()).hexdigest(),
                arc_sha256=hashlib.sha256((HERE/'arc2d.py').read_bytes()).hexdigest(),
                limits=__doc__,clips=[])
    for sd in sorted(a.root.iterdir()):
        if not (sd/'session.json').exists():
            continue
        sess=S.load_session(sd)
        for td in S.throw_dirs(sd):
            if 'rs' not in sess:
                continue
            row=run(td,sess,a.speed)
            result['clips'].append(row)
            a.out.write_text(json.dumps(result,indent=2))
            print(sd.name,td.name,'p95',round(row['wall_ms_p50_95_99'][1],2),
                  'ms; triggers',len(row['hit_frames'][0]),flush=True)


if __name__=='__main__':
    main()
