"""Timing only, not accuracy: full-frame vs 96px ROI Hough on existing Pi clips.

Uses installed OpenCV, one thread. RAM loading excluded; first call included. Fixed knobs
are an illustrative workload, not a tuned detector. Does not acquire a camera.
"""
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('root',type=Path)
    ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args()
    cv2.setNumThreads(1)
    rows=[]
    for session,clip in [('20261031_114236_loc1','t003'),('20261031_121159_loc2','t005')]:
        frames=np.load(a.root/session/clip/'frames.npy',mmap_mode='r')
        ims=[np.array(frames[i]) for i in range(40,59,2)]
        meta=json.loads((a.root/session/clip/'meta.json').read_text())
        centers=meta['live_blobs']
        for mode in ['full','roi96']:
            wall,cpu,n=[],[],[]
            for rep in range(2):
                for i,im in zip(range(40,59,2),ims):
                    if mode=='roi96':
                        # Recorder's largest blob: timing ROI only, not truth association.
                        cs=centers[i]
                        u,v=(cs[0][:2] if cs else (424,240))
                        x,y=int(np.clip(u-48,0,752)),int(np.clip(v-48,0,384))
                        im=im[y:y+96,x:x+96]
                    t,c=time.perf_counter_ns(),time.process_time_ns()
                    gray=cv2.cvtColor(im,cv2.COLOR_RGB2GRAY)
                    gray=cv2.GaussianBlur(gray,(5,5),1)
                    circles=cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1.5,16,
                                            param1=80,param2=20,minRadius=4,maxRadius=60)
                    wall.append((time.perf_counter_ns()-t)*1e-6)
                    cpu.append((time.process_time_ns()-c)*1e-6)
                    n.append(0 if circles is None else int(circles.shape[1]))
            rows.append(dict(session=session,clip=clip,mode=mode,samples=len(wall),
                             wall_ms_p50_95_99=np.percentile(wall,[50,95,99]).tolist(),
                             cpu_mean_ms=float(np.mean(cpu)),candidates_p50_95=np.percentile(n,[50,95]).tolist()))
            print(rows[-1],flush=True)
    a.out.parent.mkdir(parents=True,exist_ok=True)
    a.out.write_text(json.dumps(dict(opencv=cv2.__version__,threads=cv2.getNumThreads(),
                                   limits=__doc__,rows=rows),indent=2))


if __name__=='__main__':
    main()
