"""Bounded asynchronous witness recorder. Camera/detector never wait on disk.

Every offered RGB frame is JPEG encoded, and every offered decision is logged.
Queue losses are counted and displayed: this cannot promise lossless storage on
an overloaded SD card. The full-rate depth/IR rings are separate raw event clips.
"""
import json
from pathlib import Path
import queue
import shutil
import threading
import time
import uuid

import numpy as np
from PIL import Image

try:
    import simplejpeg
except ImportError:
    simplejpeg=None


class Witness:
    def __init__(self,root):
        self.root=Path(root)
        self.path=None
        self.queue=queue.Queue(maxsize=12)
        self.dropped=0
        self.written=0
        self.error=None
        self.enabled=False
        self.session_start_dropped=0
        self.free_gb=None
        self.thread=threading.Thread(target=self._write,daemon=True)
        self.thread.start()

    def start(self,metadata):
        if self.enabled:
            return
        self.root.mkdir(parents=True,exist_ok=True)
        self.free_gb=shutil.disk_usage(self.root).free/1e9
        if self.free_gb<1:
            self.error='Less than 1 GB free: recording not armed; originals retained'
            return
        self.path=self.root/(time.strftime('%Y%m%d_%H%M%S')+f'_{uuid.uuid4().hex[:12]}_bucket_live')
        self.path.mkdir(parents=True,exist_ok=False)
        (self.path/'witness').mkdir()
        (self.path/'session.json').write_text(json.dumps(metadata,indent=2))
        self.error=None
        self.session_start_dropped=self.dropped
        self.enabled=True

    def offer(self,frame,decision):
        if not self.enabled:
            return
        try:
            self.queue.put_nowait((self.path,frame,decision))
        except queue.Full:
            self.dropped+=1

    def _write(self):
        while True:
            item=self.queue.get()
            if item is None:
                self.queue.task_done()
                return
            path,frame,decision=item
            try:
                if self.written%60==0:
                    self.free_gb=shutil.disk_usage(path).free/1e9
                    if self.free_gb<1:
                        raise OSError('Less than 1 GB free: recording stopped; originals retained')
                k=self.written
                filename=f'witness/{k:07d}.jpg'
                if simplejpeg:
                    data=simplejpeg.encode_jpeg(frame.rgb,quality=88,colorspace='RGB')
                    (path/filename).write_bytes(data)
                else:
                    Image.fromarray(frame.rgb).save(path/filename,quality=88)
                with (path/'witness.jsonl').open('a') as f:
                    f.write(json.dumps(dict(frame=k,file=filename,t=frame.t,meta=frame.meta,
                                            decision=decision,dropped_total=self.dropped),default=float)+'\n')
                self.written+=1
            except Exception as e:
                self.error=f'{type(e).__name__}: {e}'
                self.enabled=False
            finally:
                self.queue.task_done()

    def status(self):
        return dict(active=self.enabled,path=str(self.path) if self.path else None,
                    written=self.written,dropped=self.dropped-self.session_start_dropped,
                    dropped_total=self.dropped,queued=self.queue.qsize(),error=self.error,
                    format='Full-rate RGB JPEG + decisions; raw depth/IR in event clips only',
                    free_gb=self.free_gb)

    def close(self):
        self.enabled=False
        self.queue.put(None)
        self.thread.join(timeout=20)


class DebugClip:
    """Operator-started clip for diagnosing the workflow in any stage (not only after Ready): every processed frame's
    colour image (JPEG: the analysis's exact input, so it replays) with its decisions, depth at <= depth_fps, the
    page's annotated view at <= view_fps, and the settings at start and stop. Not every captured frame: on the Pi 4
    the JPEG writer can't keep 60 fps beside the analysis (the Ready witness does that). Bounded queue, losses
    counted, the stop record never dropped; stops itself after max_s."""
    def __init__(self,root,max_s=90.,depth_fps=5.,view_fps=5.):
        self.root=Path(root)
        self.max_s,self.depth_dt,self.view_dt=max_s,1/depth_fps,1/view_fps
        self.path=None
        self.active=False
        self.started=None
        self.queue=queue.Queue(maxsize=48)
        self.dropped=self.colour=self.processed=self.depth=self.views=0
        self.last_depth=self.last_view=-1e9
        self.error=None
        self.thread=threading.Thread(target=self._write,daemon=True)
        self.thread.start()

    def start(self,metadata):
        if self.active:
            return
        self.root.mkdir(parents=True,exist_ok=True)
        if shutil.disk_usage(self.root).free<1.5e9:
            self.error='Less than 1.5 GB free: clip not started'
            return
        self.path=self.root/('clip_'+time.strftime('%Y%m%d_%H%M%S'))
        for sub in ('colour','depth','view'):
            (self.path/sub).mkdir(parents=True,exist_ok=True)
        (self.path/'start.json').write_text(json.dumps(metadata,indent=2,default=float))
        self.dropped=self.colour=self.processed=self.depth=self.views=0
        self.last_depth=self.last_view=-1e9
        self.error=None
        self.started=time.monotonic()
        self.active=True

    def stop(self,metadata=None,reason='operator'):
        if not self.active:
            return
        self.active=False
        try:
            self.queue.put((self.path,'end',dict(metadata or {},stop_reason=reason,
                                                 seconds=time.monotonic()-self.started,dropped=self.dropped)),timeout=10)
        except queue.Full:
            self.error='Clip writer stalled: end.json not written'

    def _put(self,item):
        try:
            self.queue.put_nowait((self.path,)+item)
        except queue.Full:
            self.dropped+=1

    def _check(self):
        if self.active and time.monotonic()-self.started>self.max_s:
            self.stop(reason=f'{self.max_s:.0f} s limit')
        return self.active

    def offer_processed(self,record,t,rgb,depth=None):
        if not self._check():
            return
        record=dict(record,colour_file=f'colour/{record["seq"]:07d}.jpg')
        if depth is not None and t-self.last_depth>=self.depth_dt:
            self.last_depth=t
            record['depth_file']=f'depth/{record["seq"]:07d}.npy'
        else:
            depth=None
        self._put(('processed',record,rgb,depth))

    def offer_view(self,jpeg,seq):
        now=time.monotonic()
        if self._check() and now-self.last_view>=self.view_dt:
            self.last_view=now
            self._put(('view',jpeg,seq,time.time_ns()))

    def _write(self):
        while True:
            path,kind,*rest=self.queue.get()
            try:
                if kind=='processed':
                    record,rgb,depth=rest
                    name=record['colour_file']
                    if simplejpeg:
                        (path/name).write_bytes(simplejpeg.encode_jpeg(rgb,quality=90,colorspace='RGB'))
                    else:
                        Image.fromarray(rgb).save(path/name,quality=90)
                    self.colour+=1
                    if depth is not None:
                        np.save(path/record['depth_file'],depth)
                        self.depth+=1
                    with (path/'processed.jsonl').open('a') as f:
                        f.write(json.dumps(record,default=float)+'\n')
                    self.processed+=1
                elif kind=='view':
                    jpeg,seq,host_ns=rest
                    name=f'view/{self.views:05d}.jpg'
                    (path/name).write_bytes(jpeg)
                    with (path/'view.jsonl').open('a') as f:
                        f.write(json.dumps(dict(file=name,seq=seq,host_ns=host_ns))+'\n')
                    self.views+=1
                elif kind=='end':
                    (path/'end.json').write_text(json.dumps(dict(rest[0],colour_frames=self.colour,
                                                                 processed_frames=self.processed,depth_frames=self.depth,
                                                                 view_frames=self.views),indent=2,default=float))
            except Exception as e:
                self.error=f'{type(e).__name__}: {e}'
                self.active=False
            finally:
                self.queue.task_done()

    def status(self):
        return dict(active=self.active,name=self.path.name if self.path else None,
                    seconds=time.monotonic()-self.started if self.active else None,max_s=self.max_s,
                    colour=self.colour,processed=self.processed,depth=self.depth,views=self.views,
                    dropped=self.dropped,queued=self.queue.qsize(),error=self.error)

    def close(self,metadata=None):
        self.stop(metadata,reason='lab stopped')
        self.queue.join()
