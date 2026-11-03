"""Bucket ball lab: gesture identity, tuning, causal 3D flight and recording.

python bucket_live.py --source realsense [--arducam]
python bucket_live.py --source synthetic --port 8086
One process owns the RealSense. GPIO is not used. Existing ball_view is retained.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import queue
import sys
import threading
import time
from types import SimpleNamespace

import numpy as np
from PIL import Image,ImageDraw

HERE=Path(__file__).resolve().parent
for path in (HERE.parent/'throw_track',HERE.parent/'realsense',HERE.parents[1]/'ros/src/omni_catch',HERE.parent/'src/omni_catch'):
    sys.path.insert(0,str(path))
import ball_view as B
from bucket_vision import observe,circle_proposals,blob_proposals,BlobProposer,CameraGuard
from bucket_workflow import GestureFinder,Tuner,colour_window
from bucket_record import Witness,DebugClip
from flight3d import Config,FlightDetector,trajectory,measurement,public_result
from omni_catch.detector import DetectorConfig
from omni_catch.frames import ArrayFrame


class BucketSource:
    name='realsense'
    has_depth=True
    exposure_max,gain_min,gain_max=8000,0.,128.

    def __init__(self,streams='color,depth,ir1,ir2'):
        import record_rs as R
        self.R=R
        args=SimpleNamespace(serial=None,size='848x480',fps=60,depth_size='848x480',depth_fps=90,
                             emitter='on',ir_exposure_us=1000)
        self.tap=None
        owner=self
        class CaptureRig(R.RealSenseRig):
            def _item(rig,f,host_ns):
                item=super()._item(f,host_ns)
                if owner.tap and rig._name_of(f)=='color':
                    owner.tap(item)
                return item
        self.rig=CaptureRig(args,streams.split(','),keep_s=2.4)
        try:
            self.calib=self.rig.start(2000,128,False,4200)
        except Exception:
            self.rig.close()
            raise
        self.intr=tuple(self.calib['streams']['color']['intrinsics'][k] for k in ('fx','fy','ppx','ppy'))
        self.di=tuple(self.calib['streams']['depth']['intrinsics'][k] for k in ('fx','fy','ppx','ppy'))
        ext=self.calib.get('extrinsics',{}).get('depth_to_color')
        self.rotation=np.array(ext['R']).reshape(3,3,order='F') if ext else np.eye(3)
        self.translation=np.array(ext['t']) if ext else np.zeros(3)
        self.applied=None
        self.skipped=0

    def read(self):
        it=self.rig.next_color(3.)
        while True:
            try:
                it=self.rig.cq.get_nowait()
                self.skipped+=1
            except queue.Empty:
                break
        ds=self.rig.rings['depth'].latest(8)
        # Never use a depth sample captured in the future.
        ds=[d for d in ds if 0<=it.t_ns-d.t_ns<=40_000_000]
        dep=ds[-1] if ds else None
        return B.Frame(it.arr,*self.intr,t=it.t_ns*1e-9,
                       age_ms=(time.time_ns()-it.t_ns)*1e-6,depth=dep.arr if dep else None,
                       depth_scale=self.calib.get('depth_scale',.001),
                       meta=dict(sensor_ns=it.t_ns,host_ns=it.host_ns,counter=it.counter,
                                 exposure_field_raw=it.info[0],exposure_field_units='D435 colour 100us units; optical check pending',
                                 configured_exposure_us=self.applied[0] if self.applied else 2000,
                                 gain=it.info[1],depth_sensor_ns=dep.t_ns if dep else None,
                                 depth_age_ms=(it.t_ns-dep.t_ns)*1e-6 if dep else None,
                                 skipped_to_latest=self.skipped))

    def set_controls(self,p):
        key=(p['exposure_us'],p['gain'],p['awb'],p.get('white_balance',4200))
        if key==self.applied:
            return
        self.rig._apply_options(key[0],key[1],key[2],key[3])
        self.applied=key

    def close(self):
        self.rig.close()


class ManagedRealSense:
    """Keep the web page available when the camera is absent or unplugged."""
    name='realsense'
    has_depth=True
    exposure_max,gain_min,gain_max=8000,0.,128.

    def __init__(self,streams):
        self.streams=streams
        self.driver=None
        self.controls=None
        self.error='Waiting for the RealSense camera'
        self.closed=threading.Event()
        self.generation=0
        self.tap=None

    def set_witness_tap(self,tap):
        self.tap=tap

    def __getattr__(self,name):
        driver=self.__dict__.get('driver')
        if driver is not None:
            return getattr(driver,name)
        raise AttributeError(name)

    def read(self):
        while not self.closed.is_set():
            try:
                if self.driver is None:
                    self.driver=BucketSource(self.streams)
                    self.driver.tap=self.tap
                    self.generation+=1
                    if self.controls:
                        self.driver.set_controls(self.controls)
                frame=self.driver.read()
                self.error=None
                return frame
            except (SystemExit,RuntimeError,queue.Empty) as e:
                if self.driver:
                    self.driver.close()
                    self.driver=None
                message='Camera unavailable. Check power/cable and the blue USB 3 port; reconnect to retry.'
                if message!=self.error:
                    print(message,flush=True)
                self.error=message
                self.closed.wait(2)
        raise RuntimeError('Camera source closed')

    def set_controls(self,p):
        self.controls=p
        if self.driver:
            self.driver.set_controls(p)

    def close(self):
        self.closed.set()
        if self.driver:
            self.driver.close()


class DemoSource:
    name='synthetic'
    has_depth=True
    exposure_max,gain_min,gain_max=8000,0.,128.
    def __init__(self,pace=True):
        self.k=0
        self.pace=pace
        self.start=time.monotonic()
        self.controls=dict(exposure_us=2000,gain=128)
        self.launch=None
        self.di=None
        self.rotation=self.translation=None
        self.profile='orange'
        self.calib={}

    def throw(self):
        self.launch=self.k/60
        phase=(self.launch%4)
        corners=np.array([[240,180],[400,180],[400,340],[240,340],[240,180]])
        n=int(phase)
        u,v=corners[n]+(phase-n)*(corners[n+1]-corners[n])
        self.launch_state=np.array([(u-320)/500*3,(v-240)/500*3,3.,0.,-2.5,-4.5])

    def read(self):
        t=self.k/60
        self.k+=1
        if self.pace:
            time.sleep(max(0,self.start+t-time.monotonic()))
        f,cx,cy=500.,320.,240.
        if self.launch is None:
            phase=(t%4)/4*4
            corners=np.array([[240,180],[400,180],[400,340],[240,340],[240,180]])
            n=int(phase)
            u,v=corners[n]+(phase-n)*(corners[n+1]-corners[n])
            z=3.
        else:
            tau=t-self.launch
            p=trajectory(self.launch_state,[tau],-9.81*Config().up,Config().drag)[0]
            u,v,z=measurement(p,(f,f,cx,cy))
        rgb=np.full((480,640,3),(95,105,110),np.uint8)
        depth=np.full((480,640),5000,np.uint16)
        r=f*.080345/max(z,.1)
        if z>.2 and 0<=u<640 and 0<=v<480:
            yy,xx=np.ogrid[:480,:640]
            mask=(xx-u)**2+(yy-v)**2<=r*r
            colours={'orange':(220,85,12),'yellow':(220,205,25),'purple':(95,45,115)}
            brightness=min(1.2,self.controls['exposure_us']/2000*self.controls['gain']/128)
            rgb[mask]=np.clip(np.array(colours[self.profile])*brightness,0,255)
            depth[mask]=max(1,int((z-.075)*1000))
        rgb[45:90,40:85]=(220,85,12)  # stationary decoy
        return B.Frame(rgb,f,f,cx,cy,t=t,age_ms=0,depth=depth,
                       meta=dict(counter=self.k,synthetic=True,configured_exposure_us=self.controls['exposure_us']))

    def set_controls(self,p):
        self.controls=p

    def close(self):
        pass


class BucketPipeline(B.Pipeline):
    def __init__(self,source,root,profile='orange'):
        params=B.Params(source)
        for item in [('white_balance','white balance K',2800,6500,100,4200,'camera'),
                     ('exposure_cap','tuning exposure cap us',500,8000,500,2000,'camera'),
                     ('mass_g','ball mass g (nominal)',10,300,1,73,'range'),
                     ('pitch_deg','bucket camera pitch deg',10,60,.1,30,'range')]:
            params.spec[item[0]]=item
            params.v[item[0]]=item[5]
        params.v.update(ball_mm=160.6908,exposure_us=2000,gain=128,awb=False,track=True,show_guides=False)
        super().__init__(source,params)
        self.profile=profile
        if hasattr(source,'profile'):
            source.profile=profile
        self.stage='SEARCHING'
        self.message='Move the ball around two large squares. Any comfortable pace.'
        self.gestures=GestureFinder()
        self.proposer=BlobProposer()
        self.target=None
        self.target_velocity=np.zeros(2)
        self.target_t=None
        self.tuner=None
        self.flight=None
        self.flight_status={}
        self.prediction={}
        self.last_prediction={}
        self.recorder=Witness(root)
        self.clip=DebugClip(root)
        self.root=Path(root)
        self.commands=queue.Queue()
        self.event_pending=None
        self.event_busy=False
        self.event_count=0
        self.event_error=None
        self.event_thread=None
        self.last_event=-1e9
        self.ready_hits=0
        self.profile_dir=Path.home()/'omni_ws/ball_view_settings'
        self.applied_signature=None
        self.source_generation=getattr(source,'generation',0)
        self.camera_guard=CameraGuard()
        self.aux=None
        self.aux_error=None
        self.aux_frame=None
        self.aux_stop=threading.Event()
        self.aux_thread=None
        if hasattr(source,'set_witness_tap'):
            source.set_witness_tap(self.capture_witness)

    def capture_witness(self,item):
        """Offload every captured colour frame before latest-frame processing skips."""
        if not self.recorder.enabled:
            return
        driver=self.source.driver
        if driver is None or not hasattr(driver,'intr'):
            return
        frame=B.Frame(item.arr,*driver.intr,t=item.t_ns*1e-9,
                      meta=dict(sensor_ns=item.t_ns,host_ns=item.host_ns,counter=item.counter,
                                exposure_field_raw=item.info[0],exposure_units='colour 100us field; optical check pending'))
        decision=self.bucket_status()
        decision['decision_age_s']=frame.t-self.latest.frame.t if self.latest else None
        self.recorder.offer(frame,decision)

    def command(self,verb,**kw):
        self.commands.put((verb,kw))

    def reset_workflow(self):
        self.stage='SEARCHING'
        self.message='Move the ball around two large squares. Any comfortable pace.'
        self.gestures=GestureFinder()
        self.target=None
        self.target_t=None
        self.target_velocity=np.zeros(2)
        self.tuner=None
        self.flight=None
        self.flight_status={}
        self.prediction={}
        self.last_prediction={}
        self.recorder.enabled=False
        self.ready_hits=0
        self.camera_guard=CameraGuard()
        self.event_pending=None
        if hasattr(self.source,'launch'):
            self.source.launch=None

    def _commands(self,frame):
        while True:
            try:
                verb,kw=self.commands.get_nowait()
            except queue.Empty:
                break
            if verb in ('reset','profile'):
                if verb=='profile' and kw.get('profile') in ('orange','yellow','purple'):
                    self.profile=kw['profile']
                    if hasattr(self.source,'profile'):
                        self.source.profile=self.profile
                self.reset_workflow()
            elif verb=='stop_recording':
                self.recorder.enabled=False
                self.stage='PAUSED'
                self.message='Recording paused. Identify the ball again to arm another attempt.'
            elif verb=='mark':
                if self.recorder.enabled and frame is not None:
                    self.request_event(frame.t,'manual attempt marker')
                else:
                    self.message='Wait for Ready and recording before marking an attempt.'
            elif verb=='retune' and self.target:
                self.begin_tuning()
            elif verb=='pick':
                if frame is None:
                    self.message='A live frame is needed to select the ball.'
                    continue
                u,v=kw['x']*frame.rgb.shape[1],kw['y']*frame.rgb.shape[0]
                # A click may pick a still ball: every spot's refine, plus the old contour proposals.
                cs=blob_proposals(frame.rgb,static_budget=48)+circle_proposals(frame.rgb)
                if cs:
                    c=min(cs,key=lambda c:(c[0]-u)**2+(c[1]-v)**2)
                    if math.hypot(c[0]-u,c[1]-v)<50:
                        self.target=c
                        self.target_t=frame.t
                        self.begin_tuning()
            elif verb=='demo_throw' and hasattr(self.source,'throw'):
                self.source.throw()

    def begin_tuning(self):
        self.stage='TUNING'
        self.message='Ball identified. Keep moving it; testing light and filter settings.'
        p=self.params.get()
        self.tuner=Tuner(p['exposure_cap'],start=dict(exposure_us=p['exposure_us'],gain=p['gain']))
        self.ready_hits=0
        self.recorder.enabled=False

    def colour_candidates(self,frame,p):
        cfg=self.cfg
        cfg.hue_deg=(p['hue_min'],p['hue_max'])
        cfg.min_sat,cfg.min_val=p['sat_min'],p['val_min']
        cfg.min_blob_px=int(p['min_blob_px'])
        cfg.size_gain=p['size_gain']
        cfg.coarse_stride=int(p['coarse_stride'])
        fr=ArrayFrame(frame.rgb)
        if p['hue_min']<=p['hue_max']:
            cs=self.det.candidates(fr,24)
        else:
            cs=[]
            for lo,hi in ((p['hue_min'],360),(0,p['hue_max'])):
                cfg.hue_deg=(lo,hi)
                cs.extend(self.det.candidates(fr,16))
            cfg.hue_deg=(p['hue_min'],p['hue_max'])
        return sorted(cs,key=lambda c:-c[3])[:32]

    def observation(self,frame,c):
        p=self.params.get()
        return observe(frame.t,c,(frame.fx,frame.fy,frame.ppx,frame.ppy),p['ball_mm']/2000,
                       frame.depth,frame.depth_scale,getattr(self.source,'di',None),
                       getattr(self.source,'rotation',None),getattr(self.source,'translation',None))

    def metadata(self):
        calib=getattr(self.source,'calib',{})
        params=self.params.get()
        stream=calib.get('streams',{}).get('color',{})
        camera=dict(name=self.source.name,width=stream.get('width',640),height=stream.get('height',480),
                    fps=stream.get('fps',60),exposure_us=params['exposure_us'],gain=params['gain'],
                    f_px=stream.get('intrinsics',{}).get('fx',500),fov_is_estimate=False)
        return dict(kind='bucket-live/1',profile=self.profile,
                    ball=dict(radius=self.params.get()['ball_mm']/2000,mass=self.params.get()['mass_g']/1000,
                              drag_cd=.5,source='same foam type as measured orange; individual balls unmeasured'),
                    parameters=params,rs=calib,detector=dict(asdict(self.cfg),hue_deg=[params['hue_min'],params['hue_max']]),streams=list(calib.get('streams',{})),
                    pre_s=.8,post_s=.8,
                    recording='RGB JPEG + decisions at every offered frame; raw event streams in tNNN',
                    geometry='bucket.json CAD, verify pose after bumps',camera=camera)

    def request_event(self,t,reason):
        if self.event_pending is None and not self.event_busy and t-self.last_event>1.7:
            self.event_pending=(t,reason)

    def _raw_event(self,t):
        if not self.event_pending or t<self.event_pending[0]+.8 or self.event_busy:
            return
        event_t,reason=self.event_pending
        self.event_pending=None
        self.last_event=event_t
        if not hasattr(self.source,'rig') or not self.recorder.path:
            return
        import record_rs as R
        import session as S
        snap=R.snapshot(self.source.rig,int(event_t*1e9),int(.8e9),int(.8e9))
        if self.aux is not None:
            aux=self.aux.window(int(event_t*1e9)-int(.8e9),int(event_t*1e9)+int(.8e9))
            if aux:
                snap['arducam']=R.arrays_of(aux)
        self.event_count+=1
        n=self.event_count
        self.event_busy=True
        path=self.recorder.path
        st=snap['color']['stamps']
        meta=dict(kind='realsense',reason=reason,
                  t_trigger_s=(event_t*1e9-int(st[0,0]))*1e-9,
                  trigger=int(np.searchsorted(st[:,0],event_t*1e9)),
                  profile=self.profile,decision=dict(self.flight_status))
        def write():
            try:
                S.save_throw(path,n,None,None,meta,streams=snap)
            except Exception as e:
                self.event_error=str(e)
            finally:
                self.event_busy=False
        self.event_thread=threading.Thread(target=write,daemon=False)
        self.event_thread.start()

    def process(self,frame,p=None):
        start=time.perf_counter()
        generation=getattr(self.source,'generation',0)
        if generation!=self.source_generation:
            self.reset_workflow()
            self.source_generation=generation
        self._commands(frame)
        p=self.params.get()
        signature=tuple(p[k] for k in ('hue_min','hue_max','sat_min','val_min','size_gain','ball_mm','mass_g','pitch_deg',
                                      'exposure_us','gain','white_balance','awb'))
        if self.stage in ('READY','CANDIDATE','FLIGHT') and signature!=self.applied_signature:
            self.stage='VERIFYING'
            self.ready_hits=0
            self.flight=None
            self.prediction={}
            self.message='Settings changed: rechecking the identified object before rearming.'
            self.recorder.enabled=False
        self.applied_signature=signature
        if self.stage in ('READY','CANDIDATE','FLIGHT'):
            guard=self.camera_guard.update(frame.rgb,self.target)
            if guard['moved']:
                self.stage='CAMERA_MOVED'
                self.message=guard['reason']
                self.flight=None
                self.prediction={}
            if self.recorder.error or self.recorder.dropped>self.recorder.session_start_dropped:
                self.message='Recording is losing frames or has failed. Pause and check storage before counting attempts.'
        selected=None
        cs=[]
        if self.stage=='SEARCHING':
            # The gesture's leading tracks stay proposed through a pause.
            lead=sorted((tr for tr in self.gestures.tracks if len(tr.trail)>=5),key=lambda tr:(-tr.loops,-len(tr.trail)))
            cs=self.proposer(frame.rgb,keep=[tr.point[:3] for tr in lead[:3]])
            # Known diameter/depth removes many labels, lamps and wall decorations.
            cs=[c for c in cs if self.observation(frame,c) is not None]
            found=self.gestures.update(frame.t,cs)
            if found:
                self.target=found.point
                self.target_t=frame.t
                self.target_velocity=np.zeros(2)
                self.begin_tuning()
        elif self.stage in ('TUNING','VERIFYING','READY','FLIGHT','CANDIDATE') and self.target:
            if self.stage=='TUNING':
                cs=self.proposer(frame.rgb,keep=self.target)
            else:
                cs=self.colour_candidates(frame,p)
                # Local full-resolution recovery avoids whole-frame coarse sampling misses.
                half=max(30.,3*self.target[2])
                pred=np.array(self.target[:2])+self.target_velocity*min(frame.t-self.target_t,.1)
                for lo,hi in ([(p['hue_min'],p['hue_max'])] if p['hue_min']<=p['hue_max'] else [(p['hue_min'],360),(0,p['hue_max'])]):
                    self.cfg.hue_deg=(lo,hi)
                    cs.extend(self.det._components(ArrayFrame(frame.rgb),pred[0]-half,pred[1]-half,pred[0]+half,pred[1]+half))
            pred=np.array(self.target[:2])+self.target_velocity*min(frame.t-self.target_t,.1)
            options=[c for c in cs if math.hypot(c[0]-pred[0],c[1]-pred[1])<max(30.,3*self.target[2])
                     and .3<c[2]/max(self.target[2],1)<3]
            if options:
                selected=min(options,key=lambda c:(c[0]-pred[0])**2+(c[1]-pred[1])**2)
                dt=max(frame.t-self.target_t,.001)
                vel=(np.array(selected[:2])-self.target[:2])/dt
                self.target_velocity=.4*self.target_velocity+.6*vel
                self.target,self.target_t=selected,frame.t
            elif frame.t-self.target_t>.25:
                self.stage='LOST'
                self.message='Lost the identified object. Use Identify again; recording remains available.'
                self.prediction={}
            if selected and self.stage=='TUNING':
                px=B.disc_pixels(frame.rgb,*selected[:2],.55*selected[2])
                window=colour_window(px)
                extras=30
                if window:
                    tuned=dict(p,**{k:window[k] for k in ('hue_min','hue_max','sat_min','val_min')})
                    coloured=self.colour_candidates(frame,tuned)
                    extras=sum(math.hypot(c[0]-selected[0],c[1]-selected[1])>max(10,selected[2]) for c in coloured)
                best=self.tuner.update(px,selected[2],extras)
                if self.tuner.done:
                    if best:
                        for key in ('hue_min','hue_max','sat_min','val_min','exposure_us','gain','white_balance','awb'):
                            self.params.set(key,best[key])
                        self.stage='VERIFYING'
                        self.message='Checking the chosen settings on the identified ball.'
                    else:
                        self.stage='INSUFFICIENT_LIGHT'
                        self.message='No reliable appearance at this exposure cap. Add light, raise the cap, or adjust controls and identify again.'
                else:
                    for key,value in self.tuner.current.items():
                        self.params.set(key,value)
            if self.stage=='VERIFYING':
                self.ready_hits=self.ready_hits+1 if selected and self.observation(frame,selected) else 0
                if self.ready_hits>=20 and not self.event_busy:
                    self.stage='READY'
                    self.message='Ready to throw. Recording is active; waiting for measured 3D free flight.'
                    config=Config(radius=p['ball_mm']/2000,mass=p['mass_g']/1000,pitch_deg=p['pitch_deg'])
                    self.flight=FlightDetector((frame.fx,frame.fy,frame.ppx,frame.ppy),config)
                    self.recorder.start(self.metadata())
                    if not self.recorder.enabled:
                        self.stage='STORAGE_FULL'
                        self.message=self.recorder.error or 'Recording could not be armed'
                    self.save_profile()
            if selected and self.flight and self.stage in ('READY','CANDIDATE','FLIGHT'):
                obs=self.observation(frame,selected)
                confirmed=self.flight.update(frame.t,[obs] if obs else [])
                tr=confirmed or self.flight.best
                if tr:
                    self.flight_status=public_result(tr.result)
                    if self.stage!='FLIGHT':
                        self.stage='CANDIDATE'
                        self.message=tr.result['reason']
                        if tr.result.get('speed',0)>1.2:
                            self.request_event(frame.t,'3D flight candidate (including rejected/uncertain)')
                if confirmed:
                    self.stage='FLIGHT'
                    self.message='Confirmed: measured 3D acceleration and forward observations support gravity + drag.'
                    self.prediction=self.predict_bucket(confirmed.result,frame)
                    self.last_prediction=dict(self.prediction,capture_t=frame.t)
                elif self.stage=='FLIGHT':
                    self.prediction={}
                    self.stage='CANDIDATE'
                    self.message='Flight evidence no longer holds; bucket verdict withdrawn.'
        self._raw_event(frame.t)
        ms=(time.perf_counter()-start)*1000
        self.ms.append(ms)
        self.t_frames.append(frame.t)
        self.hits.append(selected is not None)
        if frame.age_ms is not None:
            self.ages.append(frame.age_ms+ms)
        if selected:
            self.trail.append(selected[:2])
        self.last=selected
        self.seq+=1
        obs=self.observation(frame,selected) if selected else None
        size=depth=None
        if obs:
            zsize=frame.fx*p['ball_mm']/2000/max(obs.r,.5)
            size=dict(z=zsize,x=(obs.u-frame.ppx)/frame.fx*zsize,
                      y=(obs.v-frame.ppy)/frame.fy*zsize,z_1px=zsize*.5/max(obs.r-.5,.1),f_px=frame.fx)
            if obs.depth:
                depth=dict(z=obs.z,x=(obs.u-frame.ppx)/frame.fx*obs.z,y=(obs.v-frame.ppy)/frame.fy*obs.z,
                           fill=1.,sigma_z=obs.sigma_z,suggest_size_gain=p['size_gain']*zsize/obs.z,
                           size_vs_depth_pct=100*(zsize-obs.z)/obs.z)
        res=B.Result(self.seq,frame,selected,cs,[False]*len(cs),None,ms,size,depth,list(self.trail),p)
        res.bucket_snapshot=self.bucket_status()
        res.bucket_path=list(self.prediction.get('projected_path',[]))
        if not hasattr(self.source,'set_witness_tap'):
            self.recorder.offer(frame,res.bucket_snapshot)
        if self.clip.active:
            self.clip.offer_processed(self.clip_record(frame,res,cs,selected,ms),frame.t,frame.rgb,frame.depth)
        return res

    def clip_record(self,frame,res,cs,selected,ms):
        """One processed frame's decisions, for a debug clip."""
        snap=res.bucket_snapshot
        p=res.params
        def num(c):
            return [float(x) for x in c[:4]]
        return dict(seq=res.seq,t=frame.t,meta=frame.meta,process_ms=ms,
                    stage=snap['stage'],message=snap['message'],selected=num(selected) if selected else None,
                    candidates=[num(c) for c in cs[:16]],n_candidates=len(cs),
                    target=num(self.target) if self.target else None,target_velocity=self.target_velocity.tolist(),
                    gestures=[dict(id=g.id,loops=g.loops,point=[float(x) for x in g.point[:3]]) for g in self.gestures.tracks],
                    tuning=dict(trial=snap['tuning_trial'],total=snap['tuning_total']),ready_hits=self.ready_hits,
                    flight=snap['flight'],prediction=snap['prediction'],camera_guard=snap['camera_guard'],
                    skipped_for_processing=snap['skipped_for_processing'],
                    params={k:p[k] for k in ('hue_min','hue_max','sat_min','val_min','min_blob_px','size_gain',
                                            'exposure_us','gain','white_balance','awb') if k in p})

    def clip_command(self,verb):
        if verb=='clip_start':
            self.clip.start(dict(self.metadata(),knobs=self.params.get(),bucket=self.bucket_status(),
                                 started=time.strftime('%Y-%m-%dT%H:%M:%S')))
        else:
            self.clip.stop(dict(knobs=self.params.get(),bucket=self.bucket_status()))

    def predict_bucket(self,result,frame):
        # Use the measured CAD rim in depth frame, transform to colour frame.
        path=HERE.parent/'throw_track/bucket.json'
        cad=json.loads(path.read_text())
        cad['cad']['pitch_deg']=self.params.get()['pitch_deg']
        c=cad['cad']
        origin=c['origin_from_back_face_m']
        a=math.radians(c['pitch_deg'])
        s,co=math.sin(a),math.cos(a)
        mount=np.array([0.,c['camera_back_horizontal_m'],c['camera_back_height_m']])
        mount+=np.array([0.,co,s])*origin['forward']+np.array([origin['lateral'],0.,0.])+np.array([0.,-s,co])*origin['up']
        offset=np.array([0.,0.,c['rim_height_m']])-mount
        rim=np.array([offset[0],s*offset[1]-co*offset[2],co*offset[1]+s*offset[2]])
        rim_radius=c['rim_diameter_m']/2
        ball_radius=self.params.get()['ball_mm']/2000
        rim_up=np.array([0.,-co,s])
        R=getattr(self.source,'rotation',None)
        tr=getattr(self.source,'translation',None)
        R=np.eye(3) if R is None else R
        tr=np.zeros(3) if tr is None else tr
        center=R@rim+tr
        up=R@rim_up
        fit=result['fit']
        now=frame.t-fit['t0']
        ts=np.arange(max(now,0),max(now,0)+2,.008)
        points=trajectory(fit['state'],ts,fit['gravity'],fit['drag'])
        height=(points-center)@up
        crossing=np.flatnonzero((height[:-1]>=0)&(height[1:]<0))
        if not len(crossing):
            return dict(verdict='uncertain',reason='No descending rim-plane crossing within 2s')
        k=crossing[0]
        frac=height[k]/(height[k]-height[k+1])
        point=points[k]+frac*(points[k+1]-points[k])
        delta=point-center
        miss=float(np.linalg.norm(delta-up*np.dot(delta,up)))
        ttg=float(ts[k]+frac*.008-now)
        clearance=rim_radius-ball_radius-miss
        # Conservative model band, explicitly not a calibrated probability.
        band=.025+.03*ttg+result['score']*.01
        verdict='likely in' if clearance>band else ('likely out' if miss-rim_radius-ball_radius>band else 'rim / uncertain')
        return dict(verdict=verdict,radial_m=miss,clearance_m=clearance,time_to_go_s=ttg,
                    model_band_m=band,calibrated_probability=False,
                    reason='CAD geometry; conservative provisional band, not measured accuracy',
                    projected_path=measurement(points[:min(k+1,80)],(frame.fx,frame.fy,frame.ppx,frame.ppy))[:,:2].tolist())

    def bucket_status(self):
        loops=max([tr.loops for tr in self.gestures.tracks],default=0)
        result=dict(stage=self.stage,message=self.message,profile=self.profile,gesture_loops=loops,
                    target_identified=self.target is not None,ready=self.stage in ('READY','CANDIDATE','FLIGHT')
                    and not self.event_busy and not self.recorder.error
                    and self.recorder.dropped==self.recorder.session_start_dropped,
                    tuning_trial=self.tuner.index if self.tuner else 0,tuning_total=len(self.tuner.settings) if self.tuner else 0,
                    flight=self.flight_status,prediction={k:v for k,v in self.prediction.items() if k!='projected_path'},
                    last_prediction={k:v for k,v in self.last_prediction.items() if k!='projected_path'},
                    recording=self.recorder.status(),raw_events=self.event_count,raw_busy=self.event_busy,
                    raw_error=self.event_error,camera_guard=self.camera_guard.status,
                    skipped_for_processing=getattr(self.source,'skipped',0),
                    aux_available=self.aux_frame is not None,aux_error=self.aux_error,
                    aux_health=self.aux.health() if self.aux is not None else None,clip=self.clip.status())
        if getattr(self.source,'error',None):
            result.update(stage='CAMERA_UNAVAILABLE',message=self.source.error,ready=False,prediction={})
        return result

    def status(self):
        # Reuse the established controls and timing readouts, without the old
        # acquisition logic being allowed to choose the bucket's target.
        if getattr(self.source,'error',None):
            self._commands(None)
        out=super().status()
        out['bools']=[b for b in out['bools'] if b[0] in ('show_all','show_mask','show_trail','show_guides','click_picks','awb')]
        out['spec']=[s for s in out['spec'] if s[0]!='f_scale']
        out['bucket']=self.bucket_status()
        return out

    def save_profile(self):
        self.profile_dir.mkdir(parents=True,exist_ok=True)
        settings=dict(self.metadata(),knobs=self.params.get(),source=f'bucket_{self.profile}',
                      saved=time.strftime('%Y-%m-%dT%H:%M:%S'))
        path=self.profile_dir/f'bucket_{self.profile}.json'
        path.write_text(json.dumps(settings,indent=2))
        return dict(ok=True,path=str(path),settings=settings)

    def close(self):
        self.stop.set()
        self.aux_stop.set()
        if self.aux_thread:
            self.aux_thread.join(timeout=3)
        self.clip.close(dict(knobs=self.params.get()))
        self.recorder.close()
        if self.event_thread:
            self.event_thread.join(timeout=30)

    def start_arducam(self,size=(768,432),keep_s=2.4):
        """Arducam witness: the sensor's full-field 2304x1296 mode (~56 fps) scaled to `size` by the ISP into a RAM ring.
        Each raw event clip gets the ring's +-0.8 s as stream "arducam" (stamps: sensor time mapped to the host's wall
        clock, the D435's global_time domain; host arrival; counter). Uncalibrated, rolling shutter, not fused into 3D."""
        import record_rs as R
        self.aux=R.Ring(keep_s)
        def capture():
            source=None
            try:
                source=B.PiCamSource(size,mode='wide')
                source.set_controls(dict(exposure_us=4000,gain=4,awb=True))
                n=0
                while not self.aux_stop.is_set():
                    fr=source.read()
                    host=time.time_ns()
                    t_ns=host-int((fr.age_ms or 0)*1e6)
                    n+=1
                    self.aux.push(R.Item(t_ns,host,n,fr.rgb,(fr.meta.get('exposure_us') or 0,fr.meta.get('gain') or 0,0)))
                    self.aux_frame=fr
            except Exception as e:
                self.aux_error=f'{type(e).__name__}: {e}'
            finally:
                if source:
                    source.close()
        self.aux_thread=threading.Thread(target=capture,daemon=True)
        self.aux_thread.start()


BUCKET_PANEL='''<section id="bucket"><h2>Bucket experiment</h2>
<div id="stage" style="font-size:22px;font-weight:700">Starting</div><p id="instruction"></p>
<div id="lamps" style="line-height:2"></div>
<select id="profile"><option>orange</option><option>yellow</option><option>purple</option></select>
<button onclick="bucketCmd('reset')">Identify again</button><button onclick="bucketCmd('retune')">Retune</button>
<button onclick="bucketCmd('mark')">Mark attempt / save raw clip</button>
<button id="clipbtn" onclick="bucketCmd(this.dataset.on==='1'?'clip_stop':'clip_start')" style="font-weight:700">● Record clip</button>
<div id="clipinfo" style="font-size:13px"></div>
<button onclick="bucketCmd('stop_recording')">Pause recording</button>
<button onclick="bucketCmd('demo_throw')">Demo throw (synthetic only)</button>
<pre id="evidence" style="white-space:pre-wrap;font-size:12px"></pre>
<details id="auxpanel" hidden><summary>Arducam witness (uncalibrated, saved in every raw clip)</summary><img id="auximage" style="width:100%" alt="Arducam witness"></details>
<p class="note">Draw two large squares with the ball. Keep moving it during tuning. Click the ball to select it if gesture acquisition is ambiguous. A green identity is not a flight confirmation. Nominal shared foam dimensions: verify each ball.</p>
</section>'''
BUCKET_JS=r'''<script>
async function bucketCmd(verb,extra={}){await fetch('/bucket/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({verb,...extra})})}
document.getElementById('profile').onchange=e=>bucketCmd('profile',{profile:e.target.value});
setInterval(async()=>{try{const s=await(await fetch('/status')).json(),b=s.bucket;if(!b)return;
document.getElementById('stage').textContent=b.stage.replaceAll('_',' ');
document.getElementById('instruction').textContent=b.message;
document.getElementById('profile').value=b.profile;
const lamp=(yes,text)=>`<span style="color:${yes?'#4ad66d':'#8b95a5'}">${yes?'●':'○'} ${text}</span>`;
document.getElementById('lamps').innerHTML=[lamp(b.target_identified,'Ball identified'),lamp(b.ready,'Ready / recording armed'),lamp(b.stage==='FLIGHT','3D free flight confirmed'),lamp(b.prediction.verdict==='likely in','Predicted clean entry')].join('<br>');
const f=b.flight,p=b.prediction,r=b.recording;
document.getElementById('evidence').textContent=`Gesture: ${b.gesture_loops}/2 squares\nTune: ${b.tuning_trial}/${b.tuning_total}\n${f.reason||''}\n`+
(f.score!=null?`3D fit ${f.score.toFixed(2)}, forward test ${f.holdout.toFixed(2)}\nAcceleration ${f.acceleration.toFixed(1)} ± ${f.accel_sigma.toFixed(1)} m/s²; expected ${f.expected_acceleration.toFixed(1)}\nDepth observations: ${f.depth_points}\n`:'')+
`Bucket: ${p.verdict||'uncertain'}${p.time_to_go_s!=null?' · '+p.time_to_go_s.toFixed(2)+' s to go':''}\nRecording: ${r.active?'ON':'off'}, ${r.written} frames, ${r.dropped} queue losses\nRaw clips: ${b.raw_events} ${b.raw_busy?'(writing)':''}\n${r.error||b.raw_error||''}`;
document.getElementById('evidence').textContent+='\nSkipped for processing: '+b.skipped_for_processing+' (capture witness is independent)';
if(b.last_prediction.verdict)document.getElementById('evidence').textContent+='\nPrevious forecast: '+b.last_prediction.verdict+' (not a landing observation)';
const c=b.clip,cb=document.getElementById('clipbtn');if(c){cb.dataset.on=c.active?'1':'0';
cb.textContent=c.active?`■ Stop clip (${c.seconds.toFixed(0)} / ${c.max_s.toFixed(0)} s)`:'● Record clip';cb.style.color=c.active?'#ff3b30':'';
document.getElementById('clipinfo').textContent=c.name?`${c.active?'Recording':'Last clip'}: ${c.name} · ${c.processed} analysed frames, ${c.depth} depth, ${c.views} view${c.dropped?' · '+c.dropped+' lost':''}${c.queued?' · writing '+c.queued:''}${c.error?' · '+c.error:''}`:(c.error||'');}
if(b.aux_health)document.getElementById('evidence').textContent+=`\nArducam: ${b.aux_health.fps?b.aux_health.fps.toFixed(0):'-'} fps, max gap ${b.aux_health.max_gap_ms.toFixed(0)} ms${b.aux_error?' · '+b.aux_error:''}`;
if(b.aux_available){document.getElementById('auxpanel').hidden=false;const im=document.getElementById('auximage');if(document.getElementById('auxpanel').open)im.src='/aux.jpg?t='+Date.now();}
}catch(e){}},300);
</script>'''


def render(res,pipe):
    snapshot=getattr(res,'bucket_snapshot',pipe.bucket_status())
    arr=res.frame.rgb if res.params['view']=='color' or res.frame.depth is None else B.depth_colormap(res.frame.depth,res.frame.depth_scale)
    if res.params['view']=='color' and res.params['show_mask']:
        arr=arr.copy()
        cfg=DetectorConfig(hue_deg=(res.params['hue_min'],res.params['hue_max']),
                           min_sat=res.params['sat_min'],min_val=res.params['val_min'])
        if cfg.hue_deg[0]<=cfg.hue_deg[1]:
            mask=B.ball_mask(arr,cfg)
        else:
            lo,hi=cfg.hue_deg
            cfg.hue_deg=(lo,360)
            mask=B.ball_mask(arr,cfg)
            cfg.hue_deg=(0,hi)
            mask|=B.ball_mask(arr,cfg)
        arr[mask]=(.4*arr[mask]+.6*np.array([255,0,220])).astype(np.uint8)
    img=Image.fromarray(arr)
    dr=ImageDraw.Draw(img)
    if res.params['view']=='color':
        if res.params['show_trail'] and len(res.trail)>1:
            dr.line(res.trail,fill='#ffb347',width=2)
        if res.params['show_guides']:
            B.draw_guides(dr,res)
        for c in (res.cands if res.params['show_all'] else ([res.det] if res.det else [])):
            u,v,r=c[:3]
            main=res.det and math.hypot(u-res.det[0],v-res.det[1])<3
            dr.ellipse((u-r,v-r,u+r,v+r),outline='#4ad66d' if main else '#bbbbbb',width=2 if main else 1)
        if snapshot['stage']=='SEARCHING':
            moving=[tr for tr in list(pipe.gestures.tracks) if tr.moving]
            best=max(moving,key=lambda tr:(tr.loops,len(tr.trail)),default=None)
            for track in moving:
                lead=track is best
                if len(track.trail)>1:
                    dr.line(track.trail,fill='#00e5ff' if lead else '#5a8fa0',width=3 if lead else 1)
                u,v=track.point[:2]
                r=track.point[2]
                dr.ellipse((u-r,v-r,u+r,v+r),outline='#00e5ff' if lead else '#5a8fa0',width=2 if lead else 1)
                dr.text((u+r+4,v-6),f'#{track.id} squares {track.loops}/2',fill='#00e5ff' if lead else '#8fb8c4',
                        font=B.font(14 if lead else 11))
        path=getattr(res,'bucket_path',[])
        path=[tuple(p) for p in path if np.isfinite(p).all() and abs(p[0])<5000 and abs(p[1])<5000]
        if len(path)>1:
            dr.line(path,fill='#4ad6ff',width=2)
    dr.rectangle((0,0,img.width,45),fill='#111111')
    dr.text((6,3),f'{snapshot["stage"]}   {snapshot["profile"]}   frame {res.seq}   capture {res.frame.t:.3f}',fill='white',font=B.font(14))
    dr.text((6,23),snapshot['message'][:110],fill='#dddddd',font=B.font(12))
    if pipe.clip.active:
        dr.ellipse((img.width-22,8,img.width-8,22),fill='#ff3b30')
    if B.simplejpeg:
        data=bytes(B.simplejpeg.encode_jpeg(np.asarray(img),quality=80,colorspace='RGB'))
    else:
        import io
        buf=io.BytesIO()
        img.save(buf,'JPEG',quality=80)
        data=buf.getvalue()
    if pipe.clip.active:
        pipe.clip.offer_view(data,res.seq)
    return data


class Handler(B.Handler):
    def do_GET(self):
        if self.path.split('?')[0]=='/aux.jpg':
            frame=self.pipe.aux_frame
            if frame is None:
                return self._json(dict(error='Arducam witness not enabled or unavailable'),503)
            import io
            buf=io.BytesIO()
            Image.fromarray(frame.rgb).save(buf,'JPEG',quality=85)
            data=buf.getvalue()
            self.send_response(200)
            self.send_header('Content-Type','image/jpeg')
            self.send_header('Content-Length',str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        if self.path.split('?')[0]=='/':
            page=B.PAGE.replace('<title>Ball view</title>','<title>Bucket ball lab</title>')
            page=page.replace('<section><h2>Tracking</h2>',BUCKET_PANEL+'<section><h2>Tracking</h2>')
            page=page.replace('</body>',BUCKET_JS+'</body>')
            page=page.replace('<button id="cal">calibrate focal to known distance</button>','<button id="cal" hidden>calibrate focal to known distance</button>')
            data=page.encode()
            self.send_response(200)
            self.send_header('Content-Type','text/html; charset=utf-8')
            self.send_header('Content-Length',str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        return super().do_GET()

    def do_POST(self):
        if self.path=='/save':
            self.rfile.read(int(self.headers.get('Content-Length',0)))
            return self._json(self.pipe.save_profile())
        if self.path=='/load':
            self.rfile.read(int(self.headers.get('Content-Length',0)))
            path=self.pipe.profile_dir/f'bucket_{self.pipe.profile}.json'
            if not path.exists():
                return self._json(dict(ok=False,message='No saved profile for this ball yet'))
            saved=json.loads(path.read_text())
            for k,v in saved.get('knobs',{}).items():
                self.pipe.params.set(k,v)
            self.pipe.command('reset')
            return self._json(dict(ok=True,message='Profile loaded. Identify the ball again in the current scene.'))
        if self.path in ('/bucket/command','/pick'):
            try:
                body=json.loads(self.rfile.read(int(self.headers.get('Content-Length',0))) or b'{}')
                if self.path=='/pick':
                    self.pipe.command('pick',x=float(body['x']),y=float(body['y']))
                else:
                    verb=body.pop('verb')
                    if verb in ('clip_start','clip_stop'):
                        self.pipe.clip_command(verb)
                        return self._json(dict(ok=True,clip=self.pipe.clip.status()))
                    if verb not in ('reset','profile','retune','mark','stop_recording','demo_throw'):
                        raise ValueError('Unknown command')
                    self.pipe.command(verb,**body)
                return self._json(dict(ok=True))
            except (ValueError,KeyError) as e:
                return self._json(dict(ok=False,error=str(e)),400)
        return super().do_POST()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source',choices=['realsense','synthetic'],default='realsense')
    ap.add_argument('--profile',choices=['orange','yellow','purple'],default='orange')
    ap.add_argument('--port',type=int,default=8086)
    ap.add_argument('--display-fps',type=float,default=10)
    ap.add_argument('--streams',default='color,depth,ir1,ir2')
    ap.add_argument('--arducam',action='store_true',help='Arducam witness: full field at ~56 fps, 768x432, saved in each raw clip; never used as calibrated stereo')
    ap.add_argument('--record-root',type=Path,default=Path.home()/'omni_ws/bucket_sessions')
    a=ap.parse_args()
    if not {'color','depth'}.issubset(a.streams.split(',')):
        ap.error('Bucket flight requires color and depth streams')
    src=ManagedRealSense(a.streams) if a.source=='realsense' else DemoSource()
    pipe=BucketPipeline(src,a.record_root,a.profile)
    if a.arducam:
        pipe.start_arducam()
    pipe.settings_dir=None
    B.render=render
    Handler.pipe=pipe
    Handler.display_fps=a.display_fps
    server=B.ThreadingHTTPServer(('0.0.0.0',a.port),Handler)
    server.daemon_threads=True
    worker=threading.Thread(target=pipe.run,daemon=True)
    worker.start()
    import signal
    def shutdown(*_):
        threading.Thread(target=server.shutdown,daemon=True).start()
    signal.signal(signal.SIGINT,shutdown)
    signal.signal(signal.SIGTERM,shutdown)
    print(f'Bucket ball lab: http://{__import__("socket").gethostname()}.local:{a.port}/',flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        src.close()
        pipe.close()
        worker.join(timeout=4)


if __name__=='__main__':
    main()
