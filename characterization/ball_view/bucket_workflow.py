"""Gesture identity and camera-tuning state, independent of any camera driver."""
from dataclasses import dataclass,field
import math
import numpy as np


@dataclass
class GestureTrack:
    id:int
    point:tuple
    t:float
    path:list=field(default_factory=list)
    velocity:np.ndarray=field(default_factory=lambda:np.zeros(2))
    loops:int=0
    trail:list=field(default_factory=list)   # for display only: survives the path reset after each square

    @property
    def moving(self):
        return len(self.trail)>=3

    def add(self,t,c):
        dt=max(t-self.t,.001)
        self.velocity=.5*self.velocity+.5*(np.array(c[:2])-self.point[:2])/dt
        self.point,self.t=c,t
        if not self.trail or np.hypot(c[0]-self.trail[-1][0],c[1]-self.trail[-1][1])>max(3.,.2*c[2]):
            self.trail=(self.trail+[(float(c[0]),float(c[1]))])[-300:]
        if not self.path or np.linalg.norm(np.array(c[:2])-self.path[-1])>max(3.,.2*c[2]):
            self.path.append(np.array(c[:2],float))
        if len(self.path)>400:
            self.path=self.path[-400:]
        if len(self.path)<8:
            return
        xy=np.array(self.path)
        lo,hi=xy.min(axis=0),xy.max(axis=0)
        wh=hi-lo
        if min(wh)<max(45.,5*c[2]) or max(wh)/max(min(wh),1)>2.5:
            return
        # One lap = the path winds a full turn round its box's centre. (Nov 1: the old test wanted the path back
        # within a quarter of the box of its exact first point; real squares, sampled ~5 times a second, came back
        # 99 px off a 58 px limit while winding round twice.) Lines, scribbles and jitter wind ~0.
        rel=xy-(lo+hi)/2
        ang=np.unwrap(np.arctan2(rel[:,1],rel[:,0]))
        winding=abs(ang[-1]-ang[0])
        if winding<2*math.pi*.95:
            return
        norm=(xy-lo)/wh
        edge=np.minimum.reduce((norm[:,0],1-norm[:,0],norm[:,1],1-norm[:,1]))
        # Rounded corners and arbitrary pace are fine, repeated central scribbles aren't.
        if np.mean(edge)<.15:
            self.loops+=1
            self.path=[xy[-1]]


class GestureFinder:
    """Square-gesture tracks over the proposals. A track coasts through missed frames (up to `coast_s`) on its
    velocity, its gate growing with the gap, so a dropped frame doesn't restart the gesture (Nov 1: at ~7.5 fps
    analysed, two misses dropped a track and the next detection began a new one at 0 squares)."""
    def __init__(self,coast_s=1.0):
        self.tracks=[]
        self.next_id=1
        self.coast_s=coast_s

    def update(self,t,candidates):
        used=set()
        # Established tracks choose first, then the most recently seen.
        for track in sorted(self.tracks,key=lambda x:(-len(x.path),t-x.t)):
            gap=min(t-track.t,self.coast_s)
            # Half the velocity: a square's corner can come in the gap, so split the error between going straight
            # on and turning; the gate covers a 90-degree turn mid-gap.
            pred=np.array(track.point[:2])+.5*track.velocity*gap
            gate=max(30.,2.5*track.point[2])+1.2*float(np.linalg.norm(track.velocity))*gap
            options=[(np.linalg.norm(np.array(c[:2])-pred),k,c) for k,c in enumerate(candidates)
                     if k not in used and .4<c[2]/max(track.point[2],1)<2.5]
            if options:
                distance,k,c=min(options,key=lambda x:x[0])
                if distance<gate:
                    track.add(t,c)
                    used.add(k)
        self.tracks=[tr for tr in self.tracks if t-tr.t<self.coast_s]
        for k,c in enumerate(candidates):
            if k not in used and len(self.tracks)<40:
                self.tracks.append(GestureTrack(self.next_id,c,t,[np.array(c[:2],float)],
                                                trail=[(float(c[0]),float(c[1]))]))
                self.next_id+=1
        matches=[tr for tr in self.tracks if tr.loops>=2 and t-tr.t<.05]
        return matches[0] if len(matches)==1 else None


def colour_window(pixels):
    """Learn even a dim/desaturated purple ball, once identity is already known.

    A circular hue interval can wrap through red; the bucket's candidate detector
    evaluates the two intervals separately without changing robot defaults.
    """
    a=np.asarray(pixels,float)/255
    if len(a)<12:
        return None
    mx,mn=a.max(axis=1),a.min(axis=1)
    chroma=mx-mn
    valid=(mx>.035)&(chroma>.012)
    if valid.sum()<max(10,.25*len(a)):
        return None
    r,g,b=a.T
    den=np.maximum(chroma,1e-8)
    hue=np.where(mx==r,(60*(g-b)/den)%360,np.where(mx==g,60*((b-r)/den+2),60*((r-g)/den+4)))
    angles=np.deg2rad(hue[valid])
    center=math.degrees(math.atan2(np.sin(angles).mean(),np.cos(angles).mean()))%360
    delta=(hue-center+180)%360-180
    core=valid&(np.abs(delta)<40)
    if core.sum()<10:
        return None
    low,high=np.percentile(delta[core],[2,98])+np.array([-8,8])
    sat=chroma/np.maximum(mx,1e-8)
    return dict(hue_min=float((center+low)%360),hue_max=float((center+high)%360),
                sat_min=float(max(.035,.75*np.percentile(sat[core],5))),
                val_min=float(max(.025,.7*np.percentile(mx[core],5))),
                brightness=float(np.median(mx[core])),coverage=float(core.sum()/len(a)))


class Tuner:
    """Finite, repeatable settings trials, checked on the same locked object.

    Starts from the camera's current exposure and gain (the ball was just seen there) and tries the white balances;
    exposure x gain steps by 2x only while the ball's own median brightness is outside 0.3-0.75, toward 0.5, never
    above the exposure cap. (Nov 1: a fixed grid from 500 us / gain 64 upward, with a score that favoured short
    exposures, turned the room nearly black just after identification and the ball was lost.)"""
    WB=(4200,2800,5400)
    GAINS=(16,32,64,128)
    def __init__(self,cap=2000,start=None):
        start=start or {}
        self.cap=cap
        e=min(float(start.get('exposure_us',cap)),cap)
        g=float(start.get('gain',128))
        self.settings=[dict(exposure_us=e,gain=g,white_balance=w,awb=False) for w in self.WB]
        self.stepped=0
        self.index=0
        self.count=0
        self.samples=[]
        self.best=None
        self.brightness=[]

    def _step(self,factor):
        """The next exposure x gain toward `factor`, exposure first up to the cap; None at a limit."""
        cur=self.settings[self.index-1]
        e,g=cur['exposure_us'],cur['gain']
        if factor>1:
            if e<self.cap:
                e=min(self.cap,e*2)
            elif g<self.GAINS[-1]:
                g=min(self.GAINS[-1],g*2)
            else:
                return None
        else:
            if g>self.GAINS[0]:
                g=max(self.GAINS[0],g/2)
            elif e>250:
                e=e/2
            else:
                return None
        w=self.best['white_balance'] if self.best else cur['white_balance']
        return dict(exposure_us=e,gain=g,white_balance=w,awb=False)

    @property
    def current(self):
        return self.settings[min(self.index,len(self.settings)-1)]

    def update(self,pixels,radius,other_blobs):
        self.count+=1
        if self.count<5:  # controls settling; no assumption every frame has applied them
            return None
        tune=colour_window(pixels)
        if tune:
            score=tune['coverage']-min(other_blobs,30)*.015-abs(tune['brightness']-.5)*.3
            self.samples.append((score,tune))
        if self.count<8:
            return None
        if self.samples:
            score,tune=max(self.samples,key=lambda x:x[0])
            if self.best is None or score>self.best['score']:
                self.best=dict(score=score,**self.current,**tune)
            level=float(np.median([t['brightness'] for _,t in self.samples]))
        else:
            level=None
        self.index+=1
        self.count=0
        self.samples=[]
        # After the white balances, step exposure x gain while the ball is too bright or too dark (at most 3 steps).
        if self.index>=len(self.settings) and level is not None and self.stepped<3 and not .3<=level<=.75:
            nxt=self._step(2. if level<.3 else .5)
            if nxt:
                self.settings.append(nxt)
                self.stepped+=1
        return self.best if self.index>=len(self.settings) else None

    @property
    def done(self):
        return self.index>=len(self.settings)
