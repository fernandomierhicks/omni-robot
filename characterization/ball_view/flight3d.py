"""Causal fixed-camera free-flight evidence, numpy only (no scipy/ROS).

Fits pixels AND independent range samples to p'' = g - k |v| v. Gravity's
direction is measured/configured, never fitted to make a lamp look ballistic.
The last observations are withheld when judging a candidate. A straight-motion
alternative, curvature observability and consecutive successful predictions
prevent a good-looking short 2D parabola from being called a throw.

This is the bucket bench's experimental gate, not a change to ROS catch defaults.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np


@dataclass
class Config:
    radius: float = .08034539414636606
    mass: float = .073
    drag_cd: float = .5                 # estimate, not measured
    pitch_deg: float = 30.
    min_span: float = .15
    max_span: float = .55
    min_points: int = 10
    max_gap: float = .065
    max_score: float = 2.3
    max_holdout: float = 3.0
    min_improvement: float = .25
    confirmations: int = 2
    min_depth: int = 4                  # size-only is tentative, never strong confirmation
    evaluate_every: int = 2

    @property
    def up(self):
        a=math.radians(self.pitch_deg)
        return np.array([0.,-math.cos(a),math.sin(a)])

    @property
    def drag(self):
        return 1.204*math.pi*self.radius**2*self.drag_cd/(2*self.mass)


@dataclass
class Observation:
    t: float
    u: float
    v: float
    r: float
    z: float
    sigma_z: float
    depth: bool = False
    sigma_px: float = 1.8


def trajectory(state, times, gravity, drag, native=True):
    """Vectorized midpoint integration, including batches of finite differences."""
    if native:
        from flight3d_native import path
        got=path(state,times,gravity,drag)
        if got is not None:
            return got
    states=np.atleast_2d(state).astype(float)
    times=np.asarray(times,float)
    pos,vel=states[:,:3].copy(),states[:,3:].copy()
    ans=np.empty((len(states),len(times),3))
    now=0.
    for i,target in enumerate(times):
        steps=max(1,int(math.ceil((target-now)/.012)))
        dt=(target-now)/steps
        for _ in range(steps):
            acc=gravity-drag*np.linalg.norm(vel,axis=1)[:,None]*vel
            vm=vel+.5*dt*acc
            pos+=dt*vm
            vel+=dt*(gravity-drag*np.linalg.norm(vm,axis=1)[:,None]*vm)
        ans[:,i]=pos
        now=target
    return ans[0] if np.asarray(state).ndim==1 else ans


def measurement(pos, intr):
    fx,fy,cx,cy=intr
    z=np.maximum(pos[...,2],.08)
    return np.stack((fx*pos[...,0]/z+cx,fy*pos[...,1]/z+cy,pos[...,2]),axis=-1)


def fit(obs, intr, cfg, kind='flight', native=True):
    ts=np.array([o.t-obs[0].t for o in obs])
    yy=np.array([[o.u,o.v,o.z] for o in obs])
    ss=np.array([[o.sigma_px,o.sigma_px,o.sigma_z] for o in obs])
    fx,fy,cx,cy=intr
    pts=np.column_stack(((yy[:,0]-cx)/fx*yy[:,2],(yy[:,1]-cy)/fy*yy[:,2],yy[:,2]))
    gravity=-9.81*cfg.up if kind=='flight' else np.zeros(3)
    drag=cfg.drag if kind=='flight' else 0.
    if native:
        from flight3d_native import solve
        raw=np.column_stack((ts,yy,ss[:,0],ss[:,2]))
        got=solve(raw,intr,gravity,drag)
        if got is not None:
            state,score=got
            return dict(state=state,t0=obs[0].t,score=score,gravity=gravity,drag=drag,
                        position=trajectory(state,ts[-1:],gravity,drag)[-1],times=ts)
    mat=np.column_stack((np.ones(len(ts)),ts))
    lin=np.linalg.lstsq(mat,pts-.5*ts[:,None]**2*gravity,rcond=None)[0]
    state=np.r_[lin[0],lin[1]]
    eps=np.array([.001,.001,.002,.01,.01,.01])

    def residual(states):
        pp=trajectory(states,ts,gravity,drag,native=False)
        return ((measurement(pp,intr)-yy)/ss).reshape(len(np.atleast_2d(states)),-1)

    for _ in range(5):
        batch=np.tile(state,(7,1))
        batch[1:]+=np.diag(eps)
        rr=residual(batch)
        jac=((rr[1:]-rr[0])/eps[:,None]).T
        weights=np.minimum(1.,2.5/np.maximum(np.abs(rr[0]),1e-6))
        normal=jac.T@(weights[:,None]*jac)+np.eye(6)*1e-5
        try:
            step=np.linalg.solve(normal,-jac.T@(weights*rr[0]))
        except np.linalg.LinAlgError:
            return None
        step*=min(1.,3./max(np.linalg.norm(step),1e-8))
        old=np.mean(np.minimum(rr[0]**2,25))
        accepted=False
        for factor in (1.,.4,.1):
            candidate=state+factor*step
            if candidate[2] < .15 or np.linalg.norm(candidate[3:])>25:
                continue
            r2=residual(candidate)[0]
            if np.mean(np.minimum(r2**2,25)) <= old:
                state=candidate
                accepted=True
                break
        if not accepted or np.linalg.norm(step)<.0005:
            break
    pp=trajectory(state,ts,gravity,drag,native=False)
    residuals=(measurement(pp,intr)-yy)/ss
    # Do not trim the wrong points until a lamp or a hand fits.
    score=float(np.sqrt(np.mean(np.minimum(residuals**2,100))))
    if not np.isfinite(score) or not np.all(np.isfinite(state)):
        return None
    return dict(state=state,t0=obs[0].t,score=score,residuals=residuals,
                gravity=gravity,drag=drag,position=pp[-1],times=ts)


def evidence(obs, intr, cfg):
    """Candidate evidence includes true forward prediction on withheld samples."""
    if len(obs)<cfg.min_points or obs[-1].t-obs[0].t<cfg.min_span:
        return dict(ok=False,reason='Need more time to distinguish flight from handling')
    if max(np.diff([o.t for o in obs]))>cfg.max_gap:
        return dict(ok=False,reason='Observation gap: flight not confirmed')
    prefix,held=obs[:-2],obs[-2:]
    flight=fit(prefix,intr,cfg)
    linear=fit(prefix,intr,cfg,'linear')
    if flight is None or linear is None:
        return dict(ok=False,reason='3D fit failed')
    tq=np.array([o.t-flight['t0'] for o in held])
    predicted=measurement(trajectory(flight['state'],tq,flight['gravity'],flight['drag']),intr)
    truth=np.array([[o.u,o.v,o.z] for o in held])
    sigma=np.array([[o.sigma_px,o.sigma_px,o.sigma_z] for o in held])
    holdout=float(np.sqrt(np.mean(((predicted-truth)/sigma)**2)))
    improvement=linear['score']-flight['score']
    depth=sum(o.depth for o in obs)
    speed=float(np.linalg.norm(flight['state'][3:]))
    # Measure acceleration independently of the constrained fit, in gravity's
    # transverse direction. Its range sensitivity is small near the mean ray.
    # Merely fitting forced gravity within a generous pixel tolerance is NOT
    # evidence that the observed object actually accelerates that way.
    fx,fy,cx,cy=intr
    rays=np.array([[(o.u-cx)/fx,(o.v-cy)/fy,1.] for o in obs])
    ray=np.mean(rays,axis=0)
    ray/=np.linalg.norm(ray)
    axis=cfg.up-ray*np.dot(cfg.up,ray)
    axis/=max(np.linalg.norm(axis),1e-8)
    z=np.array([o.z for o in obs])
    q=(rays*z[:,None])@axis
    var=np.array([(o.sigma_px*o.z)**2*((axis[0]/fx)**2+(axis[1]/fy)**2)
                  +(np.dot(axis,r)*o.sigma_z)**2+.003**2 for o,r in zip(obs,rays)])
    ts=np.array([o.t-obs[0].t for o in obs])
    design=np.column_stack((np.ones(len(ts)),ts,.5*ts*ts))
    cov=np.linalg.pinv(design.T@((1/var)[:,None]*design))
    coef=cov@design.T@(q/var)
    acceleration=float(coef[2])
    accel_sigma=float(math.sqrt(max(cov[2,2],0)))
    midv=flight['state'][3:]+.5*ts[-1]*flight['gravity']
    expected=float(axis@(flight['gravity']-cfg.drag*np.linalg.norm(midv)*midv))
    # Curvature is measured in the image as well as in noisy depth, so a
    # stationary/linear 3D object cannot pass just because the range is uncertain.
    result=dict(ok=False,score=flight['score'],linear_score=linear['score'],
                improvement=improvement,holdout=holdout,depth_points=depth,
                span=obs[-1].t-obs[0].t,speed=speed,fit=flight,
                acceleration=acceleration,accel_sigma=accel_sigma,expected_acceleration=expected)
    if speed<1.2 or speed>18:
        reason='Held/slow object, or implausible speed'
    elif accel_sigma>max(3.,.55*abs(expected)):
        reason='Curvature is not yet observable in 3D'
    elif abs(acceleration-expected)>max(3.,2*accel_sigma) or acceleration>-.40*abs(expected):
        reason='Measured 3D acceleration disagrees with gravity and drag'
    elif flight['score']>cfg.max_score or holdout>cfg.max_holdout:
        reason='Does not predict new 3D observations with gravity and drag'
    elif improvement<cfg.min_improvement:
        reason='Ordinary movement explains this as well as free flight'
    elif depth<cfg.min_depth:
        reason='Flight-like pixels, but insufficient independent depth evidence'
    else:
        reason='Gravity + drag predicts the next 3D observations'
        result['ok']=True
    result['reason']=reason
    return result


@dataclass
class Track:
    id: int
    obs: list = field(default_factory=list)
    miss: int = 0
    checks: int = 0
    positive: int = 0
    result: dict = field(default_factory=dict)

    def predict(self,t):
        if len(self.obs)<2:
            return np.array([self.obs[-1].u,self.obs[-1].v])
        a,b=self.obs[-2:]
        dt=max(b.t-a.t,.001)
        vel=np.array([b.u-a.u,b.v-a.v])/dt
        return np.array([b.u,b.v])+vel*min(t-b.t,.065)


class FlightDetector:
    def __init__(self,intr,cfg=None):
        self.intr=tuple(intr)
        self.cfg=cfg or Config()
        self.tracks=[]
        self.next_id=1
        self.best=None
        self.confirmed=None

    def reset(self):
        self.tracks=[]
        self.best=self.confirmed=None

    def update(self,t,observations,only_id=None):
        used=set()
        for tr in sorted(self.tracks,key=lambda x:-len(x.obs)):
            pred=tr.predict(t)
            best=None
            for k,o in enumerate(observations):
                if k in used:
                    continue
                distance=np.linalg.norm(pred-[o.u,o.v])
                gate=max(20.,2.2*tr.obs[-1].r)
                if distance<gate and (best is None or distance<best[0]):
                    best=(distance,k)
            if best is None:
                tr.miss+=1
                tr.positive=0
            else:
                used.add(best[1])
                tr.obs.append(observations[best[1]])
                tr.obs=[o for o in tr.obs if t-o.t<=self.cfg.max_span]
                tr.miss=0
        self.tracks=[tr for tr in self.tracks if tr.miss<=3 and t-tr.obs[-1].t<.12]
        for k,o in enumerate(observations):
            if k not in used and len(self.tracks)<12:
                self.tracks.append(Track(self.next_id,[o]))
                self.next_id+=1
        self.best=self.confirmed=None
        for tr in self.tracks:
            if tr.miss or len(tr.obs)<self.cfg.min_points or (only_id is not None and tr.id!=only_id):
                continue
            tr.checks+=1
            if tr.checks%self.cfg.evaluate_every:
                if tr.positive>=self.cfg.confirmations:
                    self.confirmed=tr
                continue
            uv=np.array([[o.u,o.v] for o in tr.obs[-28:]])
            depths=[o.z for o in tr.obs[-28:] if o.depth]
            if np.linalg.norm(np.ptp(uv,axis=0))<8 and (len(depths)<6 or np.ptp(depths)<.35):
                tr.positive=0
                continue
            # Three suffixes let the ball leave the hand; no backward/oracle growing.
            trials=[]
            for n in (12,18,28):
                if len(tr.obs)>=n:
                    r=evidence(tr.obs[-n:],self.intr,self.cfg)
                    if 'score' in r:
                        trials.append(r)
            if not trials:
                continue
            good=[r for r in trials if r['ok']]
            chosen=min(good or trials,key=lambda r:r['holdout']+r['score'])
            tr.result=chosen
            tr.positive=tr.positive+1 if chosen['ok'] else 0
            if self.best is None or chosen['score']<self.best.result.get('score',1e9):
                self.best=tr
            if tr.positive>=self.cfg.confirmations:
                self.confirmed=tr
        return self.confirmed


def public_result(result):
    return {k:float(v) if isinstance(v,np.floating) else v for k,v in result.items() if k!='fit'}
