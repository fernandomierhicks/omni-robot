"""Physical discrimination, gesture identity and end-to-end bench workflow."""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from flight3d import Config,Observation,FlightDetector,evidence,fit,measurement,trajectory
from bucket_workflow import GestureFinder,colour_window
from bucket_vision import observe

INTR=(616.,616.,424.,240.)


def observations(kind='flight',depth=True):
    cfg=Config()
    t=np.arange(24)/60
    p0=np.array([.2,.1,4.5])
    vel=np.array([.15,-2.4,-4.])
    if kind=='flight':
        pts=trajectory(np.r_[p0,vel],t,-9.81*cfg.up,cfg.drag)
    elif kind=='linear':
        pts=p0+t[:,None]*vel
    elif kind=='wrong_gravity':
        pts=p0+t[:,None]*vel+.5*t[:,None]**2*np.array([9.81,0,0])
    else:
        pts=p0+np.column_stack((.15*np.sin(10*t),.2*np.sin(9*t),np.zeros(len(t))))
    uv=measurement(pts,INTR)
    return [Observation(float(ti),u,v,616*cfg.radius/z,z,.035,depth,1.5)
            for ti,(u,v,z) in zip(t,uv)]


def test_true_drag_flight_predicts_withheld_3d_samples():
    result=evidence(observations(),INTR,Config())
    assert result['ok'],result
    assert result['holdout']<.1


@pytest.mark.parametrize('kind',['linear','wrong_gravity','hand'])
def test_rejects_motion_that_is_not_gravity_and_drag(kind):
    assert not evidence(observations(kind),INTR,Config())['ok']


def test_monocular_size_does_not_masquerade_as_independent_depth():
    result=evidence(observations(depth=False),INTR,Config())
    assert not result['ok']
    assert 'depth' in result['reason']


def test_short_arc_remains_uncertain():
    assert not evidence(observations()[:7],INTR,Config())['ok']


def test_gate_cannot_look_at_future_observations():
    detector=FlightDetector(INTR)
    confirms=[]
    for o in observations():
        if detector.update(o.t,[o]):
            confirms.append(o.t)
    assert confirms
    assert min(confirms)>=.18


def test_bad_new_range_withdraws_confirmation():
    detector=FlightDetector(INTR)
    os=observations()
    for o in os:
        detector.update(o.t,[o])
    for i in range(4):
        o=os[-1]
        detector.update(o.t+(i+1)/60,[Observation(o.t+(i+1)/60,o.u,o.v,o.r,8.,.035,True,1.5)])
    assert detector.confirmed is None


@pytest.mark.parametrize('kind',['flight','linear','wrong_gravity'])
def test_compiled_fitter_matches_numpy_reference(kind):
    import flight3d_native
    assert flight3d_native.load() is not None
    os=observations(kind)
    for model in ('flight','linear'):
        native=fit(os,INTR,Config(),model)
        reference=fit(os,INTR,Config(),model,native=False)
        assert np.allclose(native['state'],reference['state'],atol=1e-5)
        assert native['score']==pytest.approx(reference['score'],abs=1e-6)


def test_compiled_trajectory_matches_reference_and_closed_form():
    state=np.array([.2,.1,4.,1.,-2.,-3.])
    t=np.linspace(0,.8,50)
    g=-9.81*Config().up
    for drag in (0.,Config().drag):
        native=trajectory(state,t,g,drag)
        reference=trajectory(state,t,g,drag,native=False)
        assert np.allclose(native,reference,atol=1e-10)
        if not drag:
            expected=state[:3]+t[:,None]*state[3:]+.5*t[:,None]**2*g
            assert np.allclose(native,expected,atol=1e-10)


def test_front_foreground_patch_is_not_known_diameter_ball():
    # Tiny skin patch whose size says >6m, actually on a 1m foreground person.
    assert observe(0,(424,240,6),INTR,.0803,np.full((480,848),1000,np.uint16)) is None


def test_square_gesture_ignores_static_decoy_and_pace():
    finder=GestureFinder()
    corners=np.array([[200,120],[400,120],[400,320],[200,320],[200,120]])
    match=None
    t=0.
    for loop in range(2):
        for edge in range(4):
            for alpha in np.linspace(0,1,35):
                u,v=corners[edge]*(1-alpha)+corners[edge+1]*alpha
                t+=.01+.02*(edge%2)
                match=finder.update(t,[(u,v,12,400),(600,80,20,900)]) or match
    assert match is not None
    assert match.loops==2


def test_square_gesture_survives_missed_frames_at_live_rate():
    """~7.5 analysed fps with every third detection missing, two in a row now and then: one track, two squares."""
    finder=GestureFinder()
    corners=np.array([[200,120],[400,120],[400,320],[200,320],[200,120]])
    match=None
    t,k=0.,0
    ids=set()
    for loop in range(3):                     # the second square counts once it closes, a little into the third
        for edge in range(4):
            for alpha in np.linspace(0,1,9):
                if match:
                    break
                u,v=corners[edge]*(1-alpha)+corners[edge+1]*alpha
                t+=1/7.5
                k+=1
                seen=k%3!=0 and k%17 not in (1,2)
                cands=[(u,v,12,400)] if seen else []
                match=finder.update(t,cands+[(600,80,20,900)]) or match
                ids|={tr.id for tr in finder.tracks if tr.moving}
    assert match is not None and match.loops==2
    assert len(ids)==1


def test_tuner_starts_at_current_settings_and_only_steps_when_needed():
    from bucket_workflow import Tuner
    tuner=Tuner(2000,start=dict(exposure_us=2000,gain=128))
    assert tuner.current['exposure_us']==2000 and tuner.current['gain']==128
    def run(rgb):                               # the ball's pixels scale with exposure x gain
        tuner=Tuner(2000,start=dict(exposure_us=2000,gain=128))
        while not tuner.done:
            c=tuner.current
            k=c['exposure_us']*c['gain']/(2000*128)
            tuner.update(np.tile(np.clip(np.array(rgb)*k,0,255),(200,1)),10,0)
        return tuner
    tuner=run((230,104,23))                     # brightness 0.9: too bright -> one step darker (gain, not exposure)
    assert len(tuner.settings)==4
    assert tuner.settings[-1]['gain']==64 and tuner.settings[-1]['exposure_us']==2000
    assert run((120,55,12)).settings==run((120,55,12)).settings[:3]   # 0.47: no exposure trials
    assert len(run((120,55,12)).settings)==3


def test_stationary_and_linear_objects_do_not_identify_ball():
    finder=GestureFinder()
    for i in range(200):
        assert finder.update(i/60,[(100+i,200,10,300),(500,100,12,400)]) is None


def test_far_held_ball_is_proposed_in_real_frames():
    """Nov 1 clip: a 9 px-radius ball at 5.3 m touching an arm and a picture frame's edge. The old contour
    proposals found it within 12 px in under half the frames, at radius 4-5 px."""
    pytest.importorskip('cv2')
    from PIL import Image
    from bucket_vision import BlobProposer
    root=Path(__file__).parent/'test_data/nov1_far_ball'
    frames=json.loads((root/'frames.json').read_text())['frames']
    proposer=BlobProposer()
    found=[]
    for k,f in enumerate(frames):
        cs=proposer(np.asarray(Image.open(root/f['file'])))
        if k==0:
            continue                          # no background yet: nothing moves
        u,v=f['ball_orange_centroid']
        near=[c for c in cs[:8] if math.hypot(c[0]-u,c[1]-v)<6]
        found.append(bool(near))
        if near:
            assert 7<near[0][2]<11.5
    assert sum(found)>=len(found)-1,found


SQUARES_CLIP=Path('C:/Users/ferna/throw_data/bucket_clips/clip_20261101_114812')


@pytest.mark.skipif(not SQUARES_CLIP.exists(),reason='Nov 1 squares clip not on this machine (throw_data)')
def test_real_squares_clip_identifies_the_ball():
    """Fernando's Nov 1 squares, backlit, the dark ball against a dark shirt and a bright wall, ~5 analysed fps,
    a pause after the first square: replayed through proposals, the depth check and the gesture, the ball (not the
    lamp or a decoy) is identified."""
    pytest.importorskip('cv2')
    from PIL import Image
    from bucket_vision import BlobProposer
    rows=[json.loads(s) for s in (SQUARES_CLIP/'processed.jsonl').read_text().splitlines()]
    rs=json.loads((SQUARES_CLIP/'start.json').read_text())['rs']
    ci,di=rs['streams']['color']['intrinsics'],rs['streams']['depth']['intrinsics']
    intr,dintr=tuple(ci[k] for k in ('fx','fy','ppx','ppy')),tuple(di[k] for k in ('fx','fy','ppx','ppy'))
    ext=rs['extrinsics']['depth_to_color']
    R,T=np.array(ext['R']).reshape(3,3,order='F'),np.array(ext['t'])
    proposer,finder,depth,match=BlobProposer(),GestureFinder(),None,None
    for x in rows:
        if 'depth_file' in x:
            depth=np.load(SQUARES_CLIP/x['depth_file'])
        lead=sorted((tr for tr in finder.tracks if len(tr.trail)>=5),key=lambda tr:(-tr.loops,-len(tr.trail)))
        cs=[c for c in proposer(np.asarray(Image.open(SQUARES_CLIP/x['colour_file'])),keep=[tr.point[:3] for tr in lead[:3]])
            if observe(x['t'],c,intr,.0803,depth,.001,dintr,R,T) is not None]
        match=finder.update(x['t'],cs)
        if match:
            break
    assert match is not None
    rgb=np.asarray(Image.open(SQUARES_CLIP/x['colour_file'])).astype(int)
    u,v,r=match.point[:3]
    disc=rgb[int(v)-int(r/2):int(v)+int(r/2),int(u)-int(r/2):int(u)+int(r/2)].reshape(-1,3).mean(axis=0)
    assert disc[0]>1.5*disc[2] and 12<r<24        # an orange-ish disc of the ball's size, not the lamp


@pytest.mark.parametrize('rgb',[(220,80,12),(220,205,25),(65,40,60),(100,93,90)])
def test_identified_dim_ball_can_be_tuned_without_saturation_assumption(rgb):
    assert colour_window(np.tile(rgb,(100,1))) is not None


@pytest.mark.parametrize('profile',['orange','yellow','purple'])
def test_full_gesture_tune_ready_flight_and_recording(tmp_path,profile):
    pytest.importorskip('cv2')
    import bucket_live as L
    src=L.DemoSource(pace=False)
    pipe=L.BucketPipeline(src,tmp_path,profile=profile)
    pipe.profile_dir=tmp_path/'settings'
    try:
        for _ in range(700):
            pipe.process(src.read())
            src.set_controls(pipe.params.get())
            if pipe.stage=='READY':
                break
        assert pipe.stage=='READY'
        assert pipe.recorder.enabled
        src.throw()
        stages=[]
        verdicts=[]
        for _ in range(40):
            pipe.process(src.read())
            stages.append(pipe.stage)
            if pipe.stage=='FLIGHT':
                verdicts.append(pipe.prediction.get('verdict'))
        assert 'FLIGHT' in stages
        assert verdicts and set(verdicts)<={'likely in','likely out','rim / uncertain','uncertain'}
    finally:
        pipe.close()
    path=next(p for p in tmp_path.iterdir() if p.name!='settings')
    assert (path/'session.json').exists()
    rows=[json.loads(s) for s in (path/'witness.jsonl').read_text().splitlines()]
    assert rows
    assert any(r['decision']['stage']=='FLIGHT' for r in rows)


def test_profile_and_status_work_while_camera_is_absent(tmp_path):
    import bucket_live as L
    source=L.DemoSource(pace=False)
    source.error='Camera unplugged'
    pipe=L.BucketPipeline(source,tmp_path)
    try:
        pipe.command('profile',profile='purple')
        status=pipe.status()['bucket']
        assert status['profile']=='purple'
        assert status['stage']=='CAMERA_UNAVAILABLE'
        assert not status['ready']
        assert not status['recording']['active']
    finally:
        pipe.close()


def test_bucket_browser_script_parses():
    import shutil,subprocess
    import bucket_live as L
    node=shutil.which('node')
    if node is None:
        pytest.skip('Node unavailable for the page syntax check')
    script=L.BUCKET_JS.removeprefix('<script>').removesuffix('</script>')
    r=subprocess.run([node,'--check'],input=script,text=True,encoding='utf-8',capture_output=True)
    assert r.returncode==0,r.stderr


def test_recording_profile_switch_keeps_queued_frames_in_original_session(tmp_path):
    import ball_view as B
    from bucket_record import Witness
    recorder=Witness(tmp_path)
    frame=B.Frame(np.full((16,16,3),100,np.uint8),100,100,8,8)
    recorder.start(dict(profile='orange'))
    first=recorder.path
    recorder.offer(frame,dict(profile='orange'))
    recorder.enabled=False
    recorder.start(dict(profile='purple'))
    second=recorder.path
    recorder.offer(frame,dict(profile='purple'))
    recorder.close()
    assert first!=second
    a=json.loads((first/'witness.jsonl').read_text())
    b=json.loads((second/'witness.jsonl').read_text())
    assert a['decision']['profile']=='orange'
    assert b['decision']['profile']=='purple'


def test_debug_clip_records_any_stage_with_decisions_depth_and_view(tmp_path):
    pytest.importorskip('cv2')
    import bucket_live as L
    src=L.DemoSource(pace=False)
    pipe=L.BucketPipeline(src,tmp_path)
    pipe.profile_dir=tmp_path/'settings'
    try:
        pipe.clip_command('clip_start')
        assert pipe.clip.active
        for _ in range(30):
            res=pipe.process(src.read())
            L.render(res,pipe)
        pipe.clip_command('clip_stop')
        assert not pipe.clip.active
    finally:
        pipe.close()
    clip=next(tmp_path.glob('clip_*'))
    rows=[json.loads(s) for s in (clip/'processed.jsonl').read_text().splitlines()]
    assert len(rows)==30 and rows[0]['stage']=='SEARCHING'
    assert len(list((clip/'colour').glob('*.jpg')))==30
    assert rows[0]['depth_file'] and np.load(clip/rows[0]['depth_file']).dtype==np.uint16
    assert len(list((clip/'view').glob('*.jpg')))>=1
    end=json.loads((clip/'end.json').read_text())
    assert end['stop_reason']=='operator' and end['processed_frames']==30
    assert (clip/'start.json').exists()


def test_raw_attempt_keeps_pretrigger_frames_and_finishes_on_close(tmp_path):
    from types import SimpleNamespace
    import bucket_live as L
    import record_rs as R
    source=L.DemoSource(pace=False)
    rings={name:R.Ring(3.) for name in ('color','depth')}
    for k in range(22):
        ns=k*100_000_000
        for name,ring in rings.items():
            shape=(16,16,3) if name=='color' else (16,16)
            arr=np.full(shape,80 if name=='color' else 2000,np.uint8 if name=='color' else np.uint16)
            ring.push(R.Item(ns,ns+1_000_000,k+1,arr,(1000.,16.,0.)))
    source.rig=SimpleNamespace(rings=rings)
    pipe=L.BucketPipeline(source,tmp_path)
    pipe.aux=R.Ring(3.)                      # the Arducam witness ring, at its own (offset) times
    for k in range(40):
        ns=50_000_000*k+7_000_000
        pipe.aux.push(R.Item(ns,ns+2_000_000,k+1,np.full((9,16,3),k,np.uint8),(4000.,4.,0.)))
    pipe.recorder.start(pipe.metadata())
    session=pipe.recorder.path
    pipe.request_event(1.,'manual attempt marker')
    pipe._raw_event(1.81)
    pipe.close()  # must finish the asynchronous numpy writer, not truncate it
    stamps=np.load(session/'t001/stamps.npy')
    assert stamps[0,0]==200_000_000
    assert stamps[-1,0]==1_800_000_000
    assert np.load(session/'t001/depth.npy').shape[0]==len(stamps)
    aux=np.load(session/'t001/arducam_stamps.npy')
    assert aux[0,0]==207_000_000 and aux[-1,0]==1_757_000_000
    assert np.load(session/'t001/arducam.npy').shape==(len(aux),9,16,3)
    assert json.loads((session/'t001/meta.json').read_text())['reason']=='manual attempt marker'
