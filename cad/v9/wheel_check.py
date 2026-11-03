"""Wheel-only validation. Does not build or export either robot."""
import hashlib
import json
from pathlib import Path
from build123d import Compound, Pos
from OCP.BRepAdaptor import BRepAdaptor_Surface
import assembly as A
import build as B
import dims as D
import wheel as W


def robot():
    r=A.Robot()
    for name,s,mat,inf,orient in W.printed():
        r.add_printed(name,s,s,mat,inf,orient,'wheel','POD')
    for row in (0,1):
        for loc in W.roller_locs(row):
            for name,s,mat in [('roller_core',W.roller_core(),'PLA'),('roller_tread',W.roller_tread(),'TPU medium')]:
                r.add_printed(name,s,s.moved(loc),mat,100,'axis vertical','wheel','local')
    r.add_hw(W.hardware(),lambda s,loc:s.moved(loc),'wheel')
    return r


def report():
    r=robot()
    print('Wheel clashes',flush=True)
    out={'clashes':B.clashes(r.items),'solids':B.solids(r)}
    print('Wheel threads / access',flush=True)
    out['short']=[e for e in B.engagement(r.items,only=lambda i:i.size=="M3") if not e['ok']]
    out['access']=[e for e in B.access(r.items) if e['problems']]
    hub=Compound([W.hub(k) for k in 'ABC']+[W.bearing_cartridge()])
    out['external_motor_screws']={'count':3,'size':'M2.5x6','engagement_mm':2.5,'blind_depth_mm':3.0}
    out['end_float_mm']=W.END_FLOAT
    out['journal_length_mm']=W.JOURNAL_LENGTH
    out['minimum_required_body_clearance_mm']=W.MIN_BODY_CLEARANCE
    out['core_body_clearance_mm']={}
    out['tpu_clearance_mm']={}
    body=W._revolve(W._profile(core=True))
    tread_envelope=W._revolve(W._profile(tread=True))
    for row in (0,1):
        for j,loc in enumerate(W.roller_locs(row)):
            for shift in (-W.END_FLOAT/2,0,W.END_FLOAT/2):
                print('Roller clearance',row,j,shift,flush=True)
                at=loc*Pos(0,0,shift)
                key=f'{row}/{j}/{shift}'
                out['core_body_clearance_mm'][key]=body.moved(at).distance_to(hub)
                out['tpu_clearance_mm'][key]=tread_envelope.moved(at).distance_to(hub)
    out['root_fillets']={k:sum(f.geom_type.name=='CYLINDER' and abs(__import__('OCP.BRepAdaptor',fromlist=['BRepAdaptor_Surface']).BRepAdaptor_Surface(f.wrapped).Cylinder().Radius()-2)<1e-5 for f in W.hub(k).faces()) for k in 'ABC'}
    out['web_root_fillet_radius_mm']=W.WEB_ROOT_FILLET
    out['web_root_fillets']={k:sum(f.geom_type.name=='TORUS'
                                and abs(BRepAdaptor_Surface(f.wrapped).Torus().MinorRadius()-W.WEB_ROOT_FILLET)<1e-5
                                for f in W.hub(k).faces()) for k in 'ABC'}
    out['web_end_notches_filled']={f'{k}/{row}':all(W.hub(k).is_inside(
        (loc*Pos(side*depth,-13,sign*20.8)).position)
        for loc in W.roller_locs(row) for sign in (-1,1) for depth in (1,5,10))
        for k,row,side in [('A',0,-1),('B',0,1),('B',1,-1),('C',1,1)]}
    out['bearing_mm']=W.BRG
    print('Wheel wall scan',flush=True)
    import walls
    out['walls']=walls.report(r.printed)
    out['source_sha256']={n:hashlib.sha256(Path(__file__).with_name(n).read_bytes()).hexdigest() for n in ('wheel.py','wheel_test.py','wheel_check.py')}
    out['passed']=(not any(out[k] for k in ('clashes','solids','short','access'))
                   and min(out['core_body_clearance_mm'].values())>=W.MIN_BODY_CLEARANCE-1e-6
                   and min(out['tpu_clearance_mm'].values())>=W.MIN_BODY_CLEARANCE-1e-6
                   and all(out['root_fillets'].values())
                   and out['web_root_fillets']=={'A':12,'B':24,'C':12}
                   and all(out['web_end_notches_filled'].values()))
    return out


def main():
    j=report()
    Path(__file__).with_name('out').joinpath('wheel_checks_6804.json').write_text(json.dumps(j,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in j.items() if k!='walls'},indent=2),flush=True)
    raise SystemExit(0 if j['passed'] else 1)


if __name__=='__main__':
    main()
