"""Independent nominal geometry checks for the 6804 support integration."""
from pathlib import Path
import sys,json,math,itertools
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'));sys.path.insert(0,str(ROOT.parent/'v9'))
import build123d as b
import pod_design as V
import pod as P, wheel as W, dims as D, hw

def volume(shape):return 0 if shape is None else shape.volume

def main():
    cache={}
    def native(key):
        if key not in cache:cache[key]=b.import_step(ROOT/'validation/human'/(key+'.step'))
        return cache[key]
    for method,key in {'inner_fork':'inner_fork_6804','outer_fork':'outer_fork_6804','motor_cup':'motor_cup','bench_plate':'bench_plate','pivot_shaft':'pivot_shaft','shock_case':'shock_case','shock_rod':'shock_rod','dummy_stator':'dummy_motor_stator','dummy_rotor':'dummy_motor_rotor','washer_large':'washer_M3_large','bearing_687':'bearing_687'}.items():
        setattr(V,method,lambda key=key:native(key))
    P.wheel_guard=lambda front=False:native('wheel_guard')
    P.pod_mount=lambda:native('pod_mount')
    P.fork_bar=lambda:native('fork_bar')
    W.hub=lambda which:native('wheel_hub_'+which)
    W.bearing_cartridge=lambda:native('wheel_cartridge')
    W.roller_core=lambda:native('roller_core')
    W.roller_tread=lambda:native('roller_tread')
    fixed={'guard':P.wheel_guard(),'mount':P.pod_mount(),'bench':V.bench_plate(),
           'shaft':V.pivot_shaft().moved(P.shaft_loc())}
    fork={'inner':V.inner_fork(),'outer':V.outer_fork(),'cup':V.motor_cup(),
          'bar':P.fork_bar().moved(P.fork_bar_loc())}
    wheel={k:s for k,s,_,_,_ in W.printed()}
    # Roller envelopes preserve the wheel-only verification's original contract.
    rollers={f'core_{row}_{j}':W.roller_core().moved(loc) for row in (0,1) for j,loc in enumerate(W.roller_locs(row))}
    rollers.update({f'tread_{row}_{j}':W.roller_tread().moved(loc) for row in (0,1) for j,loc in enumerate(W.roller_locs(row))})
    motor={'stator':V.dummy_stator(),'rotor':V.dummy_rotor(),'washer':V.washer_large().moved(b.Pos(0,0,-9.8)),
           'bearing':V.bearing_687().moved(b.Pos(0,0,-6.2))}
    report={'motor_pairs':[],'motor_spin':[],'fixed_pairs':[],'static_pairs':[],'travel':[],'shock_pairs':[],'solids':{}}
    for name,shape in {**fixed,**fork,**wheel,**motor}.items():
        report['solids'][name]={'count':len(shape.solids()),'valid':shape.is_valid}
    for (na,a),(nb,bb) in itertools.combinations(motor.items(),2):
        v=volume(a&bb)
        if v>.01:report['motor_pairs'].append([na,nb,v])
    for degrees in range(0,360,30):
        hits=[]
        rotor=motor['rotor'].moved(b.Rot(0,0,degrees))
        for key in ('stator','bearing'):
            v=volume(rotor&motor[key])
            if v>.01:hits.append([key,v])
        report['motor_spin'].append({'degrees':degrees,'clashes':hits})
    for (na,a),(nb,bb) in itertools.combinations(fixed.items(),2):
        v=volume(a&bb)
        if v>.01:report['fixed_pairs'].append([na,nb,v])
    for length in (40,45,50):
        v=volume(V.shock_case()&V.shock_rod().moved(b.Pos(0,0,length)))
        if v>.01:report['shock_pairs'].append([length,v])
    # Test wheel clearance against stationary fork and motor cup at nominal spin.
    for na,a in fork.items():
        for nb,bb in {**wheel,**rollers}.items():
            v=volume(a&bb)
            if v>.01:report['static_pairs'].append([na,nb,v])
    for na,a in fork.items():
        for nb,bb in motor.items():
            v=volume(a&bb.moved(P.MOTOR_LOC))
            if v>.01:report['static_pairs'].append([na,'motor_'+nb,v])
    for bump in (-3.,0.,7.5,15.):
        motion=P.arm_motion(bump);bad=[]
        for na,a in {**fork,**wheel}.items():
            aa=a.moved(motion)
            for nb,bb in fixed.items():
                v=volume(aa&bb)
                if v>.02:bad.append([na,nb,v])
        report['travel'].append({'bump_mm':bump,'clashes':bad,'inner_shock_mm':P._shock_len(*D.SHOCK_IN,bump),'outer_shock_mm':P._shock_len(*D.SHOCK_OUT,bump)})
    path=ROOT/'validation/human/design_geometry.json';path.write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2),flush=True)

if __name__=='__main__':main()

