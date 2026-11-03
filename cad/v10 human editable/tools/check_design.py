"""Independent nominal geometry checks for the 6804 support integration."""
from pathlib import Path
import sys,json,math,itertools
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT.parent/'v9'))
import build123d as b
import pod_design as V
import pod as P, wheel as W, dims as D, hw

def volume(shape):return 0 if shape is None else shape.volume

def main():
    fixed={'guard':P.wheel_guard(),'mount':P.pod_mount(),'bench':V.bench_plate(),
           'shaft':V.pivot_shaft().moved(P.shaft_loc())}
    fork={'inner':V.inner_fork(),'outer':V.outer_fork(),'cup':V.motor_cup(),
          'bar':P.fork_bar().moved(P.fork_bar_loc())}
    wheel={k:s for k,s,_,_,_ in W.printed()}
    # Roller envelopes preserve the wheel-only verification's original contract.
    rollers={f'core_{row}_{j}':W.roller_core().moved(loc) for row in (0,1) for j,loc in enumerate(W.roller_locs(row))}
    rollers.update({f'tread_{row}_{j}':W.roller_tread().moved(loc) for row in (0,1) for j,loc in enumerate(W.roller_locs(row))})
    motor={'case':V.dummy_case(),'cap':V.dummy_rear_cap(),'rotor':V.dummy_rotor(),'retainer':V.dummy_retainer(),
           'front_bearing':V.bearing_687().moved(b.Pos(0,0,-6.2)),
           'rear_bearing':V.bearing_687().moved(b.Pos(0,0,-32.7))}
    report={'motor_pairs':[],'motor_spin':[],'fixed_pairs':[],'static_pairs':[],'travel':[],'shock_pairs':[],'solids':{}}
    for name,shape in {**fixed,**fork,**wheel,**motor}.items():
        report['solids'][name]={'count':len(shape.solids()),'valid':shape.is_valid}
    for (na,a),(nb,bb) in itertools.combinations(motor.items(),2):
        v=volume(a&bb)
        if v>.01:report['motor_pairs'].append([na,nb,v])
    for degrees in range(0,360,30):
        hits=[]
        rotor=motor['rotor'].moved(b.Rot(0,0,degrees))
        for key in ('case','cap'):
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
    path=ROOT/'validation/design_geometry.json';path.write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2),flush=True)

if __name__=='__main__':main()
