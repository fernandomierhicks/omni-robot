"""STLs from the checked native motor STEP, placed in printing orientation."""
from pathlib import Path
import build123d as b
ROOT=Path(__file__).resolve().parents[1]

if __name__=='__main__':
    folder=ROOT/'print';folder.mkdir(exist_ok=True)
    settings={'dummy_motor_case':b.Pos(0,0,-3.7)*b.Rot(180,0,0),
              'dummy_motor_rotor':b.Rot(180,0,0),
              'dummy_motor_rear_cap':b.Pos(0,0,40.2),
              'dummy_motor_retainer':b.Pos(0,0,36.7)}
    for name,loc in settings.items():
        shape=b.import_step(ROOT/'validation'/('V10_'+name+'.step')).moved(loc)
        if not shape.is_valid or len(shape.solids())!=1:raise RuntimeError('Invalid motor print '+name)
        b.export_stl(shape,folder/(name+'.stl'),tolerance=.025,angular_tolerance=.08)
        print(name,shape.bounding_box().min,shape.bounding_box().max)
