from common import *

def case(sw):
    key='dummy_motor_case';m,E,path=start(sw,key,'Case Section')
    values=[('outside_radius',26.5,'AK40 body envelope radius'),('rear_face_distance',36.7,'Open rear face toward negative motor Z'),
            ('front_face_distance',3.7,'Front bearing face toward negative motor Z'),('cavity_radius',21.5,'Rear cavity radius'),
            ('cavity_end_distance',11.01,'Back of the front-bearing support web'),('bearing_seat_radius',7,'687 bearing outside radius'),
            ('bearing_shoulder_distance',8.7,'Front bearing outer-race shoulder'),('shaft_clearance_radius',4.5,'Clearance behind the bearing'),
            ('relief_inner_radius',10.5,'Inner radius of the output-boss sweep clearance'),('relief_outer_radius',16.5,'Outer radius of that clearance'),
            ('relief_end_distance',6.7,'Floor of the rotating-boss clearance recess'),('cover_screw_pcd',46.6,'Rear cover screw pitch-circle diameter'),
            ('cover_boss_diameter',6,'Reinforced printed cover-screw boss diameter'),('cover_boss_length',11,'Cover-screw boss length'),
            ('cover_pilot_diameter',2.8,'M3 thread-forming pilot'),('cover_pilot_depth',9,'Blind pilot depth; leaves a 2 mm floor'),
            ('rear_mount_pcd',47,'AK40 rear mounting pitch circle'),('rear_mount_clearance',6.3,'Clearance for the removable cover mounting bosses'),
            ('rear_mount_relief_depth',5.7,'Rear mounting boss insertion depth plus clearance')]
    for n,v,t in values:E.var(n,v,'mm',t)
    E.var('cover_first_angle',45,'deg','First cover screw from local +Y toward -X');E.var('cover_screw_count',4,'','Cover screw count')
    E.var('rear_first_angle',90,'deg','First rear mounting hole from local +Y toward -X');E.var('rear_screw_count',4,'','Rear mounting hole count')
    vs=[(21.5,36.7,'cavity_radius','rear_face_distance'),(26.5,36.7,'outside_radius','rear_face_distance'),
        (26.5,3.7,'outside_radius','front_face_distance'),(16.5,3.7,'relief_outer_radius','front_face_distance'),
        (16.5,6.7,'relief_outer_radius','relief_end_distance'),(10.5,6.7,'relief_inner_radius','relief_end_distance'),
        (10.5,3.7,'relief_inner_radius','front_face_distance'),(7,3.7,'bearing_seat_radius','front_face_distance'),
        (7,8.7,'bearing_seat_radius','bearing_shoulder_distance'),(4.5,8.7,'shaft_clearance_radius','bearing_shoulder_distance'),
        (4.5,11.01,'shaft_clearance_radius','cavity_end_distance'),(21.5,11.01,'cavity_radius','cavity_end_distance')]
    s=orthogonal_profile(m,E,'Top Plane','Case Section',vs,axis='vertical');full_revolve(m,E,s,'Case Revolve')
    make_axis(m,'Motor Axis','Top Plane','Right Plane');ref_plane(m,E,'Front Plane',-36.7,'Rear Cover Seat','rear_face_distance')
    polar_hole(m,E,'Rear Cover Seat','Cover Boss Sketch',23.3,45,6,'cover_screw_pcd / 2','cover_first_angle','cover_boss_diameter')
    f=boss_extrude(m,11,name='Cover Boss');link_feature(E,f,['cover_boss_length'])
    patterned(m,E,'Motor Axis',['Cover Boss'],4,'cover_screw_count','Cover Boss Pattern')
    polar_hole(m,E,'Rear Cover Seat','Cover Pilot Sketch',23.3,45,2.8,'cover_screw_pcd / 2','cover_first_angle','cover_pilot_diameter')
    f=cut_extrude(m,9,reverse=True,name='Cover Pilot Cut');link_feature(E,f,['cover_pilot_depth'])
    patterned(m,E,'Motor Axis',['Cover Pilot Cut'],4,'cover_screw_count','Cover Pilot Pattern')
    polar_hole(m,E,'Rear Cover Seat','Rear Mount Clearance Sketch',23.5,90,6.3,'rear_mount_pcd / 2','rear_first_angle','rear_mount_clearance')
    f=cut_extrude(m,5.7,reverse=True,name='Rear Mount Clearance Cut');link_feature(E,f,['rear_mount_relief_depth'])
    patterned(m,E,'Motor Axis',['Rear Mount Clearance Cut'],4,'rear_screw_count','Rear Mount Clearance Pattern')
    E.var('rotor_float_datum_distance',10,'mm','Offset axial datum makes the 0.3 mm rotor end-float direction unambiguous')
    ref_plane(m,E,'Front Plane',-10,'Rotor Axial Datum Plane','rotor_float_datum_distance')
    return finish(sw,m,E,key,path)

def rotor(sw):
    key='dummy_motor_rotor';m,E,path=start(sw,key,'Rotor Section')
    vals=[('flange_radius',18.5,'Output flange radius'),('flange_depth',3.5,'Output flange thickness'),('shaft_radius',3.45,'Printed shaft radius; 0.1 mm diametral clearance in 687 bearings'),
          ('shaft_end_distance',35.5,'Rear shaft end toward negative motor Z'),('inner_race_shoulder_radius',4.5,'Shoulder contacts only the front bearing inner race'),
          ('inner_race_shoulder_distance',3.7,'Front bearing inner-race contact face'),('retainer_pilot_radius',1.4,'M3 retainer thread-forming pilot radius'),
          ('retainer_pilot_depth',9,'Retainer pilot depth'),('output_screw_pcd',27,'AK40 output screw pitch-circle diameter'),('output_boss_diameter',5.5,'Reinforced printed output screw bosses'),
          ('output_boss_length',3,'Boss extension behind the output flange'),('output_pilot_diameter',2.1,'M2.5 printed thread-forming pilot')]
    for n,v,t in vals:E.var(n,v,'mm',t)
    E.var('output_first_angle',90,'deg','One AK40 output hole, measured from local +Y toward -X');E.var('output_screw_count',3,'','Output mounting screw count')
    vs=[(1.4,35.5,'retainer_pilot_radius','shaft_end_distance'),(3.45,35.5,'shaft_radius','shaft_end_distance'),
        (3.45,3.7,'shaft_radius','inner_race_shoulder_distance'),(4.5,3.7,'inner_race_shoulder_radius','inner_race_shoulder_distance'),
        (4.5,3.5,'inner_race_shoulder_radius','flange_depth'),(18.5,3.5,'flange_radius','flange_depth'),
        (18.5,0,'flange_radius',None),(0,0,None,None),(0,26.5,None,'shaft_end_distance - retainer_pilot_depth'),
        (1.4,26.5,'retainer_pilot_radius','shaft_end_distance - retainer_pilot_depth')]
    s=orthogonal_profile(m,E,'Top Plane','Rotor Section',vs,axis='vertical');full_revolve(m,E,s,'Rotor Revolve')
    make_axis(m,'Motor Axis','Top Plane','Right Plane');ref_plane(m,E,'Front Plane',-6.5,'Output Boss Ends','flange_depth + output_boss_length')
    polar_hole(m,E,'Output Boss Ends','Output Boss Sketch',13.5,90,5.5,'output_screw_pcd / 2','output_first_angle','output_boss_diameter')
    f=boss_extrude(m,3,name='Output Boss');link_feature(E,f,['output_boss_length'])
    patterned(m,E,'Motor Axis',['Output Boss'],3,'output_screw_count','Output Boss Pattern')
    polar_hole(m,E,'Front Plane','Output Pilot Sketch',13.5,90,2.1,'output_screw_pcd / 2','output_first_angle','output_pilot_diameter')
    cut_extrude(m,through=True,name='Output Pilot Cut')
    patterned(m,E,'Motor Axis',['Output Pilot Cut'],3,'output_screw_count','Output Pilot Pattern')
    return finish(sw,m,E,key,path)

if __name__=='__main__':
    sw=connect()
    try:
        for fn in (case,rotor):
            if not sys.argv[1:] or fn.__name__ in sys.argv[1:]:fn(sw)
    finally:sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,True)
