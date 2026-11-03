"""Two-part printable AK40 dummy: stator and rotor, one 687 bearing, one M3 screw and fender washer.
Matches the AK40 drawing: 40.2 overall, Ø53 body whose front face is 1 mm behind the Ø37 rotor face, Ø15 centre recess,
3 x M2.5 on Ø47.5 (front), 3 x M2.5 on Ø27 (output), 4 x M2.5 on Ø47 (rear), and the flat where the power plug cable leaves."""
from common import *
from importlib import import_module
angle_plane=import_module('08_hubs').angle_plane

def stator(sw):
    key='dummy_motor_stator';m,E,path=start(sw,key,'Stator Section')
    values=[('outside_radius',26.5,'AK40 body envelope radius'),('front_face_distance',1,'Body front face; the rotor flange protrudes 1 mm ahead of it (AK40 drawing)'),
            ('rear_face_distance',40.2,'Rear mounting face; overall length from the rotor output face'),
            ('rotor_recess_radius',18.75,'Recess for the Ø37 rotor flange: 0.25 mm radial clearance'),
            ('bearing_front_distance',4.7,'Front bearing face and floor of the rotor recess'),('bearing_seat_radius',7,'687 bearing outside radius'),
            ('bearing_width',5,'687 bearing width'),('lip_bore_radius',5.5,'Bore behind the bearing: bears on the outer race, clears the washer and screw head'),
            ('cavity_floor_distance',11.7,'Floor of the hollow, leaves a 2 mm lip behind the bearing'),('cavity_radius',20,'Radius of the hollow opening to the rear; screw and tool access'),
            ('relief_inner_radius',10.5,'Inner radius of the output-boss sweep clearance'),('relief_outer_radius',16.5,'Outer radius of that clearance'),
            ('relief_end_distance',7.7,'Floor of the rotating-boss clearance recess'),('rear_mount_pcd',47,'AK40 rear mounting pitch circle'),
            ('rear_pilot_diameter',2.1,'M2.5 thread-forming pilot'),('rear_pilot_depth',8,'Blind rear pilot depth; 6 mm of M2.5 x 10 engagement'),
            ('front_mount_pcd',47.5,'AK40 front mounting pitch circle (3 x M2.5 x 5)'),('front_pilot_diameter',2.1,'M2.5 thread-forming pilot'),
            ('front_pilot_depth',6,'Blind front pilot depth (the real motor is M2.5 x 5)'),
            ('plug_flat_width',18.7,'Width of the flat where the power plug cable leaves (AK40 drawing)'),
            ('plug_flat_start',27,'Flat starts this far behind the output face'),('plug_flat_end',34.8,'Flat ends this far behind the output face'),
            ('plug_cut_width',24,'Width of the cutting sketch; wider than the flat, which ends where the plane leaves the Ø53 cylinder'),
            ('plug_cut_depth',4,'Depth of the cutting sketch outward from the flat')]
    for n,v,t in values:E.var(n,v,'mm',t)
    E.var('rear_first_angle',90,'deg','First rear mounting hole from local +Y toward -X');E.var('rear_screw_count',4,'','Rear mounting hole count')
    E.var('front_first_angle',60,'deg','First front mounting hole from local +Y toward -X (holes at model 30, 150, 270 deg)');E.var('front_screw_count',3,'','Front mounting hole count')
    E.var('plug_angle',315,'deg','Plug direction in the motor frame, from local +X toward +Y; midway between two rear holes (AK40 drawing: 45 deg from a hole)')
    vs=[(20,40.2,'cavity_radius','rear_face_distance'),(26.5,40.2,'outside_radius','rear_face_distance'),
        (26.5,1,'outside_radius','front_face_distance'),(18.75,1,'rotor_recess_radius','front_face_distance'),
        (18.75,4.7,'rotor_recess_radius','bearing_front_distance'),(16.5,4.7,'relief_outer_radius','bearing_front_distance'),
        (16.5,7.7,'relief_outer_radius','relief_end_distance'),(10.5,7.7,'relief_inner_radius','relief_end_distance'),
        (10.5,4.7,'relief_inner_radius','bearing_front_distance'),(7,4.7,'bearing_seat_radius','bearing_front_distance'),
        (7,9.7,'bearing_seat_radius','bearing_front_distance + bearing_width'),(5.5,9.7,'lip_bore_radius','bearing_front_distance + bearing_width'),
        (5.5,11.7,'lip_bore_radius','cavity_floor_distance'),(20,11.7,'cavity_radius','cavity_floor_distance')]
    s=orthogonal_profile(m,E,'Top Plane','Stator Section',vs,axis='vertical');full_revolve(m,E,s,'Stator Revolve')
    make_axis(m,'Motor Axis','Top Plane','Right Plane')
    ref_plane(m,E,'Front Plane',-40.2,'Rear Face Plane','rear_face_distance')
    polar_hole(m,E,'Rear Face Plane','Rear Mount Pilot Sketch',23.5,90,2.1,'rear_mount_pcd / 2','rear_first_angle','rear_pilot_diameter')
    f=cut_extrude(m,8,reverse=True,name='Rear Mount Pilot Cut');link_feature(E,f,['rear_pilot_depth'])
    patterned(m,E,'Motor Axis',['Rear Mount Pilot Cut'],4,'rear_screw_count','Rear Mount Pilot Pattern')
    ref_plane(m,E,'Front Plane',-1,'Front Face Plane','front_face_distance')
    polar_hole(m,E,'Front Face Plane','Front Mount Pilot Sketch',23.75,60,2.1,'front_mount_pcd / 2','front_first_angle','front_pilot_diameter')
    f=cut_extrude(m,6,name='Front Mount Pilot Cut');link_feature(E,f,['front_pilot_depth'])
    patterned(m,E,'Motor Axis',['Front Mount Pilot Cut'],3,'front_screw_count','Front Mount Pilot Pattern')
    # Flat for the power plug: a plane through the axis turned to the plug direction, offset so that it meets the Ø53 cylinder 18.7 mm wide.
    angle_plane(m,E,'Right Plane','Motor Axis',45,'360deg - plug_angle','Plug Direction Plane')
    ref_plane(m,E,'Plug Direction Plane',24.8,'Plug Flat Plane','sqr(outside_radius * outside_radius - plug_flat_width * plug_flat_width / 4)')
    vs=[(27,12,'plug_flat_start','plug_cut_width / 2'),(34.8,12,'plug_flat_end','plug_cut_width / 2'),(34.8,-12,'plug_flat_end','plug_cut_width / 2'),(27,-12,'plug_flat_start','plug_cut_width / 2')]
    s=orthogonal_profile(m,E,'Plug Flat Plane','Plug Flat Sketch',vs,axis=None)
    m.ClearSelection2(True);s.feat.Select2(False,0)
    f=cut_extrude(m,4,reverse=True,name='Plug Flat Cut');link_feature(E,f,['plug_cut_depth'])
    E.var('rotor_float_datum_distance',10,'mm','Offset datum from the output plane; the rotor end float is measured from it')
    ref_plane(m,E,'Front Plane',-10,'Rotor Axial Datum Plane','rotor_float_datum_distance')
    return finish(sw,m,E,key,path)

def rotor(sw):
    key='dummy_motor_rotor';m,E,path=start(sw,key,'Rotor Section')
    vals=[('flange_radius',18.5,'Output flange radius (AK40 drawing: Ø37)'),('flange_depth',4.5,'Output flange thickness; the first 1 mm protrudes from the stator'),
          ('shaft_radius',3.45,'Printed shaft radius; 0.1 mm diametral clearance in the 687 bearing'),
          ('inner_race_shoulder_radius',4.5,'Shoulder contacts only the bearing inner race'),('inner_race_shoulder_distance',4.7,'Bearing front face; rotor shoulder contact plane'),
          ('bearing_width',5,'687 bearing width'),('rotor_end_float',.3,'Axial end float between the shoulder and the washer'),
          ('center_recess_radius',7.5,'Central recess in the output face (AK40 drawing: Ø15)'),('center_recess_depth',1.1,'Depth of that recess'),
          ('screw_pilot_radius',1.4,'M3 thread-forming pilot radius'),('screw_pilot_floor_distance',2.2,'Pilot floor; leaves 1.1 mm behind the recess floor'),
          ('output_screw_pcd',27,'AK40 output screw pitch-circle diameter'),('output_boss_diameter',5.5,'Reinforced printed output screw bosses'),
          ('output_boss_length',3,'Boss extension behind the output flange'),('output_pilot_diameter',2.1,'M2.5 printed thread-forming pilot')]
    for n,v,t in vals:E.var(n,v,'mm',t)
    E.var('output_first_angle',90,'deg','One AK40 output hole, measured from local +Y toward -X');E.var('output_screw_count',3,'','Output mounting screw count')
    end='inner_race_shoulder_distance + bearing_width + rotor_end_float'
    vs=[(1.4,10,'screw_pilot_radius',end),(3.45,10,'shaft_radius',end),
        (3.45,4.7,'shaft_radius','inner_race_shoulder_distance'),(4.5,4.7,'inner_race_shoulder_radius','inner_race_shoulder_distance'),
        (4.5,4.5,'inner_race_shoulder_radius','flange_depth'),(18.5,4.5,'flange_radius','flange_depth'),
        (18.5,0,'flange_radius',None),(7.5,0,'center_recess_radius',None),(7.5,1.1,'center_recess_radius','center_recess_depth'),
        (0,1.1,None,'center_recess_depth'),(0,2.2,None,'screw_pilot_floor_distance'),(1.4,2.2,'screw_pilot_radius','screw_pilot_floor_distance')]
    s=orthogonal_profile(m,E,'Top Plane','Rotor Section',vs,axis='vertical');full_revolve(m,E,s,'Rotor Revolve')
    make_axis(m,'Motor Axis','Top Plane','Right Plane');ref_plane(m,E,'Front Plane',-7.5,'Output Boss Ends','flange_depth + output_boss_length')
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
        for fn in (stator,rotor):
            if not sys.argv[1:] or fn.__name__ in sys.argv[1:]:fn(sw)
    finally:sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,True)
