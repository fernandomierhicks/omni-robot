from common import *

def shaft(sw):
    key='pivot_shaft';m,E,path=start(sw,key,'Shaft Sketch')
    for name,value,comment in [('shaft_diameter',7,'Owned 627 bearing bore'),('shaft_length',98,'Overall shaft length'),
                               ('pin_from_end',86.25,'Cross-hole centre measured from the outer shaft end'),('pin_hole_diameter',3.4,'M3 pin clearance')]:E.var(name,value,'mm',comment)
    s=Sketch(m,'Right Plane','Shaft Sketch');cc=s.circle(0,0,3.5);s.rel('sgCOINCIDENT',cc.GetCenterPoint2(),origin=True);s.ddim(cc,'shaft_diameter',(7,7));s.finish(E)
    m.ClearSelection2(True);s.feat.Select2(False,0);f=boss_extrude(m,98,mid=True,name='Shaft Extrude');link_feature(E,f,['shaft_length'])
    s=Sketch(m,'Front Plane','Cross Hole Sketch');cc=s.circle(37.25,0,1.7)
    s.hdim(cc.GetCenterPoint2(),None,'pin_from_end - shaft_length / 2',(37.25,-5),origin_b=True)
    s.rel('sgHORIZONTALPOINTS2D',cc.GetCenterPoint2(),origin=True);s.ddim(cc,'pin_hole_diameter',(42,5));s.finish(E)
    m.ClearSelection2(True);s.feat.Select2(False,0);f=cut_extrude(m,7,mid=True,name='Cross Hole Cut');link_feature(E,f,['shaft_diameter'])
    return finish(sw,m,E,key,path)

def retainer(sw):
    key='dummy_motor_retainer';m,E,path=start(sw,key,'Retainer Sketch')
    for n,v,t in [('outside_diameter',9,'Retainer outside diameter; contacts the bearing inner race'),('screw_clearance',3.4,'M3 clearance'),
                  ('retainer_thickness',1.2,'Retainer thickness'),('rear_face_distance',36.7,'Rear face measured toward negative motor Z')]:E.var(n,v,'mm',t)
    s=Sketch(m,'Front Plane','Retainer Sketch')
    for r,expr in [(4.5,'outside_diameter'),(1.7,'screw_clearance')]:
        cc=s.circle(0,0,r);s.rel('sgCOINCIDENT',cc.GetCenterPoint2(),origin=True);s.ddim(cc,expr,(r+4,4))
    s.finish(E);m.ClearSelection2(True);s.feat.Select2(False,0)
    f=boss_extrude(m,1.2,start_offset=36.7,flip_start=True,name='Retainer Extrude');link_feature(E,f,['retainer_thickness','rear_face_distance'])
    return finish(sw,m,E,key,path,math.pi*(4.5**2-1.7**2)*1.2)

def cartridge(sw):
    key='wheel_cartridge';m,E,path=start(sw,key,'Cartridge Section')
    values=[('barrel_end_distance',51,'Outer barrel end, toward negative pod X'),('flange_back_distance',44,'Back face of mounting flange'),
            ('flange_face_distance',41,'Front face of mounting flange'),('bearing_shoulder_distance',43.5,'6804 outer-race shoulder location'),
            ('barrel_radius',22,'Cartridge barrel radius'),('flange_radius',33,'Mounting flange radius'),('bearing_seat_radius',16,'6804 bearing outside radius'),
            ('shoulder_bore_radius',13.5,'Bore under the bearing outer-race shoulder'),('screw_pcd',52,'Three M3 mounting holes pitch-circle diameter'),
            ('screw_clearance',3.4,'M3 mounting clearance'),('drill_overrun',.1,'Drill overrun at each flange face')]
    for n,v,t in values:E.var(n,v,'mm',t)
    E.var('first_screw_angle',30,'deg','First mounting screw angle from pod +Y toward +Z');E.var('screw_count',3,'','Mounting screw count')
    vs=[(-51,16,'barrel_end_distance','bearing_seat_radius'),(-51,22,'barrel_end_distance','barrel_radius'),
        (-44,22,'flange_back_distance','barrel_radius'),(-44,33,'flange_back_distance','flange_radius'),
        (-41,33,'flange_face_distance','flange_radius'),(-41,13.5,'flange_face_distance','shoulder_bore_radius'),
        (-43.5,13.5,'bearing_shoulder_distance','shoulder_bore_radius'),(-43.5,16,'bearing_shoulder_distance','bearing_seat_radius')]
    s=orthogonal_profile(m,E,'Front Plane','Cartridge Section',vs);full_revolve(m,E,s,'Cartridge Revolve')
    make_axis(m,'Cartridge Axis','Front Plane','Top Plane')
    ref_plane(m,E,'Right Plane',-44.1,'Mounting Drill Plane','flange_back_distance + drill_overrun')
    s=polar_circle_sketch(m,E,'Mounting Drill Plane','Mounting Hole Sketch',26,30,3.4,'screw_pcd / 2','first_screw_angle','screw_clearance');s.finish(E)
    if s.status()!=3:raise RuntimeError('Cartridge mounting sketch is under-defined')
    m.ClearSelection2(True);s.feat.Select2(False,0);f=cut_extrude(m,3.2,reverse=True,name='Mounting Hole Cut')
    link_feature(E,f,['flange_back_distance - flange_face_distance + 2 * drill_overrun'])
    patterned(m,E,'Cartridge Axis',['Mounting Hole Cut'],3,'screw_count','Mounting Hole Pattern')
    return finish(sw,m,E,key,path)

if __name__=='__main__':
    sw=connect()
    try:
        for fn in (shaft,retainer,cartridge):
            if not sys.argv[1:] or fn.__name__ in sys.argv[1:]:fn(sw)
    finally:sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,True)
