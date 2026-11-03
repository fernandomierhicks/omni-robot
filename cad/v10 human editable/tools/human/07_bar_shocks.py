from common import *

def bar(sw):
    key='fork_bar';m,E,path=start(sw,key,'Bar Outline Sketch')
    for n,v,t in [('bar_length',104,'Cut length of 6061 bar'),('bar_width',10,'Bar width along Y'),('bar_height',20,'Bar height along Z'),('outer_screw_x',48.5,'Outer screw offset toward negative local X'),('inner_screw_x',46,'Inner screw offset toward positive local X'),('screw_half_pitch',6,'Half vertical screw pitch'),('screw_clearance',3.4,'M3 through hole diameter')]:E.var(n,v,'mm',t)
    s=orthogonal_profile(m,E,'Front Plane','Bar Outline Sketch',[(-52,-5,'bar_length / 2','bar_width / 2'),(52,-5,'bar_length / 2','bar_width / 2'),(52,5,'bar_length / 2','bar_width / 2'),(-52,5,'bar_length / 2','bar_width / 2')],axis=None)
    m.ClearSelection2(True);s.feat.Select2(False,0);f=boss_extrude(m,20,mid=True,name='Bar Extrude');link_feature(E,f,['bar_height'])
    s=Sketch(m,'Top Plane','Bar Screw Sketch')
    for x,expr in [(-48.5,'outer_screw_x'),(46,'inner_screw_x')]:
        ci=s.circle(x,-6,1.7);pt=ci.GetCenterPoint2();s.hdim(pt,None,expr,(x,-10),origin_b=True);s.vdim(pt,None,'screw_half_pitch',(x+5,-6),origin_b=True);s.ddim(ci,'screw_clearance',(x+4,-2))
    s.finish(E);m.ClearSelection2(True);s.feat.Select2(False,0);f=cut_extrude(m,10,mid=True,name='Bar Screw Cut');link_feature(E,f,['bar_width'])
    mirror_features(m,'Front Plane',['Bar Screw Cut'],'Opposite Bar Screw Holes')
    return finish(sw,m,E,key,path)

def shock(sw,rod=False):
    key='shock_rod' if rod else 'shock_case';m,E,path=start(sw,key,'Shaft Section' if rod else 'Cylinder Section')
    for n,v,t in [('eye_diameter',8,'Mock shock eye outside diameter'),('eye_width',6,'Eye width along hinge axis X'),('eye_bore',3.2,'M3 hinge clearance'),('rod_diameter',4,'Sliding rod diameter'),('rod_end_distance',27,'Rod tip distance from moving eye'),('neck_start',3,'Neck junction inside eye'),('cylinder_diameter',11,'Mock shock cylinder outside diameter'),('cylinder_start',7,'Cylinder lower shoulder'),('cylinder_end',35.5,'Cylinder open end'),('bore_start',12,'Bottom of sliding bore'),('bore_diameter',4.4,'Sliding bore diameter')]:E.var(n,v,'mm',t)
    if rod:
        vs=[(0,27,None,'rod_end_distance'),(2,27,'rod_diameter / 2','rod_end_distance'),(2,3,'rod_diameter / 2','neck_start'),(0,3,None,'neck_start')]
    else:
        vs=[(0,-3,None,'neck_start'),(2,-3,'rod_diameter / 2','neck_start'),(2,-7,'rod_diameter / 2','cylinder_start'),(5.5,-7,'cylinder_diameter / 2','cylinder_start'),(5.5,-35.5,'cylinder_diameter / 2','cylinder_end'),(2.2,-35.5,'bore_diameter / 2','cylinder_end'),(2.2,-12,'bore_diameter / 2','bore_start'),(0,-12,None,'bore_start')]
    name='Shaft Section' if rod else 'Cylinder Section';s=orthogonal_profile(m,E,'Top Plane',name,vs,axis='vertical');full_revolve(m,E,s,'Rod Revolve' if rod else 'Cylinder Revolve')
    s=Sketch(m,'Right Plane','Eye Sketch')
    ci=s.circle(0,0,4);s.rel('sgCOINCIDENT',ci.GetCenterPoint2(),origin=True);s.ddim(ci,'eye_diameter',(6,6));s.finish(E)
    m.ClearSelection2(True);s.feat.Select2(False,0);f=boss_extrude(m,6,mid=True,name='Eye Extrude');link_feature(E,f,['eye_width'])
    s=Sketch(m,'Right Plane','Eye Bore Sketch');ci=s.circle(0,0,1.6);s.rel('sgCOINCIDENT',ci.GetCenterPoint2(),origin=True);s.ddim(ci,'eye_bore',(5,5));s.finish(E)
    m.ClearSelection2(True);s.feat.Select2(False,0);f=cut_extrude(m,6,mid=True,name='Eye Bore Cut');link_feature(E,f,['eye_width'])
    return finish(sw,m,E,key,path)

if __name__=='__main__':
    sw=connect()
    try:
        if not sys.argv[1:] or 'bar' in sys.argv[1:]:bar(sw)
        if not sys.argv[1:] or 'case' in sys.argv[1:]:shock(sw)
        if not sys.argv[1:] or 'rod' in sys.argv[1:]:shock(sw,True)
    finally:sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,True)
