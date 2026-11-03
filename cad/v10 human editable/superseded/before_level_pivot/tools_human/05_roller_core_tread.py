from common import *

def variables(E):
    for n,v,t in [('wheel_radius',70,'True wheel contact radius; roller barrel is a circular arc'),('roller_axis_radius',52,'Wheel centre to roller axis'),
                  ('tread_thickness',2.5,'Radial TPU sleeve thickness'),('roller_half_length',19.5,'Core half length before journals'),
                  ('end_cone_length',1,'Axial length of the conical core tips'),('tip_radius',8,'Core radius at the tip'),
                  ('axle_clearance_diameter',3.6,'M3 axle running clearance'),('journal_diameter',8,'Rigid thrust journal diameter'),
                  ('journal_length',1.5,'Journal extension at each core end'),('keyway_floor_radius',13,'Inner radial face of each sleeve keyway'),
                  ('key_width',2.4,'Tangential key and keyway width'),('key_length',30,'Axial key length'),('key_cut_span',8,'Radial span of the keyway cutter'),
                  ('key_overlap',.2,'Key overlap into the sleeve for a single printable solid')]:E.var(n,v,'mm',t)
    E.var('key_count',3,'','Number of equally spaced anti-rotation keys')

def core(sw):
    key='roller_core';m,E,path=start(sw,key,'Core Barrel Section');variables(E)
    half=19.5;end=18.5;offset=54.5;rr=math.sqrt(70**2-end**2)-offset
    s=Sketch(m,'Top Plane','Core Barrel Section')
    a=s.line(1.8,-half,rr,-half);b=s.line(rr,-half,rr,-end)
    arc=s.arc(-offset,0,rr,-end,rr,end,True)
    d=s.line(rr,end,rr,half);e=s.line(rr,half,1.8,half);f=s.line(1.8,half,1.8,-half)
    for line in (a,e):s.rel('sgHORIZONTAL2D',line)
    for line in (b,d,f):s.rel('sgVERTICAL2D',line)
    arc_controls(s,arc,E,'roller_axis_radius + tread_thickness','wheel_radius')
    for point in (arc.GetStartPoint2(),arc.GetEndPoint2()):s.vdim(point,None,'roller_half_length - end_cone_length',(rr+4,0),origin_b=True)
    for point in (a.GetStartPoint2(),e.GetEndPoint2()):s.vdim(point,None,'roller_half_length',(2,22),origin_b=True)
    s.hdim(a.GetStartPoint2(),None,'axle_clearance_diameter / 2',(4,-24),origin_b=True)
    construction_axis(s,E);s.finish(E)
    if s.status()!=3:raise RuntimeError('Core barrel section status '+str(s.status()))
    select_revolve(s);full_revolve(m,E,s,'Core Barrel Revolve');E.solve();m.ForceRebuild3(False)
    es=edges_where(bodies(m)[0],lambda g:g[0]=='circle' and abs(g[1][2]-half)<1e-5 and abs(g[3]-rr)<1e-5)
    if len(es)!=1:raise RuntimeError('Expected one seed core tip edge')
    select_edges(m,es);ch=chamfer_equal(m,.2,'Core End Cones')
    radial=rr-8;found=False
    for d0,d1 in [(1,radial),(radial,1)]:
        cd=wrap(ch.GetDefinition(),sld.IChamferFeatureData2)
        if not cd.AccessSelections(m,None):raise RuntimeError('Core cone selection access')
        cd.Type=2;cd.EqualDistance=False;cd.SetEdgeChamferDistance(0,d0*MM);cd.SetEdgeChamferDistance(1,d1*MM)
        ok=ch.ModifyDefinition(cd,m,None);m.EditRebuild3()
        if ok and not ch.GetErrorCode2()[0]:
            ends=edges_where(bodies(m)[0],lambda g:g[0]=='circle' and abs(g[1][2]-half)<1e-5 and abs(g[3]-8)<1e-4)
            if len(ends)==1:found=True;break
    if not found:raise RuntimeError('Native core chamfer did not produce the conical tips')
    for dim in feature_own_dims(ch):
        value=wrap(m.Parameter(dim),sld.IDimension).SystemValue*1000
        E.link(dim,'end_cone_length' if abs(value-1)<1e-5 else 'sqr(wheel_radius * wheel_radius - (roller_half_length - end_cone_length) * (roller_half_length - end_cone_length)) - roller_axis_radius - tread_thickness - tip_radius')
    # Parasolid rejects a mirror of this unequal-distance chamfer at the arc seam.
    # The opposite end uses the same two controls in a second native chamfer.
    es=edges_where(bodies(m)[0],lambda g:g[0]=='circle' and abs(g[1][2]+half)<1e-5 and abs(g[3]-rr)<1e-5)
    select_edges(m,es);ch=chamfer_equal(m,.2,'Opposite End Cone')
    for d0,d1 in [(1,radial),(radial,1)]:
        cd=wrap(ch.GetDefinition(),sld.IChamferFeatureData2);cd.AccessSelections(m,None)
        cd.Type=2;cd.EqualDistance=False;cd.SetEdgeChamferDistance(0,d0*MM);cd.SetEdgeChamferDistance(1,d1*MM)
        ok=ch.ModifyDefinition(cd,m,None);m.EditRebuild3()
        ends=edges_where(bodies(m)[0],lambda g:g[0]=='circle' and abs(g[1][2]+half)<1e-5 and abs(g[3]-8)<1e-4)
        if ok and len(ends)==1:break
    else:raise RuntimeError('Opposite cone dimensions refused')
    for dim in feature_own_dims(ch):
        value=wrap(m.Parameter(dim),sld.IDimension).SystemValue*1000
        E.link(dim,'end_cone_length' if abs(value-1)<1e-5 else 'sqr(wheel_radius * wheel_radius - (roller_half_length - end_cone_length) * (roller_half_length - end_cone_length)) - roller_axis_radius - tread_thickness - tip_radius')
    ref_plane(m,E,'Front Plane',19.5,'Journal Shoulder Plane','roller_half_length')
    s=Sketch(m,'Journal Shoulder Plane','Journal Sketch')
    for radius,expr in [(4,'journal_diameter'),(1.8,'axle_clearance_diameter')]:
        cc=s.circle(0,0,radius);s.rel('sgCOINCIDENT',cc.GetCenterPoint2(),origin=True);s.ddim(cc,expr,(radius+4,4))
    s.finish(E);m.ClearSelection2(True);s.feat.Select2(False,0)
    f=boss_extrude(m,1.5,name='Thrust Journal');link_feature(E,f,['journal_length'])
    mirror_features(m,'Front Plane',['Thrust Journal'],'Opposite Thrust Journal')
    make_axis(m,'Roller Axis','Top Plane','Right Plane')
    vs=[(13,-15,'keyway_floor_radius','key_length / 2'),(21,-15,'keyway_floor_radius + key_cut_span','key_length / 2'),
        (21,15,'keyway_floor_radius + key_cut_span','key_length / 2'),(13,15,'keyway_floor_radius','key_length / 2')]
    s=orthogonal_profile(m,E,'Top Plane','Keyway Sketch',vs,axis=None);m.ClearSelection2(True);s.feat.Select2(False,0)
    f=cut_extrude(m,2.4,mid=True,name='Keyway Cut');link_feature(E,f,['key_width'])
    patterned(m,E,'Roller Axis',['Keyway Cut'],3,'key_count','Keyway Pattern')
    return finish(sw,m,E,key,path)

def tread(sw):
    key='roller_tread';m,E,path=start(sw,key,'Sleeve Section');variables(E)
    end=18.5;outer=math.sqrt(70**2-end**2)-52;inner=outer-2.5
    s=Sketch(m,'Top Plane','Sleeve Section')
    oa=s.arc(-52,0,outer,-end,outer,end,True);top=s.line(outer,end,inner,end)
    ia=s.arc(-54.5,0,inner,end,inner,-end,False);bottom=s.line(inner,-end,outer,-end)
    s.rel('sgHORIZONTAL2D',top);s.rel('sgHORIZONTAL2D',bottom)
    arc_controls(s,oa,E,'roller_axis_radius','wheel_radius');arc_controls(s,ia,E,'roller_axis_radius + tread_thickness','wheel_radius')
    for pt in (oa.GetStartPoint2(),oa.GetEndPoint2()):s.vdim(pt,None,'roller_half_length - end_cone_length',(outer+4,0),origin_b=True)
    construction_axis(s,E);s.finish(E)
    if s.status()!=3:raise RuntimeError('Sleeve section status '+str(s.status()))
    select_revolve(s);full_revolve(m,E,s,'Sleeve Revolve')
    make_axis(m,'Roller Axis','Top Plane','Right Plane')
    key_end=15;offset=54.3;rr=math.sqrt(70**2-key_end**2)-offset
    s=Sketch(m,'Top Plane','Sleeve Key Sketch')
    a=s.line(13,-key_end,rr,-key_end);arc=s.arc(-offset,0,rr,-key_end,rr,key_end,True)
    b=s.line(rr,key_end,13,key_end);c1=s.line(13,key_end,13,-key_end)
    s.rel('sgHORIZONTAL2D',a);s.rel('sgHORIZONTAL2D',b);s.rel('sgVERTICAL2D',c1)
    arc_controls(s,arc,E,'roller_axis_radius + tread_thickness - key_overlap','wheel_radius')
    for pt in (arc.GetStartPoint2(),arc.GetEndPoint2()):s.vdim(pt,None,'key_length / 2',(rr+4,0),origin_b=True)
    s.hdim(a.GetStartPoint2(),None,'keyway_floor_radius',(7,-19),origin_b=True)
    s.finish(E)
    if s.status()!=3:raise RuntimeError('Sleeve key section status '+str(s.status()))
    m.ClearSelection2(True);s.feat.Select2(False,0);f=boss_extrude(m,2.4,mid=True,name='Sleeve Key');link_feature(E,f,['key_width'])
    patterned(m,E,'Roller Axis',['Sleeve Key'],3,'key_count','Sleeve Key Pattern')
    return finish(sw,m,E,key,path)

if __name__=='__main__':
    sw=connect()
    try:
        for fn in (core,tread):
            if not sys.argv[1:] or fn.__name__ in sys.argv[1:]:fn(sw)
    finally:sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,True)
