"""Design-intent wheel hubs. Stages: base, spokes, axles, pockets."""
from common import *

def hub_vars(E):
    for n,v,t in [('row_distance',18,'Distance from wheel centre to each split plane'),('drum_outer_radius',33,'Drum outside radius'),('drum_bore_radius',30,'Lightweight drum bore radius'),('end_face_distance',41,'Hub A exterior end face toward -X'),('end_plate_thickness',4.4,'Hub A end plate thickness'),('flange_face_distance',23,'AK40 output contact face toward -X'),('flange_thickness',3.5,'Output flange plate thickness'),('access_bore_radius',16.5,'Driver access through outer hub plate'),('cartridge_boss_radius',26,'Cartridge mounting pitch radius'),('cartridge_boss_diameter',9,'Cartridge screw support outside diameter'),('cartridge_boss_length',10,'Cartridge screw engagement boss length'),('cartridge_pilot',2.8,'M3 thread-form pilot diameter'),('flange_pcd',27,'AK40 output mounting pitch circle'),('flange_clearance',2.9,'M2.5 output screw clearance'),('bearing_start',37,'Inner bearing outer face'),('bearing_end',44,'Inner bearing inner face'),('bearing_radius',16,'6804 outside radius'),('bearing_shoulder_radius',13.5,'Outer-race shoulder clearance'),('bearing_shoulder_length',2.5,'Axial bearing shoulder thickness'),('bearing_lead',.5,'Native seat entry chamfer'),('spoke_root_radius',31.5,'Root arc embedded inside drum wall'),('spoke_half_width',14,'Spoke tangential half width at root'),('spoke_tip_radius',60,'Spoke radial reach'),('nose_clip',4,'Tangential nose corner clip'),('spoke_thickness',12,'Axial thickness from split plane'),('taper_start_radius',45,'Radial start of axial spoke taper'),('taper_tip_thickness',6,'Spoke axial thickness at nose'),('nose_edge_break',1,'Native nose edge chamfer'),('joint_radius',38,'Hub joining screw pitch radius'),('joint_pad_diameter',14,'Joining screw pad diameter'),('joint_pad_fillet',2,'Pad reinforcing root fillet'),('joint_clearance',3.4,'M3 joining screw clearance'),('joint_pilot',2.8,'M3 joining thread-form pilot'),('joint_floor',1.5,'Closed floor behind hub B pilots'),('head_pocket_diameter',7.2,'M3 shallow pan head and driver clearance'),('head_pocket_depth',2.9,'Joint screw head counterbore depth'),('root_fillet',2,'Spoke to drum reinforcing fillet'),('roller_axis_radius',52,'Radius of roller axes'),('axle_block_diameter',12,'Axle support block diameter'),('axle_block_start',20,'Block near face from roller midplane'),('axle_block_end',23.5,'Block outer face from roller midplane'),('axle_clearance',3.4,'M3 roller axle clearance'),('axle_hole_length',51,'Axle cut total length'),('access_tunnel_length',45,'Axle head and nut tool tunnel length'),('nut_across_flats',5.8,'Captive M3 hex nut pocket'),('nut_depth',2.6,'Captive nut pocket depth'),('wheel_radius',70,'True roller barrel circle radius'),('roller_half_length',19.5,'Core half length before thrust journals'),('end_cone_length',1,'Core tip taper axial length'),('pocket_clearance',1.3,'Running clearance outside roller barrel'),('pocket_half_length',21.3,'Rigid journal thrust stop distance'),('pocket_cylinder_end',19.8,'End of full-width barrel relief'),('pocket_tip_radius',9,'Pocket radius at thrust stop'),('pocket_end_fillet',2,'Cutter end rounding leaves cheek reinforcing coves')]:E.var(n,v,'mm',t)
    for n,v,t in [('roller_count',6,'Rollers per row'),('cartridge_count',3,'Cartridge screws'),('flange_screw_count',3,'AK40 output screws')]:E.var(n,v,'',t)
    for n,v,t in [('outer_spoke_phase',30,'Outer spokes between rollers'),('inner_spoke_phase',60,'Inner spokes between staggered rollers'),('inner_roller_phase',30,'Stagger of second roller row'),('mount_first_angle',30,'First flange and cartridge screw from +Y toward +Z')]:E.var(n,v,'deg',t)

def resume(sw,key):
    m=open_owned(sw,key);sw.ActivateDoc3(m.GetTitle(),False,0,0);sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,False)
    m.FeatureManager.EnableFeatureTree=False;m.FeatureManager.EnableFeatureTreeWindow=False
    wrap(m.ActiveView,sld.IModelView).EnableGraphicsUpdate=False
    return m,DesignEqs(m),OUT/('V10H_'+key+'.SLDPRT')

def point_controls(s,pt,x,y,xe,ye):
    for value,expr,fn,rel in [(x,xe,s.hdim,'sgVERTICALPOINTS2D'),(y,ye,s.vdim,'sgHORIZONTALPOINTS2D')]:
        if abs(value)<1e-8:s.rel(rel,pt,origin=True)
        else:fn(pt,None,expr,(x+4,y+4),origin_b=True)

def centered_circle(m,E,plane,name,x,y,d,xe,ye,de):
    s=Sketch(m,plane,name);ci=s.circle(x,y,d/2);point_controls(s,ci.GetCenterPoint2(),x,y,xe,ye);s.ddim(ci,de,(x+5,y+5));s.finish(E)
    if s.status()!=3:raise RuntimeError(name+' status '+str(s.status()))
    m.ClearSelection2(True);s.feat.Select2(False,0);return s

def angle_plane(m,E,base,axis,angle,expression,name):
    if not angle:return base
    m.ClearSelection2(True);select(m,base,'PLANE');select(m,axis,'AXIS',0,True)
    if m.CreatePlaneAtAngle3(math.radians(angle),True,True) is None:raise RuntimeError('Cannot make '+name)
    f=named(m,name);link_feature(E,f,[expression]);return f.Name

def base(sw,which):
    key='wheel_hub_'+which;m,E,path=start(sw,key,'Hub Section');hub_vars(E)
    if which=='A':vs=[(-41,16.5,'end_face_distance','access_bore_radius'),(-41,33,'end_face_distance','drum_outer_radius'),(-18,33,'row_distance','drum_outer_radius'),(-18,30,'row_distance','drum_bore_radius'),(-23,30,'flange_face_distance','drum_bore_radius'),(-23,0,'flange_face_distance',None),(-26.5,0,'flange_face_distance + flange_thickness',None),(-26.5,30,'flange_face_distance + flange_thickness','drum_bore_radius'),(-36.6,30,'end_face_distance - end_plate_thickness','drum_bore_radius'),(-36.6,16.5,'end_face_distance - end_plate_thickness','access_bore_radius')]
    elif which=='B':vs=[(-18,30,'row_distance','drum_bore_radius'),(-18,33,'row_distance','drum_outer_radius'),(18,33,'row_distance','drum_outer_radius'),(18,30,'row_distance','drum_bore_radius')]
    else:vs=[(18,30,'row_distance','drum_bore_radius'),(18,33,'row_distance','drum_outer_radius'),(44,33,'bearing_end','drum_outer_radius'),(44,16,'bearing_end','bearing_radius'),(37,16,'bearing_start','bearing_radius'),(37,13.5,'bearing_start','bearing_shoulder_radius'),(34.5,13.5,'bearing_start - bearing_shoulder_length','bearing_shoulder_radius'),(34.5,30,'bearing_start - bearing_shoulder_length','drum_bore_radius')]
    s=orthogonal_profile(m,E,'Front Plane','Hub Section',vs,axis='horizontal');full_revolve(m,E,s,'Hub Body Revolve');make_axis(m,'Hub Axis','Top Plane','Front Plane')
    if which=='A':
        ref_plane(m,E,'Right Plane',-41,'End Face Plane','end_face_distance');ref_plane(m,E,'Right Plane',-23,'Flange Face Plane','flange_face_distance')
        polar_hole(m,E,'End Face Plane','Cartridge Boss Sketch',26,30,9,'cartridge_boss_radius','mount_first_angle','cartridge_boss_diameter')
        f=boss_extrude(m,10,name='Cartridge Boss');link_feature(E,f,['cartridge_boss_length']);patterned(m,E,'Hub Axis',['Cartridge Boss'],3,'cartridge_count','Cartridge Boss Pattern')
        polar_hole(m,E,'End Face Plane','Cartridge Pilot Sketch',26,30,2.8,'cartridge_boss_radius','mount_first_angle','cartridge_pilot')
        f=cut_extrude(m,10,reverse=True,name='Cartridge Pilot Cut');link_feature(E,f,['cartridge_boss_length']);patterned(m,E,'Hub Axis',['Cartridge Pilot Cut'],3,'cartridge_count','Cartridge Pilot Pattern')
        polar_hole(m,E,'Flange Face Plane','Motor Flange Sketch',13.5,30,2.9,'flange_pcd / 2','mount_first_angle','flange_clearance')
        f=cut_extrude(m,3.5,name='Motor Flange Clearance');link_feature(E,f,['flange_thickness']);patterned(m,E,'Hub Axis',['Motor Flange Clearance'],3,'flange_screw_count','Motor Flange Pattern')
    if which=='C':
        E.solve();m.ForceRebuild3(False);es=edges_where(bodies(m)[0],lambda g:g[0]=='circle' and abs(g[1][0]-44)<1e-5 and abs(g[3]-16)<1e-5)
        select_edges(m,es);f=chamfer_equal(m,.5,'Bearing Seat Lead In');link_feature(E,f,['bearing_lead'])
    return finish(sw,m,E,key,path)

def rows(which):return [0,1] if which=='B' else ([0] if which=='A' else [1])

def spoke(skplane,m,E,row,x,reverse):
    label='Outer' if row==0 else 'Inner';a=math.radians(30+30*row);ae='outer_spoke_phase' if row==0 else 'inner_spoke_phase'
    root=math.sqrt(31.5**2-14**2)
    pts=[(-14,root,'spoke_half_width','sqr(spoke_root_radius * spoke_root_radius - spoke_half_width * spoke_half_width)'),(-14,56,'spoke_half_width','spoke_tip_radius - nose_clip'),(-10,60,'spoke_half_width - nose_clip','spoke_tip_radius'),(10,60,'spoke_half_width - nose_clip','spoke_tip_radius'),(14,56,'spoke_half_width','spoke_tip_radius - nose_clip'),(14,root,'spoke_half_width','sqr(spoke_root_radius * spoke_root_radius - spoke_half_width * spoke_half_width)')]
    actual=[]
    for u,v,ue,ve in pts:
        ue=('- ('+ue+')') if u<0 else ue
        actual.append((u*math.cos(a)-v*math.sin(a),u*math.sin(a)+v*math.cos(a),f'abs(({ue}) * cos({ae}) - ({ve}) * sin({ae}))',f'abs(({ue}) * sin({ae}) + ({ve}) * cos({ae}))'))
    s=Sketch(m,skplane,label+' Spoke Sketch');lines=[]
    for p,q in zip(actual,actual[1:]):lines.append(s.line(p[0],p[1],q[0],q[1]))
    arc=s.arc(0,0,actual[-1][0],actual[-1][1],actual[0][0],actual[0][1],True)
    s.rel('sgCOINCIDENT',arc.GetCenterPoint2(),origin=True);s.rdim(arc,'spoke_root_radius',(-25,20))
    points=[l.GetStartPoint2() for l in lines]+[lines[-1].GetEndPoint2()]
    for j,(pt,(xx,yy,xe,ye)) in enumerate(zip(points,actual)):
        s.hdim(pt,None,xe,(xx-5,yy),origin_b=True)
        if j not in (0,5):s.vdim(pt,None,ye,(xx,yy+4),origin_b=True)
    s.finish(E)
    if s.status()!=3:raise RuntimeError(s.name+' status '+str(s.status()))
    m.ClearSelection2(True);s.feat.Select2(False,0);f=boss_extrude(m,12,reverse=reverse,name=label+' Spoke Extrude');link_feature(E,f,['spoke_thickness'])
    # Front plane rotated about X: local y is the seed spoke radial direction.
    tp=angle_plane(m,E,'Front Plane','Hub Axis',30+30*row,ae,label+' Spoke Taper Plane')
    outer=x+(-12 if reverse else 12);direction=1 if reverse else -1;oxexpr=('row_distance + spoke_thickness' if (x<0 and reverse) or (x>0 and not reverse) else 'row_distance - spoke_thickness')
    # Triangle continues 1 mm beyond the nose; the controlling slope is 6/15.
    E.var('taper_overrun',1,'mm','Cut extends beyond the spoke nose to avoid coincident edges')
    yy=61;xx=outer+direction*6.4
    xexpr=f'abs(({"-" if x<0 else ""}row_distance {"-" if reverse else "+"} spoke_thickness) {"+" if reverse else "-"} (spoke_thickness - taper_tip_thickness) * (spoke_tip_radius + taper_overrun - taper_start_radius) / (spoke_tip_radius - taper_start_radius))'
    s=orthogonal_profile(m,E,tp,label+' Taper Sketch',[(outer,45,oxexpr,'taper_start_radius'),(outer,yy,oxexpr,'spoke_tip_radius + taper_overrun'),(xx,yy,xexpr,'spoke_tip_radius + taper_overrun')],axis=None)
    m.ClearSelection2(True);s.feat.Select2(False,0);f=cut_extrude(m,28,mid=True,name=label+' Axial Spoke Taper');link_feature(E,f,['2 * spoke_half_width'])
    E.solve();m.ForceRebuild3(False)
    def nose(g):
        if g[0]!='line':return False
        p=[(g[1][j]+g[2][j])/2 for j in range(3)];rad=p[1]*math.cos(a)+p[2]*math.sin(a)
        return rad>=55.99 and min(x,outer)-1e-5<=p[0]<=max(x,outer)+1e-5
    es=edges_where(bodies(m)[0],nose);select_edges(m,es);f=chamfer_equal(m,1,label+' Nose Chamfer');link_feature(E,f,['nose_edge_break'])
    patterned(m,E,'Hub Axis',[label+' Spoke Extrude',label+' Axial Spoke Taper',label+' Nose Chamfer'],6,'roller_count',label+' Spoke Pattern')
    # The joining pads lie on the spoke centre lines.
    polar_hole(m,E,skplane,label+' Joint Pad Sketch',38,30+30*row,14,'joint_radius',ae,'joint_pad_diameter')
    f=boss_extrude(m,12,reverse=reverse,name=label+' Joint Pad');link_feature(E,f,['spoke_thickness'])
    patterned(m,E,'Hub Axis',[label+' Joint Pad'],6,'roller_count',label+' Joint Pad Pattern')

def spokes(sw,which):
    key='wheel_hub_'+which;m,E,path=resume(sw,key);delete_from(m,'Outer Row Plane' if which!='C' else 'Inner Row Plane')
    for row in rows(which):
        label='Outer' if row==0 else 'Inner';x=-18 if row==0 else 18;rev=which=='A' or (which=='B' and row==1)
        ref_plane(m,E,'Right Plane',x,label+' Row Plane','row_distance');spoke(label+' Row Plane',m,E,row,x,rev)
    return finish(sw,m,E,key,path)

def axles(sw,which):
    key='wheel_hub_'+which;m,E,path=resume(sw,key)
    first='Outer Axle Block Sketch' if which!='C' else 'Inner Roller Plane'
    delete_from(m,first)
    for row in rows(which):
        label='Outer' if row==0 else 'Inner';x=-18 if row==0 else 18;xe='row_distance'
        rev=which=='A' or (which=='B' and row==1)
        plane=angle_plane(m,E,'Front Plane','Hub Axis',30 if row else 0,'inner_roller_phase',label+' Roller Plane')
        s=Sketch(m,plane,label+' Axle Block Sketch');arc=s.arc(x,52,x,46,x,58,not rev);line=s.line(x,58,x,46)
        centre=arc.GetCenterPoint2();point_controls(s,centre,x,52,xe,'roller_axis_radius');s.rdim(arc,'axle_block_diameter / 2',(x+8,52))
        s.rel('sgVERTICAL2D',line);s.rel('sgVERTICALPOINTS2D',line.GetStartPoint2(),centre)
        s.finish(E)
        if s.status()!=3:raise RuntimeError(s.name+' status '+str(s.status()))
        m.ClearSelection2(True);s.feat.Select2(False,0)
        f=boss_extrude(m,3.5,start_offset=20,name=label+' Axle Block');link_feature(E,f,['axle_block_end - axle_block_start','axle_block_start'])
        mirror_features(m,plane,[label+' Axle Block'],label+' Opposite Axle Block')
        patterned(m,E,'Hub Axis',[label+' Axle Block',label+' Opposite Axle Block'],6,'roller_count',label+' Axle Block Pattern')
        centered_circle(m,E,plane,label+' Axle Hole Sketch',x,52,3.4,xe,'roller_axis_radius','axle_clearance')
        f=cut_extrude(m,51,mid=True,name=label+' Axle Hole');link_feature(E,f,['axle_hole_length'])
        centered_circle(m,E,plane,label+' Head Tunnel Sketch',x,52,7.2,xe,'roller_axis_radius','head_pocket_diameter')
        f=cut_extrude(m,45,start_offset=23.5,flip_start=True,name=label+' Head Access Tunnel');link_feature(E,f,['access_tunnel_length','axle_block_end'])
        s=Sketch(m,plane,label+' Nut Pocket Sketch');s.polygon(x,52,x+5.8/math.sqrt(3),52,6)
        segs=s.segments();ci=next(wrap(z,sld.ISketchArc) for z in segs if z.GetType()==1)
        point_controls(s,ci.GetCenterPoint2(),x,52,xe,'roller_axis_radius');s.ddim(ci,'nut_across_flats',(x+8,56))
        flat=next(z for z in segs if z.GetType()==0 and abs(wrap(z,sld.ISketchLine).GetStartPoint2().Y-wrap(z,sld.ISketchLine).GetEndPoint2().Y)<1e-8)
        s.rel('sgHORIZONTAL2D',flat);s.finish(E)
        m.ClearSelection2(True);s.feat.Select2(False,0);f=cut_extrude(m,2.6,reverse=True,start_offset=23.5,name=label+' Captive Nut Pocket');link_feature(E,f,['nut_depth','axle_block_end'])
        centered_circle(m,E,plane,label+' Nut Tunnel Sketch',x,52,7.2,xe,'roller_axis_radius','head_pocket_diameter')
        f=cut_extrude(m,45,reverse=True,start_offset=26.1,name=label+' Nut Access Tunnel');link_feature(E,f,['access_tunnel_length','axle_block_end + nut_depth'])
        patterned(m,E,'Hub Axis',[label+' Axle Hole',label+' Head Access Tunnel',label+' Captive Nut Pocket',label+' Nut Access Tunnel'],6,'roller_count',label+' Axle Clearance Pattern')
        # One seed joins each split, repeated between rollers.
        if which=='B':
            jp=label+' Row Plane';d=2.8;depth=10.5;cutrev=row==0
        else:
            outer=x+(-12 if rev else 12)
            ref_plane(m,E,'Right Plane',outer,label+' Joint Head Plane','row_distance + spoke_thickness')
            jp=label+' Joint Head Plane';d=3.4;depth=12;cutrev=rev
        ae='outer_spoke_phase' if row==0 else 'inner_spoke_phase'
        polar_hole(m,E,jp,label+' Joint Hole Sketch',38,30+30*row,d,'joint_radius',ae,'joint_pilot' if which=='B' else 'joint_clearance')
        f=cut_extrude(m,depth,reverse=cutrev,name=label+' Joint Hole');link_feature(E,f,['spoke_thickness - joint_floor' if which=='B' else 'spoke_thickness'])
        group=[label+' Joint Hole']
        if which!='B':
            polar_hole(m,E,jp,label+' Joint Counterbore Sketch',38,30+30*row,7.2,'joint_radius',ae,'head_pocket_diameter')
            f=cut_extrude(m,2.9,reverse=cutrev,name=label+' Joint Counterbore');link_feature(E,f,['head_pocket_depth']);group.append(label+' Joint Counterbore')
        patterned(m,E,'Hub Axis',group,6,'roller_count',label+' Joint Hole Pattern')
    E.solve();m.ForceRebuild3(False)
    def seam(g):
        if g[0]!='line':return False
        p,q=g[1:];return abs(p[0]-q[0])>5 and all(32.99<math.hypot(v[1],v[2])<33.01 for v in (p,q))
    es=edges_where(bodies(m)[0],seam)
    if not es:raise RuntimeError('No drum-spoke seams')
    select_edges(m,es);f=fillet_uniform(m,2,'Spoke Root Fillets');link_feature(E,f,['root_fillet'])
    return finish(sw,m,E,key,path)

def pocket(sw,which):
    key='wheel_hub_'+which;m,E,path=resume(sw,key);delete_from(m,'Outer Pocket Plane' if which!='C' else 'Inner Pocket Radial Plane')
    for row in rows(which):
        label='Outer' if row==0 else 'Inner';x=-18 if row==0 else 18;re='- row_distance' if row==0 else 'row_distance'
        radial=angle_plane(m,E,'Top Plane','Hub Axis',30 if row else 0,'inner_roller_phase',label+' Pocket Radial Plane')
        ref_plane(m,E,radial,52,label+' Pocket Plane','roller_axis_radius')
        yb=18.5;rb=math.sqrt(70**2-yb**2)-52+1.3;xt=x+rb
        s=Sketch(m,label+' Pocket Plane',label+' Roller Pocket Section')
        aa=s.line(x,-21.3,x+9,-21.3);bb=s.line(x+9,-21.3,xt,-19.8);cc=s.line(xt,-19.8,xt,-18.5)
        arc=s.arc(x-50.7,0,xt,-18.5,xt,18.5,True)
        dd=s.line(xt,18.5,xt,19.8);ee=s.line(xt,19.8,x+9,21.3);ff=s.line(x+9,21.3,x,21.3);gg=s.line(x,21.3,x,-21.3)
        for z in (aa,ff):s.rel('sgHORIZONTAL2D',z)
        for z in (cc,dd,gg):s.rel('sgVERTICAL2D',z)
        arc_controls(s,arc,E,f'roller_axis_radius - pocket_clearance - ({re})','wheel_radius')
        for z in (arc.GetStartPoint2(),arc.GetEndPoint2()):s.vdim(z,None,'roller_half_length - end_cone_length',(xt+5,0),origin_b=True)
        for z in (cc.GetStartPoint2(),dd.GetEndPoint2()):s.vdim(z,None,'pocket_cylinder_end',(xt+8,0),origin_b=True)
        for z in (aa.GetStartPoint2(),ff.GetEndPoint2()):s.vdim(z,None,'pocket_half_length',(x-4,0),origin_b=True)
        s.hdim(aa.GetStartPoint2(),None,'row_distance',(x,-25),origin_b=True)
        for z in (aa.GetEndPoint2(),ff.GetStartPoint2()):s.hdim(z,None,f'abs({re} + pocket_tip_radius)',(x+9,26),origin_b=True)
        # Axis is separate construction geometry, centred on the row split.
        ax=s.cline(x,-10.123,x,10.123);s.rel('sgVERTICAL2D',ax)
        s.rel('sgVERTICALPOINTS2D',ax.GetStartPoint2(),aa.GetStartPoint2())
        for pt in (ax.GetStartPoint2(),ax.GetEndPoint2()):s.vdim(pt,None,'axis_reference_halfspan',(x+2,4),origin_b=True)
        s.finish(E)
        if s.status()!=3:raise RuntimeError(s.name+' status '+str(s.status()))
        select_revolve(s);full_revolve(m,E,s,label+' Roller Pocket Tool',new_body=True);E.solve();m.ForceRebuild3(False)
        tool=next(b for b in bodies(m) if abs(b.GetMassProperties(1)[0]*1000-x)<1e-3 and abs(b.GetMassProperties(1)[1]*1000-52*math.cos(math.radians(30*row)))<1e-3)
        def end_ring(g):
            if g[0]!='circle':return False
            a=math.radians(30*row);p=g[1];along=-p[1]*math.sin(a)+p[2]*math.cos(a)
            return abs(abs(along)-21.3)<1e-4 and abs(g[3]-9)<1e-4
        es=edges_where(tool,end_ring)
        if len(es)!=2:raise RuntimeError('Expected two pocket end rings, got '+str(len(es)))
        select_edges(m,es);f=fillet_uniform(m,2,label+' Pocket End Fillets');link_feature(E,f,['pocket_end_fillet'])
        tool=next(b for b in bodies(m) if abs(b.GetMassProperties(1)[0]*1000-x)<1e-3)
        circ_pattern_bodies(m,'Hub Axis',tool,6,label+' Pocket Tool Pattern')
        for dim in feature_own_dims(last_feature(m)):
            val=wrap(m.Parameter(dim),sld.IDimension).SystemValue;E.link(dim,'roller_count' if abs(val-6)<1e-6 else 'full_turn')
        bs=bodies(m);main=max(bs,key=lambda b:b.GetMassProperties(1)[3]);tools=[b for b in bs if b._oleobj_!=main._oleobj_]
        combine_cut(m,main,tools,label+' Roller Pockets Cut')
    return finish(sw,m,E,key,path)

if __name__=='__main__':
    sw=connect();which=sys.argv[1] if len(sys.argv)>1 else 'A';stage=sys.argv[2] if len(sys.argv)>2 else 'all'
    try:
        if stage in ('all','base'):base(sw,which)
        if stage in ('all','spokes'):spokes(sw,which)
        if stage in ('all','axles'):axles(sw,which)
        if stage in ('all','pockets'):pocket(sw,which)
    finally:sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,True)
