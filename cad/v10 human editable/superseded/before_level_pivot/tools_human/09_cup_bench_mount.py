from common import *
from importlib import import_module
H=import_module('08_hubs')

def cup(sw):
    key='motor_cup';m,E,path=start(sw,key,'Cradle Section')
    for n,v,t in [('sleeve_start',21.5,'Open sleeve end distance toward -X'),('sleeve_radius',29,'Cradle outer radius'),('motor_bore_radius',26.8,'Motor body running clearance radius'),('bridge_front',17.2,'Rear bridge front face'),('bridge_back',21.2,'Rear bridge rear face'),('neck_end',37,'Fork contact face'),('neck_radius',10,'6804-compatible support neck radius'),('wire_bore_radius',3,'Central harness bore'),('rear_screw_pcd',47,'AK40 rear mounting circle'),('rear_clearance',2.9,'M2.5 rear mounting clearance'),('fork_screw_radius',6.5,'Four M3 cradle attachment screw radius'),('fork_pilot',2.8,'M3 thread-forming pilot'),('fork_thread_depth',12,'Cradle attachment pilot depth'),('channel_start',6,'Harness channel starts 29 mm behind output face'),('channel_end',37.1,'Harness cut extends past fork contact face'),('channel_half_width',6,'Harness slot half width before indexing'),('channel_floor',7.5,'Harness floor retains at least 1.4 mm to the M3 major thread'),('channel_top',35,'Harness slot upper extent before indexing'),('bore_lead',.5,'Native sleeve entry chamfer')]:E.var(n,v,'mm',t)
    E.var('rear_first_angle',90,'deg','Rear screw seed from +Y toward +Z');E.var('fork_first_angle',90,'deg','Fork screw seed from +Y toward +Z');E.var('channel_index',45,'deg','Harness diagonal index between rear screws');E.var('screw_count',4,'','Four equally spaced screws per mounting pattern')
    vs=[(-21.5,26.8,'sleeve_start','motor_bore_radius'),(-21.5,29,'sleeve_start','sleeve_radius'),(21.2,29,'bridge_back','sleeve_radius'),(21.2,10,'bridge_back','neck_radius'),(37,10,'neck_end','neck_radius'),(37,3,'neck_end','wire_bore_radius'),(17.2,3,'bridge_front','wire_bore_radius'),(17.2,26.8,'bridge_front','motor_bore_radius')]
    s=orthogonal_profile(m,E,'Front Plane','Cradle Section',vs,axis='horizontal');full_revolve(m,E,s,'Cradle Revolve');make_axis(m,'Motor Axis','Front Plane','Top Plane')
    ref_plane(m,E,'Right Plane',21.2,'Rear Screw Seat Plane','bridge_back')
    polar_hole(m,E,'Rear Screw Seat Plane','Rear Mount Sketch',23.5,90,2.9,'rear_screw_pcd / 2','rear_first_angle','rear_clearance')
    f=cut_extrude(m,4,name='Rear Mount Clearance');link_feature(E,f,['bridge_back - bridge_front']);patterned(m,E,'Motor Axis',['Rear Mount Clearance'],4,'screw_count','Rear Mount Pattern')
    ref_plane(m,E,'Right Plane',37,'Fork Contact Plane','neck_end')
    polar_hole(m,E,'Fork Contact Plane','Fork Pilot Sketch',6.5,90,2.8,'fork_screw_radius','fork_first_angle','fork_pilot')
    f=cut_extrude(m,12,name='Fork Thread Pilot');link_feature(E,f,['fork_thread_depth']);patterned(m,E,'Motor Axis',['Fork Thread Pilot'],4,'screw_count','Fork Pilot Pattern')
    ref_plane(m,E,'Right Plane',6,'Harness Start Plane','channel_start');vs=[]
    for u,v,ue,ve in [(-7.5,-6,'- channel_floor','- channel_half_width'),(-35,-6,'- channel_top','- channel_half_width'),(-35,6,'- channel_top','channel_half_width'),(-7.5,6,'- channel_floor','channel_half_width')]:
        a=math.radians(-45);xx=u*math.cos(a)-v*math.sin(a);yy=u*math.sin(a)+v*math.cos(a)
        vs.append((xx,yy,f'abs(({ue}) * cos(channel_index) + ({ve}) * sin(channel_index))',f'abs(- ({ue}) * sin(channel_index) + ({ve}) * cos(channel_index))'))
    s=orthogonal_profile(m,E,'Harness Start Plane','Diagonal Harness Sketch',vs,axis=None);m.ClearSelection2(True);s.feat.Select2(False,0)
    f=boss_or_cut=cut_extrude(m,31.1,reverse=True,name='Diagonal Harness Channel');link_feature(E,f,['channel_end - channel_start'])
    E.solve();m.ForceRebuild3(False);es=edges_where(bodies(m)[0],lambda g:g[0]=='circle' and abs(g[1][0]+21.5)<1e-5 and abs(g[3]-26.8)<1e-5)
    select_edges(m,es);f=chamfer_equal(m,.5,'Motor Sleeve Lead In');link_feature(E,f,['bore_lead'])
    return finish(sw,m,E,key,path)

def hole_set(m,E,plane,name,positions,diam,diam_expr):
    s=Sketch(m,plane,name)
    for x,y,xe,ye in positions:
        ci=s.circle(x,y,diam/2);H.point_controls(s,ci.GetCenterPoint2(),x,y,xe,ye);s.ddim(ci,diam_expr,(x+3,y+3))
    s.finish(E)
    if s.status()!=3:raise RuntimeError(name+' status '+str(s.status()))
    m.ClearSelection2(True);s.feat.Select2(False,0);return s

def foot_variables(E):
    for n,v,t in [('guard_front_foot_x',14,'Front wall foot offset toward -X'),('guard_rear_foot_x',16,'Rear wall foot offset toward +X'),('guard_foot_y',87.5,'Wall and ear foot transverse location'),('inner_ear_foot_x',40.25,'Inner ear foot at shaft pin'),('outer_ear_foot_x',31.5,'Outer ear foot offset toward -X')]:E.var(n,v,'mm',t)
    pts=[(86,22),(86,46),(90,30),(97,26),(100,33.5)]
    for j,(x,y) in enumerate(pts,1):E.var('tower_foot_'+str(j)+'_x',x,'mm','Tower foot screw '+str(j)+' X');E.var('tower_foot_'+str(j)+'_y',y,'mm','Tower foot screw '+str(j)+' Y')
    guard=[(-14,-87.5,'guard_front_foot_x','guard_foot_y'),(16,-87.5,'guard_rear_foot_x','guard_foot_y'),(-14,87.5,'guard_front_foot_x','guard_foot_y'),(16,87.5,'guard_rear_foot_x','guard_foot_y'),(40.25,87.5,'inner_ear_foot_x','guard_foot_y'),(-31.5,87.5,'outer_ear_foot_x','guard_foot_y')]
    tower=[(x,y,'tower_foot_'+str(j)+'_x','tower_foot_'+str(j)+'_y') for j,(x,y) in enumerate(pts,1)]
    return guard,tower

def bench(sw):
    key='bench_plate';m,E,path=start(sw,key,'Bench Frame Sketch');guard,tower=foot_variables(E)
    for n,v,t in [('frame_left',70,'Frame open end toward -X'),('frame_right',118,'Frame closed end toward +X'),('frame_half_width',98,'Bench frame half width'),('opening_end',62,'Wheel opening closed end'),('opening_half_width',83,'Wheel opening half width'),('bench_thickness',6,'Bench frame plate thickness'),('panel_centre_x',146,'Robot deck circle centre relative to wheel'),('relief_outer_radius',217.5,'Outer guard panel clearance radius'),('relief_inner_radius',211.5,'Inner guard panel clearance radius'),('relief_half_width',90.5,'Transverse panel clearance extent'),('clearance_cut_depth',8,'Relief cutter extends past both plate faces'),('screw_clearance',3.4,'M3 foot screw clearance'),('counterbore_diameter',7.2,'M3 pan head counterbore'),('counterbore_depth',2.9,'Counterbore from underside')]:E.var(n,v,'mm',t)
    vs=[(-70,-98,'frame_left','frame_half_width'),(118,-98,'frame_right','frame_half_width'),(118,98,'frame_right','frame_half_width'),(-70,98,'frame_left','frame_half_width'),(-70,83,'frame_left','opening_half_width'),(62,83,'opening_end','opening_half_width'),(62,-83,'opening_end','opening_half_width'),(-70,-83,'frame_left','opening_half_width')]
    s=orthogonal_profile(m,E,'Front Plane','Bench Frame Sketch',vs,axis=None);m.ClearSelection2(True);s.feat.Select2(False,0);f=boss_extrude(m,6,mid=True,name='Bench Frame Extrude');link_feature(E,f,['bench_thickness'])
    s=Sketch(m,'Front Plane','Curved Panel Clearance Sketch');xo=146-math.sqrt(217.5**2-90.5**2);xi=146-math.sqrt(211.5**2-90.5**2)
    oa=s.arc(146,0,xo,-90.5,xo,90.5,False);a=s.line(xo,90.5,xi,90.5);ia=s.arc(146,0,xi,90.5,xi,-90.5,True);b=s.line(xi,-90.5,xo,-90.5)
    arc_controls(s,oa,E,'panel_centre_x','relief_outer_radius')
    s.rel('sgCONCENTRIC',oa,ia);s.rdim(ia,'relief_inner_radius',(xi+5,0))
    for line in (a,b):s.rel('sgHORIZONTAL2D',line)
    for pt in (oa.GetStartPoint2(),oa.GetEndPoint2()):s.vdim(pt,None,'relief_half_width',(xo-5,0),origin_b=True)
    s.finish(E)
    if s.status()!=3:raise RuntimeError('Panel clearance status '+str(s.status()))
    m.ClearSelection2(True);s.feat.Select2(False,0);f=cut_extrude(m,8,mid=True,name='Curved Panel Clearance');link_feature(E,f,['clearance_cut_depth'])
    hole_set(m,E,'Front Plane','Pod Foot Clearance Sketch',guard+tower,3.4,'screw_clearance');f=cut_extrude(m,8,mid=True,name='Pod Foot Through Holes');link_feature(E,f,['clearance_cut_depth'])
    ref_plane(m,E,'Front Plane',-3,'Bench Underside Plane','bench_thickness / 2')
    hole_set(m,E,'Bench Underside Plane','Foot Counterbore Sketch',guard+tower,7.2,'counterbore_diameter');f=cut_extrude(m,2.9,reverse=True,name='Foot Counterbores');link_feature(E,f,['counterbore_depth'])
    return finish(sw,m,E,key,path)

def mount(sw):
    key='pod_mount';m,E,path=start(sw,key,'Deck Top Plane');_,tower=foot_variables(E)
    vals=[('deck_height',3,'Top of bench deck'),('foot_left',69.5,'Tower foot outer edge X'),('foot_right',92,'Tower foot inner edge X'),('foot_y_min',18,'Tower foot near edge Y'),('foot_y_max',50,'Tower foot far edge Y'),('tie_right',104,'Deck tie extension end X'),('tie_y_min',23,'Tie near edge Y'),('tie_y_max',38,'Tie far edge Y'),('foot_thickness',10,'Printed foot thickness'),('web_right',75.5,'Tower web inner face X'),('web_y_min',22,'Tower web near edge Y'),('web_y_max',46,'Tower web far edge Y'),('web_top',64.3,'Tower web top Z'),('shock_y',30,'Fixed inner shock eye Y'),('shock_z',58.3,'Fixed inner shock eye Z'),('shock_boss_start',66.5,'Shock spacer face X'),('shock_boss_diameter',9,'Shock spacer outside diameter'),('shock_clearance',3.4,'M3 shock hinge clearance'),('foot_pilot',2.8,'M3 thread-form foot pilot'),('foot_entry_chamfer',.5,'Pilot entry lead'),('web_root_fillet',2,'Tower web reinforcing root fillet')]
    for n,v,t in vals:E.var(n,v,'mm',t)
    ref_plane(m,E,'Front Plane',3,'Deck Top Plane','deck_height')
    vs=[(69.5,18,'foot_left','foot_y_min'),(92,18,'foot_right','foot_y_min'),(92,23,'foot_right','tie_y_min'),(104,23,'tie_right','tie_y_min'),(104,38,'tie_right','tie_y_max'),(92,38,'foot_right','tie_y_max'),(92,50,'foot_right','foot_y_max'),(69.5,50,'foot_left','foot_y_max')]
    s=orthogonal_profile(m,E,'Deck Top Plane','Mount Foot Sketch',vs,axis=None);m.ClearSelection2(True);s.feat.Select2(False,0);f=boss_extrude(m,10,name='Mount Foot Extrude');link_feature(E,f,['foot_thickness'])
    ref_plane(m,E,'Front Plane',13,'Foot Top Plane','deck_height + foot_thickness')
    s=orthogonal_profile(m,E,'Foot Top Plane','Tower Web Sketch',[(69.5,22,'foot_left','web_y_min'),(75.5,22,'web_right','web_y_min'),(75.5,46,'web_right','web_y_max'),(69.5,46,'foot_left','web_y_max')],axis=None)
    m.ClearSelection2(True);s.feat.Select2(False,0);f=boss_extrude(m,51.3,name='Tower Web Extrude');link_feature(E,f,['web_top - deck_height - foot_thickness'])
    E.solve();m.ForceRebuild3(False)
    es=edges_where(bodies(m)[0],lambda g:g[0]=='line' and all(abs(p[2]-13)<1e-5 for p in g[1:]) and ((all(abs(p[0]-75.5)<1e-5 for p in g[1:])) or all(abs(p[1]-46)<1e-5 for p in g[1:])))
    select_edges(m,es);f=fillet_uniform(m,2,'Tower Root Fillets');link_feature(E,f,['web_root_fillet'])
    ref_plane(m,E,'Right Plane',66.5,'Shock Spacer Plane','shock_boss_start')
    H.centered_circle(m,E,'Shock Spacer Plane','Shock Spacer Sketch',-58.3,30,9,'shock_z','shock_y','shock_boss_diameter')
    f=boss_extrude(m,3,name='Shock Spacer Boss');link_feature(E,f,['foot_left - shock_boss_start'])
    H.centered_circle(m,E,'Shock Spacer Plane','Shock Hinge Sketch',-58.3,30,3.4,'shock_z','shock_y','shock_clearance');f=cut_extrude(m,9,reverse=True,name='Shock Hinge Through Hole');link_feature(E,f,['web_right - shock_boss_start'])
    hole_set(m,E,'Deck Top Plane','Tower Foot Pilot Sketch',tower,2.8,'foot_pilot');f=cut_extrude(m,10,reverse=True,name='Tower Foot Pilots');link_feature(E,f,['foot_thickness'])
    E.solve();m.ForceRebuild3(False);es=edges_where(bodies(m)[0],lambda g:g[0]=='circle' and abs(g[1][2]-3)<1e-5 and abs(g[3]-1.4)<1e-5)
    select_edges(m,es);f=chamfer_equal(m,.5,'Tower Foot Entry Chamfers');link_feature(E,f,['foot_entry_chamfer'])
    return finish(sw,m,E,key,path)

if __name__=='__main__':
    sw=connect()
    try:
        for fn in (cup,bench,mount):
            if not sys.argv[1:] or fn.__name__ in sys.argv[1:]:fn(sw)
    finally:sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,True)
