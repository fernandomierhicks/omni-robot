from common import *
from importlib import import_module
H=import_module('08_hubs');F=import_module('09_cup_bench_mount')

def box(m,E,name,x,y,z,expressions):
    xe,ye,ze=expressions
    plane='Front Plane'
    if z[0]:ref_plane(m,E,plane,z[0],name+' Base Plane',ze[0]);plane=name+' Base Plane'
    vs=[(x[0],y[0],xe[0],ye[0]),(x[1],y[0],xe[1],ye[0]),(x[1],y[1],xe[1],ye[1]),(x[0],y[1],xe[0],ye[1])]
    s=orthogonal_profile(m,E,plane,name+' Sketch',vs,axis=None);m.ClearSelection2(True);s.feat.Select2(False,0)
    f=boss_extrude(m,z[1]-z[0],name=name+' Extrude');link_feature(E,f,[ze[1]+' - '+ze[0]])

def variables(E):
    vals=[('deck_centre_x',146,'Deck circle centre relative to wheel axle'),('deck_radius',217,'Guard outside deck radius'),('panel_thickness',5,'Curved impact panel radial thickness'),('panel_half_width',89.5,'Panel outer transverse extent'),('panel_bottom',22,'Panel lower edge toward -Z'),('panel_top',70.5,'Panel top edge Z'),('deck_height',3,'Chassis deck top'),('wall_top',57,'Full side wall and inner ear top'),('low_wall_top',28,'Outer plate clearance wall top'),('outer_ear_top',53,'Outboard ear top'),('house_y',83.5,'Side wall inside transverse coordinate'),('wall_thickness',6,'Side wall thickness'),('house_end_x',24,'Side wall inner end X'),('wall_front_x',57.01444671179024,'Panel and wall overlap start toward -X'),('foot_front_x',20,'Far side foot start toward -X'),('foot_y_inner',82.5,'Foot strip near transverse edge'),('foot_y_outer',92.5,'Foot strip outer transverse edge'),('foot_top',11,'Foot top Z'),('inner_ear_x0',36.5,'Inner ear exterior face X'),('inner_ear_x1',44,'Inner ear interior face X'),('outer_ear_x0',36.5,'Outer ear exterior face toward -X'),('outer_ear_x1',26.5,'Outer ear interior face toward -X'),('ear_y_min',58.2,'Ear near wheel edge Y'),('ear_y_max',89.5,'Ear far edge Y'),('outer_ear_wheel_clearance',72,'Wheel running clearance at outboard ear'),('pivot_y',68.2,'Suspension pivot Y'),('pivot_z',43,'Suspension pivot Z'),('shaft_bore',7.1,'627 shaft slip clearance'),('shaft_lead',.5,'Native shaft bore entry lead'),('pin_x',40.25,'Inner ear shaft retention screw X'),('pin_clearance',3.4,'M3 pin clearance above shaft'),('pin_pilot',2.8,'M3 pin thread pilot below shaft'),('pin_pilot_top',40,'Pin pilot entry below shaft centre'),('pin_pilot_depth',11,'Pin engagement hole depth'),('deck_slot_edge',78.8,'Deck lobe relief transverse boundary'),('deck_slot_depth',6.4,'Deck lobe relief slab thickness'),('cut_span',500,'Cutter boundary beyond guard envelope'),('panel_root_fillet',5,'Native side wall to curved panel reinforcing fillet'),('outer_shock_y',36,'Fixed outer shock transverse coordinate'),('outer_shock_z',58,'Fixed outer shock height'),('shock_boss_start',65.14672666178214,'Shock boss embedded end toward -X'),('shock_boss_end',58,'Shock eye seating face toward -X'),('shock_boss_diameter',9,'Fixed shock boss outside diameter'),('shock_pilot',2.8,'M3 shock thread-form pilot'),('shock_pilot_length',12,'Shock pilot cutter through panel'),('lower_eye_z',10,'Lower shock screw driver height at nominal pose'),('driver_access',7,'Lower shock driver passage diameter'),('driver_cut_length',20,'Driver clearance through panel'),('foot_pilot',2.8,'M3 deck foot pilot'),('foot_entry',.5,'Native deck pilot lead'),('foot_thread_depth',8,'Deck foot engagement hole depth')]
    for n,v,t in vals:E.var(n,v,'mm',t)
    F.foot_variables(E)

def base(sw):
    key='wheel_guard';m,E,path=start(sw,key,'Panel Base Plane');variables(E)
    ref_plane(m,E,'Front Plane',-22,'Panel Base Plane','panel_bottom')
    s=Sketch(m,'Panel Base Plane','Curved Panel Sketch');a=math.asin(89.5/217);xo=146-217*math.cos(a);xi=146-212*math.cos(a);yi=212*math.sin(a)
    oa=s.arc(146,0,xo,-89.5,xo,89.5,False);up=s.line(xo,89.5,xi,yi);ia=s.arc(146,0,xi,yi,xi,-yi,True);low=s.line(xi,-yi,xo,-89.5)
    arc_controls(s,oa,E,'deck_centre_x','deck_radius');s.rel('sgCONCENTRIC',oa,ia);s.rdim(ia,'deck_radius - panel_thickness',(xi+4,0))
    for line in (up,low):s.rel('sgCOINCIDENT',oa.GetCenterPoint2(),line)
    for pt in (oa.GetStartPoint2(),oa.GetEndPoint2()):s.vdim(pt,None,'panel_half_width',(xo-5,0),origin_b=True)
    s.finish(E)
    if s.status()!=3:raise RuntimeError('Curved panel status '+str(s.status()))
    m.ClearSelection2(True);s.feat.Select2(False,0);f=boss_extrude(m,92.5,name='Curved Panel Extrude');link_feature(E,f,['panel_top + panel_bottom'])
    box(m,E,'Far Wall',(-57.01444671179024,24),(-89.5,-83.5),(3,57),(('wall_front_x','house_end_x'),('house_y + wall_thickness','house_y'),('deck_height','wall_top')))
    box(m,E,'Far Foot',(-20,24),(-92.5,-82.5),(3,11),(('foot_front_x','house_end_x'),('foot_y_outer','foot_y_inner'),('deck_height','foot_top')))
    box(m,E,'Pivot Side Wall',(-26.5,36.5),(83.5,89.5),(3,57),(('outer_ear_x1','inner_ear_x0'),('house_y','house_y + wall_thickness'),('deck_height','wall_top')))
    box(m,E,'Outer Low Wall',(-57.01444671179024,-36.5),(83.5,89.5),(3,28),(('wall_front_x','outer_ear_x0'),('house_y','house_y + wall_thickness'),('deck_height','low_wall_top')))
    box(m,E,'Pivot Side Foot',(-36.5,44),(82.5,92.5),(3,11),(('outer_ear_x0','inner_ear_x1'),('foot_y_inner','foot_y_outer'),('deck_height','foot_top')))
    box(m,E,'Inner Shaft Ear',(36.5,44),(58.2,89.5),(3,57),(('inner_ear_x0','inner_ear_x1'),('ear_y_min','ear_y_max'),('deck_height','wall_top')))
    ref_plane(m,E,'Right Plane',-36.5,'Outer Ear Face Plane','outer_ear_x0')
    s=Sketch(m,'Outer Ear Face Plane','Outer Ear Wheel Clearance Sketch');yc=math.sqrt(72**2-3**2);xc=-math.sqrt(72**2-58.2**2)
    p=[(-3,yc),(-3,89.5),(-53,89.5),(-53,58.2),(xc,58.2)];ls=[s.line(*u,*v) for u,v in zip(p,p[1:])];ar=s.arc(0,0,*p[-1],*p[0],False)
    for j in (0,2):s.rel('sgVERTICAL2D',ls[j])
    for j in (1,3):s.rel('sgHORIZONTAL2D',ls[j])
    s.rel('sgCOINCIDENT',ar.GetCenterPoint2(),origin=True);s.rdim(ar,'outer_ear_wheel_clearance',(-45,45))
    s.hdim(ls[0].GetStartPoint2(),None,'deck_height',(-5,70),origin_b=True);s.vdim(ls[0].GetEndPoint2(),None,'ear_y_max',(-5,93),origin_b=True)
    s.hdim(ls[1].GetEndPoint2(),None,'outer_ear_top',(-57,90),origin_b=True);s.vdim(ls[2].GetEndPoint2(),None,'ear_y_min',(-57,60),origin_b=True)
    s.finish(E)
    if s.status()!=3:raise RuntimeError('Outboard ear status '+str(s.status()))
    m.ClearSelection2(True);s.feat.Select2(False,0);f=boss_extrude(m,10,name='Outer Shaft Ear');link_feature(E,f,['outer_ear_x0 - outer_ear_x1'])
    return finish(sw,m,E,key,path)

def interfaces(sw):
    key='wheel_guard';m,E,path=H.resume(sw,key);delete_from(m,'Inner Shaft Entry Plane')
    for label,x,depth,expr,rev in [('Inner',44,7.5,'inner_ear_x1 - inner_ear_x0',False),('Outer',-36.5,10,'outer_ear_x0 - outer_ear_x1',True)]:
        ref_plane(m,E,'Right Plane',x,label+' Shaft Entry Plane','inner_ear_x1' if label=='Inner' else 'outer_ear_x0')
        H.centered_circle(m,E,label+' Shaft Entry Plane',label+' Shaft Bore Sketch',-43,68.2,7.1,'pivot_z','pivot_y','shaft_bore')
        f=cut_extrude(m,depth,reverse=rev,name=label+' Shaft Bore');link_feature(E,f,[expr])
    E.solve();m.ForceRebuild3(False)
    es=edges_where(bodies(m)[0],lambda g:g[0]=='circle' and abs(g[3]-3.55)<1e-5)
    select_edges(m,es);f=chamfer_equal(m,.5,'Shaft Bore Entry Chamfers');link_feature(E,f,['shaft_lead'])
    ref_plane(m,E,'Front Plane',57,'Pin Head Plane','wall_top');H.centered_circle(m,E,'Pin Head Plane','Pin Clearance Sketch',40.25,68.2,3.4,'pin_x','pivot_y','pin_clearance')
    f=cut_extrude(m,14,name='Pin Clearance');link_feature(E,f,['wall_top - pivot_z'])
    ref_plane(m,E,'Front Plane',40,'Pin Pilot Entry Plane','pin_pilot_top');H.centered_circle(m,E,'Pin Pilot Entry Plane','Pin Pilot Sketch',40.25,68.2,2.8,'pin_x','pivot_y','pin_pilot')
    f=cut_extrude(m,11,name='Pin Thread Pilot');link_feature(E,f,['pin_pilot_depth'])
    # Relief where the surrounding deck lobes cross the impact panel.
    s=orthogonal_profile(m,E,'Front Plane','Deck Lobe Relief Sketch',[(-500,78.8,'cut_span','deck_slot_edge'),(500,78.8,'cut_span','deck_slot_edge'),(500,500,'cut_span','cut_span'),(-500,500,'cut_span','cut_span')],axis=None)
    m.ClearSelection2(True);s.feat.Select2(False,0);f=cut_extrude(m,6.4,mid=True,name='Deck Lobe Relief');link_feature(E,f,['deck_slot_depth'])
    mirror_features(m,'Top Plane',['Deck Lobe Relief'],'Opposite Deck Lobe Relief')
    ref_plane(m,E,'Right Plane',-65.14672666178214,'Shock Boss Start Plane','shock_boss_start');H.centered_circle(m,E,'Shock Boss Start Plane','Fixed Shock Boss Sketch',-58,36,9,'outer_shock_z','outer_shock_y','shock_boss_diameter')
    f=boss_extrude(m,7.14672666178214,name='Fixed Shock Boss');link_feature(E,f,['shock_boss_start - shock_boss_end'])
    ref_plane(m,E,'Right Plane',-58,'Shock Eye Seat Plane','shock_boss_end');H.centered_circle(m,E,'Shock Eye Seat Plane','Fixed Shock Pilot Sketch',-58,36,2.8,'outer_shock_z','outer_shock_y','shock_pilot')
    f=cut_extrude(m,12,name='Fixed Shock Pilot');link_feature(E,f,['shock_pilot_length'])
    H.centered_circle(m,E,'Shock Eye Seat Plane','Lower Shock Driver Sketch',-10,36,7,'lower_eye_z','outer_shock_y','driver_access');f=cut_extrude(m,20,name='Lower Shock Driver Access');link_feature(E,f,['driver_cut_length'])
    guard,_=F.foot_variables(E);ref_plane(m,E,'Front Plane',3,'Deck Foot Seat Plane','deck_height');F.hole_set(m,E,'Deck Foot Seat Plane','Deck Foot Pilot Sketch',guard,2.8,'foot_pilot')
    f=cut_extrude(m,8,reverse=True,name='Deck Foot Pilots');link_feature(E,f,['foot_thread_depth'])
    E.solve();m.ForceRebuild3(False);es=edges_where(bodies(m)[0],lambda g:g[0]=='circle' and abs(g[1][2]-3.2)<1e-5 and abs(g[3]-1.4)<1e-5)
    select_edges(m,es);f=chamfer_equal(m,.5,'Deck Foot Pilot Leads');link_feature(E,f,['foot_entry'])
    # Trim all wall overlap at the true deck circle, after the joins.
    s=orthogonal_profile(m,E,'Front Plane','Deck Envelope Trim Sketch',[(-500,-500,'cut_span','cut_span'),(500,-500,'cut_span','cut_span'),(500,500,'cut_span','cut_span'),(-500,500,'cut_span','cut_span')],axis=None)
    m.ClearSelection2(True);s.feat.Select2(False,0);m.SketchManager.InsertSketch(True);sk=wrap(m.SketchManager.ActiveSketch,sld.ISketch)
    # Continue the sketch with a dimensioned circle; the nested contour keeps its inside.
    sm=m.SketchManager;sm.AddToDB=True;ci=wrap(sm.CreateCircleByRadius(.146,0,0,.217),sld.ISketchArc);sm.AddToDB=False
    s.sm=sm;s.sk=sk;s.dims=[];s.rel('sgHORIZONTALPOINTS2D',ci.GetCenterPoint2(),origin=True);s.hdim(ci.GetCenterPoint2(),None,'deck_centre_x',(140,-5),origin_b=True);s.ddim(ci,'2 * deck_radius',(0,225));s.finish(E)
    m.ClearSelection2(True);s.feat.Select2(False,0);f=cut_extrude(m,500,mid=True,name='True Deck Circle Trim');link_feature(E,f,['cut_span'])
    return finish(sw,m,E,key,path)

def radial_cylinder(m,E,plane,name,angle,ae,start,end,se,ee,radius,re,cut=False):
    a=math.radians(angle);vs=[]
    for r,t,r_ex,t_ex in [(start,0,se,'0'),(end,0,ee,'0'),(end,radius,ee,re),(start,radius,se,re)]:
        xx=146-r*math.cos(a)+t*math.sin(a);yy=-r*math.sin(a)-t*math.cos(a)
        vs.append((xx,yy,f'abs(deck_centre_x - ({r_ex}) * cos({ae}) + ({t_ex}) * sin({ae}))',f'abs(- ({r_ex}) * sin({ae}) - ({t_ex}) * cos({ae}))'))
    s=orthogonal_profile(m,E,plane,name+' Section',vs,axis=None)
    # Re-enter the section to add the fully dimensioned radial construction axis.
    m.ClearSelection2(True);s.feat.Select2(False,0);m.SketchManager.InsertSketch(True);s.sk=wrap(m.SketchManager.ActiveSketch,sld.ISketch);s.sm=m.SketchManager;s.dims=[]
    ax=s.cline(146,0,146-10.123*math.cos(a),-10.123*math.sin(a))
    H.point_controls(s,ax.GetStartPoint2(),146,0,'deck_centre_x',None)
    H.point_controls(s,ax.GetEndPoint2(),146-10.123*math.cos(a),-10.123*math.sin(a),f'deck_centre_x - axis_reference_halfspan * cos({ae})',f'axis_reference_halfspan * sin({ae})')
    s.finish(E)
    if s.status()!=3:raise RuntimeError(name+' status '+str(s.status()))
    select_revolve(s)
    if cut:
        f=m.FeatureManager.FeatureRevolve2(True,True,False,True,False,False,0,0,2*math.pi,0,False,False,0,0,0,0,0,True,True,True)
        if f is None:raise RuntimeError('Radial cut failed '+name)
        f=named(m,name);link_feature(E,f,['full_turn'])
    else:full_revolve(m,E,s,name)

def bumper(sw):
    key='wheel_guard';m,E,path=H.resume(sw,key);delete_from(m,'Panel Wall Root Fillets')
    for n,v,t in [('bumper_boss_start',206,'Bumper boss inner radial face'),('bumper_boss_end',212.5,'Boss embedded end inside curved panel'),('bumper_boss_radius',4.5,'Bumper screw support radius'),('bumper_pilot_start',205.5,'Bumper pilot inner cutter start'),('bumper_pilot_end',217.5,'Bumper pilot outer cutter end'),('bumper_pilot_radius',1.4,'M3 bumper thread pilot radius'),('centre_bumper_height',15,'Centre bumper screw height magnitude'),('side_bumper_low',10,'Lower side bumper screw Z'),('side_bumper_high',18,'Upper side bumper screw Z'),('axis_reference_halfspan',10.123,'Construction axis display length; no physical effect')]:E.var(n,v,'mm',t)
    E.var('bumper_side_angle',23.35835493834063,'deg','Side bumper screw index from outward -X');E.var('full_turn',360,'deg','Full circular extent for revolves and patterns')
    E.solve();m.ForceRebuild3(False)
    def root(g):
        if g[0]!='line':return False
        p,q=g[1:];return abs(p[2]-q[2])>20 and abs(abs(p[1])-83.5)<1e-4 and abs(abs(q[1])-83.5)<1e-4 and -50<p[0]<-47 and abs(p[0]-q[0])<1e-5
    es=edges_where(bodies(m)[0],root)
    if len(es)!=2:raise RuntimeError('Expected two panel-wall junctions, got '+str(len(es)))
    select_edges(m,es);f=fillet_uniform(m,5,'Panel Wall Root Fillets');link_feature(E,f,['panel_root_fillet'])
    ref_plane(m,E,'Front Plane',-15,'Centre Bumper Plane','centre_bumper_height')
    radial_cylinder(m,E,'Centre Bumper Plane','Centre Bumper Boss',0,'0deg',206,212.5,'bumper_boss_start','bumper_boss_end',4.5,'bumper_boss_radius')
    radial_cylinder(m,E,'Centre Bumper Plane','Centre Bumper Pilot',0,'0deg',205.5,217.5,'bumper_pilot_start','bumper_pilot_end',1.4,'bumper_pilot_radius',True)
    mirror_features(m,'Front Plane',['Centre Bumper Boss','Centre Bumper Pilot'],'Upper Centre Bumper Mount')
    ref_plane(m,E,'Front Plane',10,'Side Bumper Plane','side_bumper_low')
    radial_cylinder(m,E,'Side Bumper Plane','Side Bumper Boss',23.35835493834063,'bumper_side_angle',206,212.5,'bumper_boss_start','bumper_boss_end',4.5,'bumper_boss_radius')
    radial_cylinder(m,E,'Side Bumper Plane','Side Bumper Pilot',23.35835493834063,'bumper_side_angle',205.5,217.5,'bumper_pilot_start','bumper_pilot_end',1.4,'bumper_pilot_radius',True)
    mirror_features(m,'Top Plane',['Side Bumper Boss','Side Bumper Pilot'],'Opposite Side Bumper Mount')
    ref_plane(m,E,'Front Plane',14,'Bumper Row Mirror Plane','(side_bumper_low + side_bumper_high) / 2')
    mirror_features(m,'Bumper Row Mirror Plane',['Side Bumper Boss','Side Bumper Pilot','Opposite Side Bumper Mount'],'Upper Side Bumper Mounts')
    return finish(sw,m,E,key,path)

if __name__=='__main__':
    sw=connect()
    try:
        stage=sys.argv[1] if sys.argv[1:] else 'all'
        if stage in ('all','base'):base(sw)
        if stage in ('all','interfaces'):interfaces(sw)
        if stage in ('all','bumper'):bumper(sw)
    finally:sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,True)
