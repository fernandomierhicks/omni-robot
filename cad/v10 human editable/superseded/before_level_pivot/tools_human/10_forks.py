"""Fork plates use tangent arc-and-line hulls, not faceted source polygons."""
from common import *
from importlib import import_module
H=import_module('08_hubs')

def tangent_hull(m,E,plane,name,circles):
    # x,y,r,x-expression,y-expression,r-expression. r=0 is a sharp vertex.
    breaks=[0,2*math.pi]
    for i,(x,y,r,*_) in enumerate(circles):
        for u,v,s,*_ in circles[i+1:]:
            dx,dy=x-u,y-v;length=math.hypot(dx,dy)
            if not length or abs(s-r)>=length:continue
            theta=math.atan2(dy,dx);delta=math.acos((s-r)/length)
            breaks.extend([(theta+delta)%(2*math.pi),(theta-delta)%(2*math.pi)])
    breaks=sorted(set(round(a,12) for a in breaks));runs=[]
    for a,b in zip(breaks,breaks[1:]):
        t=(a+b)/2;i=max(range(len(circles)),key=lambda j:circles[j][0]*math.cos(t)+circles[j][1]*math.sin(t)+circles[j][2])
        if runs and runs[-1][0]==i:runs[-1][2]=b
        else:runs.append([i,a,b])
    if len(runs)>1 and runs[0][0]==runs[-1][0]:runs[0][1]=runs[-1][1]-2*math.pi;runs.pop()
    s=Sketch(m,plane,name);pieces=[]
    for i,a,b in runs:
        x,y,r,xe,ye,re=circles[i];p=(x+r*math.cos(a),y+r*math.sin(a));q=(x+r*math.cos(b),y+r*math.sin(b))
        arc=s.arc(x,y,*p,*q,True) if r else None
        pieces.append((i,p,q,arc))
    lines=[]
    for j,(i,p,q,arc) in enumerate(pieces):
        nxt=pieces[(j+1)%len(pieces)];lines.append(s.line(*q,*nxt[1]))
    for j,(i,p,q,arc) in enumerate(pieces):
        x,y,r,xe,ye,re=circles[i]
        if arc:
            H.point_controls(s,arc.GetCenterPoint2(),x,y,xe,ye);s.rdim(arc,re,(x+5,y+5))
            s.rel('sgTANGENT',lines[j-1],arc);s.rel('sgTANGENT',arc,lines[j])
        else:H.point_controls(s,lines[j].GetStartPoint2(),x,y,xe,ye)
    s.finish(E)
    if s.status()!=3:raise RuntimeError(name+' tangent hull status '+str(s.status()))
    m.ClearSelection2(True);s.feat.Select2(False,0);return s

def variables(E,outer):
    for n,v,t in [('plate_start',48 if outer else 45,'Outer face magnitude of fork plate'),('plate_end',39 if outer else 57,'Inner face magnitude of fork plate'),('plate_thickness',9 if outer else 12,'Fork plate axial thickness'),('wheel_lobe_radius',35 if outer else 30,'Wheel-side round web lobe'),('pivot_y',68.2,'Suspension pivot Y'),('pivot_z',43,'Suspension pivot Z'),('pivot_lobe_radius',13.5,'627 bearing housing outside radius'),('shock_y',36 if outer else 30,'Lower shock hinge Y'),('shock_z',10,'Lower shock hinge Z'),('shock_lobe_radius',7.5,'Shock boss supporting web radius'),('shock_boss_radius',6.5,'Shock eye spacer outside radius'),('shock_fillet',1,'Native spacer reinforcing fillet'),('pivot_bore',16,'Pivot outer-race shoulder clearance diameter'),('pivot_seat_diameter',22,'627 bearing seat diameter'),('pivot_seat_start',46 if outer else 44.5,'Bearing seat first X face magnitude'),('pivot_seat_end',39 if outer else 51.5,'Bearing seat shoulder X magnitude'),('bearing_width',7,'627 bearing width'),('journal_radius',10,'6804 inner-race journal radius'),('journal_shoulder_radius',11.5,'6804 inner-race shoulder radius'),('wire_bore',8 if outer else 6,'Central lead clearance diameter'),('lead_chamfer',.5,'Native bearing and journal entry lead'),('bar_screw_x',43.5 if outer else 51,'Fork bar screw X magnitude'),('bar_screw_y',73,'Fork bar seating face Y'),('bar_screw_top_z',12,'Upper bar screw offset toward -Z'),('bar_screw_bottom_z',24,'Lower bar screw offset toward -Z'),('bar_thread_depth',9,'Printed bar screw engagement hole depth'),('thread_pilot',2.8,'M3 thread-forming hole diameter'),('thread_entry',.5,'Native pilot entry chamfer'),('clearance_y',73 if outer else 32.5,'Deck clearance Y boundary'),('clearance_z',29.5 if outer else 5.5,'Deck clearance Z boundary'),('clearance_span',150,'Clearance cutter overrun beyond plate outline'),('clearance_depth',40,'Clearance cutter axial overrun'),('root_fillet',2,'Native clearance corner reinforcing fillet')]:E.var(n,v,'mm',t)
    for n,v,t in [('leg_y_start',10,'Lower leg near wheel'),('leg_y_end',73,'Lower leg fork bar seat'),('leg_y_knee',64,'Lower leg top knee Y'),('leg_y_slope',59,'Lower leg upper slope end Y'),('leg_y_inner',34,'Lower leg inner shoulder Y'),('leg_y_root',20,'Lower leg wheel root Y'),('leg_bottom',28,'Lower leg bottom toward -Z'),('leg_top',8,'Lower leg top at bar toward -Z'),('leg_shoulder',13,'Lower leg inner shoulder toward -Z'),('leg_root_z',2,'Lower leg wheel root toward -Z')]:E.var(n,v,'mm',t)

def plate(sw,outer=False):
    key='outer_fork_6804' if outer else 'inner_fork_6804';m,E,path=start(sw,key,'Plate Face Plane');variables(E,outer)
    px=-48 if outer else 45;ref_plane(m,E,'Right Plane',px,'Plate Face Plane','plate_start')
    circles=[(0,0,35 if outer else 30,None,None,'wheel_lobe_radius'),(-43,68.2,13.5,'pivot_z','pivot_y','pivot_lobe_radius'),(-10,36 if outer else 30,7.5,'shock_z','shock_y','shock_lobe_radius')]
    if outer:
        E.var('lug_inner_y',57,'mm','Start of wide outer bar lug')
        circles += [(28,57,0,'leg_bottom','lug_inner_y',None),(28,73,0,'leg_bottom','leg_y_end',None),(8,57,0,'leg_top','lug_inner_y',None),(8,73,0,'leg_top','leg_y_end',None)]
    tangent_hull(m,E,'Plate Face Plane','Fork Web Outline',circles)
    f=boss_extrude(m,9 if outer else 12,name='Fork Web Extrude');link_feature(E,f,['plate_thickness'])
    # Clearance above the chassis deck. Coordinates are natural on Right plane.
    cy=73 if outer else 32.5;cz=29.5 if outer else 5.5
    s=orthogonal_profile(m,E,'Plate Face Plane','Deck Clearance Sketch',[(-cz,cy,'clearance_z','clearance_y'),(150,cy,'clearance_span','clearance_y'),(150,150,'clearance_span','clearance_span'),(-cz,150,'clearance_z','clearance_span')],axis=None)
    m.ClearSelection2(True);s.feat.Select2(False,0);f=cut_extrude(m,40,reverse=True,name='Deck Clearance Cut');link_feature(E,f,['clearance_depth'])
    if outer:
        # Restore a full-width pivot housing after the deck clearance.
        E.var('pivot_web_y_offset',8,'mm','Reinforcing pivot capsule Y offset');E.var('pivot_web_z_offset',4,'mm','Reinforcing pivot capsule Z offset')
        tangent_hull(m,E,'Plate Face Plane','Pivot Reinforcement Sketch',[(-43,68.2,13.5,'pivot_z','pivot_y','pivot_lobe_radius'),(-39,60.2,13.5,'pivot_z - pivot_web_z_offset','pivot_y - pivot_web_y_offset','pivot_lobe_radius')])
        f=boss_extrude(m,9,name='Pivot Reinforcement');link_feature(E,f,['plate_thickness'])
    else:
        E.solve();m.ForceRebuild3(False)
        es=edges_where(bodies(m)[0],lambda g:g[0]=='line' and abs(g[1][0]-g[2][0])>11.9 and all(abs(p[1]-32.5)<1e-5 and abs(p[2]-5.5)<1e-5 for p in g[1:]))
        select_edges(m,es);f=fillet_uniform(m,2,'Deck Clearance Root Fillet');link_feature(E,f,['root_fillet'])
        vs=[(28,10,'leg_bottom','leg_y_start'),(28,73,'leg_bottom','leg_y_end'),(8,73,'leg_top','leg_y_end'),(8,64,'leg_top','leg_y_knee'),(13,59,'leg_shoulder','leg_y_slope'),(13,34,'leg_shoulder','leg_y_inner'),(2,20,'leg_root_z','leg_y_root')]
        s=orthogonal_profile(m,E,'Plate Face Plane','Lower Fork Leg Sketch',vs,axis=None);m.ClearSelection2(True);s.feat.Select2(False,0);f=boss_extrude(m,12,name='Lower Fork Leg');link_feature(E,f,['plate_thickness'])
    finish(sw,m,E,key,path)

def interfaces(sw,outer=False):
    key='outer_fork_6804' if outer else 'inner_fork_6804';m,E,path=H.resume(sw,key)
    delete_from(m,'Wheel Inner Face Plane' if outer else 'Wheel Journal Section')
    for n,v,t in [('journal_start',52 if outer else 37,'Journal outer axial coordinate magnitude'),('journal_end',43.5 if outer else 44,'Journal inner axial coordinate magnitude'),('journal_shoulder_end',50.5 if outer else 45,'Journal shoulder axial coordinate magnitude'),('journal_shoulder_start',52 if outer else 44,'Journal shoulder first axial coordinate magnitude'),('shock_boss_face',51 if outer else 59.5,'Shock spacer free face X magnitude'),('shock_thread_depth',10 if outer else 8.5,'Lower shock printed pilot depth'),('cup_screw_radius',6.5,'Cradle to inner fork shared screw pitch radius'),('cup_clearance',3.2,'Cradle screw clearance diameter'),('cup_head_diameter',7.2,'Cradle screw head counterbore'),('cup_head_depth',2.9,'Cradle screw head recess depth')]:E.var(n,v,'mm',t)
    if outer:
        for n,v,t in [('cap_outer_face',54.5,'Cap exterior face toward -X'),('cap_inner_face',44.5,'Cap flange inner face toward -X'),('cap_radius',35,'6804 outer cap outside radius'),('cartridge_flange_clearance',33.5,'Cartridge flange sweep clearance radius'),('cartridge_barrel_clearance',22.5,'Cartridge barrel running clearance radius'),('cap_cavity_end',51.5,'Cap cavity blind end toward -X'),('shock_eye_clearance',9,'Lower shock eye clearance at the outer cap')]:E.var(n,v,'mm',t)
        ref_plane(m,E,'Right Plane',-39,'Wheel Inner Face Plane','plate_end')
        H.centered_circle(m,E,'Wheel Inner Face Plane','Cartridge Flange Clearance Sketch',0,0,67,None,None,'2 * cartridge_flange_clearance')
        f=cut_extrude(m,5.5,name='Cartridge Flange Clearance');link_feature(E,f,['cap_inner_face - plate_end'])
        H.centered_circle(m,E,'Wheel Inner Face Plane','Cartridge Barrel Clearance Sketch',0,0,45,None,None,'2 * cartridge_barrel_clearance')
        f=cut_extrude(m,12.5,name='Cartridge Barrel Clearance');link_feature(E,f,['cap_cavity_end - plate_end'])
        vs=[(-54.5,4,'cap_outer_face','wire_bore / 2'),(-54.5,35,'cap_outer_face','cap_radius'),(-44.5,35,'cap_inner_face','cap_radius'),(-44.5,22.5,'cap_inner_face','cartridge_barrel_clearance'),(-51.5,22.5,'cap_cavity_end','cartridge_barrel_clearance'),(-51.5,11.5,'cap_cavity_end','journal_shoulder_radius'),(-50.5,11.5,'journal_shoulder_end','journal_shoulder_radius'),(-50.5,10,'journal_shoulder_end','journal_radius'),(-43.5,10,'journal_end','journal_radius'),(-43.5,4,'journal_end','wire_bore / 2')]
        s=orthogonal_profile(m,E,'Front Plane','Outer Cap Section',vs,axis='horizontal');full_revolve(m,E,s,'Outer Cap and Journal Revolve')
        ref_plane(m,E,'Right Plane',-54.5,'Cap Exterior Plane','cap_outer_face')
        H.centered_circle(m,E,'Cap Exterior Plane','Shock Eye Relief Sketch',-10,36,9,'shock_z','shock_y','shock_eye_clearance');f=cut_extrude(m,3,reverse=True,name='Shock Eye Cap Relief');link_feature(E,f,['cap_outer_face - cap_cavity_end'])
    else:
        vs=[(37,0,'journal_start',None),(37,10,'journal_start','journal_radius'),(44,10,'journal_end','journal_radius'),(44,11.5,'journal_end','journal_shoulder_radius'),(45,11.5,'journal_shoulder_end','journal_shoulder_radius'),(45,0,'journal_shoulder_end',None)]
        s=orthogonal_profile(m,E,'Front Plane','Wheel Journal Section',vs,axis='horizontal');full_revolve(m,E,s,'Wheel Journal Revolve')
        ref_plane(m,E,'Right Plane',57,'Inner Fork Back Plane','plate_end')
        H.centered_circle(m,E,'Inner Fork Back Plane','Wire Bore Sketch',0,0,6,None,None,'wire_bore');cut_extrude(m,through=True,name='Wire Through Bore')
        E.var('cup_first_angle',90,'deg','First cradle screw from +Y toward +Z');E.var('cup_screw_count',4,'','Four cradle attachment screws')
        polar_hole(m,E,'Inner Fork Back Plane','Cradle Clearance Sketch',6.5,90,3.2,'cup_screw_radius','cup_first_angle','cup_clearance');f=cut_extrude(m,20,name='Cradle Clearance');link_feature(E,f,['plate_end - journal_start'])
        polar_hole(m,E,'Inner Fork Back Plane','Cradle Counterbore Sketch',6.5,90,7.2,'cup_screw_radius','cup_first_angle','cup_head_diameter');f=cut_extrude(m,2.9,name='Cradle Counterbore');link_feature(E,f,['cup_head_depth'])
        make_axis(m,'Wheel Axis','Front Plane','Top Plane');patterned(m,E,'Wheel Axis',['Cradle Clearance','Cradle Counterbore'],4,'cup_screw_count','Cradle Mount Pattern')
    E.solve();m.ForceRebuild3(False)
    end=-43.5 if outer else 37
    es=edges_where(bodies(m)[0],lambda g:g[0]=='circle' and abs(g[1][0]-end)<1e-5 and abs(g[3]-10)<1e-5)
    select_edges(m,es);f=chamfer_equal(m,.5,'Journal Lead Chamfer');link_feature(E,f,['lead_chamfer'])
    # One master pivot axis; seat and through bore remain independently sized.
    H.centered_circle(m,E,'Plate Face Plane','Pivot Through Bore Sketch',-43,68.2,16,'pivot_z','pivot_y','pivot_bore')
    f=cut_extrude(m,9 if outer else 12,reverse=True,name='Pivot Through Bore');link_feature(E,f,['plate_thickness'])
    ref_plane(m,E,'Right Plane',-39 if outer else 44.5,'Pivot Seat Entry Plane','pivot_seat_end' if outer else 'pivot_seat_start')
    H.centered_circle(m,E,'Pivot Seat Entry Plane','Pivot Bearing Seat Sketch',-43,68.2,22,'pivot_z','pivot_y','pivot_seat_diameter')
    f=cut_extrude(m,7,reverse=not outer,name='Pivot Bearing Seat');link_feature(E,f,['bearing_width'])
    E.solve();m.ForceRebuild3(False);entry=-39 if outer else 45
    es=edges_where(bodies(m)[0],lambda g:g[0]=='circle' and abs(g[1][0]-entry)<1e-5 and abs(g[1][1]-68.2)<1e-5 and abs(g[1][2]-43)<1e-5 and abs(g[3]-11)<1e-5)
    select_edges(m,es);f=chamfer_equal(m,.5,'Pivot Bearing Lead In');link_feature(E,f,['lead_chamfer'])
    ref_plane(m,E,'Right Plane',-51 if outer else 57,'Shock Boss Root Plane','shock_boss_face' if outer else 'plate_end')
    H.centered_circle(m,E,'Shock Boss Root Plane','Lower Shock Boss Sketch',-10,36 if outer else 30,13,'shock_z','shock_y','2 * shock_boss_radius')
    f=boss_extrude(m,3 if outer else 2.5,name='Lower Shock Boss');link_feature(E,f,['shock_boss_face - plate_start' if outer else 'shock_boss_face - plate_end'])
    E.solve();m.ForceRebuild3(False);root=-48 if outer else 57
    es=edges_where(bodies(m)[0],lambda g:g[0]=='circle' and abs(g[1][0]-root)<1e-5 and abs(g[1][1]-(36 if outer else 30))<1e-5 and abs(g[1][2]-10)<1e-5 and abs(g[3]-6.5)<1e-5)
    es=[e for e in es if any(wrap(face,sld.IFace2).Normal[0] < -.9 if outer else wrap(face,sld.IFace2).Normal[0] > .9 for face in e.GetTwoAdjacentFaces2())]
    select_edges(m,es);f=fillet_uniform(m,1,'Lower Shock Boss Root Fillet');link_feature(E,f,['shock_fillet'])
    ref_plane(m,E,'Right Plane',-51 if outer else 59.5,'Shock Screw Entry Plane','shock_boss_face')
    H.centered_circle(m,E,'Shock Screw Entry Plane','Lower Shock Pilot Sketch',-10,36 if outer else 30,2.8,'shock_z','shock_y','thread_pilot')
    f=cut_extrude(m,10 if outer else 8.5,reverse=outer,name='Lower Shock Pilot');link_feature(E,f,['shock_thread_depth'])
    ref_plane(m,E,'Top Plane',73,'Fork Bar Seat Plane','bar_screw_y')
    s=Sketch(m,'Fork Bar Seat Plane','Fork Bar Pilot Sketch')
    for z,expr in [(12,'bar_screw_top_z'),(24,'bar_screw_bottom_z')]:
        ci=s.circle(-43.5 if outer else 51,z,1.4);H.point_controls(s,ci.GetCenterPoint2(),-43.5 if outer else 51,z,'bar_screw_x',expr);s.ddim(ci,'thread_pilot',(55,z+3))
    s.finish(E);m.ClearSelection2(True);s.feat.Select2(False,0);f=cut_extrude(m,9,name='Fork Bar Thread Pilots');link_feature(E,f,['bar_thread_depth'])
    E.solve();m.ForceRebuild3(False);es=edges_where(bodies(m)[0],lambda g:g[0]=='circle' and abs(g[1][1]-73)<1e-5 and abs(g[3]-1.4)<1e-5)
    select_edges(m,es);f=chamfer_equal(m,.5,'Fork Bar Pilot Entry Chamfers');link_feature(E,f,['thread_entry'])
    return finish(sw,m,E,key,path)

if __name__=='__main__':
    sw=connect()
    try:
        if 'interfaces' not in sys.argv[1:]:plate(sw,'outer' in sys.argv[1:])
        if 'base' not in sys.argv[1:]:interfaces(sw,'outer' in sys.argv[1:])
    finally:sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,True)

