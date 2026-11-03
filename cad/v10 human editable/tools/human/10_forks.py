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

# Nov 2 2026: the pivot is level with the axle (design.md 4.7): pivot (y, z) = (80.6, 0) at nominal ride height. The plates are
# one tangent hull from the wheel lobe to the 627 lobe; the deck is slotted around them (the robot deck), so the old deck
# clearance cut and lower leg are gone. The cross bar seats in a notch under the pivot lobe.
PIVOT_Y,SHOCK_Y,SHOCK_Z,BAR_TOP,BAR_SCREWS=80.6,37,11,13,(17,29)       # bar roof 13: a 2 mm skin under the 627 (r 11)

def variables(E,outer):
    for n,v,t in [('plate_start',48 if outer else 45,'Outer face magnitude of fork plate'),('plate_end',39 if outer else 57,'Inner face magnitude of fork plate'),('plate_thickness',9 if outer else 12,'Fork plate axial thickness'),('wheel_lobe_radius',35 if outer else 30,'Wheel-side round web lobe'),('pivot_y',PIVOT_Y,'Suspension pivot Y; level with the axle (a relation holds its height at 0)'),('pivot_lobe_radius',13.5,'627 bearing housing outside radius'),('shock_y',SHOCK_Y,'Lower shock hinge Y'),('shock_z',SHOCK_Z,'Lower shock hinge Z'),('shock_lobe_radius',7.5,'Shock boss supporting web radius'),('shock_boss_radius',6.5,'Shock eye spacer outside radius'),('shock_fillet',1,'Native spacer reinforcing fillet'),('pivot_bore',16,'Pivot outer-race shoulder clearance diameter'),('pivot_seat_diameter',22,'627 bearing seat diameter'),('pivot_seat_start',46 if outer else 44.5,'Bearing seat first X face magnitude'),('pivot_seat_end',39 if outer else 51.5,'Bearing seat shoulder X magnitude'),('bearing_width',7,'627 bearing width'),('journal_radius',10,'6804 inner-race journal radius'),('journal_shoulder_radius',11.5,'6804 inner-race shoulder radius'),('wire_bore',8 if outer else 6,'Central lead clearance diameter'),('lead_chamfer',.5,'Native bearing and journal entry lead'),('bar_screw_x',43.5 if outer else 51,'Fork bar screw X magnitude'),('bar_screw_y',73,'Fork bar seating face Y'),('bar_screw_top_z',BAR_SCREWS[0],'Upper bar screw offset toward -Z'),('bar_screw_bottom_z',BAR_SCREWS[1],'Lower bar screw offset toward -Z'),('bar_thread_depth',9,'Printed bar screw engagement hole depth'),('thread_pilot',2.8,'M3 thread-forming hole diameter'),('thread_entry',.5,'Native pilot entry chamfer')]:E.var(n,v,'mm',t)
    for n,v,t in [('lug_inner_y',57,'Bar lug start Y'),('lug_seat_y',73,'Bar lug seating face Y'),('lug_bottom',32,'Bar lug bottom toward -Z'),('bar_seat_top',BAR_TOP,'Bar notch roof toward -Z, under the 627 lobe'),('bar_notch_span',150,'Bar notch cutter overrun beyond the plate outline'),('bar_notch_depth',40,'Bar notch cutter axial overrun')]:E.var(n,v,'mm',t)

def plate(sw,outer=False):
    key='outer_fork_6804' if outer else 'inner_fork_6804';m,E,path=start(sw,key,'Plate Face Plane')
    drop_globals(m,['pivot_z','clearance_y','clearance_z','clearance_span','clearance_depth','root_fillet','pivot_web_y_offset','pivot_web_z_offset']+['leg_'+n for n in ('y_start','y_end','y_knee','y_slope','y_inner','y_root','bottom','top','shoulder','root_z')])
    variables(E,outer)
    px=-48 if outer else 45;ref_plane(m,E,'Right Plane',px,'Plate Face Plane','plate_start')
    # Right-plane sketch coordinates are (-pod Z, pod Y): the pivot lobe sits on the axle's horizontal (x = 0, a relation).
    circles=[(0,0,35 if outer else 30,None,None,'wheel_lobe_radius'),(0,PIVOT_Y,13.5,None,'pivot_y','pivot_lobe_radius'),(-SHOCK_Z,SHOCK_Y,7.5,'shock_z','shock_y','shock_lobe_radius'),
             (32,57,0,'lug_bottom','lug_inner_y',None),(32,73,0,'lug_bottom','lug_seat_y',None)]
    tangent_hull(m,E,'Plate Face Plane','Fork Web Outline',circles)
    f=boss_extrude(m,9 if outer else 12,name='Fork Web Extrude');link_feature(E,f,['plate_thickness'])
    # The cross bar's seat: everything beyond the lug face and below the notch roof is cut away.
    s=orthogonal_profile(m,E,'Plate Face Plane','Bar Seat Sketch',[(BAR_TOP,73,'bar_seat_top','lug_seat_y'),(150,73,'bar_notch_span','lug_seat_y'),(150,150,'bar_notch_span','bar_notch_span'),(BAR_TOP,150,'bar_seat_top','bar_notch_span')],axis=None)
    m.ClearSelection2(True);s.feat.Select2(False,0);f=cut_extrude(m,40,reverse=True,name='Bar Seat Cut');link_feature(E,f,['bar_notch_depth'])
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
        H.centered_circle(m,E,'Cap Exterior Plane','Shock Eye Relief Sketch',-SHOCK_Z,SHOCK_Y,9,'shock_z','shock_y','shock_eye_clearance');f=cut_extrude(m,3,reverse=True,name='Shock Eye Cap Relief');link_feature(E,f,['cap_outer_face - cap_cavity_end'])
    else:
        vs=[(37,0,'journal_start',None),(37,10,'journal_start','journal_radius'),(44,10,'journal_end','journal_radius'),(44,11.5,'journal_end','journal_shoulder_radius'),(45,11.5,'journal_shoulder_end','journal_shoulder_radius'),(45,0,'journal_shoulder_end',None)]
        s=orthogonal_profile(m,E,'Front Plane','Wheel Journal Section',vs,axis='horizontal');full_revolve(m,E,s,'Wheel Journal Revolve')
        ref_plane(m,E,'Right Plane',57,'Inner Fork Back Plane','plate_end')
        H.centered_circle(m,E,'Inner Fork Back Plane','Wire Bore Sketch',0,0,6,None,None,'wire_bore');cut_extrude(m,through=True,name='Wire Through Bore')
        E.var('cup_first_angle',90,'deg','First cradle screw from +Y toward +Z');E.var('cup_screw_count',4,'','Four cradle attachment screws')
        polar_hole(m,E,'Inner Fork Back Plane','Cradle Clearance Sketch',6.5,90,3.2,'cup_screw_radius','cup_first_angle','cup_clearance');f=cut_extrude(m,20,name='Cradle Clearance');link_feature(E,f,['plate_end - journal_start'])
        polar_hole(m,E,'Inner Fork Back Plane','Cradle Counterbore Sketch',6.5,90,7.2,'cup_screw_radius','cup_first_angle','cup_head_diameter');f=cut_extrude(m,2.9,name='Cradle Counterbore');link_feature(E,f,['cup_head_depth'])
        make_axis(m,'Wheel Axis','Front Plane','Top Plane');patterned(m,E,'Wheel Axis',['Cradle Clearance','Cradle Counterbore'],4,'cup_screw_count','Cradle Mount Pattern')
        # The payload plate's corner standoff flange (r 10 at x 56.8 from the pod) sits just beyond the plate's back face at the
        # pivot: a shallow face relief around the pivot keeps 1.8 mm from it at every fork angle (the relief is round about the pivot).
        E.var('standoff_relief_radius',16,'mm','Back-face relief radius around the pivot (payload standoff clearance)');E.var('standoff_relief_depth',2,'mm','Back-face relief depth at the pivot')
        H.centered_circle(m,E,'Inner Fork Back Plane','Standoff Relief Sketch',0,PIVOT_Y,32,None,'pivot_y','2 * standoff_relief_radius')
        f=cut_extrude(m,2,name='Standoff Relief');link_feature(E,f,['standoff_relief_depth'])
    E.solve();m.ForceRebuild3(False)
    end=-43.5 if outer else 37
    es=edges_where(bodies(m)[0],lambda g:g[0]=='circle' and abs(g[1][0]-end)<1e-5 and abs(g[3]-10)<1e-5)
    select_edges(m,es);f=chamfer_equal(m,.5,'Journal Lead Chamfer');link_feature(E,f,['lead_chamfer'])
    # One master pivot axis; seat and through bore remain independently sized.
    H.centered_circle(m,E,'Plate Face Plane','Pivot Through Bore Sketch',0,PIVOT_Y,16,None,'pivot_y','pivot_bore')
    f=cut_extrude(m,9 if outer else 12,reverse=True,name='Pivot Through Bore');link_feature(E,f,['plate_thickness'])
    ref_plane(m,E,'Right Plane',-39 if outer else 44.5,'Pivot Seat Entry Plane','pivot_seat_end' if outer else 'pivot_seat_start')
    H.centered_circle(m,E,'Pivot Seat Entry Plane','Pivot Bearing Seat Sketch',0,PIVOT_Y,22,None,'pivot_y','pivot_seat_diameter')
    f=cut_extrude(m,7,reverse=not outer,name='Pivot Bearing Seat');link_feature(E,f,['bearing_width'])
    E.solve();m.ForceRebuild3(False);entry=-39 if outer else 45
    es=edges_where(bodies(m)[0],lambda g:g[0]=='circle' and abs(g[1][0]-entry)<1e-5 and abs(g[1][1]-PIVOT_Y)<1e-5 and abs(g[1][2])<1e-5 and abs(g[3]-11)<1e-5)
    select_edges(m,es);f=chamfer_equal(m,.5,'Pivot Bearing Lead In');link_feature(E,f,['lead_chamfer'])
    ref_plane(m,E,'Right Plane',-51 if outer else 57,'Shock Boss Root Plane','shock_boss_face' if outer else 'plate_end')
    H.centered_circle(m,E,'Shock Boss Root Plane','Lower Shock Boss Sketch',-SHOCK_Z,SHOCK_Y,13,'shock_z','shock_y','2 * shock_boss_radius')
    f=boss_extrude(m,3 if outer else 2.5,name='Lower Shock Boss');link_feature(E,f,['shock_boss_face - plate_start' if outer else 'shock_boss_face - plate_end'])
    E.solve();m.ForceRebuild3(False);root=-48 if outer else 57
    es=edges_where(bodies(m)[0],lambda g:g[0]=='circle' and abs(g[1][0]-root)<1e-5 and abs(g[1][1]-SHOCK_Y)<1e-5 and abs(g[1][2]-SHOCK_Z)<1e-5 and abs(g[3]-6.5)<1e-5)
    es=[e for e in es if any(wrap(face,sld.IFace2).Normal[0] < -.9 if outer else wrap(face,sld.IFace2).Normal[0] > .9 for face in e.GetTwoAdjacentFaces2())]
    select_edges(m,es);f=fillet_uniform(m,1,'Lower Shock Boss Root Fillet');link_feature(E,f,['shock_fillet'])
    ref_plane(m,E,'Right Plane',-51 if outer else 59.5,'Shock Screw Entry Plane','shock_boss_face')
    H.centered_circle(m,E,'Shock Screw Entry Plane','Lower Shock Pilot Sketch',-SHOCK_Z,SHOCK_Y,2.8,'shock_z','shock_y','thread_pilot')
    f=cut_extrude(m,10 if outer else 8.5,reverse=outer,name='Lower Shock Pilot');link_feature(E,f,['shock_thread_depth'])
    ref_plane(m,E,'Top Plane',73,'Fork Bar Seat Plane','bar_screw_y')
    s=Sketch(m,'Fork Bar Seat Plane','Fork Bar Pilot Sketch')
    for z,expr in [(BAR_SCREWS[0],'bar_screw_top_z'),(BAR_SCREWS[1],'bar_screw_bottom_z')]:
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

