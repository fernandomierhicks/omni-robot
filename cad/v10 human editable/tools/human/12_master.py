"""One-way design layout: named controls, primary hole patterns and local views."""
from common import *
from importlib import import_module
H=import_module('08_hubs')
META=REPORTS/'master_layout.json'
LENGTHS={
 'row_distance':(18,'Wheel centre to each row split'), 'roller_axis_radius':(52,'Wheel centre to roller axes'),
 'wheel_radius':(70,'Wheel contact radius'), 'tread_thickness':(2.5,'Radial TPU sleeve thickness'),
 'roller_half_length':(19.5,'Roller core half length'), 'end_cone_length':(1,'Roller tip cone length'),
 'key_width':(2.4,'Sleeve key tangential width'), 'key_length':(30,'Sleeve key axial length'), 'keyway_floor_radius':(13,'Sleeve key inner face'),
 'joint_radius':(38,'Hub joining screw pitch radius'), 'cradle_radius':(6.5,'Cradle attachment screw pitch radius'),
 'cartridge_radius':(26,'Removable cartridge screw pitch radius'), 'output_radius':(13.5,'AK40 output screw pitch radius'),
 'rear_radius':(23.5,'AK40 rear screw pitch radius'), 'cover_radius':(23.3,'Dummy cover screw pitch radius'),
 'pivot_y':(80.6,'Suspension pivot transverse coordinate'), 'pivot_z':(0,'Suspension pivot height above the axle: 0 = level with it (Nov 2)'), 'pin_x':(40.25,'Shaft retaining screw X'),
 'inner_bar_x':(51,'Inner fork bar screws X'), 'outer_bar_x':(43.5,'Outer fork bar screws toward -X'),
 'bar_mid_z':(23,'Bar screw pattern centre toward -Z'), 'bar_half_pitch':(6,'Half vertical bar screw pitch'),
 'bar_origin_x':(5,'Bar local origin X in pod frame'), 'bar_origin_z':(23,'Bar local origin toward -Z'), 'shaft_origin_x':(3,'Shaft local origin X in pod frame'),
 'inner_lower_y':(37,'Inner shock lower eye Y'), 'inner_lower_z':(11,'Inner shock lower eye Z'),
 'inner_upper_y':(37,'Inner shock upper eye Y'), 'inner_upper_z':(59.2,'Inner shock upper eye Z'),
 'outer_lower_y':(37,'Outer shock lower eye Y'), 'outer_lower_z':(11,'Outer shock lower eye Z'),
 'outer_upper_y':(37,'Outer shock upper eye Y'), 'outer_upper_z':(59.2,'Outer shock upper eye Z'),
 'guard_front_foot_x':(14,'Guard front wall foot toward -X'), 'guard_rear_foot_x':(16,'Guard rear wall foot X'),
 'guard_foot_y':(87.5,'Far side guard foot transverse position'), 'outer_ear_foot_x':(31.5,'Outer ear foot toward -X'),
 'pivot_foot_y':(94.5,'Pivot side guard foot transverse position, beyond the deck slot'), 'inner_ear_foot_x':(30,'Pivot side inner foot X'),
 'wheel_bearing_bore':(20,'6804 bearing bore'), 'wheel_bearing_od':(32,'6804 bearing outside diameter'),
 'pivot_bearing_bore':(7,'627 bearing bore and pivot shaft'), 'pivot_bearing_od':(22,'627 bearing outside diameter'),
 'motor_bearing_bore':(7,'687 bearing bore'), 'motor_bearing_od':(14,'687 bearing outside diameter')}
ANGLES={'outer_spoke_phase':(30,'Outer joint and spoke index'), 'inner_spoke_phase':(60,'Inner joint and spoke index'),
 'inner_roller_phase':(30,'Second roller row stagger'), 'cradle_phase':(90,'Cradle seed from +Y toward +Z'),
 'cartridge_phase':(30,'Cartridge seed from +Y toward +Z'), 'output_phase':(30,'Motor output seed in pod frame'),
 'rear_phase':(90,'Motor rear seed in pod frame'), 'cover_phase':(45,'Dummy cover seed in motor local frame')}
COUNTS={'roller_count':6,'cradle_count':4,'cartridge_count':3,'output_count':3,'rear_count':4,'cover_count':4}
for j,(x,y) in enumerate([(86,22),(86,46),(90,30),(97,26),(100,33.5)],1):
    LENGTHS['tower_foot_'+str(j)+'_x']=(x,'Tower foot screw '+str(j)+' X');LENGTHS['tower_foot_'+str(j)+'_y']=(y,'Tower foot screw '+str(j)+' Y')

def sketch_points(m,E,plane,name,coords):
    """Transferable ordinate dimensions refer to an in-sketch datum and anchor."""
    s=Sketch(m,plane,name);anchor=s.circle(0,0,.25);s.rel('sgCOINCIDENT',anchor.GetCenterPoint2(),origin=True);s.ddim(anchor,'anchor_diameter',(1,1))
    datum=s.circle(-100,-100,.25);dp=datum.GetCenterPoint2();s.hdim(dp,anchor.GetCenterPoint2(),'guide_datum_distance',(-60,-105));s.vdim(dp,anchor.GetCenterPoint2(),'guide_datum_distance',(-105,-60));s.ddim(datum,'anchor_diameter',(-104,-104))
    markers=[]
    for x,y,xe,ye in coords:
        ci=s.circle(x,y,.5);pt=ci.GetCenterPoint2()
        s.hdim(pt,dp,'guide_datum_distance + ('+xe+')',(x+3,y+3));dx=s.dims[-1][0]
        s.vdim(pt,dp,'guide_datum_distance + ('+ye+')',(x+5,y+5));dy=s.dims[-1][0]
        s.ddim(ci,'guide_marker_diameter',(x+2,y+2));markers.append({'index':len(markers)+2,'x':x,'y':y,'dimensions':[dx+'@'+name,dy+'@'+name]})
    for seg in s.segments():seg.ConstructionGeometry=True
    s.finish(E)
    if s.status()!=3:raise RuntimeError(name+' status '+str(s.status()))
    return {'feature':s.feat.Name,'markers':markers,'plane':plane}

def polar_points(r,a,n,re,ae,ne):
    return [(-r*math.sin(math.radians(a+360*j/n)),r*math.cos(math.radians(a+360*j/n)),f'- {re} * sin({ae} + {j} * full_turn / {ne})',f'{re} * cos({ae} + {j} * full_turn / {ne})') for j in range(n)]

def build(sw):
    m,E,path=start(sw,'master_layout','Control Measures');meta={'controls':{},'patterns':{}}
    for n,(v,t) in LENGTHS.items():E.var(n,v,'mm',t)
    for n,(v,t) in ANGLES.items():E.var(n,v,'deg',t)
    for n,v in COUNTS.items():E.var(n,v,'','Number of features in '+n.replace('_count','').replace('_',' '))
    for n,v,t in [('measure_display_pitch',3,'Control measure marker display spacing'),('guide_datum_distance',100,'In-sketch coordinate datum offset; no physical dimension'),('guide_marker_diameter',1,'Construction circle marker diameter; no physical dimension'),('anchor_diameter',.5,'Construction datum marker diameter'),('control_display_radius',10.123,'Angle control construction radius')]:E.var(n,v,'mm',t)
    E.var('full_turn',360,'deg','Full circle for layout patterns')
    s=Sketch(m,'Front Plane','Control Measures')
    for j,n in enumerate(list(LENGTHS)+list(COUNTS),1):
        if n in LENGTHS and not LENGTHS[n][0]:continue          # a diameter cannot be zero (pivot_z): its layout marker carries it
        ci=s.circle(0,j*3,(LENGTHS[n][0] if n in LENGTHS else COUNTS[n])/2);pt=ci.GetCenterPoint2();s.rel('sgVERTICALPOINTS2D',pt,origin=True);s.vdim(pt,None,f'{j} * measure_display_pitch',(3,j*3),origin_b=True);s.ddim(ci,n,(5,j*3));ci=wrap(ci,sld.ISketchSegment);ci.ConstructionGeometry=True
        meta['controls'][n]={'dimension':s.dims[-1][0]+'@Control Measures','kind':'count' if n in COUNTS else 'length'}
    s.finish(E)
    if s.status()!=3:raise RuntimeError('Control measure sketch status '+str(s.status()))
    for n,(a,_) in ANGLES.items():
        name='Angle '+n.replace('_',' ')
        s=polar_circle_sketch(m,E,'Front Plane',name,10.123,a,1,'control_display_radius',n,'guide_marker_diameter');s.finish(E)
        for seg in wrap(s.feat.GetSpecificFeature2(),sld.ISketch).GetSketchSegments():wrap(seg,sld.ISketchSegment).ConstructionGeometry=True
        meta['controls'][n]={'dimension':'D2@'+name,'kind':'angle'}
        if s.status()!=3:raise RuntimeError(name+' status '+str(s.status()))
    make_axis(m,'Layout X Axis','Top Plane','Front Plane')
    families=[('Cradle','Right Plane',6.5,90,4,'cradle_radius','cradle_phase','cradle_count'),('Outer Joint','Right Plane',38,30,6,'joint_radius','outer_spoke_phase','roller_count'),('Inner Joint','Right Plane',38,60,6,'joint_radius','inner_spoke_phase','roller_count'),('Cartridge','Right Plane',26,30,3,'cartridge_radius','cartridge_phase','cartridge_count'),('Output','Right Plane',13.5,30,3,'output_radius','output_phase','output_count'),('Output Local','Front Plane',13.5,90,3,'output_radius','full_turn / output_count - output_phase','output_count'),('Rear','Right Plane',23.5,90,4,'rear_radius','rear_phase','rear_count'),('Rear Local','Front Plane',23.5,90,4,'rear_radius','full_turn / 2 - rear_phase','rear_count'),('Cover Local','Front Plane',23.3,45,4,'cover_radius','cover_phase','cover_count')]
    for name,plane,r,a,n,re,ae,ne in families:meta['patterns'][name]=sketch_points(m,E,plane,'Layout '+name,polar_points(r,a,n,re,ae,ne))
    def add(name,plane,coords):meta['patterns'][name]=sketch_points(m,E,plane,'Layout '+name,coords)
    guard=[(-14,-87.5,'- guard_front_foot_x','- guard_foot_y'),(16,-87.5,'guard_rear_foot_x','- guard_foot_y'),(-14,94.5,'- guard_front_foot_x','pivot_foot_y'),(16,94.5,'guard_rear_foot_x','pivot_foot_y'),(30,94.5,'inner_ear_foot_x','pivot_foot_y'),(-31.5,94.5,'- outer_ear_foot_x','pivot_foot_y')]
    add('Guard Feet','Front Plane',guard)
    tower=[(LENGTHS['tower_foot_'+str(j)+'_x'][0],LENGTHS['tower_foot_'+str(j)+'_y'][0],'tower_foot_'+str(j)+'_x','tower_foot_'+str(j)+'_y') for j in range(1,6)];add('Tower Feet','Front Plane',tower)
    add('Pivot','Right Plane',[(0,80.6,'- pivot_z','pivot_y')]);add('Pin','Front Plane',[(40.25,80.6,'pin_x','pivot_y')]);add('Pin Local','Front Plane',[(37.25,0,'pin_x - shaft_origin_x','0')])
    for label,pre in [('Inner Lower','inner_lower'),('Inner Upper','inner_upper'),('Outer Lower','outer_lower'),('Outer Upper','outer_upper')]:
        add(label+' Shock','Right Plane',[(-LENGTHS[pre+'_z'][0],LENGTHS[pre+'_y'][0],'- '+pre+'_z',pre+'_y')])
    add('Fork Bar','Top Plane',[(x,z,xe,ze) for x,xe in [(-43.5,'- outer_bar_x'),(51,'inner_bar_x')] for z,ze in [(17,'bar_mid_z - bar_half_pitch'),(29,'bar_mid_z + bar_half_pitch')]])
    add('Fork Bar Local','Top Plane',[(x,z,xe,ze) for x,xe in [(-48.5,'- outer_bar_x - bar_origin_x'),(46,'inner_bar_x - bar_origin_x')] for z,ze in [(-6,'bar_mid_z - bar_half_pitch - bar_origin_z'),(6,'bar_mid_z + bar_half_pitch - bar_origin_z')]])
    coords=[(-18,52,'- row_distance','roller_axis_radius'),(18,52,'row_distance','roller_axis_radius')];add('Roller Axles','Front Plane',coords)
    rp=H.angle_plane(m,E,'Front Plane','Layout X Axis',30,'inner_roller_phase','Inner Roller Guide Plane');add('Inner Roller Axles',rp,coords)
    E.solve();m.ForceRebuild3(False);r=audit_report(m);r['pass']=not r['feature_errors'] and not r['dead_equations'] and not r['unlinked_dimensions'] and r['body_checks']==[] and all(s['status']==3 for s in r['sketches'])
    if not r['pass']:raise RuntimeError('Master audit failed '+str(r['feature_errors']))
    m.FeatureManager.EnableFeatureTree=True;m.FeatureManager.EnableFeatureTreeWindow=True;wrap(m.ActiveView,sld.IModelView).EnableGraphicsUpdate=True
    sw.ActivateDoc3(m.GetTitle(),False,0,0);ok,e,w=m.Extension.SaveAs3(str(path),0,1,None,None,0,0)
    if not ok or e:raise RuntimeError('Master save failed '+str((e,w)))
    meta['audit']=r;meta['path']=str(path);META.write_text(json.dumps(meta,indent=2));print('Master layout saved',len(meta['patterns']),'coordinate views;',len(meta['controls']),'named controls;',len(r['sketches']),'fully constrained sketches',flush=True)

if __name__=='__main__':
    sw=connect()
    try:build(sw)
    finally:sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,True)
