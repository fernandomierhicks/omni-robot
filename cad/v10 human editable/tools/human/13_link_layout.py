"""Native derived layout sketches and concentric slave constraints."""
from common import *
from importlib import import_module
H=import_module('08_hubs')
LINK='Shared Layout'
META=json.loads((REPORTS/'master_layout.json').read_text())

def source_dim(name):
    d,sk=META['controls'][name]['dimension'].split('@',1)
    value='"'+d+'@'+sk+'-'+LINK+'"'
    return '('+value+' / 1mm)' if META['controls'][name]['kind']=='count' else value

def replace_equation(em,lhs,rhs,comment='Driven by the shared master layout'):
    for i in range(em.GetCount()):
        if em.Equation(i).split('=',1)[0].strip().strip('"')==lhs:
            txt='"'+lhs+'" = '+rhs+" '"+comment
            em.SetEquation(i,txt)
            if em.Equation(i).split("'",1)[0].replace(' ','')!=txt.split("'",1)[0].replace(' ',''):raise RuntimeError('Equation setter refused: activate the target document before editing '+lhs)
            return
    raise RuntimeError('No existing equation '+lhs)

def aliases(key):
    a={}
    def use(own,master=None,factor=1):a[own]=source_dim(master or own)+((' * '+str(factor)) if factor!=1 else '')
    if key.startswith('wheel_hub_'):
        for n in ('row_distance','roller_axis_radius','wheel_radius','roller_half_length','end_cone_length','joint_radius','outer_spoke_phase','inner_spoke_phase','inner_roller_phase','roller_count'):use(n)
        use('cartridge_boss_radius','cartridge_radius');use('flange_pcd','output_radius',2);use('mount_first_angle','cartridge_phase');use('cartridge_count');use('flange_screw_count','output_count');use('bearing_radius','wheel_bearing_od',.5)
        a['bearing_shoulder_radius']=source_dim('wheel_bearing_od')+' / 2 - 2.5mm'
    if key in ('roller_core','roller_tread'):
        for n in ('wheel_radius','roller_axis_radius','tread_thickness','roller_half_length','end_cone_length','key_width','key_length','keyway_floor_radius'):use(n)
    if key=='wheel_cartridge':
        use('screw_pcd','cartridge_radius',2);use('first_screw_angle','cartridge_phase');use('screw_count','cartridge_count');use('bearing_seat_radius','wheel_bearing_od',.5)
        a['shoulder_bore_radius']=source_dim('wheel_bearing_od')+' / 2 - 2.5mm'
    if key=='motor_cup':
        use('fork_screw_radius','cradle_radius');use('fork_first_angle','cradle_phase');use('screw_count','cradle_count');use('rear_screw_pcd','rear_radius',2);use('rear_first_angle','rear_phase');use('rear_mount_count','rear_count')
    if key in ('inner_fork_6804','outer_fork_6804'):
        use('pivot_y')       # the pivot's height is level with the axle (Nov 2): a relation, not a dimension
        use('pivot_seat_diameter','pivot_bearing_od');use('journal_radius','wheel_bearing_bore',.5)
        a['journal_shoulder_radius']=source_dim('wheel_bearing_bore')+' / 2 + 1.5mm';a['pivot_bore']=source_dim('pivot_bearing_od')+' - 6mm'
        pre='outer_lower' if key.startswith('outer') else 'inner_lower';use('shock_y',pre+'_y');use('shock_z',pre+'_z')
        use('bar_screw_x','outer_bar_x' if key.startswith('outer') else 'inner_bar_x')
        a['bar_screw_top_z']=source_dim('bar_mid_z')+' - '+source_dim('bar_half_pitch');a['bar_screw_bottom_z']=source_dim('bar_mid_z')+' + '+source_dim('bar_half_pitch')
        if key.startswith('inner'):use('cup_screw_radius','cradle_radius');use('cup_first_angle','cradle_phase');use('cup_screw_count','cradle_count')
    if key in ('bench_plate','wheel_guard','pod_mount'):
        for n in ('guard_front_foot_x','guard_rear_foot_x','guard_foot_y','outer_ear_foot_x'):
            if key!='pod_mount':use(n)
        if key!='pod_mount':use('inner_ear_foot_x');use('pivot_foot_y')
        for j in range(1,6):
            for ax in ('x','y'):use('tower_foot_'+str(j)+'_'+ax)
    if key=='wheel_guard':
        for n in ('pivot_y','pin_x'):use(n)
        use('outer_shock_y','outer_upper_y');use('outer_shock_z','outer_upper_z');use('lower_eye_z','outer_lower_z');use('shaft_bore','pivot_bearing_bore')
        a['shaft_bore']+=' + 0.1mm'
    if key=='pod_mount':use('shock_y','inner_upper_y');use('shock_z','inner_upper_z')
    if key=='fork_bar':
        use('screw_half_pitch','bar_half_pitch');a['outer_screw_x']=source_dim('outer_bar_x')+' + '+source_dim('bar_origin_x');a['inner_screw_x']=source_dim('inner_bar_x')+' - '+source_dim('bar_origin_x')
    if key=='pivot_shaft':use('shaft_diameter','pivot_bearing_bore')
    if key in ('pivot_shim','pivot_outer_spacer'):a['bore_diameter']=source_dim('pivot_bearing_bore')+' + 0.2mm'
    if key=='dummy_motor_stator':
        use('rear_mount_pcd','rear_radius',2);use('rear_screw_count','rear_count')
        a['rear_first_angle']='180deg - '+source_dim('rear_phase')
        use('bearing_seat_radius','motor_bearing_od',.5)
    if key=='dummy_motor_rotor':
        use('output_screw_pcd','output_radius',2);use('output_screw_count','output_count');a['output_first_angle']='360deg / '+source_dim('output_count')+' - '+source_dim('output_phase')
        a['shaft_radius']=source_dim('motor_bearing_bore')+' / 2 - 0.05mm';a['inner_race_shoulder_radius']=source_dim('motor_bearing_bore')+' / 2 + 1mm'
    for kind,bore,od in [('6804','wheel_bearing_bore','wheel_bearing_od'),('627','pivot_bearing_bore','pivot_bearing_od'),('687','motor_bearing_bore','motor_bearing_od')]:
        if key=='bearing_'+kind:use('bore_diameter',bore);use('outside_diameter',od)
    return a

def bindings(key):
    out=[]
    def add(pattern,*sketches):out.extend((sk,pattern) for sk in sketches)
    if key.startswith('wheel_hub_'):
        if key.endswith('A'):add('Cartridge','Cartridge Boss Sketch','Cartridge Pilot Sketch');add('Output','Motor Flange Sketch')
        for label,pattern,axle in [('Outer','Outer Joint','Roller Axles'),('Inner','Inner Joint','Inner Roller Axles')]:
            if (key.endswith('A') and label=='Inner') or (key.endswith('C') and label=='Outer'):continue
            add(pattern,label+' Joint Pad Sketch',label+' Joint Hole Sketch')
            if not key.endswith('B'):add(pattern,label+' Joint Counterbore Sketch')
            add(axle,*[label+' '+n for n in ['Axle Block Sketch','Axle Hole Sketch','Head Tunnel Sketch','Nut Pocket Sketch','Nut Tunnel Sketch']])
    if key=='motor_cup':add('Cradle','Fork Pilot Sketch');add('Rear','Rear Mount Sketch')
    if key=='wheel_cartridge':add('Cartridge','Mounting Hole Sketch')
    if key=='dummy_motor_stator':add('Rear Local','Rear Mount Pilot Sketch')
    if key=='dummy_motor_rotor':add('Output Local','Output Boss Sketch','Output Pilot Sketch')
    if key in ('inner_fork_6804','outer_fork_6804'):
        add('Pivot','Pivot Through Bore Sketch','Pivot Bearing Seat Sketch');add('Fork Bar','Fork Bar Pilot Sketch')
        add('Outer Lower Shock' if key.startswith('outer') else 'Inner Lower Shock','Lower Shock Boss Sketch','Lower Shock Pilot Sketch')
        if key.startswith('inner'):add('Cradle','Cradle Clearance Sketch','Cradle Counterbore Sketch');add('Pivot','Standoff Relief Sketch')
        else:add('Outer Lower Shock','Shock Eye Relief Sketch')
    if key=='wheel_guard':
        add('Pivot','Inner Shaft Bore Sketch','Outer Shaft Bore Sketch');add('Pin','Pin Pilot Sketch');add('Outer Upper Shock','Fixed Shock Boss Sketch','Fixed Shock Pilot Sketch');add('Outer Lower Shock','Lower Shock Driver Sketch');add('Guard Feet','Deck Foot Pilot Sketch');add('Pivot','Outer Lobe Clearance Sketch')
    if key=='pod_mount':add('Inner Upper Shock','Shock Spacer Sketch','Shock Hinge Sketch');add('Tower Feet','Tower Foot Pilot Sketch')
    if key=='bench_plate':add(['Guard Feet','Tower Feet'],'Pod Foot Clearance Sketch','Foot Counterbore Sketch')
    if key=='fork_bar':add('Fork Bar Local','Bar Screw Sketch')
    if key=='pivot_shaft':add('Pin Local','Cross Hole Sketch')
    return out

def transform_point(p,mat):
    return [sum(p[i]*mat[3*i+j] for i in range(3))*mat[12]+mat[9+j] for j in range(3)]

def bind_sketch(m,skname,patterns,repair=False):
    sf=feat_by_name(m,skname)
    if sf is None:raise RuntimeError('Missing slave sketch '+skname)
    # Remove the location dimensions; diameters and arc radii remain independent fits.
    remove=[];dd=wrap(sf.GetFirstDisplayDimension(),sld.IDisplayDimension)
    while dd:
        d=wrap(dd.GetDimension2(0),sld.IDimension);name='@'.join(d.FullName.split('@')[:2])
        if name.split('@')[1]==skname and dd.Type2 not in (c.swDiameterDimension,c.swRadialDimension):remove.append(name)
        dd=wrap(sf.GetNextDisplayDimension(dd),sld.IDisplayDimension)
    sk=wrap(sf.GetSpecificFeature2(),sld.ISketch)
    circles=[wrap(z,sld.ISketchArc) for z in sk.GetSketchSegments() if wrap(z,sld.ISketchSegment).GetType()==1]
    refs=[]
    for pattern in patterns if isinstance(patterns,list) else [patterns]:
        spec=META['patterns'][pattern];f=feat_by_name(m,spec['feature']+'-'+LINK)
        if f is None:raise RuntimeError('Missing derived pattern '+pattern)
        refsk=wrap(f.GetSpecificFeature2(),sld.ISketch);segs=refsk.GetSketchSegments()
        refmat=wrap(refsk.ModelToSketchTransform,sld.IMathTransform)
        inverse=wrap(refmat.Inverse(),sld.IMathTransform).ArrayData
        ownmat=wrap(sk.ModelToSketchTransform,sld.IMathTransform).ArrayData
        if max(abs(refmat.ArrayData[i]-ownmat[i]) for i in range(9))>1e-7:raise RuntimeError('Master/slave plane orientation mismatch: '+skname+' / '+pattern)
        for marker in spec['markers']:
            seg=wrap(segs[marker['index']],sld.ISketchSegment)
            pt=wrap(wrap(seg,sld.ISketchArc).GetCenterPoint2(),sld.ISketchPoint)
            projection=transform_point(transform_point([pt.X,pt.Y,0],inverse),ownmat)
            if math.hypot(projection[0]-pt.X,projection[1]-pt.Y)>1e-7:raise RuntimeError('Master/slave sketch frame mismatch: '+skname+' / '+pattern)
            refs.append((marker,seg))
    if not remove and not repair:return {'sketch':skname,'already_bound':True}
    matched=[]
    for arc in circles:
        cp=wrap(arc.GetCenterPoint2(),sld.ISketchPoint);best=min(refs,key=lambda pair:(pair[0]['x']-cp.X*1000)**2+(pair[0]['y']-cp.Y*1000)**2)
        error=math.hypot(best[0]['x']-cp.X*1000,best[0]['y']-cp.Y*1000)
        if error>1e-4 and not repair:raise RuntimeError(skname+' centre mismatch '+str((cp.X*1000,cp.Y*1000,error)))
        matched.append((arc,best[1]))
    m.ClearSelection2(True);sf.Select2(False,0);m.SketchManager.InsertSketch(True);em=wrap(m.GetEquationMgr(),sld.IEquationMgr)
    if repair:
        rm=wrap(wrap(m.SketchManager.ActiveSketch,sld.ISketch).RelationManager,sld.ISketchRelationManager)
        for raw in rm.GetRelations(0) or []:
            rel=wrap(raw,sld.ISketchRelation)
            if rel.GetRelationType()==c.swConstraintType_CONCENTRIC:rm.DeleteRelation(rel)
    for dim in remove:
        for i in range(em.GetCount()-1,-1,-1):
            if em.Equation(i).split('=',1)[0].strip().strip('"')==dim:em.Delete(i)
        m.ClearSelection2(True)
        if not m.Extension.SelectByID2(dim,'DIMENSION',0,0,0,False,0,None,0) or not m.Extension.DeleteSelection2(0):raise RuntimeError('Cannot remove local coordinate '+dim)
    if skname=='Cross Hole Sketch':
        rm=wrap(wrap(m.SketchManager.ActiveSketch,sld.ISketch).RelationManager,sld.ISketchRelationManager)
        for raw in rm.GetRelations(0) or []:
            rel=wrap(raw,sld.ISketchRelation)
            # The original single circle has only one geometric relation: centre Y=0.
            try:
                entities=rel.GetEntities() or []
                if len(entities)==2 and all(hasattr(e,'X') for e in entities):rm.DeleteRelation(rel)
            except Exception:pass
    # A zero coordinate (the pivot level with the axle) is held by a point-alignment relation with the origin; the concentric
    # relation to the master replaces it (keeping both would over-define the centre).
    centres=[(wrap(a.GetCenterPoint2(),sld.ISketchPoint).X,wrap(a.GetCenterPoint2(),sld.ISketchPoint).Y) for a,_ in matched]
    rm=wrap(wrap(m.SketchManager.ActiveSketch,sld.ISketch).RelationManager,sld.ISketchRelationManager)
    for raw in rm.GetRelations(0) or []:
        rel=wrap(raw,sld.ISketchRelation)
        try:
            ents=rel.GetEntities() or []
            if len(ents)==2 and all(hasattr(e,'X') for e in ents) and any(abs(e.X-cx)<1e-9 and abs(e.Y-cy)<1e-9 for e in ents for cx,cy in centres):rm.DeleteRelation(rel)
        except Exception:pass
    for arc,ref in matched:
        m.ClearSelection2(True);wrap(arc,sld.ISketchSegment).Select4(False,None);ref.Select4(True,None);m.SketchAddConstraints('sgCONCENTRIC')
    status=wrap(m.SketchManager.ActiveSketch,sld.ISketch).GetConstrainedStatus();m.SketchManager.InsertSketch(True)
    if status!=3:raise RuntimeError(skname+' linked sketch status '+str(status))
    return {'sketch':skname,'pattern':patterns,'circles':len(matched),'status':status}

def apply(sw,key):
    m,E,path=H.resume(sw,key);before=json.loads((REPORTS/(key+'.json')).read_text())['volume_mm3'];part=wrap(m,sld.IPartDoc)
    if feat_by_name(m,LINK) is None:
        f=wrap(part.InsertPart3(str(OUT/'V10H_master_layout.SLDPRT'),4|8|64|1024,''),sld.IFeature)
        if f is None:raise RuntimeError('Cannot derive layout into '+key)
        f.Name=LINK;first=json.loads(REGISTRY.read_text())[key]['first_feature']
        if not m.Extension.ReorderFeature(LINK,first,2):raise RuntimeError('Cannot order derived layout before design')
    else:
        f=feat_by_name(m,LINK);fd=wrap(f.GetDefinition(),sld.IDerivedPartFeatureData)
        if not fd.ImportPlane or not fd.ImportAxis:
            if not fd.AccessSelections(m,None):raise RuntimeError('Cannot enable master datum references')
            fd.ImportPlane=True;fd.ImportAxis=True
            if not f.ModifyDefinition(fd,m,None):raise RuntimeError('Cannot update derived layout datums')
    em=wrap(m.GetEquationMgr(),sld.IEquationMgr)
    if key=='motor_cup':
        if not any(em.Equation(i).split('=',1)[0].strip()=='"rear_mount_count"' for i in range(em.GetCount())):E.var('rear_mount_count',4,'','Rear motor mounting hole count from shared layout')
        replace_equation(em,'D1@Rear Mount Pattern','"rear_mount_count"')
    existing={em.Equation(i).split('=',1)[0].strip().strip('"') for i in range(em.GetCount())}
    for name,rhs in aliases(key).items():
        if name in existing:replace_equation(em,name,rhs)
    links=[bind_sketch(m,sk,pat) for sk,pat in bindings(key)]
    for _ in range(2):em.EvaluateAll();m.ForceRebuild3(False)
    if abs(volume_mm3(m)-before)>max(.01,before*1e-7):raise RuntimeError('Layout linking changed nominal geometry '+key+' '+str(volume_mm3(m)-before))
    m.ClearSelection2(True);selected=0
    for f in features(m):
        if f.GetTypeName2()=='ProfileFeature' and f.Name.endswith('-'+LINK):f.Select2(selected>0,0);selected+=1
    if selected:m.BlankSketch()
    m.ClearSelection2(True);finish(sw,m,E,key,path)
    (REPORTS/('links_'+key+'.json')).write_text(json.dumps({'part':key,'master':META['path'],'links':links,'aliases':aliases(key)},indent=2));print('Linked',key,len(links),'sketches',flush=True)
    close_owned(sw,m,save=True)

if __name__=='__main__':
    sw=connect()
    try:
        keys=sys.argv[1:] or sorted(k for k in json.loads(REGISTRY.read_text()) if k!='master_layout' and (aliases(k) or bindings(k)))
        for key in keys:apply(sw,key)
    finally:sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,True)
